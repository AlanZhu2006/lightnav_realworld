import threading
import time
import unittest
from deployment.imagegoal.preview import LatestFramePreview


class PreviewTests(unittest.TestCase):
    def test_slow_encoder_retains_only_newest_pending_frame(self):
        entered,released,done=threading.Event(),threading.Event(),threading.Event()
        seen=[]
        def publish(frame):
            seen.append(frame)
            if frame==0:
                entered.set()
                released.wait(1)
            else:
                done.set()
        worker=LatestFramePreview(publish,hz=100,max_age_s=2)
        try:
            source=time.time_ns()
            worker.submit(0,source)
            self.assertTrue(entered.wait(1))
            for i in range(1,101):
                worker.submit(i,source+i)
            released.set()
            self.assertTrue(done.wait(1))
            self.assertEqual(seen,[0,100])
            self.assertEqual(worker.snapshot()['replaced'],99)
        finally:
            released.set()
            worker.close()

    def test_never_refreshes_old_source_and_rejects_reordering(self):
        done=threading.Event()
        seen=[]
        worker=LatestFramePreview(lambda frame:(seen.append(frame),done.set()),hz=100,max_age_s=.1)
        try:
            worker.submit('old',time.time_ns()-1_000_000_000)
            deadline=time.monotonic()+1
            while worker.snapshot()['source_age_dropped']<1 and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertEqual(worker.snapshot()['source_age_dropped'],1)
            source=time.time_ns()
            worker.submit('fresh',source)
            self.assertTrue(done.wait(1))
            worker.submit('reordered',source-1)
            self.assertEqual(worker.snapshot()['source_timestamp_ns'],source)
            self.assertEqual(seen,['fresh'])
        finally:
            worker.close()

    def test_bad_future_stamp_does_not_block_following_real_camera_frames(self):
        done=threading.Event()
        seen=[]
        worker=LatestFramePreview(lambda frame:(seen.append(frame),done.set()),hz=100,max_age_s=.3)
        try:
            worker.submit("future",time.time_ns()+60_000_000_000)
            worker.submit("real",time.time_ns())
            self.assertTrue(done.wait(1))
            self.assertEqual(seen,["real"])
            self.assertEqual(worker.snapshot()["source_age_dropped"],1)
        finally:
            worker.close()

    def test_preview_has_own_rate_limit_and_shutdown_drops_pending(self):
        first=threading.Event()
        seen=[]
        worker=LatestFramePreview(lambda frame:(seen.append(frame),first.set()),hz=2,max_age_s=2)
        source=time.time_ns()
        worker.submit(1,source)
        self.assertTrue(first.wait(1))
        worker.submit(2,source+1)
        time.sleep(.05)
        self.assertEqual(seen,[1])
        worker.close()
        self.assertFalse(worker.worker.is_alive())
        self.assertEqual(seen,[1])


if __name__=='__main__':
    unittest.main()
