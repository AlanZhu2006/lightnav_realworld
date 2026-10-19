"""Scheduler regressions using synthetic RGB and protocol completions only.

The real runtime and history validator run without a model, ROS, or Go2 SDK.
Inference dispatch and finished-session cleanup are replaced with recorders.
"""
from __future__ import annotations

import base64
import io
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image

from deployment.imagegoal import runtime as runtime_module
from deployment.imagegoal.runtime import ImageGoalRuntime, RuntimeConfig


SECOND = 1_000_000_000


class LightNavHistorySchedulingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.runtime = ImageGoalRuntime(Path(self.directory.name)/"run",RuntimeConfig(profile="simulated-replay"))
        self.runtime._schedule_lightnav_cleanup = Mock()
        self.runtime._dispatch = Mock()
        stream = io.BytesIO()
        Image.new("RGB",(32,24),(30,80,120)).save(stream,format="JPEG")
        self.encoded = base64.b64encode(stream.getvalue()).decode("ascii")
        self.now = time.time_ns()
        self.number = 0
        with patch.object(runtime_module.time,"time_ns",return_value=self.now-SECOND):
            self.runtime.create_task({"backend":"lightnav","task_mode":"imagegoal","timeout_s":120,
                                      "goal":{"rgb_jpeg_base64":self.encoded}})

    def tearDown(self):
        self.runtime.close()
        self.directory.cleanup()

    def ingest(self,stamp,*,now=None):
        self.number += 1
        with patch.object(runtime_module.time,"time_ns",return_value=now or self.now):
            self.runtime.ingest_rgb({"frame_id":f"synthetic-{self.number}","rgb_jpeg_base64":self.encoded,
                "capture_timestamp_ns":stamp,"clock_domain":"unix_epoch_ns",
                "timestamp_source":"synthetic_history_regression","frame_name":"synthetic_rgb",
                "synthetic":True})

    def backlog(self):
        self.runtime.task["goal_started_timestamp_ns"] = self.now-30*SECOND
        self.runtime.task["last_lightnav_ack_stamp_ns"] = self.now-20*SECOND
        for index in range(80):
            self.ingest(self.now-(79-index)*250_000_000)

    def tick(self,now=None):
        with patch.object(runtime_module.time,"time_ns",return_value=now or self.now):
            self.runtime.tick()

    def assert_zero(self):
        self.assertTrue(all(self.runtime.proposal[key]==0 for key in ("vx_m_s","vy_m_s","wz_rad_s")))

    def valid_feedback(self, *, include_depth=True):
        """Real ingestion paths with explicit synthetic, fresh source data."""
        common = {"capture_timestamp_ns":self.now,"clock_domain":"unix_epoch_ns",
                  "timestamp_source":"synthetic_history_regression","synthetic":True}
        with patch.object(runtime_module.time,"time_ns",return_value=self.now):
            self.runtime.ingest_odom({**common,"frame_name":"synthetic_odom",
                                     "pose_m_rad":[0.,0.,0.],"velocity_v_w":[0.,0.]})
            if include_depth:
                depth = io.BytesIO()
                Image.fromarray(np.full((24,32),2000,dtype=np.uint16)).save(depth,"PNG")
                self.runtime.ingest_depth({**common,"frame_name":"synthetic_rgb",
                    "depth_png_base64":base64.b64encode(depth.getvalue()).decode(),
                    "scale_m":.001,"aligned_to_rgb":True})

    def queue_model_stop(self, frames=None, *, capture_pose=(0.,0.,0.)):
        frames = list(self.runtime.frames) if frames is None else list(frames)
        task = self.runtime.task
        observations = [{"frame_id":frame.frame_id,"capture_timestamp_ns":frame.stamp} for frame in frames]
        identity = {"session_id":self.runtime.session_id,"task_id":task["task_id"],
                    "goal_id":task["goal_id"],"goal_version":task["goal_version"],"frame_id":observations[-1]["frame_id"]}
        metadata = {"request_id":"synthetic-stop","identity":identity,"epoch":self.runtime.epoch,
                    "backend":"lightnav","source_stamp_ns":frames[-1].stamp,"created_monotonic_ns":time.monotonic_ns(),
                    "capture_pose":None if capture_pose is None else list(capture_pose),"capture_odom_gap_s":0.}
        payload = {"identity":identity,"observations":observations,
                   "task_mode":"imagegoal","conditioning_mode":"experimental_reference_image"}
        response = {"schema_version":"lightnav.imagegoal.inference.v1","identity":identity,
                    "observation_frames":observations,"source_frame":observations[-1],
                    "task_mode":"imagegoal","implementation":"go2_imagegoal_adaptation",
                    "official_release_commit":runtime_module.OFFICIAL_COMMIT,
                    "official_core_sha256":runtime_module.OFFICIAL_CORE_DIGEST,
                    "eval_task":"vlnce","goal_image_consumed":True,
                    "conditioning_mode":"experimental_reference_image","trajectory_body_m":[[0.,0.]]*10,
                    "trajectory_body_m_rad":[[0.,0.,0.]]*10,"model_stop":True,"arrival_confirmed":False}
        self.runtime.completed.put((metadata,payload,response,10.,None))
        return metadata

    def rejection_reasons(self):
        return [reason for event in self.runtime.events if event["kind"]=="inference_rejected"
                for reason in event["reasons"]]

    def test_model_stop_completion_survives_native_batch_overflow(self):
        for index in range(4):
            self.ingest(self.now-(3-index)*250_000_000)
        self.valid_feedback()
        self.assertEqual(self.runtime._sensor_gates(self.now),[])
        task = self.runtime.task
        self.queue_model_stop()
        self.tick()
        self.assertEqual(self.runtime.state,"SUCCEEDED")
        self.assertTrue(task["model_stop_latched"])
        self.assertEqual(task["control_profile"],"go2_imagegoal_adaptation")
        outcome = dict(task["outcome"])
        self.assertEqual(outcome["reason"],"go2_imagegoal_model_stop")
        self.assertFalse(outcome["arrival_independently_validated"])
        self.runtime._schedule_lightnav_cleanup.assert_called_once_with(task["task_id"])
        # Later RGB cannot turn a completed task into a backlog failure, recreate
        # its session, or dispatch another inference request.
        future = self.now+20*SECOND
        for index in range(1,81):
            self.ingest(self.now+index*250_000_000,now=future)
        with patch.object(self.runtime,"_window",wraps=self.runtime._window) as window:
            for _ in range(3):
                self.tick(future)
            window.assert_not_called()
        self.assertEqual(self.runtime.state,"SUCCEEDED")
        self.assertEqual(task["outcome"],outcome)
        self.assertIn("no_active_task",self.runtime.gates)
        self.runtime._schedule_lightnav_cleanup.assert_called_once_with(task["task_id"])
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_stale_model_stop_does_not_latch_and_next_request_can_continue(self):
        # Keep the unsent suffix inside the bounded upload budget while making
        # this completion too old for motion. A larger backlog is covered by
        # the separate fail-fast tests below and must not be silently dropped.
        self.runtime.config.max_plan_age_s = 1.
        self.runtime.task["goal_started_timestamp_ns"] = self.now-10*SECOND
        for index in range(10):
            self.ingest(self.now-(9-index)*250_000_000)
        old_frames = list(self.runtime.frames)[:4]
        self.valid_feedback()
        self.assertEqual(self.runtime._sensor_gates(self.now),[])
        self.queue_model_stop(old_frames)
        epoch = self.runtime.epoch
        self.tick()
        self.assertFalse(self.runtime.task["model_stop_latched"])
        self.assertEqual(self.runtime.epoch,epoch)
        self.assertNotEqual(self.runtime.state,"SUCCEEDED")
        self.assertIn("plan_source_stale_or_future",self.rejection_reasons())
        self.assertNotIn("model_stop_not_arrival",self.rejection_reasons())
        self.assertEqual(self.runtime.task["last_lightnav_ack_stamp_ns"],old_frames[-1].stamp)
        self.runtime._dispatch.assert_called_once()
        pending = self.runtime._dispatch.call_args.args[0]
        self.assertEqual(pending[0].frame_id,old_frames[-1].frame_id)
        self.assertEqual(pending[-1].frame_id,self.runtime.frames[-1].frame_id)
        self.assertIsNone(self.runtime.task["outcome"])
        self.assert_zero()

    def test_fresh_model_stop_with_missing_depth_does_not_latch(self):
        for index in range(4):
            self.ingest(self.now-(3-index)*250_000_000)
        self.valid_feedback(include_depth=False)
        self.queue_model_stop()
        epoch = self.runtime.epoch
        self.tick()
        self.assertFalse(self.runtime.task["model_stop_latched"])
        self.assertEqual(self.runtime.epoch,epoch)
        self.assertIn("depth_missing",self.rejection_reasons())
        self.assertNotIn("model_stop_not_arrival",self.rejection_reasons())
        self.assertNotEqual(self.runtime.state,"SUCCEEDED")
        self.assert_zero()

    def test_fresh_model_stop_without_capture_pose_does_not_latch(self):
        for index in range(4):
            self.ingest(self.now-(3-index)*250_000_000)
        self.valid_feedback()
        self.queue_model_stop(capture_pose=None)
        epoch = self.runtime.epoch
        self.tick()
        self.assertFalse(self.runtime.task["model_stop_latched"])
        self.assertEqual(self.runtime.epoch,epoch)
        self.assertIn("capture_odom_unavailable",self.rejection_reasons())
        self.assertNotIn("model_stop_not_arrival",self.rejection_reasons())
        self.assert_zero()

    def test_model_stop_from_invalidated_epoch_does_not_latch(self):
        for index in range(4):
            self.ingest(self.now-(3-index)*250_000_000)
        self.valid_feedback()
        metadata = self.queue_model_stop()
        with patch.object(runtime_module.time,"time_ns",return_value=self.now):
            self.runtime._invalidate("offline_epoch_barrier")
        epoch = self.runtime.epoch
        self.tick()
        self.assertFalse(self.runtime.task["model_stop_latched"])
        self.assertEqual(self.runtime.epoch,epoch)
        # A verified server history commit is acknowledged independently of
        # plan eligibility. It cannot authorize this old execution epoch.
        self.assertEqual(self.runtime.task["last_lightnav_ack_stamp_ns"],metadata["source_stamp_ns"])
        self.assertEqual(self.runtime.metrics["inference_accepted"],0)
        self.assertIsNone(self.runtime.plan)
        self.assertIn("execution_epoch_invalidated",self.rejection_reasons())
        self.assertNotIn("model_stop_not_arrival",self.rejection_reasons())
        self.assertNotEqual(self.runtime.state,"SUCCEEDED")
        self.assert_zero()

    def test_pause_survives_native_batch_overflow(self):
        self.backlog()
        self.runtime.task_action("pause",{})
        with patch.object(self.runtime,"_window",wraps=self.runtime._window) as window:
            self.tick()
            window.assert_not_called()
        self.assertEqual(self.runtime.state,"PAUSED")
        self.assertIsNone(self.runtime.task["outcome"])
        self.assertIn("operator_paused",self.runtime.gates)
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_resume_checks_backlog_without_resetting_or_dropping_history(self):
        self.backlog()
        self.runtime.task_action("pause",{})
        self.tick()
        before = [(frame.frame_id,frame.stamp) for frame in self.runtime.frames]
        version = self.runtime.task["goal_version"]
        self.runtime.task_action("resume",{})
        self.tick()
        self.assertEqual(self.runtime.state,"FAILED")
        self.assertEqual(self.runtime.task["outcome"]["reason"],"lightnav_native_history_gap_exceeded")
        self.assertEqual(self.runtime.task["goal_version"],version)
        self.assertEqual(before,[(frame.frame_id,frame.stamp) for frame in self.runtime.frames])
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_inflight_request_does_not_prefetch_or_fail_a_window(self):
        self.backlog()
        self.runtime.inflight = {"request_id":"synthetic-inflight","epoch":self.runtime.epoch,
                                 "identity":{"task_id":self.runtime.task["task_id"]},"source_stamp_ns":self.now}
        with patch.object(self.runtime,"_window",wraps=self.runtime._window) as window:
            self.tick()
            window.assert_not_called()
        self.assertEqual(self.runtime.state,"PLANNING")
        self.assertIsNone(self.runtime.task["outcome"])
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_active_native_batch_overflow_still_fails(self):
        self.backlog()
        self.tick()
        self.assertEqual(self.runtime.state,"FAILED")
        self.assertEqual(self.runtime.task["outcome"]["reason"],"lightnav_native_history_gap_exceeded")
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_active_missing_ack_overlap_still_fails(self):
        self.runtime.task["goal_started_timestamp_ns"] = self.now-30*SECOND
        self.runtime.task["last_lightnav_ack_stamp_ns"] = self.now-20*SECOND
        for index in range(4):
            self.ingest(self.now-(3-index)*250_000_000)
        self.tick()
        self.assertEqual(self.runtime.state,"FAILED")
        self.assertEqual(self.runtime.task["outcome"]["reason"],"lightnav_native_history_gap_exceeded")
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_active_continuous_history_dispatches_all_unsent_frames(self):
        self.runtime.task["goal_started_timestamp_ns"] = self.now-3*SECOND
        for index in range(9):
            self.ingest(self.now-(8-index)*250_000_000)
        self.runtime.task["last_lightnav_ack_stamp_ns"] = self.runtime.frames[3].stamp
        self.tick()
        self.runtime._dispatch.assert_called_once()
        self.assertEqual(self.runtime._dispatch.call_args.args[0],list(self.runtime.frames)[3:])
        self.assertIsNone(self.runtime.task["outcome"])

    def test_active_source_gap_remains_blocked(self):
        self.runtime.task["goal_started_timestamp_ns"] = self.now-3*SECOND
        for offset in (2.,1.75,.25,0.):
            self.ingest(self.now-int(offset*SECOND))
        self.tick()
        self.assertEqual(self.runtime.state,"WAITING_INPUT")
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()

    def test_model_stop_still_obeys_explicit_task_timeout(self):
        for index in range(4):
            self.ingest(self.now-(3-index)*250_000_000)
        self.valid_feedback()
        self.queue_model_stop()
        self.runtime.task["deadline_monotonic_ns"] = time.monotonic_ns()-1
        self.tick()
        self.assertEqual(self.runtime.state,"TIMED_OUT")
        self.assertEqual(self.runtime.task["outcome"]["reason"],"task_budget_exhausted")
        self.assertFalse(self.runtime.task["model_stop_latched"])
        self.assertIn("task_budget_exhausted",self.rejection_reasons())
        self.runtime._dispatch.assert_not_called()
        self.assert_zero()


if __name__ == "__main__":
    unittest.main()
