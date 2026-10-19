"""CPU checks for releasing only this runtime's finished model sessions."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from . import runtime as runtime_module
from .runtime import ImageGoalRuntime
from .audit_lightnav_native import _config


class ImmediateThread:
    def __init__(self, *, target, **kwargs):
        self.target=target

    def start(self):
        self.target()


class CleanupTests(unittest.TestCase):
    def test_retirement_is_sent_immediately_even_while_image_request_is_inflight(self):
        with tempfile.TemporaryDirectory() as temporary:
            runtime=ImageGoalRuntime(Path(temporary)/"runtime",_config())
            runtime.inflight={"identity":{"task_id":"finished-task"}}
            try:
                with patch.object(runtime_module.threading,"Thread",ImmediateThread), \
                     patch.object(runtime_module,"request_json",return_value=({"ok":True,"retired":True},1.)) as reset, \
                     patch.object(runtime.stop_event,"wait",side_effect=AssertionError("retirement waited for inference")):
                    runtime._schedule_lightnav_cleanup("finished-task")
                reset.assert_called_once()
                self.assertTrue(reset.call_args.kwargs["payload"]["retire"])
                self.assertNotIn("finished-task",runtime.pending_session_cleanup)
            finally:
                runtime.inflight=None
                runtime.close()

    def test_delayed_inference_cleanup_is_not_abandoned_after_fifteen_seconds(self):
        with tempfile.TemporaryDirectory() as temporary:
            runtime=ImageGoalRuntime(Path(temporary)/"runtime",_config())
            clock=[0.]
            attempts=[]
            def reset(url, *, timeout_s, payload):
                attempts.append((url,timeout_s,payload))
                if len(attempts)==1:
                    clock[0]=30.
                    raise TimeoutError("old GPU inference still running")
                return {"ok":True,"sessions_cleared":1,"retired":True},1.
            try:
                with patch.object(runtime_module.threading,"Thread",ImmediateThread), \
                     patch.object(runtime_module.time,"monotonic",side_effect=lambda:clock[0]), \
                     patch.object(runtime_module,"request_json",side_effect=reset), \
                     patch.object(runtime.stop_event,"wait",return_value=False):
                    runtime._schedule_lightnav_cleanup("finished-task")
                self.assertEqual(len(attempts),2)
                self.assertEqual(attempts[-1][1],3.)
                self.assertEqual(attempts[-1][2],{"identity":{"session_id":runtime.session_id,"task_id":"finished-task"},"retire":True})
                self.assertNotIn("finished-task",runtime.pending_session_cleanup)
                self.assertIn("lightnav_session_released",[event["kind"] for event in runtime.events])
            finally:
                runtime.close()

    def test_shutdown_preserves_unconfirmed_cleanup_in_pending_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            runtime=ImageGoalRuntime(Path(temporary)/"runtime",_config())
            def interrupt(timeout):
                runtime.stop_event.set()
                return True
            try:
                with patch.object(runtime_module.threading,"Thread",ImmediateThread), \
                     patch.object(runtime_module,"request_json",side_effect=TimeoutError("unavailable")), \
                     patch.object(runtime.stop_event,"wait",side_effect=interrupt):
                    runtime._schedule_lightnav_cleanup("finished-task")
                self.assertIn("finished-task",runtime.pending_session_cleanup)
                self.assertIn("lightnav_session_cleanup_interrupted",[event["kind"] for event in runtime.events])
                self.assertNotIn("lightnav_session_released",[event["kind"] for event in runtime.events])
            finally:
                runtime.close()


if __name__=="__main__":
    unittest.main()
