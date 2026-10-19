"""Offline fault checks for bounded LightNav uploads and task isolation.

The real runtime dispatcher and completion handling run against fake HTTP. No
network service, model, robot SDK, subprocess or physical motion is permitted.
"""
from __future__ import annotations

import base64
import time
import unittest
from unittest.mock import Mock, patch

from . import runtime as runtime_module
from . import audit_lightnav_dispatch as dispatch_audit
from .audit_lightnav_dispatch import session_status_fixture
from .audit_native_runtime import FakeManager
from .lightnav_http import LightNavHTTPError
from . import lightnav_http as http_module


class BacklogTests(unittest.TestCase):
    setUp = dispatch_audit.RuntimeDispatchTests.setUp
    tearDown = dispatch_audit.RuntimeDispatchTests.tearDown
    build = dispatch_audit.RuntimeDispatchTests.build

    def submit(self, runtime, *, preflight=session_status_fixture, inference=None):
        inference = inference or Mock(return_value=({"offline_response":True,"server_boot_id":"fixture-server"},1.))
        with patch.object(runtime_module.LightNavHTTPClient,"request_json",side_effect=preflight) as control, \
             patch.object(runtime.lightnav_http,"request_json",side_effect=inference) as upload:
            runtime._dispatch(list(runtime.frames))
            completion = runtime.completed.get(timeout=2.)
        return completion,control,upload

    def accept(self, runtime, completion):
        runtime.completed.put(completion)
        with runtime.lock:
            runtime._accept_completed()

    @staticmethod
    def valid_completion(runtime, request_id="history-ack"):
        frames=list(runtime.frames)
        identity={"session_id":runtime.session_id,"task_id":runtime.task["task_id"],
                  "goal_id":runtime.task["goal_id"],"goal_version":runtime.task["goal_version"],
                  "frame_id":frames[-1].frame_id}
        source=[{"frame_id":f.frame_id,"capture_timestamp_ns":f.stamp} for f in frames]
        metadata={"request_id":request_id,"identity":identity,"epoch":runtime.epoch,
                  "backend":"lightnav","task_mode":"objnav","source_stamp_ns":frames[-1].stamp,
                  "created_monotonic_ns":time.monotonic_ns(),"capture_pose":[1.,2.,.3],"capture_odom_gap_s":0.}
        payload={"identity":identity,"observations":[f.wire() for f in frames]}
        trajectory=[[.1*(i+1),0.,0.] for i in range(10)]
        response={"schema_version":"lightnav.imagegoal.inference.v1","identity":dict(identity),
                  "server_boot_id":"fixture-server",
                  "observation_frames":source,"source_frame":source[-1],"conditioning_mode":"native_language",
                  "task_mode":"objnav","implementation":"official_release","goal_image_consumed":False,
                  "official_release_commit":runtime_module.OFFICIAL_COMMIT,
                  "official_core_sha256":runtime_module.OFFICIAL_CORE_DIGEST,"eval_task":"vlnce",
                  "trajectory_body_m_rad":trajectory,"trajectory_body_m":[p[:2] for p in trajectory],
                  "model_stop":False,"model_visible":True,"arrival_confirmed":False,"policy_history_count":len(frames)}
        return metadata,payload,response,20.,None

    def append(self, runtime, count):
        last = runtime.frames[-1]
        for index in range(count):
            stamp=last.stamp+(index+1)*100_000_000
            with patch.object(runtime_module.time,"time_ns",return_value=stamp):
                runtime.ingest_rgb({**last.metadata,"frame_id":f"pending-{index}",
                    "capture_timestamp_ns":stamp,
                    "rgb_jpeg_base64":base64.b64encode(last.jpeg).decode(),
                    "provenance":{"offline_backlog_fixture":True}})

    def test_first_timeout_ends_episode_and_new_frames_never_trigger_second_upload(self):
        runtime,_ = self.build("lightnav")
        runtime.native_executor = FakeManager()
        runtime.native_executor.running = True
        runtime.plan = {"offline_previous_plan":True}
        cancel = Mock(wraps=runtime.lightnav_http.cancel_inflight)
        runtime.lightnav_http.cancel_inflight = cancel
        completion,control,upload = self.submit(runtime,inference=Mock(side_effect=TimeoutError("ACK lost")))
        self.accept(runtime,completion)
        self.assertEqual(runtime.state,"FAILED")
        self.assertEqual(runtime.task["outcome"]["reason"],"lightnav_transport_failed_no_image_retry")
        self.assertIsNone(runtime.plan)
        self.assertFalse(runtime.native_executor.running)
        self.assertEqual(runtime.proposal["vx_m_s"],0.)
        cancel.assert_called_once()
        runtime._schedule_lightnav_cleanup.assert_called_once_with(runtime.task["task_id"])
        self.append(runtime,3)
        with patch.object(runtime.lightnav_http,"request_json",side_effect=AssertionError("second image upload")):
            runtime.tick()
            runtime.tick()
        self.assertEqual(upload.call_count,1)
        self.assertEqual(control.call_count,1)
        self.assertIsNone(runtime.task["last_lightnav_ack_stamp_ns"])

    def test_capacity_full_preflight_rejects_before_any_image_upload(self):
        runtime,_ = self.build("lightnav")
        def full(*args,**kwargs):
            snapshot,rtt = session_status_fixture(*args,**kwargs)
            return dict(snapshot,active_session_count=2),rtt
        completion,control,upload = self.submit(runtime,preflight=full)
        self.accept(runtime,completion)
        self.assertEqual(runtime.state,"FAILED")
        upload.assert_not_called()
        control.assert_called_once()
        self.assertIn("session_capacity",completion[-1])

    def test_typed_409_on_preflight_stops_without_upload_or_retry(self):
        runtime,_ = self.build("lightnav")
        refusal=LightNavHTTPError(409,b'{"error_code":"session_capacity","retryable":false}')
        completion,control,upload = self.submit(runtime,preflight=Mock(side_effect=refusal))
        self.accept(runtime,completion)
        upload.assert_not_called()
        control.assert_called_once()
        self.assertEqual(runtime.task["transport_failure"]["code"],"session_capacity")
        self.assertEqual(runtime.state,"FAILED")

    def test_racing_capacity_409_after_admission_never_retries_images(self):
        runtime,_=self.build("lightnav")
        refusal=LightNavHTTPError(409,b'{"error_code":"session_capacity","retryable":false}')
        completion,_,upload=self.submit(runtime,inference=Mock(side_effect=refusal))
        self.accept(runtime,completion)
        self.assertEqual(runtime.state,"FAILED")
        self.assertEqual(runtime.task["transport_failure"]["code"],"session_capacity")
        self.append(runtime,2)
        with patch.object(runtime.lightnav_http,"request_json",side_effect=AssertionError("409 retried JPEGs")):
            runtime.tick()
        upload.assert_called_once()

    def test_retired_task_cannot_upload_after_preflight(self):
        runtime,_ = self.build("lightnav")
        def retired(*args,**kwargs):
            snapshot,rtt=session_status_fixture(*args,**kwargs)
            return dict(snapshot,retired=True),rtt
        completion,_,upload=self.submit(runtime,preflight=retired)
        upload.assert_not_called()
        self.assertIn("task_retired",completion[-1])

    def test_preflight_identity_mismatch_cannot_upload(self):
        runtime,_ = self.build("lightnav")
        def wrong(*args,**kwargs):
            snapshot,rtt=session_status_fixture(*args,**kwargs)
            return dict(snapshot,identity={"session_id":"someone-else","task_id":"other"}),rtt
        completion,_,upload=self.submit(runtime,preflight=wrong)
        upload.assert_not_called()
        self.assertIn("identity mismatch",completion[-1])

    def test_server_restarting_between_preflight_and_reply_fails_episode(self):
        runtime,_=self.build("lightnav")
        completion,_,upload=self.submit(runtime,inference=Mock(return_value=(
            {"offline_response":True,"server_boot_id":"restarted-server"},1.)))
        self.accept(runtime,completion)
        upload.assert_called_once()
        self.assertIsNotNone(completion[-1])
        self.assertEqual(runtime.task["outcome"]["reason"],"lightnav_transport_failed_no_image_retry")
        self.assertIsNone(runtime.plan)

    def test_frame_budget_ends_episode_without_truncating_history(self):
        runtime,_=self.build("lightnav")
        self.append(runtime,5)
        originals=[frame.wire() for frame in runtime.frames]
        with patch.object(runtime.lightnav_http,"request_json",side_effect=AssertionError("budget exceeded uploaded")):
            self.assertIsNone(runtime._window(runtime.frames[-1].stamp))
        self.assertEqual(runtime.state,"FAILED")
        self.assertEqual(runtime.task["outcome"]["reason"],"lightnav_upload_backlog_exceeded")
        self.assertEqual([frame.wire() for frame in runtime.frames],originals)

    def test_pending_age_budget_ends_episode_without_truncating_history(self):
        runtime,_=self.build("lightnav")
        originals=[frame.wire() for frame in runtime.frames]
        now=runtime.frames[0].stamp+int((runtime.config.lightnav_max_pending_age_s+.01)*1e9)
        self.assertIsNone(runtime._window(now))
        self.assertEqual(runtime.task["outcome"]["reason"],"lightnav_upload_backlog_exceeded")
        self.assertEqual([frame.wire() for frame in runtime.frames],originals)

    def test_serialized_byte_budget_checks_real_payload_before_any_http(self):
        runtime,_=self.build("lightnav")
        runtime.config.lightnav_max_request_bytes=1024
        originals=[frame.wire() for frame in runtime.frames]
        with patch.object(runtime_module.LightNavHTTPClient,"request_json",side_effect=AssertionError("budget exceeded used HTTP")):
            runtime._dispatch(list(runtime.frames))
        self.assertEqual(runtime.task["outcome"]["reason"],"lightnav_upload_bytes_exceeded")
        self.assertEqual(runtime.metrics["inference_started"],0)
        self.assertEqual([frame.wire() for frame in runtime.frames],originals)

    def test_history_expiring_during_preflight_is_not_uploaded(self):
        runtime,_=self.build("lightnav")
        wall=[time.time_ns()]
        def slow(*args,**kwargs):
            result=session_status_fixture(*args,**kwargs)
            wall[0]+=3_000_000_000
            return result
        with patch.object(runtime_module.time,"time_ns",side_effect=lambda:wall[0]):
            completion,_,upload=self.submit(runtime,preflight=slow)
        upload.assert_not_called()
        self.assertIn("history aged out",completion[-1])

    def test_late_old_task_error_does_not_fail_or_clear_new_inflight(self):
        runtime,_=self.build("lightnav")
        old,_,_=self.submit(runtime,inference=Mock(side_effect=TimeoutError("late old ACK")))
        runtime.create_task({"backend":"lightnav","task_mode":"objnav","instruction":"Go to the new door.",
                             "expected_epoch":runtime.epoch,"timeout_s":60})
        new_inflight={"request_id":"new-request","epoch":runtime.epoch,
                      "source_stamp_ns":runtime.frames[-1].stamp,
                      "identity":{"session_id":runtime.session_id,"task_id":runtime.task["task_id"],"goal_version":1}}
        runtime.inflight=new_inflight
        epoch,task=runtime.epoch,runtime.task["task_id"]
        self.accept(runtime,old)
        self.assertEqual(runtime.task["task_id"],task)
        self.assertEqual(runtime.epoch,epoch)
        self.assertNotEqual(runtime.state,"FAILED")
        self.assertIs(runtime.inflight,new_inflight)

    def test_stop_between_active_check_and_send_permanently_closes_only_old_task_client(self):
        runtime,_=self.build("lightnav")
        runtime.task["lightnav_server_boot_id"]="fixture-server"
        old_client=runtime.lightnav_http
        old_request=old_client.request_json
        old_task=runtime.task["task_id"]

        def stop_in_send_gap(url, *, timeout_s, payload):
            # The dispatcher has passed its final active-task check. Reproduce
            # an operator stop and restart immediately before HTTP enters send.
            with runtime.lock:
                runtime._finish("CANCELLED","offline_stop_at_send_gap")
                runtime.create_task({"backend":"lightnav","task_mode":"objnav",
                    "instruction":"Go to the new door.","expected_epoch":runtime.epoch,"timeout_s":60})
            return old_request(url,timeout_s=timeout_s,payload=payload)

        with patch.object(http_module.http.client,"HTTPConnection",side_effect=AssertionError("closed old task opened a connection")):
            completion,_,upload=self.submit(runtime,inference=stop_in_send_gap)
        upload.assert_called_once()
        self.assertIn("client is closed",completion[-1])
        self.assertNotEqual(runtime.task["task_id"],old_task)
        self.assertIsNot(runtime.lightnav_http,old_client)
        self.accept(runtime,completion)
        self.assertNotEqual(runtime.state,"FAILED")

        # Use the real HTTP client and an in-memory connection, proving that
        # closing the old owner did not close the replacement task's transport.
        response=Mock(status=200,will_close=False)
        response.getheader.return_value="11"
        response.read1.side_effect=[b'{"ok":true}',b'']
        connection=Mock()
        connection.getresponse.return_value=response
        with patch.object(http_module.http.client,"HTTPConnection",return_value=connection) as connect:
            result,_=runtime.lightnav_http.request_json("http://offline.invalid:18884/infer",timeout_s=1.,
                payload={"identity":{"session_id":runtime.session_id,"task_id":runtime.task["task_id"]}})
        self.assertEqual(result,{"ok":True})
        connect.assert_called_once()
        connection.request.assert_called_once()

    def test_healthy_four_frame_request_keeps_protocol_and_skips_repeated_preflight(self):
        runtime,_=self.build("lightnav")
        first,control,upload=self.submit(runtime)
        metadata,payload,_,_,error=first
        self.assertIsNone(error)
        self.assertEqual(control.call_count,1)
        self.assertEqual(upload.call_count,1)
        self.assertEqual(payload["conditioning_mode"],"native_language")
        self.assertEqual(payload["task_mode"],"objnav")
        self.assertEqual(len(payload["observations"]),4)
        self.assertNotIn("goal",payload)
        self.assertEqual(payload["server_boot_id"],"fixture-server")
        self.assertEqual([f["frame_id"] for f in payload["observations"]],[f.frame_id for f in runtime.frames])
        self.assertEqual(metadata["source_stamp_ns"],runtime.frames[-1].stamp)
        self.assertLessEqual(metadata["transport"]["serialized_bytes"],runtime.config.lightnav_max_request_bytes)
        self.assertEqual(runtime.task["lightnav_server_boot_id"],"fixture-server")
        _,control2,upload2=self.submit(runtime)
        control2.assert_not_called()
        upload2.assert_called_once()

    def test_valid_history_ack_survives_execution_epoch_expiry_without_rearming_motion(self):
        runtime,_=self.build("lightnav")
        completion=self.valid_completion(runtime)
        runtime.inflight=completion[0]
        runtime._invalidate("plan_input_too_old_hold",require_post_stop_rgb=True)
        self.accept(runtime,completion)
        self.assertEqual(runtime.task["last_lightnav_ack_stamp_ns"],runtime.frames[-1].stamp)
        self.assertIsNone(runtime.plan)
        self.assertEqual(runtime.metrics["inference_accepted"],0)
        self.assertEqual(runtime.proposal["vx_m_s"],0.)

    def test_wrong_source_cannot_advance_history_ack_after_epoch_expiry(self):
        runtime,_=self.build("lightnav")
        completion=self.valid_completion(runtime)
        completion[2]["source_frame"]={"frame_id":"wrong-source","capture_timestamp_ns":runtime.frames[-1].stamp}
        runtime.inflight=completion[0]
        runtime._invalidate("plan_input_too_old_hold",require_post_stop_rgb=True)
        self.accept(runtime,completion)
        self.assertIsNone(runtime.task["last_lightnav_ack_stamp_ns"])
        self.assertIsNone(runtime.plan)

    def test_wrong_task_or_goal_identity_cannot_advance_history_ack(self):
        runtime,_=self.build("lightnav")
        for key,value in (("task_id","other-task"),("goal_id","other-goal"),("goal_version",2)):
            with self.subTest(key=key):
                completion=self.valid_completion(runtime,request_id="wrong-"+key)
                completion[2]["identity"][key]=value
                runtime.inflight=completion[0]
                runtime._invalidate("plan_input_too_old_hold",require_post_stop_rgb=True)
                self.accept(runtime,completion)
                self.assertIsNone(runtime.task["last_lightnav_ack_stamp_ns"])
                self.assertIsNone(runtime.plan)

    def test_nomad_keeps_existing_consecutive_error_policy(self):
        runtime,_=self.build("nomad")
        identity={"session_id":runtime.session_id,"task_id":runtime.task["task_id"],"goal_version":1}
        for index in range(runtime.config.max_inference_errors):
            metadata={"request_id":f"nomad-error-{index}","identity":identity,"epoch":runtime.epoch,"backend":"nomad"}
            self.accept(runtime,(metadata,{"identity":identity},None,None,"TimeoutError: offline"))
            if index+1<runtime.config.max_inference_errors:
                self.assertNotEqual(runtime.state,"FAILED")
        self.assertEqual(runtime.task["outcome"]["reason"],"repeated_inference_errors")


if __name__=="__main__":
    unittest.main()
