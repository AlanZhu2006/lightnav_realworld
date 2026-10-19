"""CPU integration for adapted LightNav ImageGoal alongside official modes.

Sensor images, time, model replies and the robot executor are explicit fixtures.
The real task runtime, reference adapters, MPC and lease validator are exercised.
"""
import base64
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from . import runtime as runtime_module
from .runtime import ImageGoalRuntime
from .audit_lightnav_native import _config, _goal, _inject, _seed
from .audit_lightnav_dispatch import session_status_fixture
from .audit_native_runtime import FakeManager, RecordedClock
from .audit_tracking import _completion
from deployment.go2.native_imagegoal import validate_native_status


class LightNavImageGoalRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.clock = RecordedClock(1_800_000_000_000_000_000, source_ns=0, source_mono_ns=10_000_000_000)
        self.clock_patch = patch.object(runtime_module,"time",self.clock)
        self.clock_patch.start()
        self.runtime = ImageGoalRuntime(Path(self.directory.name)/"runtime",_config())
        self.runtime.native_executor = FakeManager()
        self.runtime._schedule_lightnav_cleanup = Mock()
        self.runtime._dispatch = Mock()
        self.image = _goal()

    def tearDown(self):
        self.runtime.close()
        self.clock_patch.stop()
        self.directory.cleanup()

    def start(self, **changes):
        request = {"expected_session_id":self.runtime.session_id,"expected_epoch":self.runtime.epoch,
                   "backend":"lightnav","task_mode":"imagegoal","goal":self.image,
                   "operator_present":True,"timeout_s":60,**changes}
        if "selection_id" in request:
            request.pop("goal")
        result = self.runtime.start_native(request,runtime_url="http://offline.invalid:8093")
        _seed(self.runtime,self.clock,self.image)
        return result

    @staticmethod
    def response():
        points = [[.05*(i+1),.001*i*i,.01*i] for i in range(10)]
        return {"schema_version":"lightnav.imagegoal.inference.v1","arrival_confirmed":False,
                "trajectory_body_m_rad":points,"trajectory_body_m":[p[:2] for p in points],"model_stop":False}

    def test_frozen_goal_starts_adapted_profile_without_language(self):
        selected = self.runtime.select_goal({"expected_session_id":self.runtime.session_id,
            "expected_epoch":self.runtime.epoch,"goal":self.image},source="import")
        status = self.start(selection_id=selected["goal_selection"]["selection_id"])
        self.assertEqual(status["task"]["control_profile"],"go2_imagegoal_adaptation")
        self.assertEqual(status["task"]["instruction"],"")
        self.assertTrue(status["execution_capability"]["lightnav_imagegoal_adapter"])
        self.assertEqual(self.runtime.goal_jpeg,base64.b64decode(self.image["rgb_jpeg_base64"]))
        self.assertTrue(status["lightnav_control"]["trajectory_resampling"])

    def test_actual_dispatch_sends_full_goal_and_explicit_adapted_mode(self):
        self.start()
        with patch.object(runtime_module.LightNavHTTPClient,"request_json",side_effect=session_status_fixture), \
             patch.object(self.runtime.lightnav_http,"request_json",return_value=({"fixture":True,"server_boot_id":"fixture-server"},1.)) as http:
            ImageGoalRuntime._dispatch(self.runtime,list(self.runtime.frames))
            self.runtime.completed.get(timeout=2)
        payload = http.call_args.kwargs["payload"]
        self.assertEqual(payload["goal"],self.image)
        self.assertEqual(payload["conditioning_mode"],"experimental_reference_image")
        self.assertEqual(payload["task_mode"],"imagegoal")
        self.assertEqual(payload["instruction"],"")
        self.assertEqual(len(payload["observations"]),4)

    def test_resampled_real_mpc_and_compensation_reach_valid_native_lease(self):
        self.start()
        self.runtime.lightnav_controller.solve = Mock(side_effect=AssertionError("official solver used for adapted profile"))
        with patch.object(runtime_module,"native_speed_gate",side_effect=AssertionError("legacy unbounded gate used")):
            status = _inject(self.runtime,self.clock,self.response(),request_id="adapted-motion")
        self.assertEqual(status["state"],"TRACKING")
        self.assertGreater(len(self.runtime.plan["sampled"].points_body_m_rad),10)
        self.assertEqual(status["plan"]["control_profile"],"go2_imagegoal_adaptation")
        self.assertIn("command_adaptation",status["plan"])
        decision = validate_native_status(status,session_id=self.runtime.session_id,
                                          task_id=self.runtime.task["task_id"],now_ns=self.clock.time_ns())
        self.assertTrue(decision["accepted"],decision["reasons"])
        self.assertLessEqual(decision["command"][0],.3)
        self.assertLessEqual(abs(decision["command"][2]),.55)
        for tamper in ("capability","profile"):
            forged = copy.deepcopy(status)
            if tamper=="capability":
                forged["execution_capability"]["lightnav_imagegoal_adapter"]=False
            else:
                forged["native_execution"]["control_profile"]="official_release"
            self.assertFalse(validate_native_status(forged,session_id=self.runtime.session_id,
                task_id=self.runtime.task["task_id"],now_ns=self.clock.time_ns())["accepted"])

    def test_disabled_adaptations_leave_raw_reference_and_small_commands(self):
        for feature in ("resample","speed_floor","heading_filter","short_actions"):
            setattr(self.runtime.config,"lightnav_adapt_"+feature,False)
        self.start()
        self.runtime.controller.solve = Mock(return_value=((.012,.003),np.zeros((6,3))))
        status = _inject(self.runtime,self.clock,self.response(),request_id="no-compensation")
        self.assertEqual(status["state"],"TRACKING")
        self.assertIsNone(self.runtime.plan["sampled"])
        self.assertEqual(len(status["plan"]["trajectory_odom_m_rad"]),10)
        self.assertEqual(status["proposal"]["vx_m_s"],.012)
        self.assertEqual(status["proposal"]["wz_rad_s"],.003)
        self.runtime.controller.solve.assert_called_once()

    def test_model_stop_finishes_without_any_speed_boost(self):
        self.start()
        with patch.object(runtime_module,"postprocess_command",side_effect=AssertionError("STOP reached speed compensation")):
            status = _inject(self.runtime,self.clock,self.response(),request_id="adapted-stop",model_stop=True)
        self.assertEqual(status["state"],"SUCCEEDED")
        self.assertEqual(status["task"]["outcome"]["reason"],"go2_imagegoal_model_stop")
        self.assertFalse(status["task"]["outcome"]["arrival_independently_validated"])
        self.assertFalse(status["native_execution"]["authorized"])
        self.assertFalse(self.runtime.native_executor.running)

    def test_wrong_conditioning_cannot_produce_a_motion_plan(self):
        self.start()
        item = _completion(self.runtime,self.clock,request_id="wrong-profile")
        self.runtime.completed.put(item)
        self.runtime.tick()
        self.assertIsNone(self.runtime.plan)
        self.assertEqual(self.runtime.metrics["inference_accepted"],0)

    def test_wrong_official_core_digest_is_rejected(self):
        self.start()
        metadata,payload,response,rtt,error = _completion(self.runtime,self.clock,request_id="wrong-core")
        response.update(conditioning_mode="experimental_reference_image",task_mode="imagegoal",
                        implementation="go2_imagegoal_adaptation",goal_image_consumed=True,
                        eval_task="vlnce",official_core_sha256="wrong")
        self.runtime.completed.put((metadata,payload,response,rtt,error))
        self.runtime.tick()
        self.assertIsNone(self.runtime.plan)
        self.assertEqual(self.runtime.metrics["inference_accepted"],0)

    def test_pure_shadow_also_compensates_before_depth_guard(self):
        self.start()
        self.runtime.config.execution_profile="shadow"
        self.runtime.task["execution_mode"]="shadow"
        self.runtime.depth={"depth_m":np.ones((3,3))*2,"metadata":dict(self.runtime.frames[-1].metadata),
                            "clearance_m":2.,"aligned_to_rgb":True,"shape_hw":[3,3]}
        def depth_guard(command, depth, config):
            self.assertEqual(command.linear_x,.077)
            self.assertEqual(command.angular_z,.033)
            return self.runtime.control_module.SafetyResult(command=command,clearance_m=2.,reason="clear")
        with patch.object(runtime_module,"postprocess_command",return_value=(.077,.033,{"fixture":True})) as post, \
             patch.object(self.runtime.control_module,"apply_depth_safety",side_effect=depth_guard):
            status=_inject(self.runtime,self.clock,self.response(),request_id="shadow-adaptation")
        post.assert_called_once()
        self.assertEqual(status["state"],"TRACKING")
        self.assertEqual(status["proposal"]["vx_m_s"],.077)
        self.assertFalse(status["native_execution"]["authorized"])

    def test_missing_goal_or_cross_mode_conditioning_cannot_change_epoch(self):
        for change in ({"instruction":""},{"instruction":"go","conditioning_mode":"native_language","goal":self.image}):
            with self.assertRaises(ValueError):
                self.runtime.create_task({"backend":"lightnav","task_mode":"imagegoal",**change})
            self.assertIsNone(self.runtime.task)
            self.assertEqual(self.runtime.epoch,0)

    def test_goal_replacement_advances_generation_for_imagegoal(self):
        self.start()
        task_id, epoch = self.runtime.task["task_id"],self.runtime.epoch
        status = self.runtime.replace_goal({"goal":self.image,"expected_epoch":epoch})
        self.assertEqual(status["task"]["task_id"],task_id)
        self.assertEqual(status["task"]["goal_version"],2)
        self.assertGreater(status["epoch"],epoch)
        self.assertIsNone(self.runtime.plan)


if __name__ == "__main__":
    unittest.main()
