"""CPU conformance checks against the original LightNav MPC and robot flow.

Images, model responses, hardware and time are fixtures. The optimizer is real;
no model inference, network request or robot motion is performed.
"""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from deployment.lightnav.official_control import GO2_MPC
from deployment.nomad.vendor.lightnav import geometry
from . import runtime as runtime_module
from .runtime import ImageGoalRuntime
from .audit_lightnav_native import _config, _goal, _inject, _request, _seed
from .audit_native_runtime import FakeManager, RecordedClock
from .audit_tracking import _completion


class OfficialLightNavTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.clock = RecordedClock(1_800_000_000_000_000_000, source_ns=0, source_mono_ns=10_000_000_000)
        self.clock_patch = patch.object(runtime_module, "time", self.clock)
        self.clock_patch.start()
        self.runtime = ImageGoalRuntime(Path(self.directory.name)/"runtime", _config())
        self.runtime._schedule_lightnav_cleanup = Mock()
        self.runtime._dispatch = Mock()
        self.runtime.native_executor = FakeManager()
        self.goal = _goal()
        self.runtime.start_native(_request(self.runtime, self.goal), runtime_url="http://offline.invalid")
        _seed(self.runtime, self.clock, self.goal)

    def tearDown(self):
        self.runtime.close()
        self.clock_patch.stop()
        self.directory.cleanup()

    def source_response(self, points):
        return {"trajectory_body_m_rad":points, "trajectory_body_m":[point[:2] for point in points],
                "schema_version":"lightnav.imagegoal.inference.v1", "arrival_confirmed":False,
                "model_stop":False}

    def test_raw_ten_poses_and_reference_match_original_without_origin_or_resampling(self):
        points = np.array([[.03*i, .004*i*i, .05*i] for i in range(1,11)])
        capture, current = [1.,2.,.3], np.array([1.08,2.02,.32])
        control = self.runtime.lightnav_controller
        packet = control.reference(points, capture, current, self.clock.time_ns(), "go2_odom")
        expected = np.asarray(geometry.project_body_to_odom(points, geometry.TimedPose(0,*capture)))
        reference = self.runtime.core.reference_builder(expected,current,horizon=5,weights=(10.,10.,1.))
        np.testing.assert_array_equal(packet.trajectory_odom_m_rad, expected)
        np.testing.assert_array_equal(packet.reference_odom_m_rad, reference)
        self.assertEqual(len(packet.trajectory_odom_m_rad),10)

    def test_two_solves_match_original_local_mpc_previous_control_and_output_scale(self):
        control = self.runtime.lightnav_controller
        expected_controller = self.runtime.core.controller_type(**GO2_MPC)
        pose = np.array([3.,-2.,.6])
        local = np.array([[.03*i,.002*i*i,.02*i] for i in range(1,6)])
        origin = geometry.TimedPose(0,*pose)
        reference = geometry.project_local_to_odom(local, origin)
        previous = (0.,0.)
        for mode,vmax in (("objnav",.8),("tracking",1.5)):
            expected, prediction = expected_controller.solve(np.zeros(3),local,previous,vmax)
            actual, result = control.solve(pose, reference, mode)
            np.testing.assert_allclose(actual,(expected[0]*2.,expected[1]),rtol=1e-6,atol=1e-7)
            np.testing.assert_allclose(result,geometry.project_local_to_odom(prediction,origin),rtol=1e-6,atol=1e-7)
            np.testing.assert_allclose(control.previous_control,expected,rtol=1e-6,atol=1e-7)
            previous = expected

    def test_small_valid_velocity_and_yaw_survive_native_pipeline(self):
        points = [[.003*(i+1),.0001*i,.001*i] for i in range(10)]
        self.runtime.lightnav_controller.solve = Mock(return_value=((.012,.003),np.zeros((6,3))))
        with patch.object(runtime_module,"native_speed_gate",side_effect=AssertionError("custom gate used")):
            status = _inject(self.runtime,self.clock,self.source_response(points),request_id="small-motion")
        self.assertEqual(status["state"],"TRACKING")
        self.assertEqual(status["proposal"]["vx_m_s"],.012)
        self.assertEqual(status["proposal"]["wz_rad_s"],.003)
        np.testing.assert_array_equal(self.runtime.plan["native_path"],points)

    def test_fresh_stop_completes_and_releases_hardware_without_independent_arrival_claim(self):
        status = _inject(self.runtime,self.clock,self.source_response([[.1,0.,0.]]*10),request_id="stop",model_stop=True)
        self.assertEqual(status["state"],"SUCCEEDED")
        self.assertIsNone(self.runtime.plan)
        self.assertFalse(status["native_execution"]["authorized"])
        self.assertFalse(self.runtime.native_executor.running)
        self.assertFalse(status["task"]["outcome"]["arrival_independently_validated"])
        self.runtime._schedule_lightnav_cleanup.assert_called_once()

    def test_stale_stop_cannot_finish_active_navigation(self):
        metadata,payload,response,rtt,error = _completion(self.runtime,self.clock,request_id="stale-stop",model_stop=True)
        response.update(conditioning_mode="native_language",task_mode="objnav",eval_task="vlnce")
        self.clock.wall_ns += 3_000_000_000
        self.runtime.completed.put((metadata,payload,response,rtt,error))
        self.runtime._accept_completed()
        self.assertNotEqual(self.runtime.state,"SUCCEEDED")
        self.assertFalse(self.runtime.task["model_stop_latched"])
        self.assertTrue(self.runtime.native_executor.running)

    def test_reference_image_requests_cannot_supersede_current_task(self):
        task_id,epoch = self.runtime.task["task_id"],self.runtime.epoch
        for changes in ({"goal":self.goal},{"goal_image":self.goal},{"selection_id":"frozen"},{"task_mode":"imagegoal"},{"instruction":" "},
                        {"conditioning_mode":"experimental_reference_image"}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    self.runtime.create_task({**_request(self.runtime,self.goal),**changes})
                self.assertEqual(self.runtime.task["task_id"],task_id)
                self.assertEqual(self.runtime.epoch,epoch)

    def test_stop_after_task_deadline_is_timeout_not_success(self):
        metadata,payload,response,rtt,error = _completion(self.runtime,self.clock,request_id="late-stop",model_stop=True)
        response.update(conditioning_mode="native_language",task_mode="objnav",eval_task="vlnce")
        self.runtime.task["deadline_monotonic_ns"] = self.clock.monotonic_ns()
        self.runtime.completed.put((metadata,payload,response,rtt,error))
        self.runtime._accept_completed()
        self.assertEqual(self.runtime.state,"TIMED_OUT")
        self.assertFalse(self.runtime.native_executor.running)


if __name__ == "__main__":
    unittest.main()
