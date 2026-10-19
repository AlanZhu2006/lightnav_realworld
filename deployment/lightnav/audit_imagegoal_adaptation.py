"""CPU-only action-geometry regressions for the explicit Go2 adaptation."""
from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from .go2_adaptation import prepare_path, postprocess_command


def config(**changes):
    values = dict(v_ref_mps=.2, max_v_mps=.3, max_w_rad_s=.55,
                  min_effective_v_mps=.1, min_effective_w_rad_s=.2,
                  horizon=5, tick_hz=10., lightnav_adapt_resample=True,
                  lightnav_adapt_speed_floor=True, lightnav_adapt_heading_filter=True,
                  lightnav_adapt_short_actions=True)
    values.update(changes)
    return SimpleNamespace(**values)


def path(length=1.5, yaw=0., lateral=0.):
    phase = np.linspace(.1, 1., 10)
    return np.column_stack((phase*length, phase*lateral, phase*yaw))


class AdaptationTests(unittest.TestCase):
    def apply(self, raw=None, vx=.05, wz=0., heading=0., refyaw=0.,
              remaining=.5, remaining_yaw=0., **changes):
        return postprocess_command(vx, wz, heading, refyaw,
               path() if raw is None else raw, remaining, remaining_yaw, config(**changes))

    def test_nominal_resampling_keeps_existing_rates(self):
        sampled, telemetry = prepare_path(path(yaw=.5), 12345, config())
        self.assertAlmostEqual(telemetry["effective_v_ref_mps"], .2)
        self.assertAlmostEqual(telemetry["effective_w_ref_rad_s"], .55)
        self.assertEqual(sampled.capture_stamp_ns, 12345)
        np.testing.assert_allclose(sampled.points_body_m_rad[-1], [1.5, 0., .5])

    def test_short_translation_lowers_reference_rate_from_local_action_extent(self):
        sampled, telemetry = prepare_path(path(length=.05), 1, config())
        self.assertAlmostEqual(telemetry["raw_path_length_m"], .05)
        self.assertAlmostEqual(telemetry["effective_v_ref_mps"], .1)
        self.assertGreater(sampled.points_body_m_rad[-1, 0], 0)
        self.assertFalse(telemetry["arrival_evaluated"])
        self.assertIn("not_image_goal_distance", telemetry["metric_semantics"])

    def test_pure_rotation_uses_positive_unused_translation_rate(self):
        yaw = math.radians(5.)
        sampled, telemetry = prepare_path(path(length=0., yaw=yaw), 1, config())
        self.assertTrue(sampled.has_motion)
        self.assertEqual(telemetry["raw_path_length_m"], 0.)
        self.assertAlmostEqual(telemetry["effective_w_ref_rad_s"], yaw/.5)
        self.assertGreater(telemetry["effective_v_ref_mps"], 0.)
        np.testing.assert_allclose(sampled.points_body_m_rad[:, :2], 0.)

    def test_resampling_ablation_returns_none_without_changing_raw(self):
        raw = path(length=.02, yaw=.03)
        before = raw.copy()
        sampled, telemetry = prepare_path(raw, 1, config(lightnav_adapt_resample=False))
        self.assertIsNone(sampled)
        self.assertFalse(telemetry["resampled"])
        np.testing.assert_array_equal(raw, before)

    def test_short_action_ablation_restores_nominal_reference_rates(self):
        _, telemetry = prepare_path(path(length=.01, yaw=.01), 1,
                                    config(lightnav_adapt_short_actions=False))
        self.assertEqual(telemetry["effective_v_ref_mps"], .2)
        self.assertEqual(telemetry["effective_w_ref_rad_s"], .55)

    def test_short_translation_floor_and_deadband_scale_together(self):
        vx, wz, telemetry = self.apply(raw=path(length=.05), vx=.025, remaining=.05)
        self.assertAlmostEqual(vx, .05)
        self.assertEqual(wz, 0.)
        self.assertAlmostEqual(telemetry["effective_v_deadband_mps"], .02)
        vx, _, telemetry = self.apply(raw=path(length=.01), vx=.006, remaining=.01)
        self.assertAlmostEqual(vx, .01)
        self.assertAlmostEqual(telemetry["effective_v_deadband_mps"], .004)

    def test_zero_mpc_components_never_become_motion(self):
        raw = path(length=.05, yaw=.5)
        vx, wz, _ = self.apply(raw=raw, vx=0., wz=0., heading=.5,
                               refyaw=.5, remaining_yaw=.5)
        self.assertEqual((vx, wz), (0., 0.))
        vx, wz, _ = self.apply(raw=raw, vx=0., wz=.05, heading=.5,
                               refyaw=.5, remaining_yaw=.5)
        self.assertEqual(vx, 0.)
        self.assertGreater(wz, 0.)

    def test_long_straight_tiny_rvq_yaw_keeps_nominal_angular_filters(self):
        _, wz, telemetry = self.apply(raw=path(yaw=6e-5), wz=.03,
                                     heading=math.radians(2.), refyaw=6e-5,
                                     remaining_yaw=6e-5)
        self.assertEqual(wz, 0.)
        self.assertFalse(telemetry["protected_short_turn"])
        self.assertAlmostEqual(telemetry["effective_heading_deadband_rad"], math.radians(8.))
        self.assertEqual(telemetry["effective_w_deadband_rad_s"], .04)

    def test_small_pure_turn_survives_zero_bearing_and_preserves_sign(self):
        for sign in (-1., 1.):
            yaw = sign*math.radians(5.)
            vx, wz, telemetry = self.apply(raw=path(length=0., yaw=yaw), vx=0., wz=sign*.01,
                                           refyaw=yaw, remaining=0., remaining_yaw=yaw)
            self.assertEqual(vx, 0.)
            self.assertGreater(wz*sign, 0.)
            self.assertTrue(telemetry["protected_short_turn"])
            self.assertLessEqual(abs(wz), abs(yaw)/.5)

    def test_short_translation_with_tiny_explicit_turn_is_not_erased(self):
        vx, wz, telemetry = self.apply(raw=path(length=.05, yaw=.02), vx=.025, wz=.001,
                                       refyaw=.02, remaining=.05, remaining_yaw=.02)
        self.assertGreater(vx, 0.)
        self.assertGreater(wz, 0.)
        self.assertTrue(telemetry["protected_short_turn"])
        self.assertLessEqual(telemetry["effective_heading_deadband_rad"], .02/3.)
        self.assertLessEqual(wz, .02/.5)

    def test_lateral_local_reference_can_turn_without_predicted_yaw(self):
        _, wz, telemetry = self.apply(raw=path(length=.02, lateral=.02), wz=.01,
                                     heading=.3, refyaw=0., remaining=.02)
        self.assertGreater(wz, 0.)
        self.assertTrue(telemetry["protected_short_turn"])

    def test_angular_suppression_requires_both_bearing_and_reference_yaw_small(self):
        for heading, yaw in ((0., .2), (.2, 0.)):
            _, wz, _ = self.apply(wz=.05, heading=heading, refyaw=yaw, remaining_yaw=yaw)
            self.assertGreater(wz, 0.)
        _, wz, _ = self.apply(wz=.05, heading=.02, refyaw=.02)
        self.assertEqual(wz, 0.)

    def test_remaining_local_extent_bounds_floor_without_clipping_original_mpc(self):
        vx, _, telemetry = self.apply(vx=.0005, remaining=.001)
        self.assertLessEqual(vx, .001/.5)
        self.assertLessEqual(telemetry["effective_v_floor_mps"], .001/.5)
        vx, _, telemetry = self.apply(vx=.15, remaining=.001)
        self.assertEqual(vx, .15)
        self.assertFalse(telemetry["original_mpc_command_clipped_by_local_budget"])
        _, wz, telemetry = self.apply(raw=path(length=0., yaw=.1), vx=0., wz=.0001,
                                     refyaw=.001, remaining=0., remaining_yaw=.001,
                                     lightnav_adapt_heading_filter=False)
        self.assertLessEqual(wz, .001/.5)
        _, wz, _ = self.apply(raw=path(length=0., yaw=.1), vx=0., wz=.1,
                              refyaw=.001, remaining=0., remaining_yaw=.001,
                              lightnav_adapt_heading_filter=False)
        self.assertEqual(wz, .1)

    def test_floor_ablation_preserves_small_nonzero_mpc_when_filter_disabled(self):
        vx, wz, _ = self.apply(vx=.01, wz=-.005, lightnav_adapt_speed_floor=False,
                               lightnav_adapt_heading_filter=False)
        self.assertEqual((vx, wz), (.01, -.005))

    def test_output_caps_and_nonnegative_forward_are_preserved(self):
        vx, wz, _ = self.apply(vx=9., wz=-9., heading=-2., refyaw=-2.)
        self.assertEqual((vx, wz), (.3, -.55))
        vx, _, _ = self.apply(vx=-.1)
        self.assertEqual(vx, 0.)

    def test_angle_extent_uses_wrapped_increment_and_raw_is_immutable(self):
        raw = path(length=.05)
        raw[:, 2] = np.deg2rad([179., -179., -179., -179., -179., -179., -179., -179., -179., -179.])
        before = raw.copy()
        _, telemetry = prepare_path(raw, 1, config())
        self.assertAlmostEqual(telemetry["raw_angular_travel_rad"], math.radians(181.))
        self.apply(raw=raw)
        np.testing.assert_array_equal(raw, before)

    def test_invalid_shapes_nonfinite_inputs_and_nonboolean_flags_rejected(self):
        for raw in (np.zeros((8, 3)), np.zeros((10, 2)), np.full((10, 3), np.nan)):
            with self.assertRaises(ValueError):
                prepare_path(raw, 1, config())
        with self.assertRaises(ValueError):
            self.apply(vx=math.nan)
        with self.assertRaises(ValueError):
            self.apply(remaining=-1.)
        with self.assertRaises(ValueError):
            self.apply(lightnav_adapt_short_actions="false")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(AdaptationTests))
    report = {"schema_version": "lightnav.go2_imagegoal_adaptation_audit.v1",
              "ok": result.wasSuccessful(), "tests_run": result.testsRun,
              "failures": len(result.failures), "errors": len(result.errors),
              "skipped": len(result.skipped), "details": stream.getvalue(),
              "synthetic_local_action_fixtures": True,
              "goal_distance_or_arrival_evaluated": False,
              "gpu_inference_executed": False, "sdk_initialized": False,
              "network_used": False, "hardware_motion_sent": False}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
