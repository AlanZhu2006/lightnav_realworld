"""CPU regression checks for released input/sample/decoder equivalence.

The generation boundary is stubbed; these tests do not claim real inference.
Upstream buffering, prompt/sample building, entry-point dispatch and RVQ STOP
semantics execute as normal. No model weights, CUDA or robot is initialized.
"""
from __future__ import annotations

import copy
import hashlib
import unittest

import numpy as np
import torch
from PIL import Image

from deployment.lightnav import policy as policy_module
from deployment.lightnav.policy import HistoryConflictError
from deployment.lightnav.server import parse_request
from deployment.lightnav.tests.test_tracking import policy, request


class OfficialReleaseTests(unittest.TestCase):
    def test_pinned_mirror_matches_independent_official_module_hashes(self):
        manifest = policy_module.verify_sources()
        self.assertEqual(manifest["commit"], "ddfd37059c49e8090217e11c4ffa6f6b3ff9d554")
        self.assertEqual(policy_module.OFFICIAL_RELEASE_COMMIT, "3015508b70fb6e30bbd66701e94c9b916dcb8df8")
        self.assertEqual(len(policy_module.OFFICIAL_CORE_SHA256), 42)
        for relative, expected in policy_module.OFFICIAL_CORE_SHA256.items():
            with self.subTest(module=relative):
                actual = hashlib.sha256((policy_module.VENDOR / relative).read_bytes()).hexdigest()
                self.assertEqual(actual, expected)

    def test_native_policy_rejects_photo_and_invalid_modes_without_mutation(self):
        instance = policy()
        frames, _, identity, instruction, mode, task_mode = parse_request(request("native_language"))
        for kwargs in ({"goal_image": Image.new("RGB", (8, 8))},
                       {"conditioning_mode": "unknown"},
                       {"instruction": ""}, {"task_mode": "imagegoal"}):
            args = dict(frames=frames, goal_image=None, identity=identity,
                        instruction=instruction, conditioning_mode=mode, task_mode=task_mode)
            args.update(kwargs)
            with self.subTest(case=kwargs), self.assertRaises(ValueError):
                instance.predict(**args)
            self.assertFalse(instance.sessions)
            self.assertFalse(instance.engine.samples)

    def test_official_slowfast_sample_and_history_are_preserved(self):
        from lightnav.inference.samples import build_vln_traj_sample

        instance = policy()
        from lightnav.slowfast import DEFAULT_SLOWFAST_TIERS_SPAN, validate_slowfast_tiers
        instance.engine.bundle.slowfast_tiers = validate_slowfast_tiers(DEFAULT_SLOWFAST_TIERS_SPAN)
        payload = request("native_language")
        for index in range(1, 7):
            payload["identity"]["frame_id"] = f"frame-{index}"
            payload["observations"][0].update(frame_id=f"frame-{index}", capture_timestamp_ns=index * 250_000_000)
            response = instance.predict(*parse_request(payload))
        observed = instance.sessions[("session", "task")]["policy"]
        expected = build_vln_traj_sample(observed._get_video_tensor(), payload["instruction"],
                                         observed._history_frame_ids, instance.engine.bundle)
        actual = instance.engine.samples[-1]
        self.assertEqual(actual["conversations"], expected["conversations"])
        self.assertEqual(response["prompt"], expected["conversations"][0]["value"])
        self.assertEqual(len(actual["video_segments"]), len(expected["video_segments"]))
        for got, wanted in zip(actual["video_segments"], expected["video_segments"]):
            self.assertTrue(torch.equal(got["video"], wanted["video"]))
            self.assertEqual(got["frame_indices"], wanted["frame_indices"])
        self.assertIs(actual["_allow_vit_cache"], True)
        self.assertEqual(len(observed._history), 6)
        self.assertEqual(observed._history_frame_ids, list(range(6)))
        self.assertEqual(response["policy_history_count"], 6)
        self.assertEqual(instance.engine.resets, 1)

    def test_instruction_and_mode_change_require_new_generation(self):
        instance = policy()
        payload = request("native_language")
        instance.predict(*parse_request(payload))
        with self.assertRaises(HistoryConflictError):
            instance.predict(*parse_request({**payload, "task_mode": "vln"}))
        newer = copy.deepcopy(payload)
        newer["identity"]["goal_version"] += 1
        newer["task_mode"] = "vln"
        response = instance.predict(*parse_request(newer))
        self.assertEqual(response["task_mode"], "vln")
        self.assertEqual(response["policy_history_reset_reason"], "new_goal_generation")

    def test_real_upstream_decoder_owns_trajectory_and_stop(self):
        from lightnav.tracking import TrackingAgent

        class TinyRVQ:
            horizon = 10
            levels = [8, 256, 256]
            decoded = np.tile([0.2, 0.04, 0.1], (10, 1)).astype(np.float32)
            calls = []

            def is_stop(self, codes):
                return codes[0] == 6

            def decode_waypoints(self, codes):
                self.calls.append(codes)
                return self.decoded.copy()

        instance = policy()
        rvq = TinyRVQ()
        instance.decoder = TrackingAgent(instance.engine, rvq_bundle=rvq)
        instance.engine.text = "<apos_1><opos_2><act_l0_1><act_l1_2><act_l2_3>"
        moving = instance.predict(*parse_request(request("native_language")))
        np.testing.assert_allclose(moving["trajectory_body_m_rad"], rvq.decoded)
        self.assertEqual(rvq.calls, [[1, 2, 3]])
        self.assertFalse(moving["model_stop"])
        instance.engine.text = "<apos_1299><opos_2><act_l0_6><act_l1_122><act_l2_174>"
        stopped = instance.predict(*parse_request(request("native_language")))
        self.assertTrue(stopped["model_stop"])
        self.assertFalse(stopped["arrival_confirmed"])
        np.testing.assert_array_equal(stopped["trajectory_body_m_rad"], np.zeros((10, 3)))
        self.assertEqual(rvq.calls, [[1, 2, 3]])  # STOP follows the official decoder's branch
        instance.engine.text = "<act_l0_9><act_l1_2><act_l2_3>"
        with self.assertRaisesRegex(ValueError, "out of range"):
            instance.predict(*parse_request(request("native_language")))


if __name__ == "__main__":
    unittest.main()
