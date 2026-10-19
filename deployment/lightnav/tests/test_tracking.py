"""CPU contract checks; no model weights, hardware, server or GPU is started."""
from __future__ import annotations

import base64
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from deployment.lightnav import policy as policy_module
from deployment.lightnav.policy import HistoryConflictError, LightNavImageGoalPolicy, build_tracking_engine
from deployment.lightnav.server import parse_request

VENDOR_SOURCE = str(policy_module.VENDOR / "src")
if VENDOR_SOURCE not in sys.path:
    sys.path.insert(0, VENDOR_SOURCE)


def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (32, 24), (100, 80, 60)).save(output, format="JPEG")
    return {"rgb_jpeg_base64": base64.b64encode(output.getvalue()).decode("ascii")}


def request(mode="native_tracking"):
    payload = {
        "identity": {"session_id": "session", "task_id": "task", "goal_id": "instruction-v1",
                     "goal_version": 1, "frame_id": "frame-1"},
        "observations": [{**jpeg(), "frame_id": "frame-1", "capture_timestamp_ns": 1_000_000_000}],
        "instruction": "Follow the person wearing a white T-shirt.",
        "conditioning_mode": mode,
    }
    return payload


def bundle(history=3):
    return SimpleNamespace(video_size=(32, 32), video_fps=4, num_history_frames=history,
                           slowfast_tiers=None, pool_enable=False, pool_spatial=1,
                           pool_mode="avg", pool_stage="post_vit", predict_horizon=10,
                           action_method="rvq", processor=SimpleNamespace(video_processor=None))


class FakeEngine:
    def __init__(self, processing_bundle):
        self.bundle = processing_bundle
        self.aspect_mode = "stretch"
        self.samples = []
        self.frame_calls = []
        self.resets = 0
        self.text = "<apos_1><opos_1><act_l0_1><act_l1_2><act_l2_3>"

    def reset_episode_state(self):
        self.resets += 1

    def generate_from_frames(self, *args, **kwargs):
        from lightnav.inference.engine import VLNInferenceEngine
        self.frame_calls.append((args, kwargs))
        return VLNInferenceEngine.generate_from_frames(self, *args, **kwargs)

    def generate(self, sample, *args, **kwargs):
        self.samples.append(sample)
        return self.text, {}


class FakeDecoder:
    stopped = False

    def decode_waypoints(self, text):
        result = np.zeros((10, 3), dtype=np.float32)
        if not self.stopped:
            result[:, 0] = np.arange(1, 11) / 10
        return result, text


def policy():
    instance = LightNavImageGoalPolicy.__new__(LightNavImageGoalPolicy)
    instance.engine = FakeEngine(bundle(history=5))
    instance.bundle = instance.engine.bundle
    instance.tracking_engine = FakeEngine(bundle(history=3))
    instance.decoder = FakeDecoder()
    instance.sessions = {}
    instance.max_new_tokens = 12
    instance.max_session_frames = 4096
    instance.max_sessions = 2
    return instance


class TrackingRequestTests(unittest.TestCase):
    def test_tracking_needs_instruction_and_no_goal(self):
        parsed = parse_request(request())
        self.assertIsNone(parsed[1])
        self.assertEqual(parsed[4], "native_tracking")
        self.assertEqual(parsed[2]["goal_id"], "instruction-v1")
        for change in ({"goal": jpeg()}, {"goal": None}, {"instruction": " "},
                       {"instruction": None}, {"instruction": "x" * 8193},
                       {"task_mode": "imagegoal"}):
            with self.subTest(change=list(change)), self.assertRaises(ValueError):
                parse_request({**request(), **change})

    def test_native_language_needs_instruction_and_rejects_goal(self):
        payload = request("native_language")
        parsed = parse_request(payload)
        self.assertIsNone(parsed[1])
        self.assertEqual(parsed[4:], ("native_language", "objnav"))
        for change in ({"goal": jpeg()}, {"goal": None}, {"goal_image": jpeg()},
                       {"instruction": " "}, {"instruction": None},
                       {"task_mode": "imagegoal"}, {"task_mode": "tracking"},
                       {"conditioning_mode": "experimental_reference_image"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                parse_request({**payload, **change})
        del payload["conditioning_mode"]
        self.assertEqual(parse_request(payload)[4:], ("native_language", "objnav"))
        self.assertEqual(parse_request({**payload, "task_mode": "vln"})[5], "vln")


class TrackingPolicyTests(unittest.TestCase):
    def test_official_tracking_builder_and_task_engine_without_goal_segment(self):
        from lightnav.inference import samples

        instance = policy()
        with patch.object(samples, "build_tracking_sample", wraps=samples.build_tracking_sample) as tracking, \
             patch.object(samples, "build_vln_traj_sample", side_effect=AssertionError("VLN fallback")):
            response = instance.predict(*parse_request(request()))
        self.assertEqual(tracking.call_count, 1)  # telemetry builder; engine has its own upstream import
        self.assertIs(tracking.call_args.args[-1], instance.tracking_engine.bundle)
        self.assertEqual(len(instance.engine.samples), 0)
        sample = instance.tracking_engine.samples[-1]
        self.assertEqual(len(sample["video_segments"]), 1)
        self.assertNotIn("Separate reference goal view", response["prompt"])
        self.assertEqual(response["task_mode"], "tracking")
        self.assertEqual(response["eval_task"], "trackvla")
        self.assertFalse(response["goal_image_consumed"])
        self.assertFalse(response["arrival_confirmed"])
        self.assertIsNone(response["goal_encoding"])
        self.assertIs(response["model_visible"], True)
        self.assertEqual(np.asarray(response["trajectory_body_m_rad"]).shape, (10, 3))

    def test_tracking_signals_preserve_invisible_unknown_and_stop(self):
        for raw, visible, stopped in (("<opos_0>", False, False),
                                      ("<act_l0_1>", None, False),
                                      ("<opos_1>", True, True)):
            with self.subTest(visible=visible, stopped=stopped):
                instance = policy()
                instance.tracking_engine.text = raw
                instance.decoder.stopped = stopped
                response = instance.predict(*parse_request(request()))
                self.assertIs(response["model_visible"], visible)
                self.assertIs(response["model_stop"], stopped)
                self.assertFalse(response["arrival_confirmed"])

    def test_instruction_version_history_dedup_and_task_configuration(self):
        instance = policy()
        original = request()
        first = instance.predict(*parse_request(original))
        retry = instance.predict(*parse_request(original))
        self.assertEqual(first["policy_history_count"], 1)
        self.assertEqual(retry["policy_history_frames_appended"], 0)
        with self.assertRaises(HistoryConflictError):
            instance.predict(*parse_request({**original, "instruction": "Follow a different person."}))
        next_generation = copy.deepcopy(original)
        next_generation["identity"].update(goal_id="instruction-v2", goal_version=2)
        next_generation["instruction"] = "Follow a different person."
        response = instance.predict(*parse_request(next_generation))
        self.assertEqual(response["policy_history_reset_reason"], "new_goal_generation")
        for index in range(2, 6):
            payload = copy.deepcopy(next_generation)
            payload["identity"]["frame_id"] = f"frame-{index}"
            payload["observations"][0].update(frame_id=f"frame-{index}", capture_timestamp_ns=index * 1_000_000_000)
            instance.predict(*parse_request(payload))
        latest = instance.tracking_engine.samples[-1]["video_segments"][0]
        self.assertEqual(latest["frame_indices"], [2, 3, 4])
        self.assertEqual(latest["video"].shape[0], 3)
        instance.reset({"session_id": "session"})
        self.assertFalse(instance.sessions)

    def test_language_stays_on_navigation_builder_and_official_hf_entrypoint(self):
        from lightnav.inference import samples

        instance = policy()
        with patch.object(samples, "build_tracking_sample", side_effect=AssertionError("tracking")):
            response = instance.predict(*parse_request(request("native_language")))
        self.assertEqual(len(instance.tracking_engine.samples), 0)
        self.assertEqual(response["eval_task"], "vlnce")
        self.assertEqual(response["task_mode"], "objnav")
        self.assertEqual(response["implementation"], "official_release")
        self.assertFalse(response["goal_image_consumed"])
        sample = instance.engine.samples[-1]
        self.assertEqual(len(sample["video_segments"]), 1)
        self.assertIs(sample["_allow_vit_cache"], True)
        self.assertEqual(instance.engine.frame_calls[0][1], {
            "predict_horizon": 1, "frame_ids": [0], "max_new_tokens": 12,
            "task_type": "vlnce_traj",
        })
        self.assertEqual(instance.engine.resets, 1)
        instance.predict(*parse_request(request("native_language")))
        self.assertEqual(instance.engine.resets, 1)  # no per-prediction cache reset


class TrackingBundleTests(unittest.TestCase):
    def test_distinct_task_parameters_share_loaded_model_without_weight_reload(self):
        from lightnav.inference import model

        processor = SimpleNamespace(tokenizer=SimpleNamespace(
            convert_tokens_to_ids=lambda token: 1, encode=lambda value, **kwargs: [1]))
        navigation = FakeEngine(bundle())
        navigation.bundle.model = object()
        navigation.bundle.processor = processor
        navigation.bundle.device = "cpu"
        navigation.bundle.position_id_func = lambda *args: None
        config = {"common": {"video_size": [32, 32]}, "tasks": {
            "vlnce": {"timestamp_relative": False},
            "trackvla": {"num_history_frames": 7, "predict_horizon": 10, "video_fps": 6,
                         "action_tokenizer": {"method": "rvq", "bundle_path": "action_tokenizer"}},
        }}
        with tempfile.TemporaryDirectory() as temporary:
            Path(temporary, "eval_config.json").write_text(json.dumps(config))
            with patch.object(model, "load_hf_model", side_effect=AssertionError("second weight load")):
                result = build_tracking_engine(navigation, Path(temporary), 12)
        self.assertIs(result.bundle.model, navigation.bundle.model)
        self.assertIs(result.bundle.processor, processor)
        self.assertEqual(result.bundle.num_history_frames, 7)
        self.assertEqual(result.bundle.video_fps, 6)
        self.assertIsNot(result.bundle, navigation.bundle)
        self.assertEqual(result.backend, "hf")

    def test_missing_tracking_task_fails_without_vln_fallback(self):
        with tempfile.TemporaryDirectory() as temporary:
            Path(temporary, "eval_config.json").write_text(json.dumps({"tasks": {"vlnce": {}}}))
            with self.assertRaisesRegex(ValueError, "tasks.trackvla"):
                build_tracking_engine(FakeEngine(bundle()), Path(temporary), 12)


if __name__ == "__main__":
    unittest.main()
