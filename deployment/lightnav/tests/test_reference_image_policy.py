"""ImageGoal adapter boundary checks; CPU only, no real model or robot."""
from __future__ import annotations

import base64
import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import torch

from deployment.lightnav import policy as policy_module
from deployment.lightnav.policy import HistoryConflictError
from deployment.lightnav.server import parse_request
from deployment.lightnav.tests.test_tracking import policy, request


def goal_jpeg(color=(20, 200, 40)):
    stream = io.BytesIO()
    Image.new("RGB", (40, 30), color).save(stream, "JPEG")
    return {"rgb_jpeg_base64": base64.b64encode(stream.getvalue()).decode()}


def imagegoal_request():
    value = request("experimental_reference_image")
    value.update(task_mode="imagegoal", goal=goal_jpeg())
    value.pop("instruction")
    return value


class ReferenceImageRequestTests(unittest.TestCase):
    def test_explicit_adapter_requires_photo_and_optional_instruction(self):
        payload = imagegoal_request()
        parsed = parse_request(payload)
        self.assertIsInstance(parsed[1], Image.Image)
        self.assertIsNone(parsed[3])
        self.assertEqual(parsed[4:], ("experimental_reference_image", "imagegoal"))
        without_task = copy.deepcopy(payload)
        without_task.pop("task_mode")
        self.assertEqual(parse_request(without_task)[5], "imagegoal")
        for changes in ({"goal": None}, {"goal_image": goal_jpeg()},
                        {"instruction": 3}, {"instruction": "x" * 8193},
                        {"task_mode": "objnav"}, {"task_mode": "tracking"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                parse_request({**payload, **changes})
        del payload["goal"]
        with self.assertRaises(ValueError):
            parse_request(payload)

    def test_target_image_never_implicitly_selects_adapter(self):
        payload = imagegoal_request()
        payload.pop("conditioning_mode")
        payload["instruction"] = "Go to the doorway."
        with self.assertRaises(ValueError):
            parse_request(payload)
        for mode in ("native_language", "native_tracking"):
            value = request(mode)
            value["goal"] = goal_jpeg()
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                parse_request(value)


class ReferenceImagePolicyTests(unittest.TestCase):
    def test_goal_is_separate_static_tubelet_not_robot_history(self):
        instance = policy()
        frames, goal, identity, instruction, mode, task = parse_request(imagegoal_request())
        response = instance.predict(frames, goal, identity, instruction, mode, task)
        observed = instance.sessions[("session", "task")]["policy"]
        sample = instance.engine.samples[-1]
        self.assertEqual(instance.engine.frame_calls, [])
        self.assertEqual(len(sample["video_segments"]), 2)
        target = sample["video_segments"][0]
        self.assertEqual(target["frame_indices"], [0, 0])
        self.assertEqual(target["total_frames"], 1)
        self.assertEqual(target["video"].shape[0], 2)
        self.assertTrue(torch.equal(target["video"][0], target["video"][1]))
        expected_target = observed._convert_frame(np.asarray(goal, dtype=np.uint8))
        self.assertTrue(torch.equal(target["video"][0], expected_target))
        self.assertFalse(torch.equal(target["video"][0], observed._get_video_tensor()[0]))
        self.assertEqual(len(observed._history), 1)
        self.assertEqual(observed._history_frame_ids, [0])
        self.assertEqual(len(observed._get_video_tensor()), 1)
        self.assertEqual(response["policy_history_count"], 1)
        self.assertEqual(response["policy_history_frames_appended"], 1)
        self.assertEqual(response["observation_frames"], [{"frame_id": "frame-1", "capture_timestamp_ns": 1_000_000_000}])
        self.assertIs(sample["_allow_vit_cache"], False)
        self.assertIn("Separate reference goal view", response["prompt"])
        self.assertIn("most recent robot observation", response["prompt"])
        self.assertEqual(response["instruction"], policy_module.DEFAULT_INSTRUCTION)
        self.assertEqual(response["implementation"], "go2_imagegoal_adaptation")
        self.assertEqual(response["official_release_commit"], policy_module.OFFICIAL_RELEASE_COMMIT)
        self.assertEqual(response["official_core_sha256"], policy_module.OFFICIAL_CORE_DIGEST)
        self.assertTrue(response["goal_image_consumed"])
        self.assertFalse(response["native_imagegoal_support"])
        self.assertFalse(response["arrival_confirmed"])
        self.assertEqual(response["goal_encoding"], "separate_static_two_frame_video_tubelet")
        self.assertEqual(response["task_mode"], "imagegoal")
        self.assertEqual(response["eval_task"], "vlnce")

    def test_custom_instruction_and_goal_generation_are_preserved(self):
        instance = policy()
        value = imagegoal_request()
        value["instruction"] = "Go to the scene in the reference photo."
        response = instance.predict(*parse_request(value))
        self.assertEqual(response["instruction"], value["instruction"])
        retry = instance.predict(*parse_request(value))
        self.assertEqual(retry["policy_history_frames_appended"], 0)
        self.assertEqual(instance.engine.resets, 1)
        replaced = copy.deepcopy(value)
        replaced["goal"] = goal_jpeg((250, 10, 70))
        with self.assertRaisesRegex(HistoryConflictError, "Goal pixels"):
            instance.predict(*parse_request(replaced))
        replaced["identity"]["goal_version"] += 1
        changed = instance.predict(*parse_request(replaced))
        self.assertEqual(changed["policy_history_reset_reason"], "new_goal_generation")
        self.assertEqual(changed["policy_history_count"], 1)

    def test_imagegoal_cache_and_prefix_do_not_leak_into_native_sessions(self):
        instance = policy()
        instance.predict(*parse_request(imagegoal_request()))
        language = request("native_language")
        language["identity"]["task_id"] = "native-task"
        response = instance.predict(*parse_request(language))
        sample = instance.engine.samples[-1]
        self.assertEqual(len(sample["video_segments"]), 1)
        self.assertIs(sample["_allow_vit_cache"], True)
        self.assertNotIn("Separate reference goal view", response["prompt"])
        self.assertFalse(response["goal_image_consumed"])
        self.assertEqual(response["implementation"], "official_release")
        self.assertIsNone(response["goal_encoding"])
        self.assertEqual(len(instance.engine.frame_calls), 1)
        self.assertEqual(instance.engine.frame_calls[-1][1]["task_type"], "vlnce_traj")
        reset_count = instance.engine.resets
        instance.predict(*parse_request(language))
        self.assertEqual(instance.engine.resets, reset_count)
        self.assertTrue(instance.engine.samples[-1]["_allow_vit_cache"])

    def test_blank_optional_imagegoal_instruction_resolves_to_same_default(self):
        instance = policy()
        value = imagegoal_request()
        instance.predict(*parse_request(value))
        for instruction in (None, "", "  "):
            with self.subTest(instruction=instruction):
                response = instance.predict(*parse_request({**value, "instruction": instruction}))
                self.assertEqual(response["instruction"], policy_module.DEFAULT_INSTRUCTION)
                self.assertEqual(response["policy_history_frames_appended"], 0)


class ReplayClientProfileTests(unittest.TestCase):
    def run_replay(self, imagegoal):
        from deployment.lightnav import client

        instance = policy()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frames = []
            for index in range(4):
                path = root / f"frame-{index}.jpg"
                Image.new("RGB", (32, 24), (100, 80, 60)).save(path, "JPEG")
                frames.append({"path": path.name, "frame_id": f"frame-{index}",
                               "capture_timestamp_ns": 1_000_000_000 + index * 250_000_000})
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"timestamp_source": "recorded_sensor_time", "frames": frames}))
            output = root / "replay"
            argv = ["client", "--manifest", str(manifest), "--output", str(output), "--max-requests", "1"]
            if imagegoal:
                goal = root / "goal.jpg"
                Image.new("RGB", (40, 30), (20, 200, 40)).save(goal, "JPEG")
                argv += ["--conditioning-mode", "experimental_reference_image", "--goal", str(goal)]
            else:
                argv += ["--instruction", "Go to the doorway."]
            requests = []

            def fake_post(url, payload, timeout):
                requests.append((url, payload))
                if url.endswith("/reset"):
                    return instance.reset(payload["identity"], retire=payload.get("retire", False))
                return instance.predict(*parse_request(payload))

            with patch("sys.argv", argv), patch.object(client, "post", side_effect=fake_post), redirect_stdout(io.StringIO()):
                client.main()
            run = json.loads((output / "run.json").read_text())
            result = json.loads((output / "results.jsonl").read_text())["response"]
            self.assertEqual(run["implementation"], "go2_imagegoal_adaptation" if imagegoal else "official_release")
            self.assertIs(result["goal_image_consumed"], imagegoal)
            self.assertEqual("goal" in requests[0][1], imagegoal)
            self.assertTrue(requests[-1][0].endswith("/reset"))
            self.assertFalse(instance.sessions)
            if imagegoal:
                self.assertEqual(result["instruction"], policy_module.DEFAULT_INSTRUCTION)
                self.assertIsNotNone(run["goal_jpeg_sha256"])
            else:
                self.assertEqual(result["instruction"], "Go to the doorway.")
                self.assertIsNone(run["goal_jpeg_sha256"])

    def test_native_replay_omits_goal_and_selects_official_response(self):
        self.run_replay(False)

    def test_imagegoal_replay_supplies_full_photo_and_default_instruction(self):
        self.run_replay(True)

    def test_goal_argument_cannot_change_default_native_profile(self):
        from deployment.lightnav import client

        argv = ["client", "--manifest", "unused.json", "--output", "unused-output", "--goal", "unused.jpg"]
        with patch("sys.argv", argv), patch.object(client, "post") as post, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as exit_code:
                client.main()
        self.assertEqual(exit_code.exception.code, 2)
        post.assert_not_called()


if __name__ == "__main__":
    unittest.main()
