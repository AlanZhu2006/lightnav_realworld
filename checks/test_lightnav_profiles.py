"""Portable profile structure checks; no service, camera or SDK is started."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from deployment.bringup import ensure_observation as observation
from deployment.go2 import start_native_imagegoal as native
from deployment.operator import open_console as console

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    return json.loads((ROOT / "deployment/bringup" / name).read_text())


class LightNavProfiles(unittest.TestCase):
    def test_shadow_profile_exact_allowlist_and_no_nomad_model(self):
        value = load("jetson_lightnav.json")
        observation.validate_config(value, ROOT / observation.LIGHTNAV_CONFIG_RELATIVE)
        self.assertNotIn("nomad", [item["name"] for item in value["components"]])

    def test_shadow_rejects_extra_model_or_motion_command(self):
        value = load("jetson_lightnav.json")
        candidates = [copy.deepcopy(value), copy.deepcopy(value)]
        candidates[0]["components"][0]["command"].append("--execute")
        candidates[1]["components"].append(next(item for item in load("jetson_unitree_camera.json")["components"] if item["name"] == "nomad"))
        for candidate in candidates:
            with self.assertRaises(ValueError):
                observation.validate_config(candidate, ROOT / observation.LIGHTNAV_CONFIG_RELATIVE)

    def test_native_is_combined_profile_without_optional_model(self):
        combined, lightnav = load("jetson_native_imagegoal.json"), load("jetson_native_lightnav.json")
        combined["name"] = "jetson_native_lightnav"
        combined["components"] = [item for item in combined["components"] if item["name"] != "nomad"]
        self.assertEqual(combined, lightnav)
        command = next(item["command"] for item in lightnav["components"] if item["name"] == "runtime")
        self.assertIn("supervised-native", command)
        self.assertIn("--operator-supervised", command)
        self.assertNotIn("--execute-supervised", command)

    def test_dry_run_starts_no_process_and_creates_no_state(self):
        with patch.object(observation, "read_json", return_value=load("jetson_lightnav.json")):
            with patch.object(observation, "validate_config", return_value=load("jetson_lightnav.json")):
                called = []
                result = observation.ensure_observation(
                    ROOT / observation.LIGHTNAV_CONFIG_RELATIVE, dry_run=True,
                    status_probe=lambda: None, bridge_probe=lambda: False,
                    alive=lambda item: False, run=lambda *args, **kwargs: called.append(args),
                )
        self.assertEqual(result["action"], "start_required")
        self.assertEqual(called, [])

    def test_native_launcher_knows_both_lightnav_ownership_profiles(self):
        self.assertIn(ROOT / "deployment/bringup/jetson_lightnav.json", native.OBSERVATION_PROFILES)
        self.assertIn(native.LIGHTNAV_PROFILE, native.OBSERVATION_PROFILES)

    def test_console_rejects_invalid_remote_path_before_ssh(self):
        with patch.object(console.subprocess, "run") as run, self.assertRaises(SystemExit):
            console.main(["--backend", "lightnav", "--remote-root", "relative/path", "--no-open"])
        run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
