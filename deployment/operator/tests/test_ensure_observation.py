from copy import deepcopy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from deployment.bringup.ensure_observation import (
    CONFIG_RELATIVE, CPU_ENV, PROFILE, PROJECT, RUNTIME_URL,
    SAFE_COMMANDS, SAFE_HEALTH, SAFE_PROBES,
    ensure_observation, validate_config,
)


def profile():
    components = []
    for name, command in SAFE_COMMANDS.items():
        component = {"name": name, "command": deepcopy(command), "startup_timeout_s": 20,
                     "adopt_existing": name in {"go2_readonly_telemetry", "nomad"}}
        if name in {"runtime", "ros_observer"}:
            component["env"] = dict(CPU_ENV)
        component.update({"health_command": deepcopy(SAFE_PROBES[name])} if name in SAFE_PROBES else deepcopy(SAFE_HEALTH[name]))
        components.append(component)
    return {"name": PROFILE, "runtime_url": RUNTIME_URL, "components": components}


def healthy_status():
    return {"schema_version": "imagegoal.runtime.v1", "session_id": "unchanged-session", "state": "IDLE",
            "profile": "external", "hardware_transport": "absent",
            "execution_capability": {"operator_run_configured": False}}


class EnsureObservationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.project = Path(self.temporary.name).resolve()
        self.config_path = self.project / CONFIG_RELATIVE
        self.config_path.parent.mkdir(parents=True)
        self.config = profile()
        self.write_config()
        self.status, self.bridge, self.started = None, False, []

    def tearDown(self):
        self.temporary.cleanup()

    def write_config(self):
        self.config_path.write_text(json.dumps(self.config))

    def write_state(self, value):
        state_path = self.project / ".runtime/stack" / PROFILE / "state.json"
        state_path.parent.mkdir(parents=True)
        state_path.write_text(json.dumps(value))

    def start(self, command, **kwargs):
        self.started.append((command, kwargs))
        self.status, self.bridge = healthy_status(), True
        return subprocess.CompletedProcess(command, 0, "", "")

    def ensure(self, **kwargs):
        return ensure_observation(self.config_path, project=self.project,
                                  status_probe=lambda: deepcopy(self.status), bridge_probe=lambda: self.bridge,
                                  alive=lambda item: item.get("alive", False), run=self.start, **kwargs)

    def test_healthy_stack_preserves_session_without_restart_or_state_write(self):
        self.status, self.bridge = healthy_status(), True
        report = self.ensure()
        self.assertTrue(report["ok"])
        self.assertEqual(report["action"], "already_healthy")
        self.assertEqual(report["session_id"], "unchanged-session")
        self.assertEqual(self.started, [])
        self.assertFalse((self.project / ".runtime").exists())

    def test_stale_running_state_with_all_processes_dead_can_start(self):
        self.write_state({"state": "running", "components": [{"name": "runtime", "ownership": "managed", "alive": False}]})
        report = self.ensure()
        self.assertTrue(report["ok"])
        self.assertEqual(report["action"], "started")
        self.assertEqual(len(self.started), 1)
        command, kwargs = self.started[0]
        self.assertEqual(command[-1], "start")
        self.assertIn("deployment.bringup", command)
        self.assertEqual(Path(command[-2]), self.config_path)
        self.assertEqual(kwargs["cwd"], self.project)
        self.assertNotIn("stop", command)

    def test_partial_live_stack_is_preserved_without_start_or_stop(self):
        self.write_state({"state": "running", "components": [{"name": "go2_readonly_camera", "ownership": "managed", "alive": True}]})
        report = self.ensure()
        self.assertFalse(report["ok"])
        self.assertEqual(report["action"], "preserve_partial_stack")
        self.assertEqual(report["managed_alive"], ["go2_readonly_camera"])
        self.assertEqual(self.started, [])

    def test_live_model_owned_by_another_profile_does_not_prevent_cold_start(self):
        self.write_state({"state": "running", "components": [{"name": "nomad", "ownership": "external_healthy", "alive": True}]})
        self.assertEqual(self.ensure()["action"], "started")
        self.assertEqual(len(self.started), 1)

    def test_dry_run_does_not_create_state_or_start_processes(self):
        before = sorted(str(path) for path in self.project.rglob("*"))
        report = self.ensure(dry_run=True)
        self.assertEqual(report["action"], "start_required")
        self.assertTrue(report["dry_run"])
        self.assertEqual(self.started, [])
        self.assertEqual(before, sorted(str(path) for path in self.project.rglob("*")))

    def test_operator_run_runtime_is_never_accepted_as_healthy(self):
        self.status, self.bridge = healthy_status(), True
        self.status["execution_capability"]["operator_run_configured"] = True
        report = self.ensure()
        self.assertFalse(report["ok"])
        self.assertEqual(report["action"], "preserve_existing_endpoint")
        self.assertEqual(self.started, [])

    def test_runtime_without_bridge_is_preserved_without_replacement(self):
        self.status = healthy_status()
        report = self.ensure()
        self.assertFalse(report["ok"])
        self.assertEqual(report["action"], "preserve_existing_endpoint")
        self.assertEqual(self.started, [])

    def test_nonshadow_or_drive_commands_are_rejected_before_start(self):
        for name, command in (("runtime", SAFE_COMMANDS["runtime"] + ["--go2", "drive"]),
                              ("runtime", [part.replace("shadow", "operator-run") for part in SAFE_COMMANDS["runtime"]]),
                              ("go2_readonly_camera", ["bash", "deployment/go2/run_go2.sh", "execute"]),
                              ("go2_readonly_telemetry", ["bash", "deployment/go2/run_go2.sh", "hardware-locked"])):
            self.config = profile()
            next(item for item in self.config["components"] if item["name"] == name)["command"] = command
            self.write_config()
            with self.subTest(name=name, command=command), self.assertRaises(ValueError):
                self.ensure()
        self.assertEqual(self.started, [])

    def test_health_shell_and_environment_cannot_bypass_command_allowlist(self):
        for kind in ("health_command", "env"):
            self.config = profile()
            component = self.config["components"][0]
            component[kind] = ["bash", "-c", "unexpected command"] if kind == "health_command" else {"LD_PRELOAD": "unexpected"}
            self.write_config()
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.ensure()
        self.assertEqual(self.started, [])

    def test_old_realsense_profile_path_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_config(profile(), self.project / "deployment/bringup/jetson.json", self.project)

    def test_shipped_current_profile_matches_the_allowlisted_contract(self):
        path = PROJECT / CONFIG_RELATIVE
        if not path.exists():
            self.skipTest("current native-camera profile not included in this checkout")
        self.assertEqual(validate_config(json.loads(path.read_text()), path)["name"], PROFILE)


if __name__ == "__main__":
    unittest.main()
