"""LightNav startup lifecycle checks: mocked models, real filesystem and locks."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deployment.lightnav.launcher import (
    CONFIG_RELATIVE, HEALTH_EXPECT, MODEL_REVISION, MODEL_SHA256, OFFICIAL_RELEASE_COMMIT, OFFICIAL_CORE_DIGEST, SERVER_IMPLEMENTATION, PROFILE,
    ensure_local_lightnav, health_matches, probe_health,
)


def healthy():
    return {"ok": True, "model_loaded": True, "contract": {
        "model": HEALTH_EXPECT["contract.model"], "model_revision": MODEL_REVISION,
        "model_sha256": MODEL_SHA256, "device": "cuda",
        "implementation": SERVER_IMPLEMENTATION, "official_release_commit": OFFICIAL_RELEASE_COMMIT,
        "official_core_sha256": OFFICIAL_CORE_DIGEST, "imagegoal_adaptation_support": True,
        "goal_image_consumed": False,
        "retirement_protocol": {"supported": True, "schema_version": "lightnav.owned_retirement.v1"}}}


class LightNavLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.project = Path(self.temporary.name).resolve()
        self.config_path = self.project / CONFIG_RELATIVE
        self.config_path.parent.mkdir(parents=True)
        self.config = {"name": PROFILE, "components": [{
            "name": "lightnav", "command": ["bash", "deployment/lightnav/run_server.sh"],
            "health_url": "http://127.0.0.1:18884/healthz",
            "health_expect": {key: value for key, value in HEALTH_EXPECT.items() if key != "contract.device"},
            "adopt_existing": True, "startup_timeout_s": 180}]}
        self.write_config()
        python = self.project / "deployment/lightnav/.venv/bin/python"
        python.parent.mkdir(parents=True)
        python.symlink_to(sys.executable)
        self.weights = self.project / "deployment/lightnav/.runtime/weights"
        self.weights.mkdir(parents=True)
        entries = []
        for name in ("config.json", "action_tokenizer/manifest.json", "model.safetensors"):
            path = self.weights / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b"fake")
            entries.append({"path": name, "size": 4, "sha256": MODEL_SHA256 if name.endswith(".safetensors") else "0" * 64})
        self.manifest = {"revision": MODEL_REVISION, "files": entries}
        self.write_manifest()
        self.status = None
        self.occupied = False
        self.calls = []

    def tearDown(self):
        self.temporary.cleanup()

    def write_config(self):
        self.config_path.write_text(json.dumps(self.config))

    def write_manifest(self):
        (self.weights / "SOURCE_MANIFEST.json").write_text(json.dumps(self.manifest))

    def write_state(self, state):
        path = self.project / ".runtime/stack" / PROFILE / "state.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(state))
        return path

    def start_model(self, command, **kwargs):
        self.calls.append((command, kwargs))
        self.status = healthy()
        return subprocess.CompletedProcess(command, 0, "{}", "")

    def ensure(self, **kwargs):
        arguments = {"project": self.project, "health_probe": lambda port: deepcopy(self.status),
                     "port_probe": lambda port: self.occupied,
                     "alive": lambda item: item.get("alive", False), "run": self.start_model,
                     "environ": {}, "hostname": "original-workstation"}
        arguments.update(kwargs)
        return ensure_local_lightnav(**arguments)

    def test_online_service_adopted_without_config_assets_or_writes(self):
        self.status = healthy()
        self.config_path.unlink()
        (self.weights / "model.safetensors").unlink()
        report = self.ensure()
        self.assertEqual(report["action"], "adopted")
        self.assertFalse(report["robot_commands_sent"])
        self.assertEqual(self.calls, [])
        self.assertFalse((self.project / ".runtime").exists())

    def test_wrong_model_on_occupied_port_preserved(self):
        self.status, self.occupied = healthy(), True
        self.status["contract"]["model_revision"] = "wrong"
        with self.assertRaisesRegex(RuntimeError, "保留现有服务"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_service_without_durable_retirement_is_not_adopted(self):
        for protocol in (None, {"supported": False, "schema_version": "lightnav.owned_retirement.v1"},
                         {"supported": True, "schema_version": "old"}):
            self.status, self.occupied = healthy(), True
            self.status["contract"]["retirement_protocol"] = protocol
            with self.subTest(protocol=protocol), self.assertRaisesRegex(RuntimeError, "保留现有服务"):
                self.ensure()
        self.assertEqual(self.calls, [])

    def test_native_only_official_service_cannot_satisfy_adapter_contract(self):
        self.status, self.occupied = healthy(), True
        self.status["contract"].update(implementation="official_release", imagegoal_adaptation_support=False)
        self.assertFalse(health_matches(self.status))
        with self.assertRaisesRegex(RuntimeError, "保留现有服务"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_legacy_experimental_service_on_same_port_is_never_adopted(self):
        self.status, self.occupied = healthy(), True
        for field in ("implementation", "official_release_commit", "official_core_sha256", "imagegoal_adaptation_support", "goal_image_consumed"):
            self.status["contract"].pop(field)
        self.status["contract"].update(conditioning_mode="experimental_reference_image", goal_image_consumed=True)
        self.assertFalse(health_matches(self.status))
        with self.assertRaisesRegex(RuntimeError, "保留现有服务"):
            self.ensure()
        self.assertEqual(self.calls, [])
        self.assertFalse((self.project / ".runtime").exists())

    def test_official_release_identity_and_no_goal_consumption_are_mandatory(self):
        for field, value in (("implementation", "experimental"),
                             ("official_release_commit", "another-release"),
                             ("official_core_sha256", "wrong-hash"),
                             ("imagegoal_adaptation_support", False),
                             ("goal_image_consumed", True),
                             ("goal_image_consumed", 0)):
            health = healthy()
            health["contract"][field] = value
            with self.subTest(field=field, value=value):
                self.assertFalse(health_matches(health))
        for field in ("implementation", "official_release_commit", "official_core_sha256", "imagegoal_adaptation_support", "goal_image_consumed"):
            health = healthy()
            del health["contract"][field]
            with self.subTest(missing=field):
                self.assertFalse(health_matches(health))

    def test_cold_start_profile_must_verify_official_release_contract(self):
        for key in ("contract.implementation", "contract.official_release_commit", "contract.official_core_sha256", "contract.imagegoal_adaptation_support", "contract.goal_image_consumed"):
            with self.subTest(missing=key):
                original = self.config["components"][0]["health_expect"].pop(key)
                self.write_config()
                with self.assertRaisesRegex(RuntimeError, "官方模型合同"):
                    self.ensure()
                self.config["components"][0]["health_expect"][key] = original
                self.write_config()
        self.assertEqual(self.calls, [])

    def test_non_json_server_on_port_preserved(self):
        self.occupied = True
        with self.assertRaisesRegex(RuntimeError, "已占用"):
            self.ensure()
        self.assertFalse((self.project / ".runtime").exists())

    def test_cpu_or_unloaded_services_not_accepted(self):
        for field, value in (("device", "cpu"), ("device", "cuda:0"), ("model_sha256", "wrong")):
            health = healthy()
            health["contract"][field] = value
            self.assertFalse(health_matches(health))
        health = healthy()
        health["model_loaded"] = 1
        self.assertFalse(health_matches(health))
        self.assertFalse(health_matches(None))
        self.assertFalse(health_matches({"ok": True}))

    def test_cold_start_uses_existing_manager_and_exact_final_health(self):
        report = self.ensure()
        self.assertEqual(report["action"], "started")
        self.assertEqual(len(self.calls), 1)
        command, kwargs = self.calls[0]
        self.assertEqual(command, [sys.executable, "-m", "deployment.bringup", "--config", str(self.config_path), "start"])
        self.assertEqual(kwargs["cwd"], self.project)
        self.assertEqual(kwargs["timeout"], 225)
        self.assertNotIn("stop", command)
        self.assertNotIn("restart", command)
        self.assertTrue(health_matches(report["health"]))
        self.assertFalse((self.project / ".runtime/stack" / PROFILE / "state.json").exists())

    def test_custom_port_online_adopted_offline_not_started(self):
        self.status = healthy()
        self.assertEqual(self.ensure(port=28884)["action"], "adopted")
        self.status = None
        with self.assertRaisesRegex(RuntimeError, "自定义"):
            self.ensure(port=28884)
        self.assertEqual(self.calls, [])

    def test_alive_manager_preserved_including_state_contents(self):
        path = self.write_state({"state": "starting", "components": [
            {"name": "lightnav", "ownership": "managed", "alive": True}]})
        original = path.read_bytes()
        with self.assertRaisesRegex(RuntimeError, "存活进程"):
            self.ensure()
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(self.calls, [])

    def test_dead_manager_state_does_not_block_cold_start_or_get_overwritten(self):
        path = self.write_state({"state": "running", "components": [
            {"name": "lightnav", "ownership": "managed", "alive": False}]})
        original = path.read_bytes()
        self.assertEqual(self.ensure()["action"], "started")
        self.assertEqual(path.read_bytes(), original)

    def test_lock_rechecks_model_started_by_another_invocation(self):
        probes = iter((None, healthy()))
        report = self.ensure(health_probe=lambda port: next(probes))
        self.assertEqual(report["action"], "adopted")
        self.assertEqual(self.calls, [])

    def test_two_concurrent_invocations_load_only_one_model(self):
        barrier = threading.Barrier(2)
        local = threading.local()
        def probe(port):
            if not getattr(local, "initial_probe", False):
                local.initial_probe = True
                barrier.wait(timeout=3)
                return None
            return deepcopy(self.status)
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(self.ensure, health_probe=probe) for _ in range(2)]
            results = [future.result(timeout=5) for future in futures]
        self.assertCountEqual([item["action"] for item in results], ["started", "adopted"])
        self.assertEqual(len(self.calls), 1)

    def test_maxwell_hostname_environment_and_path_reject_cold_start(self):
        for override in ({"hostname": "maxwell"}, {"hostname": "Maxwell.lab"},
                         {"environ": {"MAXWELL_DATA_ROOT": ""}},
                         {"project": Path("/local_data/example-user/projects/DejaView")}):
            with self.subTest(override=override), self.assertRaisesRegex(RuntimeError, "GPU 尚需协调"):
                self.ensure(**override)
        self.assertEqual(self.calls, [])
        self.assertFalse((self.project / ".runtime").exists())

    def test_maxwell_can_adopt_existing_healthy_model(self):
        self.status = healthy()
        report = self.ensure(hostname="maxwell", environ={"MAXWELL_DATA_ROOT": "/local_data/example-user"})
        self.assertEqual(report["action"], "adopted")
        self.assertEqual(self.calls, [])

    def test_missing_python_preserves_without_installing(self):
        (self.project / "deployment/lightnav/.venv/bin/python").unlink()
        with self.assertRaisesRegex(RuntimeError, "独立环境未准备好"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_truncated_checkpoint_never_starts(self):
        (self.weights / "model.safetensors").write_bytes(b"truncated")
        with self.assertRaisesRegex(RuntimeError, "权重缺失或不完整"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_wrong_manifest_revision_never_starts(self):
        self.manifest["revision"] = "wrong"
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, "revision"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_manifest_path_escape_rejected(self):
        self.manifest["files"][0]["path"] = "../escape"
        self.write_manifest()
        with self.assertRaisesRegex(RuntimeError, "路径越界"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_changed_manager_command_never_runs(self):
        self.config["components"][0]["command"] = ["other-script"]
        self.write_config()
        with self.assertRaisesRegex(RuntimeError, "启动命令"):
            self.ensure()
        self.assertEqual(self.calls, [])

    def test_invalid_manager_state_preserved(self):
        path = self.write_state([])
        original = path.read_bytes()
        with self.assertRaisesRegex(RuntimeError, "状态无效"):
            self.ensure()
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(self.calls, [])

    def test_failed_manager_reports_without_stop_or_restart(self):
        def fail(command, **kwargs):
            self.calls.append((command, kwargs))
            return subprocess.CompletedProcess(command, 1, "startup failed", "")
        with self.assertRaisesRegex(RuntimeError, "startup failed"):
            self.ensure(run=fail)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual(self.calls[0][0][-1], "start")

    def test_timeout_preserves_model_loading_process(self):
        def timeout(command, **kwargs):
            self.calls.append((command, kwargs))
            raise subprocess.TimeoutExpired(command, kwargs["timeout"])
        with self.assertRaisesRegex(RuntimeError, "保留现有进程"):
            self.ensure(run=timeout)
        self.assertEqual(len(self.calls), 1)

    def test_post_start_wrong_device_rejected_without_cleanup(self):
        def wrong_device(command, **kwargs):
            result = self.start_model(command, **kwargs)
            self.status["contract"]["device"] = "cpu"
            return result
        with self.assertRaisesRegex(RuntimeError, "CUDA 健康检查未通过"):
            self.ensure(run=wrong_device)
        self.assertEqual(len(self.calls), 1)

    def test_host_port_overrides_cannot_start_an_unchecked_endpoint(self):
        for variable, value in (("LIGHTNAV_HOST", "0.0.0.0"), ("LIGHTNAV_PORT", "28884")):
            with self.subTest(variable=variable), self.assertRaisesRegex(RuntimeError, "LIGHTNAV_HOST/PORT"):
                self.ensure(environ={variable: value})
        self.assertEqual(self.calls, [])

    def test_health_probe_reads_health_only_with_bound(self):
        class Response:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                pass
            def read(self, maximum):
                self.maximum = maximum
                return json.dumps(healthy()).encode()
        response = Response()
        with patch("deployment.lightnav.launcher.urlopen", return_value=response) as request:
            self.assertTrue(health_matches(probe_health(18884)))
        request.assert_called_once_with("http://127.0.0.1:18884/healthz", timeout=3)
        self.assertEqual(response.maximum, 1024 * 1024 + 1)


if __name__ == "__main__":
    unittest.main()
