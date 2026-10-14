import json
import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

from deployment.configuration import RobotConfig, ServerConfig, websocket_url
from deployment.launch import _stop_group, main, run_server
from deployment.preflight import checkpoint_files


def test_commands_preserve_paths_and_arguments_without_shell(tmp_path):
    root = tmp_path / "repo with spaces"
    config = ServerConfig.from_env({"LIGHTNAV_MODEL_PATH": "model with spaces"}, root)
    command = config.command(root / "runtime/server.ready")
    assert command[command.index("--model_path") + 1] == str(root / "model with spaces")
    assert command[command.index("--task") + 1] == "vln"
    assert command[command.index("--record_dir") + 1] == str(root / "recordings-inference/episodes")
    robot = RobotConfig.from_env({"LIGHTNAV_SERVER_URL": "ws://gpu-host:8050"}, root)
    assert f"recording_directory:={root / 'recordings'}" in robot.command()
    assert "linear_limit:=0.3" in robot.command()
    assert "angular_limit:=0.5" in robot.command()


@pytest.mark.parametrize(
    "settings",
    [
        {"LIGHTNAV_PORT": "0"},
        {"LIGHTNAV_PORT": "65536"},
        {"LIGHTNAV_GPU_MEMORY_UTILIZATION": "nan"},
        {"LIGHTNAV_GPU_MEMORY_UTILIZATION": "1.1"},
        {"LIGHTNAV_MAX_BATCH_SIZE": "0"},
        {"LIGHTNAV_BACKEND": "remote"},
        {"LIGHTNAV_HOST_TELEMETRY": "yes"},
    ],
)
def test_invalid_server_settings_fail(settings):
    with pytest.raises(ValueError):
        ServerConfig.from_env(settings)


@pytest.mark.parametrize(
    "settings",
    [
        {"LIGHTNAV_LINEAR_LIMIT": "-1"},
        {"LIGHTNAV_ANGULAR_LIMIT": "inf"},
        {"LIGHTNAV_IMAGE_TRANSPORT": "mjpeg"},
        {"ROS_DOMAIN_ID": "-1"},
        {"LIGHTNAV_IMAGE_TOPIC": "relative/topic"},
    ],
)
def test_invalid_robot_settings_fail(settings):
    with pytest.raises(ValueError):
        RobotConfig.from_env({"LIGHTNAV_SERVER_URL": "ws://gpu:8050", **settings})


@pytest.mark.parametrize(
    "url", ["", "http://gpu:8050", "ws://u:p@gpu:8050", "ws://gpu:0", "ws://gpu:8050/#x"]
)
def test_invalid_endpoint_fails(url):
    with pytest.raises(ValueError):
        websocket_url(url)


def test_dry_run_never_launches_or_checks_devices(monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail("dry-run started a process or checked a runtime")

    monkeypatch.setenv("LIGHTNAV_SERVER_URL", "ws://gpu:8050")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    monkeypatch.setattr("deployment.launch.check_server", forbidden)
    monkeypatch.setattr("deployment.launch.check_robot", forbidden)
    for role in ("server", "robot"):
        assert main([role, "--dry-run"]) == 0
        assert json.loads(capsys.readouterr().out)["role"] == role


def test_checkpoint_rejects_missing_weight_shard(tmp_path):
    for name in ("config.json", "eval_config.json", "processor_config.json", "tokenizer.json"):
        (tmp_path / name).write_text("{}")
    (tmp_path / "model.safetensors.index.json").write_text(
        json.dumps({"weight_map": {"x": "missing.safetensors"}})
    )
    with pytest.raises(ValueError, match="weight shards"):
        checkpoint_files(tmp_path)


def test_server_exit_is_propagated_and_ready_removed(tmp_path, monkeypatch):
    config = ServerConfig.from_env({"LIGHTNAV_HOST_TELEMETRY": "0"}, tmp_path)
    monkeypatch.setattr(
        ServerConfig, "command", lambda self, ready: [sys.executable, "-c", "raise SystemExit(7)"]
    )
    previous = signal.getsignal(signal.SIGTERM)
    assert run_server(config) == 7
    descriptor = json.loads((tmp_path / "runtime/server.json").read_text())
    assert not Path(descriptor["ready_file"]).exists()
    assert signal.getsignal(signal.SIGTERM) == previous


def test_cleanup_only_targets_its_process_group():
    owned = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True
    )
    unrelated = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)"], start_new_session=True
    )
    try:
        _stop_group(owned, timeout=3)
        assert owned.poll() is not None
        assert unrelated.poll() is None
        assert os.getpgid(unrelated.pid) == unrelated.pid
    finally:
        _stop_group(owned, timeout=3)
        _stop_group(unrelated, timeout=3)
