"""Ensure the existing read-only Go2/Foxglove observation profile at startup.

This is a one-shot cold-start check, not a process watchdog. A healthy stack is
left intact. Any surviving managed process in an unhealthy stack is preserved
for explicit inspection; this helper never calls stop, signals, or navigation.
"""
from __future__ import annotations

import argparse
import fcntl
import json
from pathlib import Path
import socket
import subprocess
import sys
from urllib.request import urlopen

from .__main__ import PROJECT, owned_alive, read_json


PROFILE = "jetson_unitree_camera"
CONFIG_RELATIVE = Path("deployment/bringup/jetson_unitree_camera.json")
LIGHTNAV_CONFIG_RELATIVE = Path("deployment/bringup/jetson_lightnav.json")
RUNTIME_URL = "http://127.0.0.1:8093"
SAFE_COMMANDS = {
    "go2_readonly_telemetry": ["bash", "deployment/go2/run_go2.sh", "telemetry", "--watch"],
    "nomad": ["bash", "deployment/sensors/run_nomad_jetson.sh"],
    "runtime": [".venv-nomad-cpu/bin/python", "-m", "deployment.imagegoal",
                "--profile", "external", "--execution-profile", "shadow",
                "--output", "{run}/runtime", "--host", "127.0.0.1", "--port", "8093",
                "--nomad-url", "http://127.0.0.1:18883", "--lightnav-url", "http://127.0.0.1:18884",
                "--static-dir", "deployment/operator/web"],
    "go2_readonly_camera": ["bash", "deployment/go2/run_go2.sh", "camera", "--runtime-url", RUNTIME_URL],
    "ros_observer": ["bash", "-c", "source /opt/ros/humble/setup.bash && exec .venv-nomad-cpu/bin/python -m deployment.imagegoal.ros_bridge --runtime-url http://127.0.0.1:8093 --odom-topic /go2/odometry --preview-compressed-topic /go2/front_camera/image_raw/compressed --preview-timestamp-source go2_video_rpc_receive_host_system_time_not_sensor_source_time"],
    "foxglove": ["bash", "deployment/operator/run_foxglove.sh"],
}
SAFE_PROBES = {
    "go2_readonly_telemetry": ["python3", "-c", "import json,time,pathlib; s=json.loads(pathlib.Path('.runtime/go2/telemetry.json').read_text()); assert s['ready'] and s['read_only'] and not s['synthetic'] and not s['motion_api_imported']; assert s['boot_id']==pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(); assert all(0<=(time.monotonic_ns()-s[k])/1e9<.3 for k in ['sport_received_monotonic_ns','lowstate_received_monotonic_ns']); print('Live read-only Go2 feedback')"],
    "go2_readonly_camera": ["python3", "-c", "import json,time,pathlib,urllib.request; c=json.loads(pathlib.Path('.runtime/go2/camera.json').read_text()); assert c['ready'] and 0<=(time.time_ns()-c['timestamp_ns'])/1e9<1; s=json.load(urllib.request.urlopen('http://127.0.0.1:8093/status',timeout=2)); r=s['sensors']['rgb']; assert r and 0<=r['age_s']<1 and not r['synthetic'] and r['timestamp_source']=='go2_video_rpc_receive_host_system_time_not_sensor_source_time'; print('Live native RGB; receive-time proxy, exposure time unavailable')"],
    "ros_observer": ["python3", "-c", "import json,urllib.request; s=json.load(urllib.request.urlopen('http://127.0.0.1:8093/status',timeout=2)); assert all(s['sensors'][k] and 0<=s['sensors'][k]['age_s']<1 for k in ['rgb','odom']); assert not s['execution_capability']['operator_run_configured']; print('Native RGB + measured odom; no aligned depth, shadow only')"],
}
SAFE_HEALTH = {
    "nomad": {"health_url": "http://127.0.0.1:18883/healthz", "health_expect": {
        "ok": True, "model_loaded": True, "contract.device": "cuda",
        "contract.checkpoint.sha256": "70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333"}},
    "runtime": {"health_url": RUNTIME_URL + "/status", "health_expect": {
        "schema_version": "imagegoal.runtime.v1", "profile": "external", "hardware_transport": "absent",
        "execution_capability.operator_run_configured": False}},
    "foxglove": {"tcp": ["127.0.0.1", 8766]},
}
CPU_ENV = {"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"}


def validate_config(config, path, project=PROJECT):
    """Accept the current explicit shadow/read-only command surface only."""
    lightnav_only = Path(path).resolve() == (Path(project) / LIGHTNAV_CONFIG_RELATIVE).resolve()
    if not lightnav_only and Path(path).resolve() != (Path(project) / CONFIG_RELATIVE).resolve():
        raise ValueError("only the allowlisted native-camera observation profiles are allowed")
    expected_profile = "jetson_lightnav" if lightnav_only else PROFILE
    commands = {name: command for name, command in SAFE_COMMANDS.items() if not lightnav_only or name != "nomad"}
    if not isinstance(config, dict) or config.get("name") != expected_profile or config.get("runtime_url") != RUNTIME_URL:
        raise ValueError("unexpected observation profile name or runtime endpoint")
    if set(config) != {"name", "runtime_url", "components"}:
        raise ValueError("unexpected observation profile fields")
    components = config.get("components")
    if not isinstance(components, list) or [item.get("name") if isinstance(item, dict) else None for item in components] != list(commands):
        raise ValueError("observation components must match the current read-only profile and order")
    for item in components:
        name = item["name"]
        if item.get("command") != commands[name]:
            raise ValueError(f"{name}: command is not the allowlisted shadow/read-only command")
        if item.get("env", {}) != (CPU_ENV if name in {"runtime", "ros_observer"} else {}):
            raise ValueError(f"{name}: unexpected process environment")
        if item.get("adopt_existing", False) is not (name in {"go2_readonly_telemetry", "nomad"}):
            raise ValueError(f"{name}: unexpected lifecycle ownership policy")
        expected_health = {"health_command": SAFE_PROBES[name]} if name in SAFE_PROBES else SAFE_HEALTH[name]
        for key in ("health_command", "health_url", "health_expect", "tcp"):
            if item.get(key) != expected_health.get(key):
                raise ValueError(f"{name}: unexpected health probe")
        allowed = {"name", "command", "env", "adopt_existing", "startup_timeout_s", "probe_timeout_s", *expected_health}
        if set(item) - allowed:
            raise ValueError(f"{name}: unexpected component fields")
        for key, default, maximum in (("startup_timeout_s", 30, 120), ("probe_timeout_s", 8, 30)):
            value = item.get(key, default)
            if type(value) not in (int, float) or not 0 < value <= maximum:
                raise ValueError(f"{name}: invalid {key}")
    return config


def runtime_status():
    try:
        with urlopen(RUNTIME_URL + "/status", timeout=3) as response:
            body = response.read(4 * 1024 * 1024 + 1)
        if len(body) > 4 * 1024 * 1024:
            return None
        value = json.loads(body)
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def bridge_ready():
    try:
        with socket.create_connection(("127.0.0.1", 8766), timeout=3):
            return True
    except OSError:
        return False


def safe_runtime(status):
    return bool(isinstance(status, dict)
                and status.get("schema_version") == "imagegoal.runtime.v1"
                and status.get("profile") == "external"
                and status.get("hardware_transport") == "absent"
                and isinstance(status.get("session_id"), str) and status["session_id"]
                and isinstance(status.get("execution_capability"), dict)
                and status["execution_capability"].get("operator_run_configured") is False)


def assess(state, status, bridge_ok, alive=owned_alive):
    if safe_runtime(status) and bridge_ok:
        return {"ok": True, "action": "already_healthy", "session_id": status["session_id"],
                "state": status.get("state"), "processes_preserved": True}
    surviving = [item.get("name", "unknown") for item in state.get("components", [])
                 if isinstance(item, dict) and item.get("ownership") == "managed" and alive(item)]
    if surviving:
        return {"ok": False, "action": "preserve_partial_stack", "managed_alive": surviving,
                "error": "observation stack is partly running; preserving every process for explicit inspection"}
    # A live endpoint with missing/stale state might belong to somebody else.
    # Never try a new stack against such endpoints merely because no PID is owned.
    if status is not None or bridge_ok:
        return {"ok": False, "action": "preserve_existing_endpoint",
                "error": "an existing endpoint is unhealthy or incomplete; preserving it without starting another stack"}
    return {"ok": True, "action": "start_required", "previous_state": state.get("state", "not_started")}


def ensure_observation(config_path=None, *, project=PROJECT, dry_run=False,
                       status_probe=runtime_status, bridge_probe=bridge_ready,
                       alive=owned_alive, run=subprocess.run):
    project = Path(project).resolve()
    path = Path(config_path) if config_path is not None else project / CONFIG_RELATIVE
    if not path.is_absolute():
        path = project / path
    config = validate_config(read_json(path), path, project)
    state_path = project / ".runtime" / "stack" / config["name"] / "state.json"

    def inspect():
        state = read_json(state_path)
        if not isinstance(state, dict):
            raise ValueError("observation state must be an object; preserving existing processes")
        return assess(state, status_probe(), bridge_probe(), alive)

    report = inspect()
    if dry_run or report["action"] != "start_required":
        return {**report, "dry_run": dry_run, "profile": config["name"]}
    # Serialize this helper's startup attempts separately from bringup's own
    # state lock, which the child needs to acquire for its normal start action.
    state_path.parent.mkdir(parents=True, exist_ok=True)
    with (state_path.parent / "ensure.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        report = inspect()
        if report["action"] != "start_required":
            return {**report, "dry_run": False, "profile": config["name"]}
        command = [sys.executable, "-m", "deployment.bringup", "--config", str(path), "start"]
        completed = run(command, cwd=project, check=False, capture_output=True, text=True, timeout=240)
        if completed.returncode:
            return {"ok": False, "action": "start_failed", "profile": config["name"],
                    "error": "bringup start failed; inspect its state and per-component logs"}
        status = status_probe()
        if not safe_runtime(status) or not bridge_probe():
            return {"ok": False, "action": "readiness_failed", "profile": config["name"],
                    "error": "started observation stack is not ready; existing processes were preserved"}
        return {"ok": True, "action": "started", "profile": config["name"],
                "session_id": status["session_id"], "state": status.get("state"), "dry_run": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT / CONFIG_RELATIVE)
    parser.add_argument("--dry-run", action="store_true", help="report the decision without creating files or starting processes")
    args = parser.parse_args(argv)
    try:
        report = ensure_observation(args.config, dry_run=args.dry_run)
    except (OSError, ValueError, KeyError, TypeError, subprocess.TimeoutExpired) as exc:
        report = {"ok": False, "action": "preserve_error", "error": str(exc)}
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
