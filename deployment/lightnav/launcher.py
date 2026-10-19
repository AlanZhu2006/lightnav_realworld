"""Prepare the existing workstation LightNav service without changing tasks.

Healthy services are reused, including services on Maxwell. Cold starts use the
existing bringup manager only on an ordinary workstation; this helper never
resets a model session, submits inference, stops processes, or selects a GPU.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
from urllib.request import urlopen

from deployment.bringup.__main__ import owned_alive
from .policy import MODEL_REVISION, MODEL_SHA256, OFFICIAL_RELEASE_COMMIT, SERVER_IMPLEMENTATION, OFFICIAL_CORE_DIGEST


ROOT = Path(__file__).resolve().parents[2]
PROFILE = "workstation_lightnav"
CONFIG_RELATIVE = Path("deployment/bringup/workstation_lightnav.json")
HEALTH_EXPECT = {
    "ok": True,
    "model_loaded": True,
    "contract.model": "LightOriginsHQ/LightNav-0",
    "contract.model_revision": MODEL_REVISION,
    "contract.model_sha256": MODEL_SHA256,
    "contract.device": "cuda",
    "contract.implementation": SERVER_IMPLEMENTATION,
    "contract.official_core_sha256": OFFICIAL_CORE_DIGEST,
    "contract.imagegoal_adaptation_support": True,
    "contract.official_release_commit": OFFICIAL_RELEASE_COMMIT,
    "contract.goal_image_consumed": False,
    "contract.retirement_protocol.supported": True,
    "contract.retirement_protocol.schema_version": "lightnav.owned_retirement.v1",
}


def health_matches(payload: dict) -> bool:
    """Require the pinned official core, deployment profiles, and CUDA-loaded service."""
    if not isinstance(payload, dict):
        return False
    for key, expected in HEALTH_EXPECT.items():
        actual = payload
        for part in key.split("."):
            actual = actual.get(part) if isinstance(actual, dict) else None
        if type(actual) is not type(expected) or actual != expected:
            return False
    return True


def probe_health(port: int):
    """GET a bounded health document; do not touch inference or session state."""
    try:
        with urlopen(f"http://127.0.0.1:{port}/healthz", timeout=3) as response:
            body = response.read(1024 * 1024 + 1)
        if len(body) > 1024 * 1024:
            return None
        payload = json.loads(body)
        return payload if isinstance(payload, dict) else None
    except (OSError, ValueError):
        return None


def port_occupied(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1):
            return True
    except OSError:
        return False


def _load_json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"无法读取 LightNav 配置或状态，保留现有进程：{path}") from exc


def _validate_profile(project: Path) -> dict:
    config = _load_json(project / CONFIG_RELATIVE)
    components = config.get("components") if isinstance(config, dict) else None
    if not isinstance(config, dict) or config.get("name") != PROFILE or not isinstance(components, list) or len(components) != 1:
        raise RuntimeError("LightNav 模型 profile 不匹配；保留现有进程。")
    component = components[0]
    if not isinstance(component, dict):
        raise RuntimeError("LightNav 模型 profile 不匹配；保留现有进程。")
    expected = component.get("health_expect")
    if (component.get("name") != "lightnav"
            or component.get("command") != ["bash", "deployment/lightnav/run_server.sh"]
            or component.get("health_url") != "http://127.0.0.1:18884/healthz"
            or component.get("adopt_existing") is not True
            or component.get("env", {}) != {}
            or not isinstance(expected, dict)
            or any(expected.get(key) != value for key, value in HEALTH_EXPECT.items()
                   if key != "contract.device")
            or expected.get("contract.device", "cuda") != "cuda"):
        raise RuntimeError("LightNav profile 的启动命令或官方模型合同不匹配；未启动服务。")
    timeout = component.get("startup_timeout_s", 30)
    if type(timeout) not in (int, float) or not 0 < timeout <= 600:
        raise RuntimeError("LightNav 模型启动等待时间无效；未启动服务。")
    return component


def _validate_assets(project: Path, environ: dict) -> None:
    if environ.get("LIGHTNAV_PORT", "18884") != "18884" or environ.get("LIGHTNAV_HOST", "127.0.0.1") != "127.0.0.1":
        raise RuntimeError("LightNav 自动冷启动仅支持本机 127.0.0.1:18884；请检查 LIGHTNAV_HOST/PORT。")
    python = Path(environ.get("LIGHTNAV_PYTHON", str(project / "deployment/lightnav/.venv/bin/python")))
    weights = Path(environ.get("LIGHTNAV_MODEL_PATH", str(project / "deployment/lightnav/.runtime/weights")))
    if not python.is_absolute():
        python = project / python
    if not weights.is_absolute():
        weights = project / weights
    weights = weights.resolve()
    if not python.is_file() or not os.access(python, os.X_OK):
        raise RuntimeError(f"本机 LightNav 独立环境未准备好：{python}；未安装或启动新环境。")
    manifest = _load_json(weights / "SOURCE_MANIFEST.json")
    entries = manifest.get("files") if isinstance(manifest, dict) else None
    if not isinstance(manifest, dict) or manifest.get("revision") != MODEL_REVISION or not isinstance(entries, list):
        raise RuntimeError("LightNav 权重 manifest 不属于固定的官方 revision；未启动模型。")
    weight_found = False
    required = {"config.json", "action_tokenizer/manifest.json"}
    for item in entries:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            raise RuntimeError("LightNav 权重 manifest 无效；未启动模型。")
        path = (weights / item["path"]).resolve()
        try:
            path.relative_to(weights)
        except ValueError as exc:
            raise RuntimeError("LightNav 权重 manifest 路径越界；未启动模型。") from exc
        if (type(item.get("size")) is not int or item["size"] <= 0
                or not path.is_file() or path.stat().st_size != item["size"]):
            raise RuntimeError(f"LightNav 权重缺失或不完整：{item['path']}；未启动模型。")
        required.discard(item["path"])
        if item["path"].endswith(".safetensors"):
            if item.get("sha256") != MODEL_SHA256:
                raise RuntimeError("LightNav 权重 manifest 的模型 SHA 不匹配；未启动模型。")
            weight_found = True
    if not weight_found or required:
        raise RuntimeError("LightNav 官方模型或 RVQ 资源不完整；未启动模型。")
    # The server rehashes every asset before loading. No CUDA preflight or second
    # model load is performed merely to prepare a connection.


def ensure_local_lightnav(project=ROOT, port=18884, *, health_probe=None,
                          port_probe=None, alive=None, run=None,
                          hostname=None, environ=None) -> dict:
    """Reuse LightNav or start its existing manager once; never replace it.

    Injectable read-only probes and subprocess runner allow lifecycle checks
    without loading a model. Custom ports are adopt-only. A cold start on
    Maxwell requires the separate coordinated GPU workflow and is rejected.
    """
    if type(port) is not int or not 1024 <= port <= 65535:
        raise RuntimeError("LightNav 端口必须在 1024..65535。")
    project = Path(project).resolve()
    health_probe = health_probe or probe_health
    port_probe = port_probe or port_occupied
    alive = alive or owned_alive
    run = run or subprocess.run
    environ = dict(os.environ if environ is None else environ)
    hostname = socket.gethostname() if hostname is None else hostname

    def inspect():
        health = health_probe(port)
        if health_matches(health):
            return health
        if port_probe(port):
            raise RuntimeError(f"本机 {port} 已占用，但 LightNav 官方模型合同未通过；保留现有服务。")
        return None

    def report(action, health):
        return {"ok": True, "action": action, "port": port,
                "health": health, "robot_commands_sent": False}

    health = inspect()
    if health is not None:
        return report("adopted", health)
    if port != 18884:
        raise RuntimeError("自定义 LightNav 端口仅复用健康服务；自动冷启动固定使用 18884。")
    maxwell_root = Path("/local_data")
    if ("maxwell" in str(hostname).lower() or "MAXWELL_DATA_ROOT" in environ
            or project == maxwell_root or maxwell_root in project.parents):
        raise RuntimeError("Maxwell 的 GPU 尚需协调；禁止在此入口自动冷启动 LightNav，可复用已在线的健康服务。")
    component = _validate_profile(project)
    state_dir = project / ".runtime/stack" / PROFILE
    state_dir.mkdir(parents=True, exist_ok=True)
    with (state_dir / "ensure.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        health = inspect()
        if health is not None:
            return report("adopted", health)
        state_path = state_dir / "state.json"
        state = _load_json(state_path) if state_path.exists() else {}
        if not isinstance(state, dict) or not isinstance(state.get("components", []), list):
            raise RuntimeError("LightNav manager 状态无效；保留现有进程。")
        if any(isinstance(item, dict) and item.get("ownership") == "managed" and alive(item)
               for item in state.get("components", [])):
            raise RuntimeError("LightNav manager 仍有存活进程，但模型尚未就绪；保留进程，请检查日志。")
        _validate_assets(project, environ)
        command = [sys.executable, "-m", "deployment.bringup", "--config", str(project / CONFIG_RELATIVE), "start"]
        try:
            completed = run(command, cwd=project, env=environ, capture_output=True,
                            text=True, check=False, timeout=component.get("startup_timeout_s", 30) + 45)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError("LightNav 模型启动失败或仍在加载；保留现有进程，请检查 bringup status 和日志。") from exc
        if completed.returncode:
            detail = (getattr(completed, "stderr", "") or getattr(completed, "stdout", "") or "")[-1200:].strip()
            raise RuntimeError("LightNav 模型启动未成功；请检查 bringup status 和日志。" + ("\n" + detail if detail else ""))
        health = health_probe(port)
        if not health_matches(health):
            raise RuntimeError("LightNav 启动返回后官方模型/CUDA 健康检查未通过；保留服务，请检查日志。")
        return report("started", health)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18884)
    args = parser.parse_args(argv)
    try:
        report = ensure_local_lightnav(port=args.port)
    except RuntimeError as exc:
        report = {"ok": False, "error": str(exc)}
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
