"""Prepare or explicitly start a supervised native-camera ImageGoal task.

The default invocation only reads state and probes socket availability. Motion
requires both operator flags. Existing tasks are never resumed after a restart.
"""
from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import fcntl
import hashlib
import io
import json
from pathlib import Path
import signal
import socket
import subprocess
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


PROJECT = Path(__file__).resolve().parents[2]
PROFILE = PROJECT / "deployment/bringup/jetson_native_imagegoal.json"
LIGHTNAV_PROFILE = PROJECT / "deployment/bringup/jetson_native_lightnav.json"
RUNTIME_URL = "http://127.0.0.1:8093"
SDK_PYTHON = PROJECT / ".venv-go2-runtime/bin/python"
# Only these known profiles are stopped, through their normal ownership-aware
# manager. A separate shared NoMaD service and external services are preserved.
OBSERVATION_PROFILES = (
    PROJECT / "deployment/bringup/jetson_unitree_camera.json",
    PROFILE,
    PROJECT / "deployment/bringup/jetson_lightnav.json",
    LIGHTNAV_PROFILE,
    PROJECT / "deployment/bringup/jetson.json",
)
TERMINAL = {"IDLE", "CANCELLED", "SUCCEEDED", "FAILED", "TIMED_OUT"}


class NoFrozenGoal(ValueError):
    pass


def read_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


def network_probe() -> dict:
    """Run before any state/service/task mutation, without importing an SDK."""
    try:
        for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
            with socket.socket(socket.AF_INET, kind) as channel:
                channel.bind(("127.0.0.1", 0))
        return {"available": True, "sdk_imported": False}
    except OSError as exc:
        return {"available": False, "error": f"{type(exc).__name__}: {exc}",
                "blocked": "execution_environment_network_denied", "sdk_imported": False}


def state_for(config: Path) -> dict:
    name = read_json(config)["name"]
    return read_json(PROJECT / ".runtime/stack" / name / "state.json")


def current_goal(explicit: Path | None = None) -> tuple[bytes, dict]:
    """Keep the full frozen JPEG; the UI selection takes priority over a task."""
    candidates = []
    if explicit is not None:
        path = explicit.expanduser().resolve(strict=True)
        candidates.append((path, {"source": "explicit_goal", "path": str(path)}))
    else:
        states = []
        for config in OBSERVATION_PROFILES[:-1]:
            state = state_for(config)
            if state.get("state") not in {"running", "starting"} or not state.get("run"):
                continue
            status_path = Path(state["run"]) / "runtime/status.json"
            status = read_json(status_path)
            if status:
                states.append((status.get("timestamp_ns", 0), status_path, status))
        for _, status_path, status in sorted(states, key=lambda row: row[0], reverse=True):
            output = status_path.parent
            selection, task = status.get("goal_selection") or {}, status.get("task") or {}
            # selection_path is the current goal-selection schema. goal_path
            # also supports older receipts and remains the preferred key.
            selected_path = selection.get("goal_path") or selection.get("selection_path")
            if selection.get("ready") is True and selected_path:
                candidates.append((output / selected_path,
                                   {"source": "frozen_goal_selection", "status": str(status_path),
                                    "expected_sha256": selection.get("sha256")}))
            # A present selection with ready=False means the user cleared the
            # target. Only legacy runtimes without this field may reuse a
            # task's image; clearing a goal must never revive it on restart.
            elif "goal_selection" not in status and task.get("goal_path"):
                candidates.append((output / task["goal_path"],
                                   {"source": "previous_task_goal_only", "status": str(status_path),
                                    "expected_sha256": task.get("goal_sha256")}))
            # Never fall back to an older profile's target when the active
            # newest profile has no frozen target.
            break
    if not candidates:
        raise NoFrozenGoal("没有冻结的完整目标图；请先在网页选择目标，或传入 --goal /path/goal.jpg")
    path, provenance = candidates[0]
    blob = path.read_bytes()
    if not blob or len(blob) > 4 * 1024 * 1024:
        raise ValueError("完整目标 JPEG 必须小于等于 4 MiB")
    from PIL import Image
    with Image.open(io.BytesIO(blob)) as image:
        if image.format != "JPEG" or image.width * image.height > 4096 * 2160:
            raise ValueError("目标须为完整 JPEG，像素数不超过 4096×2160")
        image.load()
        dimensions = [image.width, image.height]
    digest = hashlib.sha256(blob).hexdigest()
    if provenance.get("expected_sha256") not in {None, digest}:
        raise ValueError("保存目标的 SHA256 与状态记录不一致；停止准备")
    return blob, {**provenance, "path": str(path.resolve()), "sha256": digest,
                  "size_bytes": len(blob), "dimensions_wh": dimensions,
                  "full_jpeg_preserved": True, "old_task_restored": False}


def http_json(path: str, value: dict | None = None) -> dict:
    request = Request(RUNTIME_URL + path,
                      data=None if value is None else json.dumps(value).encode(),
                      headers={} if value is None else {"Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=5) as response:
            return json.load(response)
    except HTTPError as exc:
        raise RuntimeError(f"runtime HTTP {exc.code}: {exc.read(2000).decode(errors='replace')}") from exc


def managed_runtime_session() -> set[str]:
    # This import does not initialize ROS, DDS or a robot SDK.
    from deployment.bringup.__main__ import owned_alive
    sessions = set()
    for config in OBSERVATION_PROFILES:
        for item in state_for(config).get("components", []):
            if item.get("name") == "runtime" and owned_alive(item):
                identifier = item.get("runtime_session_id")
                if isinstance(identifier, str) and identifier:
                    sessions.add(identifier)
    return sessions


def preserve_external_runtime() -> None:
    try:
        live = http_json("/status")
    except OSError:
        return  # No runtime yet; bringup independently checks occupied ports.
    if live.get("session_id") not in managed_runtime_session():
        raise RuntimeError("8093 上的 runtime 不属于已知观测 profile；保留外部服务，不停止或替换")


def bringup(config: Path, action: str) -> None:
    subprocess.run(["python3", "-m", "deployment.bringup", "--config", str(config), action],
                   cwd=PROJECT, check=True)


def run_executor(task_id: str, duration_s: float, output: Path) -> int:
    command = [str(SDK_PYTHON), "-m", "deployment.go2.native_imagegoal",
               "--execute-supervised", "--operator-present", "--task-id", task_id,
               "--runtime-url", RUNTIME_URL, "--duration-s", str(duration_s),
               "--output", str(output)]
    process = subprocess.Popen(command, cwd=PROJECT)
    old_handlers = {}
    try:
        def stop_child(_signum, _frame):
            if process.poll() is None:
                process.terminate()  # Executor's SIGTERM handler zeros and releases.
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, stop_child)
        return process.wait()
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        for signum, old in old_handlers.items():
            signal.signal(signum, old)


def cancel_same_task(session_id: str, task_id: str) -> dict:
    live = http_json("/status")
    if live.get("session_id") != session_id or (live.get("task") or {}).get("task_id") != task_id:
        return {"cancelled": False, "reason": "session_or_task_changed"}
    if live.get("state") in TERMINAL:
        return {"cancelled": False, "reason": "task_already_terminal"}
    result = http_json("/task/cancel", {"expected_session_id": session_id,
                                       "expected_task_id": task_id,
                                       "expected_epoch": live["epoch"],
                                       "reason": "native_imagegoal_executor_finished"})
    return {"cancelled": result.get("state") == "CANCELLED", "state": result.get("state")}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--execute-supervised", action="store_true")
    operation.add_argument("--prepare-only", action="store_true",
                           help="仅切换到监督观测 profile，随后从网页或 Foxglove 开始真机任务")
    parser.add_argument("--operator-present", action="store_true")
    parser.add_argument("--goal", type=Path, help="完整 JPEG；省略时保存当前网页已冻结的目标")
    parser.add_argument("--backend", choices=("nomad", "lightnav"), default="nomad")
    parser.add_argument("--stack-profile", choices=("combined", "lightnav"), default="combined",
                        help="lightnav omits the optional NoMaD model service")
    parser.add_argument("--instruction", default="", help="可选 LightNav 指令；ImageGoal 可留空")
    parser.add_argument("--duration-s", type=float, default=60.)
    args = parser.parse_args(argv)
    profile = LIGHTNAV_PROFILE if args.stack_profile == "lightnav" else PROFILE
    if args.stack_profile == "lightnav" and args.execute_supervised and args.backend != "lightnav":
        parser.error("LightNav-only stack requires --backend lightnav for direct execution")
    if (args.execute_supervised or args.prepare_only) and not args.operator_present:
        parser.error("--execute-supervised/--prepare-only requires --operator-present")
    if not 5 <= args.duration_s <= 120:
        parser.error("--duration-s must be between 5 and 120 seconds")
    if len(args.instruction) > 4096:
        parser.error("--instruction must be at most 4096 characters")
    report = {"schema_version": "imagegoal.native_imagegoal_launch.v1",
              "execute_requested": args.execute_supervised, "prepare_only": args.prepare_only,
              "operator_present": args.operator_present,
              "backend": args.backend,
              "mode": "read_only_preflight", "network": network_probe(),
              "sdk_initialized_by_launcher": False, "old_task_restored": False,
              "navigation_success_validated": False, "blocked": [],
              "notice": "监督真机测试 · 宇树原生避障 · 手动停止",
              "limits": {"max_v_mps": .3, "max_w_rad_s": .55,
                         "min_effective_v_mps": .1, "min_effective_w_rad_s": .2},
              "geometry": "官方前相机默认朝向；未宣称获得官方外参标定矩阵",
              "time": "主机接收时间近似；未宣称曝光时间与 odom 已标定"}
    if not report["network"]["available"]:
        report["mode"] = "blocked_before_service_or_task_change"
        report["blocked"].append("execution_environment_network_denied")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2
    run = None
    session_id = task_id = None
    lock = None
    try:
        try:
            blob, goal = current_goal(args.goal)
        except NoFrozenGoal:
            if not args.prepare_only or args.goal is not None:
                raise
            blob, goal = None, None
        report["goal"] = goal
        if not SDK_PYTHON.exists() or not profile.exists():
            raise RuntimeError("缺少 Go2 Python 环境或监督测试 profile")
        if not args.execute_supervised and not args.prepare_only:
            report["next_command"] = "bash deployment/go2/run_go2.sh native-imagegoal --prepare-only --operator-present"
            return 0
        preserve_external_runtime()
        lock_path = PROJECT / ".runtime/go2/native_imagegoal_launch.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock = lock_path.open("a")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        run = PROJECT / "runs" / ("native_imagegoal_session_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
        run.mkdir(parents=True, exist_ok=False)
        # Freeze bytes before cancelling or restarting any prior runtime.
        if blob is not None:
            (run / "goal.jpg").write_bytes(blob)
            (run / "goal.sha256").write_text(goal["sha256"] + "  goal.jpg\n")
            (run / "goal_provenance.json").write_text(json.dumps(goal, ensure_ascii=False, indent=2) + "\n")
        report["run"] = str(run)
        for config in OBSERVATION_PROFILES:
            if state_for(config).get("state") in {"running", "starting", "failed"}:
                bringup(config, "stop")
        bringup(profile, "start")
        status = http_json("/status")
        capability = status.get("execution_capability") or {}
        if (status.get("profile") != "external"
                or capability.get("supervised_native_configured") is not True
                or capability.get("operator_supervised") is not True):
            raise RuntimeError("新 runtime 不满足监督原生避障执行配置；未提交任务")
        if args.prepare_only:
            if blob is not None:
                imported = http_json("/goal/import", {"expected_epoch": status["epoch"],
                                                     "expected_session_id": status["session_id"],
                                                     "goal": {"rgb_jpeg_base64": base64.b64encode(blob).decode("ascii")}})
                report["preserved_selection_id"] = imported["goal_selection"]["selection_id"]
            report["mode"] = "prepared_observation_only_no_task_or_executor"
            report["session_id"] = status["session_id"]
            print("监督观测栈已就绪。请在网页或 Foxglove 点击开始真机；尚未创建任务或启动执行器。", flush=True)
            return 0
        created = http_json("/tasks", {"expected_epoch": status["epoch"],
                                     "expected_session_id": status["session_id"],
                                     "goal": {"rgb_jpeg_base64": base64.b64encode(blob).decode("ascii")},
                                     "backend": args.backend, "execution_mode": "supervised_native",
                                     "timeout_s": args.duration_s, "instruction": args.instruction})
        session_id, task_id = created["session_id"], created["task"]["task_id"]
        report.update(mode="supervised_native_task", session_id=session_id, task_id=task_id)
        # The executor binds this fresh task and waits at zero until its native
        # command lease is authorized; it never resumes an old shadow task.
        print(f"监督 ImageGoal 已创建：{task_id}；最长 {args.duration_s:g} 秒，手动停止。", flush=True)
        result = run_executor(task_id, args.duration_s, run / "executor")
        report["executor_returncode"] = result
        report["mode"] = "executor_finished_observation_services_preserved"
        return result if result >= 0 else 2
    except (Exception, KeyboardInterrupt) as exc:
        report["mode"] = "blocked_or_stopped"
        report["blocked"].append(f"{type(exc).__name__}: {exc}")
        return 2
    finally:
        if session_id and task_id:
            try:
                report["task_cleanup"] = cancel_same_task(session_id, task_id)
            except Exception as exc:
                report["task_cleanup"] = {"cancelled": False, "error": str(exc)}
        if lock is not None:
            lock.close()
        report["finished_timestamp_ns"] = time.time_ns()
        if run is not None:
            (run / "launch_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
