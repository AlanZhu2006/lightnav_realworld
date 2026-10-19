"""Offline audit of native Start/Stop controls; no socket, SDK, or subprocess.

Transport, process creation and HTTP handlers use fakes. The concurrent Stop
checks exercise the real two-worker Foxglove adapter with bounded barriers.
"""
from __future__ import annotations

import argparse
from email.message import Message
import io
import json
from pathlib import Path
import signal
import socket
import sys
import tempfile
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch

from .foxglove_control import OperatorControl, parse_command
from .http_api import Handler
from .native_control import NativeExecutorManager
from deployment.go2 import start_native_imagegoal as launcher


def command(request_id, action, *, epoch=7, session="session-1"):
    payload = {} if action == "native_stop" else {
        "selection_id":"selection-1", "expected_epoch":epoch, "operator_present":True,
        "backend":"nomad", "execution_mode":"supervised_native", "timeout_s":60}
    return json.dumps({"request_id":request_id,"session_id":session,"action":action,"payload":payload})


def require(condition, message):
    if not condition:
        raise AssertionError(message)


class Harness:
    def __init__(self, request):
        self.condition = threading.Condition()
        self.results = []
        self.control = OperatorControl("http://offline.invalid:8093", self.publish, request_json=request)

    def publish(self, result):
        with self.condition:
            self.results.append(result)
            self.condition.notify_all()

    def result(self, request_id):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: any(item["request_id"] == request_id for item in self.results), timeout=2)
            require(ready, f"no bounded result for {request_id}")
            return next(item for item in self.results if item["request_id"] == request_id)

    def close(self):
        self.control.close()
        require(not any(worker.is_alive() for worker in self.control.workers), "control worker leaked")


def audit_adapter(checks):
    for payload in ({}, {"expected_epoch":-1}, {"expected_epoch":True}):
        raw = json.loads(command("bad-start", "native_start"))
        raw["payload"] = payload
        try:
            parse_command(json.dumps(raw))
        except ValueError:
            pass
        else:
            raise AssertionError("native Start accepted a missing/invalid epoch")
    require(parse_command(command("stop-without-epoch", "native_stop")).action == "native_stop",
            "native Stop must remain usable without an epoch")
    checks["native_start_requires_epoch_stop_does_not"] = True

    calls = []
    def request(url, *, timeout_s, payload=None):
        calls.append((url, payload))
        return ({"session_id":"session-1","epoch":7} if payload is None
                else {"task":{"task_id":"fresh-native-task"}}), {}
    harness = Harness(request)
    try:
        raw = command("fresh-start", "native_start")
        require(harness.control.submit(raw), "native Start was not queued")
        require(harness.result("fresh-start")["ok"], "native Start failed")
        require(len(calls) == 2 and calls[1][0].endswith("/native/start"), "wrong native Start endpoint")
        body = calls[1][1]
        require(body["expected_epoch"] == 7 and body["expected_session_id"] == "session-1"
                and body["selection_id"] == "selection-1" and body["operator_present"] is True,
                "native Start lost goal, epoch or supervision identity")
        checks["foxglove_native_start_preserves_identity_and_route"] = True
        require(not harness.control.submit(raw) and len(calls) == 2, "duplicate native Start reexecuted")
        checks["duplicate_native_start_does_not_launch_twice"] = True
    finally:
        harness.close()

    for name, live in (("stale_epoch", {"session_id":"session-1","epoch":8}),
                       ("stale_session", {"session_id":"session-2","epoch":7})):
        calls = []
        def stale(url, *, timeout_s, payload=None):
            calls.append((url, payload))
            return live, {}
        harness = Harness(stale)
        try:
            harness.control.submit(command(name, "native_start"))
            result = harness.result(name)
            require(not result["ok"] and len(calls) == 1 and calls[0][1] is None,
                    "stale native Start was rearmed or posted")
            checks["native_start_rejects_" + name + "_without_post"] = True
        finally:
            harness.close()

    calls = []
    def stop_only(url, *, timeout_s, payload=None):
        require(payload is not None, "native Stop waited for a status GET")
        calls.append((url, timeout_s, payload))
        return {"state":"CANCELLED"}, {}
    harness = Harness(stop_only)
    try:
        harness.control.submit(command("direct-stop", "native_stop"))
        require(harness.result("direct-stop")["ok"], "native Stop failed")
        require(len(calls) == 1 and calls[0][0].endswith("/native/stop")
                and calls[0][1] <= .5 and calls[0][2]["expected_session_id"] == "session-1",
                "native Stop route, bounded request or session changed")
        checks["native_stop_posts_directly_without_status_get"] = True
    finally:
        harness.close()

    entered, release = threading.Event(), threading.Event()
    posts = []
    def blocked_start(url, *, timeout_s, payload=None):
        if payload is None:
            entered.set()
            require(release.wait(2), "test did not release blocked status GET")
            return {"session_id":"session-1","epoch":8}, {}
        posts.append(url)
        require(url.endswith("/native/stop"), "stopped native Start reached the API")
        return {"state":"CANCELLED"}, {}
    harness = Harness(blocked_start)
    try:
        harness.control.submit(command("blocked-start", "native_start"))
        require(entered.wait(2), "normal command worker did not enter fake GET")
        harness.control.submit(command("queued-start", "native_start"))
        harness.control.submit(command("priority-stop", "native_stop"))
        require(harness.result("priority-stop")["ok"] and not release.is_set(),
                "native Stop waited for the normal command worker")
        require(not harness.result("queued-start")["ok"], "Stop did not discard queued Start")
        release.set()
        require(not harness.result("blocked-start")["ok"] and posts == ["http://offline.invalid:8093/native/stop"],
                "in-flight Start survived the Stop generation barrier")
        checks["native_stop_preempts_blocked_and_queued_starts"] = True
    finally:
        release.set()
        harness.close()


def fake_runtime(output):
    runtime = SimpleNamespace(output=output, lock=threading.RLock(), task={"task_id":"task-1"},
                              state="TRACKING", finished=[])
    def finish(state, reason):
        runtime.state = state
        runtime.finished.append((state, reason))
    runtime._finish = finish
    return runtime


def audit_manager(checks, output):
    runtime = fake_runtime(output / "failed-process")
    manager = NativeExecutorManager(runtime)
    with patch("deployment.imagegoal.native_control.subprocess.Popen", side_effect=OSError("fake launch denied")) as popen:
        try:
            manager.start("task-1", "http://offline.invalid:8093", 60)
        except OSError:
            pass
        else:
            raise AssertionError("manager hid process creation failure")
        require(manager.process is None and manager.log_handle is None and not manager.status()["running"],
                "failed process creation left a process, open log or running claim")
        args = popen.call_args.args[0]
        require("native-executor" in args and "--execute-supervised" in args and "--operator-present" in args
                and args[args.index("--task-id") + 1] == "task-1", "manager lost bounded task identity")
    checks["manager_launch_failure_has_no_running_child_or_open_log"] = True

    class FakeProcess:
        pid = 12345
        def __init__(self):
            self.exited = threading.Event()
            self.signals = []
        def poll(self):
            return 0 if self.exited.is_set() else None
        def wait(self):
            require(self.exited.wait(2), "fake owned process did not receive Stop")
            return 0
        def send_signal(self, signum):
            self.signals.append(signum)
            self.exited.set()

    runtime = fake_runtime(output / "owned-process")
    manager, process = NativeExecutorManager(runtime), FakeProcess()
    def spawn(command, **kwargs):
        execution = Path(command[command.index("--output") + 1])
        require(not execution.exists(), "child output directory already exists before process creation")
        require(Path(kwargs["stdout"].name).parent == execution.parent,
                "executor log should be outside the child's new output directory")
        return process
    with patch("deployment.imagegoal.native_control.subprocess.Popen", side_effect=spawn) as popen:
        started = manager.start("task-1", "http://offline.invalid:8093", 60)
        require(started["running"] and started["task_id"] == "task-1", "manager did not bind fake child")
        args = popen.call_args.args[0]
        execution = Path(args[args.index("--output") + 1])
        require(Path(started["report_path"]) == execution / "report.json"
                and Path(started["live_path"]) == execution / "live.json",
                "manager live/report paths do not match the child's output directory")
        checks["manager_child_output_is_new_and_receipts_match"] = True
        manager.stop("fake operator Stop")
        require(process.signals == [signal.SIGTERM], "Stop did not signal the exact owned child")
        # Obtain the owned watcher's completion without a polling sleep.
        handle = manager.log_handle
        for thread in threading.enumerate():
            if thread.name == "native-executor-owner":
                thread.join(timeout=2)
        require(handle.closed and not manager.status()["running"] and runtime.finished,
                "owned executor completion did not close log and cancel its task")
    checks["manager_stop_signals_owned_child_and_finishes_matching_task"] = True


def audit_cleared_goal(checks, output):
    run = output / "goal-fixture"
    runtime = run / "runtime"
    runtime.mkdir(parents=True)
    path = runtime / "status.json"
    status = {"timestamp_ns":1,"goal_selection":{"ready":False,"mode":"live"},
              "task":{"goal_path":"old-task-goal.jpg"}}
    path.write_text(json.dumps(status))
    (runtime / "old-task-goal.jpg").write_bytes(b"old target must not be read")
    with patch.object(launcher, "state_for", return_value={"state":"running","run":str(run)}):
        try:
            launcher.current_goal()
        except launcher.NoFrozenGoal:
            pass
        else:
            raise AssertionError("cleared selection revived the previous task goal")
    checks["cleared_selection_never_restores_previous_task_goal"] = True


def route_request(runtime, path, payload=None):
    # Bypass BaseHTTPRequestHandler.__init__/setup: no server/socket is opened.
    handler = Handler.__new__(Handler)
    handler.server = SimpleNamespace(runtime=runtime, server_address=("127.0.0.1", 19093))
    handler.path = path
    responses = []
    handler._json = lambda code, value: responses.append((code, value))
    if payload is None:
        handler.do_GET()
    else:
        blob = json.dumps(payload).encode()
        handler.headers = Message()
        handler.headers["Content-Type"] = "application/json"
        handler.headers["Content-Length"] = str(len(blob))
        handler.rfile = io.BytesIO(blob)
        handler.do_POST()
    require(len(responses) == 1, "fake HTTP handler returned no unique response")
    return responses[0]


def audit_http_routes(checks):
    start, stop = Mock(return_value={"state":"WAITING_INPUT"}), Mock(return_value={"state":"CANCELLED"})
    runtime = SimpleNamespace(start_native=start, stop_native=stop,
                              status=lambda: {"native_executor":{"running":False},"native_execution":{"authorized":False}})
    payload = {"expected_session_id":"session-1","expected_epoch":7,"operator_present":True,"selection_id":"selection-1"}
    require(route_request(runtime, "/api/native/start", payload)[0] == 200, "native Start HTTP route failed")
    start.assert_called_once_with(payload, runtime_url="http://127.0.0.1:19093")
    require(route_request(runtime, "/api/native/stop", {"expected_session_id":"session-1"})[0] == 200,
            "native Stop HTTP route failed")
    stop.assert_called_once_with({"expected_session_id":"session-1"})
    checks["native_http_routes_dispatch_with_actual_runtime_port"] = True
    code, value = route_request(runtime, "/native/status")
    require(code == 200 and value["native_execution"]["authorized"] is False
            and value["native_executor"]["running"] is False, "native status HTTP route failed")
    checks["native_http_status_exposes_contract_and_executor"] = True
    runtime.start_native = Mock(side_effect=ValueError("expected_epoch mismatch"))
    code, value = route_request(runtime, "/native/start", payload)
    require(code == 400 and "expected_epoch" in value["error"], "stale Start was not an HTTP client error")
    checks["native_http_start_propagates_stale_epoch_error"] = True


def audit():
    checks = {}
    before_sdk = {name for name in sys.modules if name.startswith("unitree_sdk2py")}
    with tempfile.TemporaryDirectory(prefix="native-control-audit-") as temporary:
        output = Path(temporary)
        # An accidental socket/process call must fail this offline audit.
        with patch.object(socket, "socket", side_effect=AssertionError("audit must not open a socket")), \
             patch("subprocess.Popen", side_effect=AssertionError("audit must not launch a subprocess")):
            audit_adapter(checks)
            audit_manager(checks, output)
            audit_cleared_goal(checks, output)
            audit_http_routes(checks)
    require({name for name in sys.modules if name.startswith("unitree_sdk2py")} == before_sdk,
            "audit imported a robot SDK")
    checks["no_socket_subprocess_or_robot_sdk"] = True
    return {"schema_version":"imagegoal.native_controls_audit.v1", "ok":all(checks.values()),
            "not_for_motion":True, "sdk_initialized":False, "all_transports_fake":True, "checks":checks}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="optional JSON audit receipt")
    args = parser.parse_args(argv)
    report = audit()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
