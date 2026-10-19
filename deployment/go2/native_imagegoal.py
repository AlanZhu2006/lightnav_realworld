"""Explicit supervised ImageGoal execution through Unitree native avoidance.

Only ``native_execution`` is consumed; the older execution envelope cannot
authorize this path. Factory front-camera orientation and host receive times
are explicit assumptions, not verified extrinsics or exposure timestamps.
SDK imports occur only in the CLI after its read-only checks and operator flags.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import fcntl
import json
import math
from pathlib import Path
import signal
import socket
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen

from .preflight import check_feedback, load_config, preflight


CLAIM = "factory_front_assumed_receive_time_unverified"
TERMINAL = {"CANCELLED", "FAILED", "TIMED_OUT", "SUCCEEDED"}
NATIVE_SOURCES = {
    "rgb": ("go2_front_camera_optical_frame", "go2_video_rpc_receive_host_system_time_not_sensor_source_time"),
    "odom": ("go2_odom", "go2_dds_receive_host_system_time_not_sensor_source_time"),
}


def _age_valid(stamp, now, maximum):
    return type(stamp) is int and 0 <= now-stamp <= maximum


def _sensor_reasons(sensors, now):
    reasons = []
    sensors = sensors if isinstance(sensors, dict) else {}
    for name, age in (("rgb", 2_000_000_000), ("odom", 300_000_000)):
        item = sensors.get(name)
        if not isinstance(item, dict):
            reasons.append(f"{name}_missing")
            continue
        if not _age_valid(item.get("capture_timestamp_ns"), now, age):
            reasons.append(f"{name}_stale_or_future")
        if item.get("synthetic") is not False or item.get("commissioning", False) is not False:
            reasons.append(f"{name}_not_real")
        if item.get("clock_domain") != "unix_epoch_ns":
            reasons.append(f"{name}_clock_invalid")
        for key in ("timestamp_source", "frame_name"):
            if not isinstance(item.get(key), str) or not item[key].strip():
                reasons.append(f"{name}_{key}_missing")
        frame, provenance = NATIVE_SOURCES[name]
        if item.get("frame_name") != frame or item.get("timestamp_source") != provenance:
            reasons.append(f"{name}_native_source_mismatch")
        if name == "odom" and item.get("body_frame_name") != "base_link":
            reasons.append("odom_body_frame_mismatch")
    return reasons


def _contract_reasons(status, session_id, task_id, now):
    if not isinstance(status, dict):
        return ["runtime_status_not_object"]
    reasons = []
    if not isinstance(session_id, str) or not session_id or not isinstance(task_id, str) or not task_id:
        reasons.append("explicit_binding_missing")
    if status.get("schema_version") != "imagegoal.runtime.v1":
        reasons.append("runtime_schema_invalid")
    if status.get("session_id") != session_id:
        reasons.append("runtime_session_changed")
    task = status.get("task")
    if not isinstance(task, dict) or task.get("task_id") != task_id:
        reasons.append("runtime_task_changed")
    elif task.get("execution_mode") != "supervised_native" or task.get("backend") not in {"nomad", "lightnav"}:
        reasons.append("task_not_supervised_native_supported_backend")
    if status.get("synthetic_feedback") is not False:
        reasons.append("runtime_feedback_not_real")
    if not _age_valid(status.get("timestamp_ns"), now, 250_000_000):
        reasons.append("runtime_status_stale_or_future")
    envelope = status.get("native_execution")
    if not isinstance(envelope, dict):
        return reasons + ["native_execution_missing"]
    if envelope.get("schema_version") != "imagegoal.native_execution.v1":
        reasons.append("native_schema_invalid")
    if envelope.get("session_id") != session_id or envelope.get("task_id") != task_id:
        reasons.append("native_identity_changed")
    if envelope.get("supervised") is not True or envelope.get("calibration_claim") != CLAIM:
        reasons.append("supervised_assumption_contract_invalid")
    if envelope.get("safety_backend") != "unitree_obstacles_avoid_only":
        reasons.append("native_safety_backend_invalid")
    if not isinstance(envelope.get("goal_id"), str) or not envelope["goal_id"]:
        reasons.append("native_goal_invalid")
    if any(type(envelope.get(key)) is not int or envelope[key] < 0 for key in ("goal_version", "epoch")):
        reasons.append("native_generation_invalid")
    if isinstance(task, dict):
        # Older NoMaD runtimes did not include this redundant field. Preserve
        # that contract while checking it whenever a newer runtime sends it.
        if "backend" in envelope and envelope["backend"] != task.get("backend"):
            reasons.append("task_native_backend_mismatch")
        task_mode = task.get("task_mode","imagegoal")
        if (not isinstance(task_mode,str) or task_mode not in {"imagegoal","objnav","vln","tracking"}
                or (task_mode!="imagegoal" and task.get("backend")!="lightnav")):
            reasons.append("task_native_mode_invalid")
        if (task_mode=="imagegoal" and task.get("backend")=="lightnav" and
                (task.get("control_profile")!="go2_imagegoal_adaptation" or
                 status.get("execution_capability",{}).get("lightnav_imagegoal_adapter") is not True)):
            reasons.append("lightnav_imagegoal_adapter_not_declared")
        if "control_profile" in task and envelope.get("control_profile") != task["control_profile"]:
            reasons.append("task_native_control_profile_mismatch")
        if envelope.get("task_mode","imagegoal") != task_mode:
            reasons.append("task_native_mode_mismatch")
        for key in ("goal_id", "goal_version"):
            if task.get(key) != envelope.get(key):
                reasons.append(f"task_native_{key}_mismatch")
    if status.get("epoch") != envelope.get("epoch"):
        reasons.append("runtime_native_epoch_mismatch")
    return reasons


def validate_native_status(status, *, session_id, task_id, now_ns=None):
    """Pure validation of the separate supervised native contract."""
    now = time.time_ns() if now_ns is None else now_ns
    reasons = _contract_reasons(status, session_id, task_id, now)
    envelope = status.get("native_execution", {}) if isinstance(status, dict) else {}
    envelope = envelope if isinstance(envelope, dict) else {}
    if not isinstance(status, dict) or status.get("state") != "TRACKING":
        reasons.append("runtime_not_tracking")
    if envelope.get("authorized") is not True:
        reasons.append("native_not_authorized")
    if not _age_valid(envelope.get("evaluated_timestamp_ns"), now, 250_000_000):
        reasons.append("controller_evaluation_stale_or_future")
    evaluated, expiry = envelope.get("evaluated_timestamp_ns"), envelope.get("expires_timestamp_ns")
    if (type(expiry) is not int or type(evaluated) is not int
            or not now < expiry <= evaluated+250_000_000 or expiry <= evaluated):
        reasons.append("execution_lease_expired_or_invalid")
    if not _age_valid(envelope.get("source_stamp_ns"), now, 2_000_000_000):
        reasons.append("source_rgb_stale_or_future")
    reasons.extend(_sensor_reasons(envelope.get("sensors"), now))
    command = envelope.get("command")
    values = None
    if isinstance(command, dict):
        raw = tuple(command.get(key) for key in ("vx_m_s", "vy_m_s", "wz_rad_s"))
        if all(type(v) in (int, float) and math.isfinite(v) for v in raw):
            if 0 <= raw[0] <= .3 and raw[1] == 0 and abs(raw[2]) <= .55:
                values = raw
    if values is None:
        reasons.append("command_invalid_or_above_limits")
    return {"accepted": not reasons, "reasons": list(dict.fromkeys(reasons)),
            "command": values, "envelope": envelope}


class NativeImageGoalExecutor:
    """Fresh controller leases, independent watchdog, and serialized RPCs.

    ``check_feedback`` must independently validate actual reception ages,
    source progression, host boot, latched faults and idle remote sticks, and
    raise on failure. Network status reads never hold the RPC lock. The client
    must have a bounded timeout; a blocked SDK call cannot be preempted.
    """

    def __init__(self, client, *, session_id, task_id, fetch_status, check_feedback,
                 stop_event=None, max_duration_s=60., wall_clock=time.time_ns,
                 mono_clock=time.monotonic_ns, live_path=None):
        if (type(max_duration_s) not in (int, float) or not math.isfinite(max_duration_s)
                or not 0 < max_duration_s <= 120):
            raise ValueError("execution duration must be in (0, 120] seconds")
        if not session_id or not task_id or not callable(fetch_status) or not callable(check_feedback):
            raise ValueError("explicit session/task and status/feedback callbacks are required")
        self.client, self.session_id, self.task_id = client, session_id, task_id
        self.fetch_status, self.check_feedback = fetch_status, check_feedback
        self.stop_event = stop_event or threading.Event()
        self.wall_clock, self.mono_clock = wall_clock, mono_clock
        self.max_duration_s = float(max_duration_s)
        self.live_path = Path(live_path) if live_path is not None else None
        self._lock = threading.RLock()
        self._live_write_lock = threading.Lock()
        self._done = threading.Event()
        self._threads = []
        self._armed = self._closed = self._motion_active = False
        self._api_control = False
        self._fault = self._latched = None
        self._reason = "not_started"
        self._runtime_state = "STARTING"
        self._generation = None
        self._backend = None
        self._task_mode = None
        self._decision = None
        self._last_eval = -1
        self._last_signature = None
        self._received_mono = self._last_http_mono = None
        self._latest_sensors = {}
        self._minimum_source_ns = 0
        self._next_switch_mono = 0
        self._end_mono = None
        self._last_command = None
        self._events = deque(maxlen=12000)
        self._counts = {key: 0 for key in ("status_reads", "status_errors", "accepted_evaluations",
                        "duplicate_evaluations", "discarded_evaluations", "zero_requests",
                        "nonzero_requests", "switch_checks", "rpc_errors", "stops")}

    def _event(self, kind, **fields):
        self._events.append({"timestamp_ns": self.wall_clock(), "monotonic_ns": self.mono_clock(),
                             "kind": kind, **fields})

    def _feedback(self):
        if self.stop_event.is_set():
            raise RuntimeError("operator_stop")
        if self.check_feedback() is False:
            raise RuntimeError("feedback_check_returned_false")

    @staticmethod
    def _hard_health(reason):
        return any(key in reason.lower() for key in ("remote", "operator_stop", "latched_fault",
                                                      "feedback_no_latched_fault", "boot"))

    def _move_locked(self, values):
        moving = any(values)
        self._counts["nonzero_requests" if moving else "zero_requests"] += 1
        self._last_command = {"vx_m_s": values[0], "vy_m_s": values[1], "wz_rad_s": values[2],
                              "timestamp_ns": self.wall_clock()}
        self._event("move_requested", **self._last_command)
        # Mark an attempted nonzero active before the RPC: failure is not proof
        # that the robot did not receive it, so cleanup must still send zero.
        if moving:
            self._motion_active = True
        try:
            code = self.client.Move(*values)
            if code != 0:
                raise RuntimeError(f"code={code}")
        except Exception as exc:
            self._counts["rpc_errors"] += 1
            raise RuntimeError(f"native_Move:{type(exc).__name__}:{exc}") from exc
        self._motion_active = moving

    def _release_locked(self):
        self._api_control = None
        try:
            code = self.client.UseRemoteCommandFromApi(False)
            if code != 0:
                raise RuntimeError(f"code={code}")
            self._api_control = False
            self._event("api_released")
        except Exception as exc:
            self._counts["rpc_errors"] += 1
            self._fault = self._fault or f"api_release:{exc}"
            self._event("api_release_failed", reason=str(exc))

    def _stop_locked(self, reason, *, latch=False, fault=False, force=False):
        must_zero = force or self._motion_active or self._decision is not None
        self._decision = None
        self._reason = str(reason)
        if must_zero:
            self._counts["stops"] += 1
            self._event("stop", reason=self._reason)
            try:
                self._move_locked((0., 0., 0.))
                # This is a source fence after zero dispatch, not a measured
                # physical standstill time. Repeated polls cannot move it.
                self._minimum_source_ns = max(self._minimum_source_ns, self.wall_clock())
            except Exception as exc:
                self._fault = self._fault or str(exc)
                latch = True
        if fault:
            self._fault = self._fault or str(reason)
        if latch or fault:
            self._latched = self._latched or str(reason)
            self._armed = False
            self._done.set()
            self._release_locked()

    def _switch_locked(self):
        self._counts["switch_checks"] += 1
        code, enabled = self.client.SwitchGet()
        self._event("switch_checked", code=code, enabled=enabled)
        self._next_switch_mono = self.mono_clock()+500_000_000
        if code != 0 or enabled is not True:
            raise RuntimeError(f"native_avoidance_unavailable:code={code},enabled={enabled}")

    def arm(self, initial_status):
        with self._lock:
            if self._closed or self._latched:
                raise RuntimeError("executor is closed or latched")
            if self._armed:
                return self.snapshot()
            now = self.wall_clock()
            reasons = _contract_reasons(initial_status, self.session_id, self.task_id, now)
            reasons.extend(_sensor_reasons((initial_status.get("native_execution") or {}).get("sensors"), now))
            if initial_status.get("state") in TERMINAL:
                reasons.append("task_is_terminal")
            if reasons:
                raise ValueError(";".join(reasons))
            self._backend = initial_status["task"]["backend"]
            self._task_mode = initial_status["task"].get("task_mode","imagegoal")
            try:
                self._feedback()
                self._switch_locked()
                self._api_control = None
                code = self.client.UseRemoteCommandFromApi(True)
                if code != 0:
                    raise RuntimeError(f"native_api_acquire:code={code}")
                self._api_control = True
                self._feedback()
                self._armed = True
                self._end_mono = self.mono_clock()+int(self.max_duration_s*1e9)
                self._event("armed", session_id=self.session_id, task_id=self.task_id)
            except Exception as exc:
                self._stop_locked(f"arm_failed:{exc}", latch=True, fault=True, force=True)
                raise RuntimeError(self._fault) from exc
        self.submit_status(initial_status)
        return self.snapshot()

    def submit_status(self, status):
        with self._lock:
            if self._closed or self._latched:
                return False
            now, mono = self.wall_clock(), self.mono_clock()
            self._counts["status_reads"] += 1
            contract = _contract_reasons(status, self.session_id, self.task_id, now)
            if self._backend is not None and (status.get("task") or {}).get("backend") != self._backend:
                contract.append("runtime_task_backend_changed")
            if self._task_mode is not None and (status.get("task") or {}).get("task_mode","imagegoal") != self._task_mode:
                contract.append("runtime_task_mode_changed")
            # Identity changes or a missing/mismatched contract cannot be
            # authorized by falling back to the old execution envelope.
            severe = [r for r in contract if r != "runtime_status_stale_or_future"]
            if severe:
                self._stop_locked(";".join(severe), latch=True, force=True)
                return False
            if status.get("state") in TERMINAL:
                self._runtime_state = status["state"]
                self._stop_locked("task_terminal:"+status["state"], latch=True, force=True)
                return False
            self._runtime_state = status.get("state", "WAITING_INPUT")
            envelope = status["native_execution"]
            generation = tuple(envelope[key] for key in ("goal_id", "goal_version", "epoch"))
            if self._generation is not None:
                old = self._generation
                if generation[1] < old[1] or generation[2] < old[2]:
                    self._counts["discarded_evaluations"] += 1
                    return False
                if generation[1] == old[1] and generation[0] != old[0]:
                    self._stop_locked("goal_changed_without_version", latch=True, force=True)
                    return False
                if generation != old:
                    self._generation = generation
                    # The watchdog may already have dispatched zero before
                    # the runtime reports its invalidated epoch. Preserve
                    # that source fence: an epoch echo must not discard RGB
                    # captured after the successful stop. An active command
                    # or pending decision still makes _stop_locked send zero.
                    self._stop_locked("goal_or_epoch_advanced_waiting_new_source")
                    return False
            else:
                self._generation = generation
            decision = validate_native_status(status, session_id=self.session_id,
                                              task_id=self.task_id, now_ns=now)
            self._last_http_mono = mono
            if not decision["accepted"]:
                self._stop_locked(";".join(decision["reasons"]))
                return False
            self._latest_sensors = json.loads(json.dumps(envelope["sensors"], allow_nan=False))
            evaluated = envelope["evaluated_timestamp_ns"]
            immutable = {key: envelope.get(key) for key in (
                "schema_version", "session_id", "task_id", "goal_id", "goal_version", "epoch",
                "evaluated_timestamp_ns", "expires_timestamp_ns", "source_stamp_ns", "command",
                "supervised", "calibration_claim")}
            signature = json.dumps(immutable, sort_keys=True, allow_nan=False)
            if evaluated < self._last_eval:
                self._counts["discarded_evaluations"] += 1
                return False
            if evaluated == self._last_eval:
                if signature == self._last_signature:
                    self._counts["duplicate_evaluations"] += 1
                    return self._current_reason_locked() is None
                self._counts["discarded_evaluations"] += 1
                return False
            if envelope["source_stamp_ns"] <= self._minimum_source_ns:
                self._stop_locked("waiting_rgb_captured_after_zero_dispatch")
                return False
            self._decision = {**immutable, "command_values": decision["command"]}
            self._last_eval, self._last_signature = evaluated, signature
            self._received_mono = mono
            self._counts["accepted_evaluations"] += 1
            self._reason = "supervised_native_authorized"
            self._event("evaluation_accepted", evaluated_timestamp_ns=evaluated,
                        source_stamp_ns=envelope["source_stamp_ns"], goal_version=envelope["goal_version"],
                        epoch=envelope["epoch"])
            return True

    def _current_reason_locked(self):
        if self._decision is None:
            return "no_current_native_evaluation"
        now, mono = self.wall_clock(), self.mono_clock()
        if self._last_http_mono is None or not 0 <= mono-self._last_http_mono <= 250_000_000:
            return "runtime_http_watchdog_expired"
        if self._received_mono is None or not 0 <= mono-self._received_mono <= 250_000_000:
            return "evaluation_receive_lease_expired"
        if (now >= self._decision["expires_timestamp_ns"]
                or not _age_valid(self._decision["evaluated_timestamp_ns"], now, 250_000_000)):
            return "native_evaluation_expired_or_clock_regressed"
        if not _age_valid(self._decision["source_stamp_ns"], now, 2_000_000_000):
            return "source_rgb_stale_or_future"
        sensors = _sensor_reasons(self._latest_sensors, now)
        return ";".join(sensors) if sensors else None

    def poll_once(self):
        try:
            status = self.fetch_status()
            return self.submit_status(status)
        except Exception as exc:
            with self._lock:
                self._counts["status_errors"] += 1
                self._stop_locked(f"runtime_http_error:{type(exc).__name__}:{exc}")
            return False

    def send_once(self):
        with self._lock:
            if not self._armed or self._closed or self._latched:
                return False
            try:
                self._feedback()
            except Exception as exc:
                self._stop_locked(str(exc), latch=self._hard_health(str(exc)))
                return False
            if self._end_mono is not None and self.mono_clock() >= self._end_mono:
                self._stop_locked("maximum_execution_duration", latch=True, force=True)
                return False
            reason = self._current_reason_locked()
            if reason:
                self._stop_locked(reason)
                return False
            try:
                self._move_locked(self._decision["command_values"])
            except Exception as exc:
                self._stop_locked(str(exc), latch=True, fault=True, force=True)
                return False
            # The original deadline is rechecked after a blocked SDK call;
            # successful sends never renew either a source or a lease.
            try:
                self._feedback()
            except Exception as exc:
                self._stop_locked(str(exc), latch=self._hard_health(str(exc)))
                return False
            if self._end_mono is not None and self.mono_clock() >= self._end_mono:
                self._stop_locked("maximum_execution_duration", latch=True, force=True)
                return False
            reason = self._current_reason_locked()
            if reason:
                self._stop_locked(reason)
                return False
            return True

    def watchdog_once(self):
        with self._lock:
            if not self._armed or self._closed or self._latched:
                return
            try:
                self._feedback()
            except Exception as exc:
                self._stop_locked(str(exc), latch=self._hard_health(str(exc)))
                return
            if self._end_mono is not None and self.mono_clock() >= self._end_mono:
                self._stop_locked("maximum_execution_duration", latch=True, force=True)
                return
            if self.mono_clock() >= self._next_switch_mono:
                try:
                    self._switch_locked()
                except Exception as exc:
                    self._counts["rpc_errors"] += 1
                    self._stop_locked(str(exc), latch=True, fault=True, force=True)
                    return
            reason = self._current_reason_locked()
            if reason:
                self._stop_locked(reason)

    def run(self, initial_status=None):
        try:
            self.arm(self.fetch_status() if initial_status is None else initial_status)
            def poll():
                while not self._done.is_set():
                    began = time.monotonic()
                    self.poll_once()
                    self._done.wait(max(0., .05-(time.monotonic()-began)))
            def watch():
                while not self._done.wait(.02):
                    self.watchdog_once()
            def publish():
                while not self._done.is_set():
                    self._publish_live()
                    self._done.wait(.2)
            for target, name in ((poll, "native-imagegoal-http"), (watch, "native-imagegoal-watchdog"),
                                 (publish, "native-imagegoal-live")):
                thread = threading.Thread(target=target, name=name, daemon=True)
                self._threads.append(thread)
                thread.start()
            while not self._done.is_set():
                began = time.monotonic()
                self.send_once()
                self._done.wait(max(0., .05-(time.monotonic()-began)))
        finally:
            self.close()
        return self.snapshot()

    def close(self):
        with self._lock:
            if not self._closed:
                self._closed = True
                self._stop_locked("executor_closed" if not self._latched else self._latched,
                                  latch=True, force=True)
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1.)
        self._publish_live()
        return self.snapshot()

    def live_status(self):
        with self._lock:
            running = self._armed and not self._closed and not self._latched
            if self._fault:
                state = "ERROR"
            elif self._closed or self._latched:
                state = "STOPPED"
            elif self._motion_active:
                state = "EXECUTING"
            elif running:
                state = self._runtime_state if self._runtime_state != "TRACKING" else "WAITING_INPUT"
            else:
                state = "STARTING"
            return {"schema_version": "imagegoal.native_executor_live.v1", "timestamp_ns": self.wall_clock(),
                    "session_id": self.session_id, "task_id": self.task_id, "running": running,
                    "backend": self._backend,"task_mode":self._task_mode,
                    "state": state, "runtime_state": self._runtime_state,
                    "command_active": self._motion_active, "last_command": self._last_command,
                    "error": self._fault, "reason": self._reason, "latched_reason": self._latched,
                    "counts": dict(self._counts), "supervised": True, "calibration_claim": CLAIM}

    def _publish_live(self):
        if self.live_path is None:
            return
        try:
            with self._live_write_lock:
                _atomic_json(self.live_path, self.live_status())
        except Exception as exc:
            with self._lock:
                self._stop_locked(f"live_receipt_write_failed:{exc}", latch=True, fault=True)

    def snapshot(self):
        with self._lock:
            return {"schema_version": "imagegoal.native_executor.v1", "session_id": self.session_id,
                    "task_id": self.task_id, "armed": self._armed, "closed": self._closed,
                    "backend": self._backend,"task_mode":self._task_mode,
                    "active": self._motion_active, "fault": self._fault, "latched_reason": self._latched,
                    "reason": self._reason, "api_control_acquired": self._api_control,
                    "minimum_post_stop_source_ns": self._minimum_source_ns,
                    "last_evaluation_ns": self._last_eval, "received_monotonic_ns": self._received_mono,
                    "generation": self._generation, "last_command": self._last_command,
                    "counts": dict(self._counts), "events": [dict(row) for row in self._events],
                    "supervised": True, "calibration_claim": CLAIM,
                    "transport": "ObstaclesAvoidClient.Move; no SportClient fallback",
                    "move_return_zero_proves_physical_execution": False,
                    "navigation_success_validated": False, "collision_avoidance_behavior_validated": False,
                    "independent_physical_standstill_verified": False}


def _fetch_status(runtime_url):
    with urlopen(runtime_url.rstrip("/")+"/status", timeout=.2) as response:
        raw = response.read(2*1024*1024+1)
    if len(raw) > 2*1024*1024:
        raise ValueError("runtime status exceeds 2 MiB")
    return json.loads(raw)


def _atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)+"\n")
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-supervised", action="store_true")
    parser.add_argument("--operator-present", action="store_true")
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--runtime-url", default="http://127.0.0.1:8093")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--feedback-status", type=Path, default=Path(".runtime/go2/telemetry.json"))
    parser.add_argument("--duration-s", type=float, default=60.)
    parser.add_argument("--stop-file", type=Path, default=Path(".runtime/go2/native_imagegoal.stop"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.execute_supervised and not args.operator_present:
        parser.error("--execute-supervised requires --operator-present")
    if not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 120:
        parser.error("duration must be in (0, 120] seconds")
    parsed = urlsplit(args.runtime_url)
    if (parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}
            or parsed.username or parsed.password or parsed.path not in {"", "/"} or parsed.query or parsed.fragment):
        parser.error("runtime-url must be a loopback HTTP origin")
    output = args.output or Path("runs")/(
        "native_imagegoal_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema_version": "imagegoal.native_execution_receipt.v1", "created_timestamp_ns": time.time_ns(),
              "execute_requested": args.execute_supervised, "operator_present": args.operator_present,
              "sdk_initialized": False, "nonzero_commands_requested": False, "blocked": [],
              "supervised": True, "calibration_claim": CLAIM, "task_id": args.task_id,
              "navigation_success_validated": False, "collision_avoidance_behavior_validated": False,
              "independent_physical_standstill_verified": False, "mode": "read_only_preparation"}
    executor, lock_file = None, None
    shutdown = threading.Event()
    old_handlers = {}
    _atomic_json(output/"live.json", {"schema_version": "imagegoal.native_executor_live.v1",
                 "timestamp_ns": time.time_ns(), "running": False, "state": "STARTING",
                 "task_id": args.task_id, "command_active": False, "last_command": None,
                 "error": None, "counts": {}})
    try:
        config = load_config(args.config)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM):
            pass
        report["preflight"] = preflight(config, args.feedback_status)
        if not report["preflight"]["ok"]:
            raise RuntimeError("preflight:"+";".join(report["preflight"]["blocked"]))
        initial = _fetch_status(args.runtime_url)
        session = initial.get("session_id")
        now = time.time_ns()
        reasons = _contract_reasons(initial, session, args.task_id, now)
        native = initial.get("native_execution")
        reasons.extend(_sensor_reasons((native or {}).get("sensors"), now))
        if initial.get("state") in TERMINAL:
            reasons.append("task_is_terminal")
        if reasons:
            raise RuntimeError("runtime_preflight:"+";".join(reasons))
        if args.stop_file.exists():
            raise RuntimeError("operator_stop_file_present")
        report["initial_native_execution"] = native
        report["initial_state"] = initial.get("state")
        if not args.execute_supervised:
            return 0
        lock_path = Path(".runtime/go2/native_motion.lock")
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock_file = lock_path.open("a")
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, lambda *_: shutdown.set())
        def healthy_feedback():
            if shutdown.is_set() or args.stop_file.exists():
                raise RuntimeError("operator_stop")
            checks, _ = check_feedback(args.feedback_status, config)
            failed = [key for key, passed in checks.items() if not passed]
            if failed:
                raise RuntimeError("feedback:"+";".join(failed))
        healthy_feedback()
        # No real SDK import, participant or authority change precedes these gates.
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
        ChannelFactoryInitialize(config["dds_domain_id"], config["net_if"])
        client = ObstaclesAvoidClient()
        client.SetTimeout(.20)
        client.Init()
        report["sdk_initialized"] = True
        executor = NativeImageGoalExecutor(client, session_id=session, task_id=args.task_id,
                    fetch_status=lambda: _fetch_status(args.runtime_url), check_feedback=healthy_feedback,
                    stop_event=shutdown, max_duration_s=args.duration_s, live_path=output/"live.json")
        report["mode"] = "supervised_native_imagegoal"
        _atomic_json(output/"report.json", report)
        # DDS/client initialization can take seconds. Keep the SDK-before-use
        # preflight above, but reacquire status before arm rather than reuse an
        # inevitably stale pre-initialization evaluation/odometry snapshot.
        report["executor"] = executor.run()
        report["mode"] = "stopped"
        return 2 if report["executor"]["fault"] else 0
    except Exception as exc:
        report["blocked"].append(f"{type(exc).__name__}: {exc}")
        report["mode"] = "blocked_or_stopped"
        return 2
    finally:
        if executor is not None:
            executor.close()
            report["executor"] = executor.snapshot()
            report["nonzero_commands_requested"] = report["executor"]["counts"]["nonzero_requests"] > 0
        for signum, previous in old_handlers.items():
            signal.signal(signum, previous)
        if lock_file is not None:
            lock_file.close()
        if executor is None:
            _atomic_json(output/"live.json", {"schema_version": "imagegoal.native_executor_live.v1",
                         "timestamp_ns": time.time_ns(), "running": False,
                         "state": "ERROR" if report["blocked"] else "READ_ONLY",
                         "task_id": args.task_id, "command_active": False, "last_command": None,
                         "error": ";".join(report["blocked"]) or None, "counts": {}})
        report["finished_timestamp_ns"] = time.time_ns()
        _atomic_json(output/"report.json", report)
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())
