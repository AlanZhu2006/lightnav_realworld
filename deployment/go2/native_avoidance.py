"""Bounded, supervised commands through an injected Unitree avoidance client.

This module imports no SDK and constructs no hardware client. The caller must
set its client's RPC timeout (normally 0.2 s). A successful no-reply Move means
only that the request was sent, never that motion or obstacle avoidance worked.
"""
from __future__ import annotations

from collections import deque
import math
import threading
import time


class NativeAvoidanceGuard:
    """Serialize avoidance RPCs and stop on health loss or a fixed deadline.

    ``health_check`` returns None/True when healthy and raises (or returns
    False) on lost input, joystick takeover, task stop, or caller cancellation.
    It must itself be bounded. RPCs cannot be interrupted while inside the
    injected client: a delayed RPC is followed by another deadline check and
    zero, and is never allowed to start a fresh command period.
    """

    def __init__(self, client, *, health_check, max_vx=.08, max_wz=.20,
                 max_burst_s=.25):
        for name, value in (("max_vx", max_vx), ("max_wz", max_wz),
                            ("max_burst_s", max_burst_s)):
            self._number(name, value, positive=True)
        if max_vx > .2 or max_wz > .4 or max_burst_s > .25:
            raise ValueError("supervised limits must not exceed .2 m/s, .4 rad/s, .25 s")
        if not callable(health_check):
            raise TypeError("health_check must be callable")
        self.client, self.health_check = client, health_check
        self.max_vx, self.max_wz = float(max_vx), float(max_wz)
        self.max_burst_s = float(max_burst_s)
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._shutdown = threading.Event()
        self._watchdog = None
        self._armed = self._active = self._closed = self._released = False
        self._api_control = False
        self._fault = None
        self._deadline = None
        self._generation = 0
        self._next_switch_check = 0.
        self._last_switch = None
        self._last_command = None
        self._events = deque(maxlen=512)
        self._counts = {key: 0 for key in (
            "health_checks", "switch_checks", "api_acquire", "api_release",
            "move_requests", "nonzero_move_requests", "zero_move_requests",
            "rpc_errors", "bursts_started", "bursts_completed")}

    @staticmethod
    def _number(name, value, *, positive=False):
        if (type(value) not in (int, float) or not math.isfinite(value)
                or (positive and value <= 0)):
            raise ValueError(f"{name} must be a finite {'positive ' if positive else ''}number")

    def _event(self, kind, **fields):
        self._events.append({"timestamp_ns": time.time_ns(),
                             "monotonic_ns": time.monotonic_ns(),
                             "kind": kind, **fields})

    def _health_locked(self):
        self._counts["health_checks"] += 1
        try:
            if self.health_check() is False:
                raise RuntimeError("health_check returned False")
        except Exception as exc:
            raise RuntimeError(f"health_check:{type(exc).__name__}:{exc}") from exc

    def _switch_locked(self, *, force=False):
        if not force and time.monotonic() < self._next_switch_check:
            return
        self._counts["switch_checks"] += 1
        try:
            code, enabled = self.client.SwitchGet()
        except Exception as exc:
            self._counts["rpc_errors"] += 1
            raise RuntimeError(f"SwitchGet:{type(exc).__name__}:{exc}") from exc
        self._last_switch = {"timestamp_ns": time.time_ns(), "code": code,
                             "enabled": enabled}
        self._next_switch_check = time.monotonic() + .2
        self._event("switch_checked", code=code, enabled=enabled)
        if code != 0 or enabled is not True:
            self._counts["rpc_errors"] += int(code != 0)
            raise RuntimeError(f"avoidance_unavailable:code={code},enabled={enabled}")

    def _move_locked(self, vx, wz):
        self._counts["move_requests"] += 1
        self._counts["nonzero_move_requests" if vx or wz else "zero_move_requests"] += 1
        self._last_command = {"vx": vx, "vy": 0., "wz": wz,
                              "timestamp_ns": time.time_ns()}
        self._event("move_requested", vx=vx, vy=0., wz=wz)
        try:
            code = self.client.Move(vx, 0., wz)
        except Exception as exc:
            self._counts["rpc_errors"] += 1
            raise RuntimeError(f"avoidance_Move:{type(exc).__name__}:{exc}") from exc
        if code != 0:
            self._counts["rpc_errors"] += 1
            raise RuntimeError(f"avoidance_Move:code={code}")

    def _release_locked(self):
        self._counts["api_release"] += 1
        self._api_control = None
        try:
            code = self.client.UseRemoteCommandFromApi(False)
            if code != 0:
                raise RuntimeError(f"code={code}")
            self._api_control = False
            self._event("api_control_released")
        except Exception as exc:
            self._counts["rpc_errors"] += 1
            reason = f"api_release:{type(exc).__name__}:{exc}"
            if self._fault is None:
                self._fault = reason
            self._event("release_failed", reason=reason)

    def _stop_locked(self, reason, *, release=False):
        self._active = False
        self._deadline = None
        self._generation += 1
        self._event("stop", reason=str(reason), release=bool(release))
        try:
            self._move_locked(0., 0.)
        except Exception as exc:
            if self._fault is None:
                self._fault = str(exc)
            self._event("zero_failed", reason=str(exc))
            release = True
        if release or self._fault is not None:
            self._armed = False
            self._released = True
            self._shutdown.set()
            self._release_locked()
        self._condition.notify_all()

    def _fault_locked(self, reason):
        if self._fault is None:
            self._fault = str(reason)
            self._event("fault_latched", reason=self._fault)
        self._stop_locked(reason, release=True)

    def _expire_locked(self):
        if self._active and time.monotonic() >= self._deadline:
            self._counts["bursts_completed"] += 1
            self._stop_locked("burst_deadline")
            return True
        return False

    def _live_locked(self):
        self._health_locked()
        if self._expire_locked():
            return
        self._switch_locked()
        # A health check or SwitchGet can block. Never use its old decision to
        # send a late nonzero command or extend the original burst deadline.
        if self._expire_locked():
            return
        self._health_locked()
        self._expire_locked()

    def arm(self):
        with self._lock:
            if self._closed or self._released or self._fault:
                raise RuntimeError("guard is closed, released, or faulted")
            if self._armed:
                return self.status()
            try:
                self._health_locked()
                self._switch_locked(force=True)
                self._counts["api_acquire"] += 1
                self._api_control = None
                code = self.client.UseRemoteCommandFromApi(True)
                if code != 0:
                    self._counts["rpc_errors"] += 1
                    raise RuntimeError(f"api_acquire:code={code}")
                self._api_control = True
                self._health_locked()
                self._armed = True
                self._event("armed")
            except Exception as exc:
                self._fault_locked(f"arm_failed:{type(exc).__name__}:{exc}")
                raise RuntimeError(self._fault) from exc
            self._watchdog = threading.Thread(target=self._watch,
                                              name="native-avoidance-watchdog", daemon=True)
            self._watchdog.start()
            return self.status()

    def _watch(self):
        while not self._shutdown.wait(.02):
            with self._lock:
                if self._closed or self._released or self._fault:
                    return
                try:
                    if not self._expire_locked():
                        self._live_locked()
                except Exception as exc:
                    self._fault_locked(str(exc))
                    return

    def burst(self, vx, wz, duration_s=.25):
        for name, value in (("vx", vx), ("wz", wz), ("duration_s", duration_s)):
            self._number(name, value, positive=name == "duration_s")
        if abs(vx) > self.max_vx or abs(wz) > self.max_wz:
            raise ValueError("burst velocity exceeds configured limits")
        if duration_s > self.max_burst_s:
            raise ValueError("burst duration exceeds configured hard maximum")
        with self._condition:
            if not self._armed or self._closed or self._released or self._fault:
                raise RuntimeError("guard must be armed and healthy")
            if self._active:
                raise RuntimeError("a burst is already active; its deadline cannot be extended")
            self._active = True
            self._deadline = time.monotonic() + float(duration_s)
            self._generation += 1
            generation = self._generation
            self._counts["bursts_started"] += 1
            self._event("burst_started", vx=float(vx), wz=float(wz),
                        duration_s=float(duration_s), deadline_monotonic_s=self._deadline)
            next_send = time.monotonic()
            while self._active and self._generation == generation:
                try:
                    self._live_locked()
                    if not self._active or self._generation != generation:
                        break
                    now = time.monotonic()
                    if now >= self._deadline:
                        self._expire_locked()
                        break
                    if now >= next_send:
                        self._move_locked(float(vx), float(wz))
                        if not self._expire_locked():
                            self._health_locked()
                            self._expire_locked()
                        next_send = time.monotonic() + .05
                    if self._active:
                        self._condition.wait(max(0., min(next_send, self._deadline)-time.monotonic()))
                except Exception as exc:
                    self._fault_locked(str(exc))
                    break
            if self._fault:
                raise RuntimeError(self._fault)
            return self.status()

    def stop(self, reason="operator_stop", release=False):
        with self._lock:
            self._stop_locked(reason, release=release or self._closed or self._released)
            return self.status()

    def close(self):
        with self._lock:
            if not self._closed:
                self._closed = True
                self._stop_locked("guard_closed", release=True)
            watcher = self._watchdog
        if watcher is not None and watcher is not threading.current_thread():
            watcher.join(timeout=1.)
        return self.status()

    def status(self):
        with self._lock:
            return {"schema_version": "imagegoal.native_avoidance_guard.v1",
                    "armed": self._armed, "active": self._active,
                    "closed": self._closed, "released": self._released,
                    "fault": self._fault, "api_control_acquired": self._api_control,
                    "deadline_monotonic_s": self._deadline,
                    "remaining_s": (max(0., self._deadline-time.monotonic())
                                    if self._deadline is not None else 0.),
                    "limits": {"max_vx": self.max_vx, "max_wz": self.max_wz,
                               "max_burst_s": self.max_burst_s},
                    "last_switch": dict(self._last_switch) if self._last_switch else None,
                    "last_command": dict(self._last_command) if self._last_command else None,
                    "command_counts": dict(self._counts),
                    "events": [dict(event) for event in self._events],
                    "transport": "unitree_obstacles_avoid_only",
                    "move_return_zero_proves_physical_execution": False,
                    "physical_standstill_verified": False,
                    "obstacle_avoidance_behavior_validated": False}
