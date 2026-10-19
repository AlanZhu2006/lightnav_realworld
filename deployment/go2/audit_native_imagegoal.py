"""Offline native ImageGoal authorization, lease and transport audit."""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import threading
import time
import tempfile

from .native_imagegoal import CLAIM, NATIVE_SOURCES, NativeImageGoalExecutor, validate_native_status


class Clock:
    def __init__(self):
        self.wall, self.mono = 1_800_000_000_000_000_000, 10_000_000_000

    def advance(self, seconds):
        self.wall += int(seconds*1e9)
        self.mono += int(seconds*1e9)


class FakeClient:
    def __init__(self):
        self.calls = []
        self.enabled = True
        self.acquire_code = self.move_code = 0
        self.on_move = None

    def SwitchGet(self):
        self.calls.append(("switch", self.enabled))
        return 0, self.enabled

    def UseRemoteCommandFromApi(self, enabled):
        self.calls.append(("authority", enabled))
        return self.acquire_code if enabled else 0

    def Move(self, vx, vy, wz):
        self.calls.append(("move", (vx, vy, wz)))
        if any((vx, vy, wz)):
            if self.on_move:
                self.on_move()
            return self.move_code
        return 0


def fixture(clock, *, goal="goal-1", version=1, epoch=1, state="TRACKING", source=None):
    stamp = clock.wall-10_000_000 if source is None else source
    sensors = {name: {"capture_timestamp_ns": stamp, "clock_domain": "unix_epoch_ns",
                         "timestamp_source": provenance, "frame_name": frame,
                         "synthetic": False, "commissioning": False}
               for name, (frame, provenance) in NATIVE_SOURCES.items()}
    sensors["odom"]["body_frame_name"] = "base_link"
    native = {"schema_version": "imagegoal.native_execution.v1", "authorized": state == "TRACKING",
              "session_id": "session-1", "task_id": "task-1", "goal_id": goal,
              "goal_version": version, "epoch": epoch, "evaluated_timestamp_ns": clock.wall,
              "expires_timestamp_ns": clock.wall+200_000_000, "source_stamp_ns": stamp,
              "command": {"vx_m_s": .15, "vy_m_s": 0., "wz_rad_s": .2},
              "sensors": sensors, "supervised": True, "calibration_claim": CLAIM,
              "safety_backend": "unitree_obstacles_avoid_only"}
    return {"schema_version": "imagegoal.runtime.v1", "session_id": "session-1",
            "timestamp_ns": clock.wall, "synthetic_feedback": False, "state": state,
            "epoch": epoch, "task": {"task_id": "task-1", "backend": "nomad",
            "execution_mode": "supervised_native", "goal_id": goal, "goal_version": version},
            "native_execution": native, "execution_envelope": {"not_for_motion": True}}


def setup(*, feedback=None):
    clock, client = Clock(), FakeClient()
    executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                    fetch_status=lambda: fixture(clock), check_feedback=feedback or (lambda: None),
                    wall_clock=lambda: clock.wall, mono_clock=lambda: clock.mono)
    executor.arm(fixture(clock))
    return clock, client, executor


def rejected(status, clock):
    return not validate_native_status(status, session_id="session-1", task_id="task-1",
                                      now_ns=clock.wall)["accepted"]


def audit():
    checks = {}
    clock = Clock()
    good = fixture(clock)
    assert not rejected(good, clock)
    for name, value in (("vx_m_s", .301), ("vx_m_s", -.1), ("vy_m_s", .001), ("wz_rad_s", .551),
                        ("vx_m_s", math.nan), ("wz_rad_s", math.inf), ("vx_m_s", True)):
        bad = copy.deepcopy(good)
        bad["native_execution"]["command"][name] = value
        assert rejected(bad, clock)
    for field, value in (("supervised", False), ("calibration_claim", "verified"),
                         ("safety_backend", "sport_direct"),
                         ("authorized", False), ("source_stamp_ns", clock.wall-2_000_000_001),
                         ("evaluated_timestamp_ns", clock.wall-250_000_001),
                         ("expires_timestamp_ns", clock.wall+250_000_001)):
        bad = copy.deepcopy(good)
        bad["native_execution"][field] = value
        assert rejected(bad, clock)
    for name in ("rgb", "odom"):
        bad = copy.deepcopy(good)
        bad["native_execution"]["sensors"][name]["synthetic"] = True
        assert rejected(bad, clock)
        for key in ("frame_name", "timestamp_source"):
            bad = copy.deepcopy(good)
            bad["native_execution"]["sensors"][name][key] = "wrong_source"
            assert rejected(bad, clock)
    bad = copy.deepcopy(good)
    bad["native_execution"]["sensors"]["odom"]["body_frame_name"] = "wrong_body"
    assert rejected(bad, clock)
    old = copy.deepcopy(good)
    del old["native_execution"]
    old["execution_envelope"] = {"eligible_for_robot_tracking": True}
    assert rejected(old, clock)
    checks["strict_native_only_contract_and_numeric_caps"] = True

    for backend in ("nomad", "lightnav"):
        clock, client = Clock(), FakeClient()
        native = fixture(clock)
        native["task"]["backend"] = native["native_execution"]["backend"] = backend
        native["task"]["task_mode"] = native["native_execution"]["task_mode"] = "objnav" if backend=="lightnav" else "imagegoal"
        assert not rejected(native, clock)
        wrong = copy.deepcopy(native)
        wrong["native_execution"]["backend"] = "lightnav" if backend == "nomad" else "nomad"
        assert rejected(wrong, clock)
        executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                    fetch_status=lambda: native, check_feedback=lambda: None,
                    wall_clock=lambda: clock.wall, mono_clock=lambda: clock.mono)
        executor.arm(native)
        assert executor.snapshot()["backend"] == backend and executor.send_once()
        changed = copy.deepcopy(native)
        changed["task"]["backend"] = changed["native_execution"]["backend"] = wrong["native_execution"]["backend"]
        changed["task"]["task_mode"] = changed["native_execution"]["task_mode"] = "objnav" if changed["task"]["backend"]=="lightnav" else "imagegoal"
        assert not executor.submit_status(changed)
        assert "runtime_task_backend_changed" in executor.snapshot()["latched_reason"].split(";")
        assert client.calls[-2:] == [("move", (0., 0., 0.)), ("authority", False)]
        assert not executor.send_once()
        executor.close()
    checks["both_backends_execute_and_backend_switch_latches_zero_release"] = True
    unsupported = copy.deepcopy(good)
    unsupported["task"]["backend"] = "unregistered"
    assert rejected(unsupported, Clock())
    checks["unsupported_and_mismatched_backend_contracts_rejected"] = True

    clock, client, executor = setup()
    assert not any(row[0] == "move" and any(row[1]) for row in client.calls)
    assert executor.send_once()
    assert client.calls[-1] == ("move", (.15, 0., .2))
    executor.close()
    assert client.calls[-1] == ("authority", False)
    checks["arm_then_native_move_and_zero_release"] = True

    clock, client = Clock(), FakeClient()
    waiting = fixture(clock, state="PLANNING")
    waiting["native_execution"]["source_stamp_ns"] = None
    executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                    fetch_status=lambda: fixture(clock), check_feedback=lambda: None,
                    wall_clock=lambda: clock.wall, mono_clock=lambda: clock.mono)
    executor.arm(waiting)
    assert executor.snapshot()["armed"] and not executor.send_once()
    assert not any(row[0] == "move" and any(row[1]) for row in client.calls)
    clock.advance(.03)
    assert executor.submit_status(fixture(clock)) and executor.send_once()
    executor.close()
    checks["initial_unready_model_waits_without_rejecting_supervised_task"] = True

    # Reproduce real DDS/client startup latency without a real SDK: a status
    # fetched before initialization has expired by the time arm is attempted.
    # run() must fetch again, retaining the original session/task binding.
    clock, client = Clock(), FakeClient()
    before_sdk = fixture(clock, state="PLANNING")
    clock.advance(1.2)
    reads = [0]
    def after_sdk_status():
        reads[0] += 1
        return fixture(clock, state="PLANNING" if reads[0] == 1 else "CANCELLED")
    executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                    fetch_status=after_sdk_status, check_feedback=lambda: None,
                    wall_clock=lambda: clock.wall, mono_clock=lambda: clock.mono)
    try:
        executor.arm(before_sdk)
        raise AssertionError("pre-SDK snapshot was incorrectly accepted after startup delay")
    except ValueError as exc:
        assert "stale_or_future" in str(exc)
    assert not client.calls
    final = executor.run()
    assert reads[0] >= 2 and ("authority", True) in client.calls
    assert final["closed"] and final["latched_reason"] == "task_terminal:CANCELLED"
    assert not any(row[0] == "move" and any(row[1]) for row in client.calls)
    checks["fresh_status_after_slow_sdk_init_can_arm_without_motion"] = True

    for changed in ("session", "task"):
        clock, client = Clock(), FakeClient()
        clock.advance(1.2)
        refreshed = fixture(clock, state="PLANNING")
        if changed == "session":
            refreshed["session_id"] = refreshed["native_execution"]["session_id"] = "restarted-runtime"
        else:
            refreshed["task"]["task_id"] = refreshed["native_execution"]["task_id"] = "replacement-task"
        executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                        fetch_status=lambda: refreshed, check_feedback=lambda: None,
                        wall_clock=lambda: clock.wall, mono_clock=lambda: clock.mono)
        try:
            executor.run()
            raise AssertionError("post-SDK refresh changed the explicit binding")
        except ValueError as exc:
            assert "changed" in str(exc)
        assert ("authority", True) not in client.calls
        assert not any(row[0] == "move" and any(row[1]) for row in client.calls)
    checks["post_sdk_refresh_cannot_rebind_session_or_task"] = True

    clock, client, executor = setup()
    first = fixture(clock)
    received = executor.snapshot()["received_monotonic_ns"]
    assert executor.send_once()
    clock.advance(.1)
    repeated = copy.deepcopy(first)
    repeated["timestamp_ns"] = clock.wall
    assert executor.submit_status(repeated)
    assert executor.snapshot()["received_monotonic_ns"] == received
    tampered = copy.deepcopy(repeated)
    tampered["native_execution"]["expires_timestamp_ns"] += 10_000_000
    assert not executor.submit_status(tampered)
    assert executor.snapshot()["received_monotonic_ns"] == received
    clock.advance(.101)
    executor.watchdog_once()
    assert not executor.snapshot()["active"]
    assert not executor.submit_status(repeated)
    assert not executor.send_once()
    executor.close()
    checks["duplicate_or_tampered_poll_cannot_renew_original_lease"] = True

    clock, client, executor = setup()
    assert executor.send_once()
    clock.advance(.1)
    zero_before = executor.snapshot()["counts"]["zero_requests"]
    newer = fixture(clock, goal="goal-2", version=2, epoch=2)
    assert not executor.submit_status(newer)
    fence = executor.snapshot()["minimum_post_stop_source_ns"]
    assert not executor.snapshot()["active"]
    assert executor.snapshot()["counts"]["zero_requests"] == zero_before+1
    assert client.calls[-1] == ("move", (0., 0., 0.))
    assert not executor.submit_status(newer)
    assert executor.snapshot()["minimum_post_stop_source_ns"] == fence
    clock.advance(.03)
    fresh = fixture(clock, goal="goal-2", version=2, epoch=2)
    assert executor.submit_status(fresh) and executor.send_once()
    old = fixture(clock, goal="goal-1", version=1, epoch=1)
    assert not executor.submit_status(old) and executor.snapshot()["active"]
    executor.close()
    checks["goal_epoch_advance_zero_then_new_source_and_old_generation_discard"] = True

    # A real source deadline can stop the executor shortly before the
    # runtime's corresponding epoch change reaches its HTTP poller. RGB
    # captured between those events is already post-stop and must survive
    # the epoch echo without a redundant zero moving the source fence.
    clock, client, executor = setup()
    assert executor.send_once()
    clock.advance(.201)
    executor.watchdog_once()
    stopped = executor.snapshot()
    fence = stopped["minimum_post_stop_source_ns"]
    zeros = stopped["counts"]["zero_requests"]
    assert zeros == 1 and not stopped["active"]
    clock.advance(.03)
    source = clock.wall
    clock.advance(.01)
    echoed = fixture(clock, epoch=2, source=source)
    assert not executor.submit_status(echoed)
    after_echo = executor.snapshot()
    assert after_echo["minimum_post_stop_source_ns"] == fence
    assert after_echo["counts"]["zero_requests"] == zeros
    assert executor.submit_status(echoed) and executor.send_once()
    checks["expired_zero_then_new_rgb_survives_epoch_echo_without_second_fence"] = True

    received = executor.snapshot()["received_monotonic_ns"]
    clock.advance(.1)
    repeated = copy.deepcopy(echoed)
    repeated["timestamp_ns"] = clock.wall
    assert executor.submit_status(repeated)
    assert executor.snapshot()["received_monotonic_ns"] == received
    assert executor.snapshot()["minimum_post_stop_source_ns"] == fence
    clock.advance(.101)
    executor.watchdog_once()
    expired = executor.snapshot()
    assert not expired["active"] and expired["counts"]["zero_requests"] == zeros+1
    fence = expired["minimum_post_stop_source_ns"]
    zeros = expired["counts"]["zero_requests"]
    for _ in range(3):
        clock.advance(.01)
        repeated["timestamp_ns"] = clock.wall
        assert not executor.submit_status(repeated)
        executor.watchdog_once()
        assert executor.snapshot()["received_monotonic_ns"] == received
        assert executor.snapshot()["minimum_post_stop_source_ns"] == fence
        assert executor.snapshot()["counts"]["zero_requests"] == zeros
    executor.close()
    checks["epoch_echo_repeated_polls_neither_renew_lease_nor_move_stop_fence"] = True

    for state in ("PAUSED", "MODEL_STOP"):
        clock, client, executor = setup()
        executor.send_once()
        clock.advance(.02)
        assert not executor.submit_status(fixture(clock, state=state))
        assert not executor.snapshot()["active"] and not executor.snapshot()["latched_reason"]
        clock.advance(.03)
        assert executor.submit_status(fixture(clock)) and executor.send_once()
        executor.close()
    checks["pause_and_model_stop_zero_without_task_success_claim"] = True

    for reason in ("CANCELLED", "SUCCEEDED", "session_change", "task_change"):
        clock, client, executor = setup()
        executor.send_once()
        clock.advance(.02)
        status = fixture(clock, state=reason if reason in {"CANCELLED", "SUCCEEDED"} else "TRACKING")
        if reason == "session_change":
            status["session_id"] = status["native_execution"]["session_id"] = "other"
        if reason == "task_change":
            status["task"]["task_id"] = status["native_execution"]["task_id"] = "other"
        assert not executor.submit_status(status)
        assert executor.snapshot()["latched_reason"] and not executor.snapshot()["active"]
        assert client.calls[-1] == ("authority", False)
        assert not executor.submit_status(fixture(clock)) and not executor.send_once()
        executor.close()
    checks["cancel_success_or_identity_change_latches_zero_and_release"] = True

    unhealthy = [None]
    def health():
        if unhealthy[0]:
            raise RuntimeError(unhealthy[0])
    clock, client, executor = setup(feedback=health)
    executor.send_once()
    unhealthy[0] = "feedback:remote_sticks_idle"
    executor.watchdog_once()
    assert executor.snapshot()["latched_reason"] and client.calls[-1] == ("authority", False)
    executor.close()
    checks["remote_takeover_releases_api_and_latches"] = True

    unhealthy[0] = None
    clock, client, executor = setup(feedback=health)
    executor.send_once()
    unhealthy[0] = "feedback:feedback_sport_fresh"
    executor.watchdog_once()
    assert not executor.snapshot()["active"] and not executor.snapshot()["latched_reason"]
    unhealthy[0] = None
    clock.advance(.03)
    assert executor.submit_status(fixture(clock)) and executor.send_once()
    executor.fetch_status = lambda: (_ for _ in ()).throw(TimeoutError("HTTP stalled"))
    assert not executor.poll_once() and not executor.snapshot()["active"]
    fence = executor.snapshot()["minimum_post_stop_source_ns"]
    clock.advance(.03)
    assert not executor.submit_status(fixture(clock, source=fence-1))
    assert executor.submit_status(fixture(clock)) and executor.send_once()
    executor.close()
    checks["feedback_or_http_loss_stops_and_requires_post_zero_source"] = True

    clock, client, executor = setup()
    executor.send_once()
    client.enabled = False
    clock.advance(.6)
    executor.watchdog_once()
    assert executor.snapshot()["fault"] and client.calls[-1] == ("authority", False)
    executor.close()
    clock, client, executor = setup()
    client.move_code = 9
    assert not executor.send_once()
    assert executor.snapshot()["fault"] and client.calls[-1] == ("authority", False)
    executor.close()
    checks["service_or_move_error_zero_release_without_sport_fallback"] = True

    clock, client = Clock(), FakeClient()
    client.acquire_code = 5
    executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                    fetch_status=lambda: fixture(clock), check_feedback=lambda: None,
                    wall_clock=lambda: clock.wall, mono_clock=lambda: clock.mono)
    try:
        executor.arm(fixture(clock))
        raise AssertionError("failed authority acquisition was accepted")
    except RuntimeError:
        pass
    assert executor.snapshot()["fault"] and client.calls[-1] == ("authority", False)
    assert not any(row[0] == "move" and any(row[1]) for row in client.calls)
    executor.close()
    checks["failed_arm_attempts_zero_and_authority_release"] = True

    clock, client, executor = setup()
    client.on_move = lambda: clock.advance(.3)
    assert not executor.send_once()
    assert not executor.snapshot()["active"]
    assert sum(row[0] == "move" and any(row[1]) for row in client.calls) == 1
    assert client.calls[-1] == ("move", (0., 0., 0.))
    executor.close()
    checks["late_move_return_rechecks_deadline_and_zeros"] = True

    clock, client, executor = setup()
    entered, release = threading.Event(), threading.Event()
    def block():
        entered.set()
        assert release.wait(1.)
    client.on_move = block
    sender = threading.Thread(target=executor.send_once)
    sender.start()
    assert entered.wait(1.)
    closer = threading.Thread(target=executor.close)
    closer.start()
    release.set()
    sender.join(1.); closer.join(1.)
    assert not sender.is_alive() and not closer.is_alive()
    count = len(client.calls)
    assert not executor.send_once() and not executor.submit_status(fixture(clock))
    assert len(client.calls) == count
    checks["close_is_serialized_and_no_post_close_nonzero"] = True

    # Run the real thread orchestration against fake RPCs, ending through a
    # terminal webpage status. No test imports a real SDK or creates a socket.
    client = FakeClient()
    began = time.monotonic()
    def live_status():
        clock = Clock()
        clock.wall = time.time_ns()
        clock.mono = time.monotonic_ns()
        return fixture(clock, state="CANCELLED" if time.monotonic()-began > .08 else "TRACKING")
    with tempfile.TemporaryDirectory(prefix="native-imagegoal-audit-") as directory:
        live_path = Path(directory)/"live.json"
        executor = NativeImageGoalExecutor(client, session_id="session-1", task_id="task-1",
                        fetch_status=live_status, check_feedback=lambda: None, max_duration_s=.5,
                        live_path=live_path)
        final = executor.run()
        live = json.loads(live_path.read_text())
        assert not live["running"] and live["state"] == "STOPPED" and not live["command_active"]
        assert live["counts"]["nonzero_requests"] > 0
    assert final["closed"] and final["latched_reason"] == "task_terminal:CANCELLED"
    assert final["counts"]["nonzero_requests"] > 0 and client.calls[-1] == ("authority", False)
    checks["threaded_run_observes_web_cancel_and_finishes"] = True
    checks["atomic_live_receipt_reports_final_stop"] = True

    return {"schema_version": "imagegoal.native_executor_audit.v1", "ok": all(checks.values()),
            "checks": checks, "sdk_initialized": False, "network_used": False,
            "hardware_motion_sent": False, "navigation_success_validated": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = audit()
    if args.output:
        args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
