"""Offline regression for post-solver latency failure diagnostics.

Reuse a recorded LightNav numeric response with fixture RGB, odometry and
clock. The actual runtime, MPC and latency guard run; no model, network,
subprocess or robot SDK is used. Artificial solver time is explicit.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import socket
import sys
import tempfile
from unittest.mock import patch

from . import runtime as runtime_module
from .audit_lightnav_native import DEFAULT_RESPONSE, _config, _goal, _inject, _request, _seed
from .audit_native_runtime import RecordedClock
from .runtime import ImageGoalRuntime
from deployment.go2.native_imagegoal import NATIVE_SOURCES, validate_native_status


def _fresh_sensors(runtime, clock, goal):
    common = {"capture_timestamp_ns": clock.time_ns(), "clock_domain": "unix_epoch_ns",
              "synthetic": False, "commissioning": False}
    frame, source = NATIVE_SOURCES["rgb"]
    runtime.ingest_rgb({**common, **goal, "frame_id": "offline-rgb-after-solver",
                        "frame_name": frame, "timestamp_source": source,
                        "provenance": {"offline_fixture": True}})
    frame, source = NATIVE_SOURCES["odom"]
    runtime.ingest_odom({**common, "frame_name": frame, "timestamp_source": source,
                         "body_frame_name": "base_link", "pose_m_rad": [1., 2., .3],
                         "velocity_v_w": [0., 0.]})


def audit(response_path=DEFAULT_RESPONSE):
    response_path = Path(response_path).resolve()
    response = json.loads(response_path.read_text())["response"]
    assert response["model_stop"] is False
    clock = RecordedClock(1_800_000_000_000_000_000, source_ns=0,
                          source_mono_ns=10_000_000_000)
    goal, checks, cases = _goal(), {}, {}
    before_sdk = {name for name in sys.modules if name.startswith("unitree_sdk2py")}
    with tempfile.TemporaryDirectory(prefix="post-solver-failure-offline-") as temporary, \
         patch.object(runtime_module, "time", clock), \
         patch.object(runtime_module, "request_json",
                      side_effect=AssertionError("offline audit must not use HTTP")), \
         patch.object(ImageGoalRuntime, "_schedule_lightnav_cleanup"), \
         patch.object(socket, "socket",
                      side_effect=AssertionError("offline audit must not open sockets")), \
         patch("subprocess.Popen",
               side_effect=AssertionError("offline audit must not start a subprocess")):
        for refresh in (True, False):
            name = "fresh_sensors" if refresh else "expired_sensors"
            runtime = ImageGoalRuntime(Path(temporary)/name, _config())
            runtime._dispatch = lambda frames: None
            try:
                runtime.create_task({**_request(runtime, goal),
                                     "execution_mode": "supervised_native"})
                _seed(runtime, clock, goal)
                normal = _inject(runtime, clock, response, request_id="offline-normal-"+name)
                assert normal["state"] == "TRACKING" and runtime.metrics["mpc_solves"] == 1
                assert normal["native_execution"]["authorized"] and runtime.depth is None
                decision = validate_native_status(normal, session_id=runtime.session_id,
                           task_id=runtime.task["task_id"], now_ns=clock.time_ns())
                assert decision["accepted"] and any(decision["command"]), decision["reasons"]
                checks["normal_native_real_mpc_and_latency_guard_remain_tracking"] = True

                source = runtime.plan["source_stamp_ns"]
                epoch = runtime.epoch
                original_solve = runtime.controller.solve
                original_apply = runtime.latency_guard.apply
                original_safety_result = runtime.control_module.SafetyResult
                observed = {"solver_calls": 0, "native_success_labels": []}

                def slow_solve(*args, **kwargs):
                    observed["solver_calls"] += 1
                    observed["pre_solver_plan_age_s"] = (clock.time_ns()-source)/1e9
                    observed["pre_solver_sensor_gates"] = runtime._sensor_gates(clock.time_ns())
                    assert 0 <= observed["pre_solver_plan_age_s"] < runtime.config.max_plan_age_s
                    assert observed["pre_solver_sensor_gates"] == []
                    result = original_solve(*args, **kwargs)
                    # Simulate elapsed solve time; do not alter the actual
                    # solver's numeric result or bypass the real age guard.
                    clock.wall_ns += int((runtime.config.max_plan_age_s+.1)*1e9)
                    if refresh:
                        _fresh_sensors(runtime, clock, goal)
                    return result

                def observe_guard(*args, **kwargs):
                    result = original_apply(*args, **kwargs)
                    observed["guard_reason"] = result.reason
                    observed["guard_source_age_s"] = result.plan_input_age_s
                    return result

                def observe_safety(*args, **kwargs):
                    observed["native_success_labels"].append(kwargs.get("reason"))
                    return original_safety_result(*args, **kwargs)

                with patch.object(runtime.controller, "solve", side_effect=slow_solve), \
                     patch.object(runtime.latency_guard, "apply", side_effect=observe_guard), \
                     patch.object(runtime.control_module, "SafetyResult", side_effect=observe_safety):
                    runtime.tick()
                failed = runtime.status()
                events = [row for row in runtime.events if row["kind"] == "execution_invalidated"]
                event = events[-1]
                assert observed["solver_calls"] == 1
                assert observed["guard_reason"] == "plan_input_too_old_hold"
                assert observed["guard_source_age_s"] > runtime.config.max_plan_age_s
                assert observed["native_success_labels"] == ["native_avoidance_executor_required"]
                assert "plan_input_too_old_hold" in failed["gates"]
                assert "native_avoidance_executor_required" not in failed["gates"]
                assert "native_avoidance_executor_required" not in event["reason"]
                assert event["reason"].startswith("post_solver_gate:plan_input_too_old_hold")
                assert not failed["native_execution"]["authorized"] and runtime.plan is None
                assert runtime.epoch == epoch+1 and runtime.metrics["mpc_errors"] == 0
                assert runtime.metrics["mpc_solves"] == 1
                assert all(failed["proposal"][key] == 0
                           for key in ("vx_m_s", "vy_m_s", "wz_rad_s"))
                if refresh:
                    assert failed["gates"] == ["plan_input_too_old_hold"]
                    assert event["reason"] == "post_solver_gate:plan_input_too_old_hold"
                    assert runtime._sensor_gates(clock.time_ns()) == []
                    checks["post_solver_source_age_failure_is_not_hidden_by_native_success_label"] = True
                else:
                    assert "rgb_stale_or_future" in failed["gates"]
                    assert "odom_stale_or_future" in failed["gates"]
                    assert "plan_input_too_old_hold" in event["reason"]
                    assert "rgb_stale_or_future" in event["reason"]
                    assert "odom_stale_or_future" in event["reason"]
                    checks["simultaneous_latency_and_sensor_failures_are_all_reported"] = True
                checks["post_solver_expiry_invalidates_plan_and_emits_zero_without_success"] = True
                cases[name] = {**observed, "gates": failed["gates"],
                               "invalidation_event": event,
                               "state": failed["state"], "proposal": failed["proposal"]}
            finally:
                runtime.close()
    assert {name for name in sys.modules if name.startswith("unitree_sdk2py")} == before_sdk
    checks["no_network_subprocess_model_or_robot_sdk"] = True
    return {"schema_version": "imagegoal.post_solver_failure_audit.v1",
            "ok": all(checks.values()), "checks": checks, "cases": cases,
            "offline_fixture": True, "synthetic_sensor_and_identity_fixture": True,
            "simulated_solver_elapsed_time": True, "actual_mpc_and_latency_guard": True,
            "recorded_model_response": str(response_path),
            "recorded_model_response_sha256": hashlib.sha256(response_path.read_bytes()).hexdigest(),
            "gpu_inference_executed": False, "sdk_initialized": False,
            "network_used": False, "hardware_motion_sent": False,
            "navigation_success_validated": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--response-fixture", type=Path, default=DEFAULT_RESPONSE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = audit(args.response_fixture)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
