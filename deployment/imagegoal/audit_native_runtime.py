"""Offline recorded-NoMaD -> real runtime/MPC -> native-contract integration.

The clock is replayed at recorded wall times. Original RGB and sensor stamps
are preserved. Only task/session identity is rebound for the offline fixture;
this audit does not run a GPU model, start an HTTP server, or import a robot SDK.
"""
from __future__ import annotations

import argparse
import base64
from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from unittest.mock import patch

import numpy as np

from . import runtime as runtime_module
from .runtime import ImageGoalRuntime, RuntimeConfig
from .audit_run import audit_run
from deployment.go2.execution_envelope import validate_envelope
from deployment.go2.native_imagegoal import validate_native_status
from deployment.go2.preflight import load_config


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RUN = ROOT/"runs/imagegoal_20261017T012852_905845Z/runtime"


class RecordedClock:
    def __init__(self, wall_ns, *, source_ns, source_mono_ns):
        self.wall_ns = wall_ns
        self.offset = source_mono_ns-source_ns

    def time_ns(self):
        return self.wall_ns

    def monotonic_ns(self):
        return self.wall_ns+self.offset

    def monotonic(self):
        return self.monotonic_ns()/1e9


class FakeManager:
    def __init__(self):
        self.running, self.starts, self.stops = False, [], []

    def status(self):
        return {"running": self.running, "state": "offline_fake", "task_id": None, "error": None}

    def start(self, task_id, runtime_url, duration_s):
        self.starts.append((task_id, runtime_url, duration_s))
        self.running = True

    def stop(self, reason):
        self.stops.append(reason)
        self.running = False


def _recording(directory, request_id):
    request_path = directory/"requests"/(request_id+".request.json")
    response_path = directory/"requests"/(request_id+".response.json")
    request = json.loads(request_path.read_text())
    response_record = json.loads(response_path.read_text())
    before = after = returned = None
    source = request["source_stamp_ns"]
    for line in (directory/"status.jsonl").open():
        status = json.loads(line)
        odom = status.get("sensors", {}).get("odom")
        if odom:
            stamp = odom["capture_timestamp_ns"]
            if stamp <= source and (before is None or stamp > before["capture_timestamp_ns"]):
                before = odom
            if stamp >= source and (after is None or stamp < after["capture_timestamp_ns"]):
                after = odom
        if returned is None and (status.get("preview") or {}).get("request_id") == request_id:
            returned = status
    if before is None or after is None or returned is None:
        raise ValueError("recorded capture-bracketing odom and response-time snapshot are required")
    assert source-before["capture_timestamp_ns"] < 150_000_000
    assert after["capture_timestamp_ns"]-source < 150_000_000
    goal = (directory/request["goal_path"]).read_bytes()
    frames = [(row, (directory/row["stored_path"]).read_bytes()) for row in request["frames"]]
    assert len(frames) == 4
    return {"request": request, "response_record": response_record, "before": before,
            "after": after, "returned": returned, "goal": goal, "frames": frames,
            "request_path": request_path, "response_path": response_path}


def _config(profile):
    return RuntimeConfig(profile="external", execution_profile=profile,
                operator_supervised=profile == "supervised-native", max_v_mps=.3,
                max_w_rad_s=.55, max_rgb_age_s=2., max_plan_age_s=2.,
                min_effective_v_mps=.1, min_effective_w_rad_s=.2)


def _build(output, profile, fixture, clock):
    clock.wall_ns = fixture["frames"][0][0]["capture_timestamp_ns"]-1
    runtime = ImageGoalRuntime(output, _config(profile))
    run_path = runtime.output/"run.json"
    run_record = json.loads(run_path.read_text())
    run_record.update(offline_fixture=True, gpu_inference_executed_this_audit=False,
                      identity_rebound_for_offline_test=True)
    run_path.write_text(json.dumps(run_record)+"\n")
    runtime._dispatch = lambda frames: None  # Never start an inference HTTP worker.
    goal = {"rgb_jpeg_base64": base64.b64encode(fixture["goal"]).decode()}
    runtime.create_task({"goal": goal, "backend": "nomad", "timeout_s": 60,
                         "execution_mode": "supervised_native" if profile == "supervised-native" else "shadow",
                         "expected_epoch": runtime.epoch})
    for row, blob in fixture["frames"]:
        clock.wall_ns = row["capture_timestamp_ns"]
        runtime.ingest_rgb({**row, "rgb_jpeg_base64": base64.b64encode(blob).decode()})
    clock.wall_ns = fixture["returned"]["timestamp_ns"]
    odom_rows = {row["capture_timestamp_ns"]: row for row in (
        fixture["before"], fixture["after"], fixture["returned"]["sensors"]["odom"])}
    for stamp in sorted(odom_rows):
        runtime.ingest_odom(odom_rows[stamp])
    newest = runtime.frames[-1]
    identity = {"session_id": runtime.session_id, "task_id": runtime.task["task_id"],
                "goal_id": runtime.task["goal_id"], "goal_version": runtime.task["goal_version"],
                "frame_id": newest.frame_id}
    payload = {"identity": identity, "observations": [frame.wire() for frame in runtime.frames], "goal": goal}
    metadata = copy.deepcopy(fixture["request"])
    metadata.update(request_id="offline-fixture-"+fixture["request"]["request_id"],
                    identity=identity, epoch=runtime.epoch, offline_fixture=True,
                    fixture_note="Recorded numeric output rebound to an offline task; no GPU inference in this audit")
    response = copy.deepcopy(fixture["response_record"]["response"])
    response["identity"] = identity
    (runtime.output/"requests"/(metadata["request_id"]+".request.json")).write_text(json.dumps(metadata)+"\n")
    runtime.completed.put((metadata, payload, response, fixture["response_record"]["round_trip_ms"], None))
    runtime.inflight = metadata
    runtime._event("offline_recorded_response_injected", original_request=str(fixture["request_path"]),
                   offline_fixture=True, gpu_inference_executed=False)
    runtime.tick()
    return runtime, goal


def audit(directory=DEFAULT_RUN, request_id="000094"):
    directory = Path(directory).resolve()
    fixture = _recording(directory, request_id)
    newest = fixture["frames"][-1][0]
    clock = RecordedClock(newest["capture_timestamp_ns"], source_ns=newest["capture_timestamp_ns"],
                          source_mono_ns=newest["provenance"]["received_monotonic_ns"])
    checks, rejected_physical = {}, []
    roots = []
    report = None

    def record_status(runtime, status):
        with (runtime.output/"status.jsonl").open("a") as stream:
            stream.write(json.dumps({**status, "offline_fixture": True})+"\n")

    def physical_rejected(status):
        result = validate_envelope(status["execution_envelope"], binding={}, config=load_config(), now_ns=clock.time_ns())
        assert not result.accepted, "old physical transport accepted a native/fixture envelope"
        rejected_physical.append(list(result.reasons))

    with tempfile.TemporaryDirectory(prefix="native-runtime-offline-") as temporary, \
         patch.object(runtime_module, "time", clock), \
         patch.object(runtime_module, "request_json", side_effect=AssertionError("HTTP forbidden in offline fixture")), \
         patch("deployment.imagegoal.native_control.subprocess.Popen", side_effect=AssertionError("Popen forbidden in offline fixture")):
        try:
            runtime, goal = _build(Path(temporary)/"native", "supervised-native", fixture, clock)
            roots.append(runtime)
            status = runtime.status()
            assert runtime.depth is None and runtime.plan is not None
            assert runtime.metrics["mpc_solves"] == 1 and runtime.metrics["mpc_errors"] == 0
            assert status["native_execution"]["authorized"], status["native_execution"]["blocking_reasons"]
            decision = validate_native_status(status, session_id=runtime.session_id,
                                              task_id=runtime.task["task_id"], now_ns=clock.time_ns())
            assert decision["accepted"], decision["reasons"]
            vx, _, wz = decision["command"]
            assert vx == 0 or .1 <= abs(vx) <= .3
            assert wz == 0 or .2 <= abs(wz) <= .55
            assert abs(vx)+abs(wz) > 0
            assert runtime.plan["capture_pose"].tolist() == fixture["request"]["capture_pose"]
            bracket = runtime_module.pose_at_stamp([item[0] for item in runtime.odom],
                        fixture["request"]["source_stamp_ns"], runtime.config.max_capture_odom_gap_s)
            assert bracket is not None
            bracket_pose = [bracket[0].x, bracket[0].y, bracket[0].yaw]
            delta = np.asarray(bracket_pose)-np.asarray(fixture["request"]["capture_pose"])
            assert np.linalg.norm(delta[:2]) < .005 and abs(math.atan2(math.sin(delta[2]),math.cos(delta[2]))) < .01
            assert status["plan"]["trajectory_odom_m_rad"] and status["plan"]["prediction_odom_m_rad"]
            physical_rejected(status)
            record_status(runtime, status)
            checks["recorded_rgb_nomad_real_mpc_without_depth_authorizes_native_only"] = True
            checks["effective_speed_floors_and_engineering_caps"] = True
            command = dict(status["native_execution"]["command"])

            # These separate fault cases preserve every recorded timestamp.
            # Moving only the virtual audit clock tests staleness without
            # relabeling an old sample as new real feedback.
            saved_odom = deque(runtime.odom, maxlen=runtime.odom.maxlen)
            runtime.odom.clear()
            missing = runtime.status()
            assert not missing["native_execution"]["authorized"]
            assert "odom_missing" in missing["native_execution"]["blocking_reasons"]
            physical_rejected(missing)
            runtime.odom = saved_odom
            clock.wall_ns += 400_000_000
            stale = runtime.status()
            assert not stale["native_execution"]["authorized"]
            assert "odom_stale_or_future" in stale["native_execution"]["blocking_reasons"]
            physical_rejected(stale)
            clock.wall_ns -= 400_000_000
            checks["missing_and_old_odom_revoke_native_authority"] = True

            original_source = runtime.frames[-1].metadata["timestamp_source"]
            runtime.frames[-1].metadata["timestamp_source"] = "offline_invalid_camera_source_fixture"
            wrong = runtime.status()
            assert not wrong["native_execution"]["authorized"]
            assert "native_rgb_source_mismatch" in wrong["native_execution"]["blocking_reasons"]
            physical_rejected(wrong)
            runtime.frames[-1].metadata["timestamp_source"] = original_source
            checks["wrong_native_camera_provenance_revokes_authority"] = True

            paused = runtime.task_action("pause", {})
            assert not paused["native_execution"]["authorized"] and paused["state"] == "PAUSED"
            physical_rejected(paused)
            record_status(runtime, paused)
            runtime.task_action("resume", {})
            changed = runtime.replace_goal({"goal": goal, "expected_epoch": runtime.epoch})
            assert not changed["native_execution"]["authorized"] and runtime.plan is None
            record_status(runtime, changed)
            cancelled = runtime.task_action("cancel", {})
            assert not cancelled["native_execution"]["authorized"] and cancelled["state"] == "CANCELLED"
            physical_rejected(cancelled)
            record_status(runtime, cancelled)
            checks["pause_goal_change_and_cancel_revoke_native_authority"] = True

            shadow, _ = _build(Path(temporary)/"shadow", "shadow", fixture, clock)
            roots.append(shadow)
            shadow_status = shadow.status()
            assert shadow.plan is None and "depth_missing" in shadow_status["gates"]
            assert not shadow_status["native_execution"]["authorized"]
            try:
                shadow.create_task({"goal": goal, "backend": "nomad", "execution_mode": "supervised_native",
                                    "expected_epoch": shadow.epoch})
                raise AssertionError("shadow task elevated itself into native execution")
            except ValueError:
                pass
            physical_rejected(shadow_status)
            record_status(shadow, shadow_status)
            checks["ordinary_shadow_still_requires_depth_and_cannot_elevate"] = True

            # A fake manager exercises the exact web Start/Stop barrier while
            # a patched Popen makes an accidental real spawn a failing test.
            manager = FakeManager()
            runtime.native_executor = manager
            clock.wall_ns = fixture["returned"]["timestamp_ns"]
            stale_epoch = runtime.epoch-1
            start = {"operator_present": True, "backend": "nomad", "goal": goal, "timeout_s": 60,
                     "expected_session_id": runtime.session_id, "expected_epoch": stale_epoch}
            try:
                runtime.start_native(start, runtime_url="http://127.0.0.1:8093")
                raise AssertionError("stale start crossed the stop epoch barrier")
            except ValueError:
                pass
            assert not manager.starts
            old_task = runtime.task["task_id"]
            start["expected_epoch"] = runtime.epoch
            runtime.start_native(start, runtime_url="http://127.0.0.1:8093")
            assert len(manager.starts) == 1
            new_task = runtime.task["task_id"]
            stops_before = len(manager.stops)
            try:
                runtime.stop_native({"expected_session_id": runtime.session_id, "expected_task_id": old_task})
                raise AssertionError("old-task stop cancelled a replacement task")
            except ValueError:
                pass
            assert runtime.task["task_id"] == new_task and runtime.state not in {"CANCELLED", "FAILED"}
            assert len(manager.stops) == stops_before, "old-task stop signalled the replacement executor"
            epoch_before_stop = runtime.epoch
            stopped = runtime.stop_native({"expected_session_id": runtime.session_id,
                          "expected_task_id": new_task, "expected_epoch": stale_epoch})
            assert stopped["state"] == "CANCELLED" and runtime.epoch > epoch_before_stop
            physical_rejected(stopped)
            record_status(runtime, stopped)
            checks["start_stop_epoch_barrier_and_expected_task_binding"] = True

            report = {"schema_version": "imagegoal.native_runtime_audit.v1", "ok": all(checks.values()),
                    "offline_fixture": True, "checks": checks, "sdk_initialized": False,
                    "hardware_motion_sent": False, "http_server_started": False,
                    "gpu_inference_executed_this_audit": False, "navigation_success_validated": False,
                    "model_output_source": str(fixture["response_path"]),
                    "original_request": str(fixture["request_path"]),
                    "recorded_replay_wall_timestamp_ns": fixture["returned"]["timestamp_ns"],
                    "recorded_source_timestamp_ns": fixture["request"]["source_stamp_ns"],
                    "recorded_response_rtt_ms": fixture["response_record"]["round_trip_ms"],
                    "rgb_sha256": [hashlib.sha256(blob).hexdigest() for _, blob in fixture["frames"]],
                    "goal_sha256": hashlib.sha256(fixture["goal"]).hexdigest(),
                    "capture_pose": fixture["request"]["capture_pose"],
                    "pose_interpolated_from_saved_status_samples": bracket_pose,
                    "capture_bracketing_odom_stamps_ns": [fixture[name]["capture_timestamp_ns"] for name in ("before", "after")],
                    "mpc_source_sha256": runtime.core.source_sha256,
                    "mpc_solves": runtime.metrics["mpc_solves"], "native_candidate_command": command,
                    "old_physical_validator_rejections": rejected_physical,
                    "limitations": ["Recorded NoMaD numbers are reused, not recomputed by a GPU in this audit.",
                       "Original sensor timestamps are preserved under a mocked recording-time clock.",
                       "Rebound task identities and fake manager are offline test fixtures.",
                       "No independent camera timing/extrinsic or physical navigation validation is claimed."]}
            return report
        finally:
            for runtime in roots:
                runtime.close()
            if report is not None:
                artifacts = [audit_run(runtime.output) for runtime in roots]
                assert all(item["ok"] for item in artifacts), [item["errors"] for item in artifacts]
                assert artifacts[0]["native_authorized_statuses"] == 1
                report["artifact_audits"] = artifacts
                checks["native_and_shadow_artifact_audits_pass"] = True
                report["ok"] = all(checks.values())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recorded-run", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--request-id", default="000094")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = audit(args.recorded_run, args.request_id)
    if args.output:
        args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    raise SystemExit(main())
