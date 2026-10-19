"""Exercise official LightNav instruction/history/reset/STOP contracts over HTTP.

Uses real recorded source RGB; every model request and response is retained.
This is a deployment acceptance driver, not a navigation success benchmark.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import sys
from pathlib import Path
import time
import urllib.error
import urllib.request
import uuid

from .client import jpeg
from .cleanup import reset_owned_session
from .policy import MODEL_REVISION, MODEL_SHA256, OFFICIAL_RELEASE_COMMIT, SERVER_IMPLEMENTATION, OFFICIAL_CORE_DIGEST


def request_json(url, payload=None, timeout=120):
    request = urllib.request.Request(url, data=None if payload is None else json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        return error.code, json.load(error)


def inspect_response(response, request):
    observations = [{key: frame[key] for key in ("frame_id", "capture_timestamp_ns")} for frame in request["observations"]]
    assert response["schema_version"] == "lightnav.imagegoal.inference.v1"
    assert response["identity"] == request["identity"]
    assert response["observation_frames"] == observations
    assert response["source_frame"] == observations[-1]
    assert response["conditioning_mode"] == request["conditioning_mode"]
    assert response["native_imagegoal_support"] is False
    assert response["implementation"] == "official_release"
    assert response["official_release_commit"] == OFFICIAL_RELEASE_COMMIT
    assert response["official_core_sha256"] == OFFICIAL_CORE_DIGEST
    assert response["task_mode"] == request.get("task_mode", "tracking" if request["conditioning_mode"] == "native_tracking" else "objnav")
    assert response["goal_image_consumed"] is False
    assert response["goal_encoding"] is None
    assert response["arrival_confirmed"] is False
    trajectory = response["trajectory_body_m_rad"]
    assert len(trajectory) == 10 and all(len(row) == 3 for row in trajectory)
    assert all(type(value) in (int, float) and math.isfinite(value) for row in trajectory for value in row)
    assert response["trajectory_body_m"] == [row[:2] for row in trajectory]
    assert type(response["model_stop"]) is bool
    assert response["model_stop"] == all(value == 0 for row in trajectory for value in row)
    assert isinstance(response["raw_text"], str) and all(f"<act_l{i}_" in response["raw_text"] for i in range(3))
    assert "Separate reference goal view" not in response["prompt"]
    return {"identity_and_history_echo": True, "metric_se2_10x3_finite": True,
            "goal_consumption_declared": True, "stop_equals_zero_action_not_arrival": True,
            "official_three_rvq_levels_present": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--alternate-instruction", default="Go to the doorway and stop.")
    parser.add_argument("--tracking-instruction", default="Follow the person wearing a white T-shirt.")
    parser.add_argument("--server-url", default="http://127.0.0.1:18884")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=120)
    args = parser.parse_args()
    if not args.instruction.strip() or not args.alternate_instruction.strip() or not args.tracking_instruction.strip():
        parser.error("Instructions must be nonempty")
    if args.instruction == args.alternate_instruction:
        parser.error("Alternate instruction must differ from --instruction")
    args.output.mkdir(parents=True, exist_ok=False)
    source = json.loads(args.manifest.read_text())
    entries = source["frames"]
    if len(entries) < 8:
        raise ValueError("At least eight distinct real source frames are required")
    frames, sources = [], []
    for value in entries[:8]:
        if not value.get("timestamp_source", source.get("timestamp_source")):
            raise ValueError("Source capture timestamp provenance is required")
        path = args.manifest.parent / value["path"]
        blob = jpeg(path)
        frames.append({"frame_id": str(value["frame_id"]), "capture_timestamp_ns": value["capture_timestamp_ns"],
                       "rgb_jpeg_base64": base64.b64encode(blob).decode()})
        sources.append({"path": str(path.resolve()), "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "jpeg_sha256": hashlib.sha256(blob).hexdigest(),
                        "frame_id": frames[-1]["frame_id"], "capture_timestamp_ns": frames[-1]["capture_timestamp_ns"]})
    if len({frame["frame_id"] for frame in frames}) != 8 or any(a["capture_timestamp_ns"] >= b["capture_timestamp_ns"] for a,b in zip(frames,frames[1:])):
        raise ValueError("Real source frames require unique IDs and strictly increasing stamps")
    instructions = [args.instruction, args.alternate_instruction]
    goal_ids = [hashlib.sha256(value.encode()).hexdigest() for value in instructions]
    base = args.server_url.rstrip("/")
    session = "lightnav_commission_" + uuid.uuid4().hex
    run = {"session_id": session, "source_manifest": str(args.manifest.resolve()), "frames": sources,
           "instructions": instructions, "instruction_sha256": goal_ids, "implementation": "official_release",
           "not_for_motion": True, "navigation_validation": False, "robot_feedback_used": False,
           "purpose": "actual_model_contract_acceptance_on_recorded_camera_RGB"}
    (args.output / "run.json").write_text(json.dumps(run, indent=2) + "\n")
    cases, stop_observed = [], False

    def payload(task, observations, version=0, goal_index=0, mode="native_language", instruction=None):
        return {"identity": {"session_id": session, "task_id": task, "goal_id": goal_ids[goal_index],
                             "goal_version": version, "frame_id": observations[-1]["frame_id"]},
                "observations": observations, "task_mode": "tracking" if mode == "native_tracking" else "objnav",
                "conditioning_mode": mode, "instruction": instruction or instructions[goal_index]}

    def call(name, value, expected=200, endpoint="/infer"):
        nonlocal stop_observed
        (args.output / f"{name}.request.json").write_text(json.dumps(value) + "\n")
        started = time.monotonic()
        status, response = request_json(base + endpoint, value, args.timeout_s)
        result = {"case": name, "http_status": status, "expected_status": expected,
                  "round_trip_ms": (time.monotonic() - started) * 1000, "response": response}
        (args.output / f"{name}.response.json").write_text(json.dumps(result, indent=2) + "\n")
        assert status == expected, (name, status, response)
        if status == 200 and endpoint == "/infer":
            result["checks"] = inspect_response(response, value)
            stop_observed |= response["model_stop"]
        cases.append(result)
        print(json.dumps({"case": name, "http_status": status, "round_trip_ms": result["round_trip_ms"],
                          "raw_text": response.get("raw_text"), "history_count": response.get("policy_history_count")}), flush=True)
        return response

    try:
        status, health = request_json(base + "/healthz", timeout=args.timeout_s)
        assert status == 200 and health["ok"] and health["model_loaded"]
        contract = health["contract"]
        assert contract["model_revision"] == MODEL_REVISION and contract["model_sha256"] == MODEL_SHA256
        assert contract["backend"] == "hf" and contract["native_imagegoal_support"] is False
        assert contract["implementation"] == SERVER_IMPLEMENTATION and contract["official_release_commit"] == OFFICIAL_RELEASE_COMMIT
        assert contract["official_core_sha256"] == OFFICIAL_CORE_DIGEST and contract["imagegoal_adaptation_support"] is True
        assert set(contract["supported_conditioning_modes"]) == {"native_language", "native_tracking", "experimental_reference_image"}
        (args.output / "health.json").write_text(json.dumps(health, indent=2) + "\n")
        call("malformed_identity_rejected", {"identity": {}}, expected=400)
        native = payload("native_go", frames[:4], mode="native_language", instruction="Move forward a short distance.")
        native_response = call("native_language", native)
        assert native_response["policy_history_count"] == 4
        call("native_cleanup", {"identity": {"session_id": session, "task_id": "native_go"}}, endpoint="/reset")
        stopping = payload("native_stop", frames[:4], mode="native_language", instruction="Stop immediately and remain stationary.")
        stop_response = call("native_stop_instruction", stopping)
        call("stop_cleanup", {"identity": {"session_id": session, "task_id": "native_stop"}}, endpoint="/reset")
        tracking = payload("tracking", frames[:4], mode="native_tracking", instruction=args.tracking_instruction)
        call("native_tracking", tracking)
        call("tracking_cleanup", {"identity": {"session_id": session, "task_id": "tracking"}}, endpoint="/reset")
        initial = payload("navigation", frames[:4])
        initial_response = call("instruction_navigation", initial)
        assert initial_response["policy_history_count"] == 4 and initial_response["policy_history_frames_appended"] == 4
        call("goal_image_rejected", {**initial, "goal": {"rgb_jpeg_base64": frames[0]["rgb_jpeg_base64"]}}, expected=400)
        call("adapter_missing_goal_rejected", {**initial, "conditioning_mode": "experimental_reference_image", "task_mode": "imagegoal"}, expected=400)
        call("missing_instruction_rejected", {**initial, "instruction": None}, expected=400)
        duplicate = call("exact_history_retry", initial)
        assert duplicate["policy_history_count"] == 4 and duplicate["policy_history_frames_appended"] == 0
        overlap = payload("navigation", frames[1:5])
        appended = call("overlapping_history_append", overlap)
        assert appended["policy_history_count"] == 5 and appended["policy_history_frames_appended"] == 1
        call("old_newest_frame_rejected", initial, expected=409)
        conflict = payload("navigation", frames[1:5])
        conflict["instruction"] = instructions[1]
        call("instruction_without_generation_rejected", conflict, expected=409)
        switched = payload("navigation", frames[2:6], version=1, goal_index=1)
        changed = call("new_goal_generation", switched)
        assert changed["policy_history_count"] == 4 and changed["policy_history_reset_reason"] == "new_goal_generation"
        call("late_old_goal_generation_rejected", payload("navigation", frames[3:7]), expected=409)
        reset = call("explicit_session_reset", {"identity": {"session_id": session, "task_id": "navigation"}}, endpoint="/reset")
        assert reset["sessions_cleared"] == 1
        restarted = call("history_after_reset", payload("navigation", frames[4:8], version=1, goal_index=1))
        assert restarted["policy_history_count"] == 4 and restarted["policy_history_reset_reason"] == "new_task_session"
        summary = {"actual_model_inference_cases": sum("checks" in item for item in cases),
                   "all_contract_cases_passed": True, "cases_completed": len(cases),
                   "native_stop_instruction_produced_stop": stop_response["model_stop"],
                   "model_stop_observed_in_real_inference": stop_observed,
                   "not_for_motion": True, "navigation_validation": False,
                   "implementation": "official_release", "native_imagegoal_support": False}
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)
    finally:
        failed = sys.exc_info()[0] is not None
        def cleanup_post(url, payload, timeout):
            status, response = request_json(url, payload, timeout)
            if status != 200:
                raise urllib.error.HTTPError(url, status, str(response), {}, None)
            return response
        cleanup = reset_owned_session(base, session, args.timeout_s, cleanup_post)
        (args.output / "final_cleanup.json").write_text(json.dumps(cleanup, indent=2) + "\n")
        if not cleanup["ok"]:
            message = "LightNav commissioning session cleanup failed; see final_cleanup.json"
            if failed:
                print(message, file=sys.stderr)
            else:
                raise RuntimeError(message)


if __name__ == "__main__":
    main()
