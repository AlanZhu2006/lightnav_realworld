"""Replay real RGB histories through a selected native or ImageGoal LightNav profile.

This is a model/client verification tool. It does not publish robot commands.
Use deployment.imagegoal for the shared task lifecycle and MPC shadow runtime.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import statistics
import sys
import time
import urllib.request
import uuid

from .cleanup import reset_owned_session
from .policy import DEFAULT_INSTRUCTION, IMAGEGOAL_IMPLEMENTATION, OFFICIAL_RELEASE_COMMIT, OFFICIAL_CORE_DIGEST


def jpeg(path):
    from PIL import Image
    with Image.open(path) as image:
        output = io.BytesIO()
        image.convert("RGB").save(output, "JPEG", quality=95)
    return output.getvalue()


def post(url, payload, timeout):
    request = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--server-url", default="http://127.0.0.1:18884")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--history-frames", type=int, default=16)
    parser.add_argument("--minimum-frames", type=int, default=4)
    parser.add_argument("--max-requests", type=int, default=0)
    parser.add_argument("--instruction")
    parser.add_argument("--goal", type=Path, help="reference photo; only with experimental_reference_image")
    parser.add_argument("--conditioning-mode", choices=["native_language", "native_tracking", "experimental_reference_image"], default="native_language")
    parser.add_argument("--task-mode", choices=["objnav", "vln", "tracking", "imagegoal"])
    parser.add_argument("--timeout-s", type=float, default=120)
    args = parser.parse_args()
    if not 1 <= args.minimum_frames <= args.history_frames <= 64:
        parser.error("Require 1 <= minimum <= history <= 64")
    if args.max_requests < 0:
        parser.error("max-requests cannot be negative")
    imagegoal = args.conditioning_mode == "experimental_reference_image"
    if imagegoal != (args.goal is not None):
        parser.error("--goal is required only for explicit experimental_reference_image mode")
    if not imagegoal and not (args.instruction and args.instruction.strip()):
        parser.error("Native modes require a nonempty --instruction")
    task_mode = args.task_mode or ("imagegoal" if imagegoal else "tracking" if args.conditioning_mode == "native_tracking" else "objnav")
    allowed = {"imagegoal"} if imagegoal else {"tracking"} if args.conditioning_mode == "native_tracking" else {"objnav", "vln"}
    if task_mode not in allowed:
        parser.error("--task-mode and --conditioning-mode disagree")
    instruction = args.instruction if args.instruction and args.instruction.strip() else DEFAULT_INSTRUCTION
    implementation = IMAGEGOAL_IMPLEMENTATION if imagegoal else "official_release"
    goal_bytes = jpeg(args.goal) if imagegoal else None
    args.output.mkdir(parents=True, exist_ok=False)
    source = json.loads(args.manifest.read_text())
    entries = source["frames"]
    if len(entries) < args.minimum_frames:
        raise ValueError("Insufficient real source frames")
    frames, lineage = [], []
    for value in entries:
        path = args.manifest.parent / value.get("path", value.get("file", ""))
        if not path.is_file():
            raise ValueError(f"Missing source frame: {path}")
        if not value.get("timestamp_source", source.get("timestamp_source")):
            raise ValueError("Capture timestamp provenance is required")
        stamp, fid = value.get("capture_timestamp_ns"), str(value.get("frame_id", ""))
        if type(stamp) is not int or stamp <= 0 or not fid:
            raise ValueError("Invalid source frame lineage")
        encoded = jpeg(path)
        frames.append({"frame_id": fid, "capture_timestamp_ns": stamp,
                       "rgb_jpeg_base64": base64.b64encode(encoded).decode()})
        lineage.append({"frame_id": fid, "capture_timestamp_ns": stamp, "path": str(path.resolve()),
                        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "jpeg_sha256": hashlib.sha256(encoded).hexdigest()})
    if len({f["frame_id"] for f in frames}) != len(frames) or any(a["capture_timestamp_ns"] >= b["capture_timestamp_ns"] for a, b in zip(frames, frames[1:])):
        raise ValueError("Source history must be strictly chronological with distinct IDs")
    instruction_digest = hashlib.sha256(instruction.encode()).hexdigest()
    goal_digest = hashlib.sha256(goal_bytes).hexdigest() if goal_bytes else None
    generation_id = hashlib.sha256((instruction_digest + (goal_digest or "")).encode()).hexdigest()
    session = "lightnav_" + uuid.uuid4().hex
    run = {"mode": "recorded_model_verification", "not_for_motion": True,
           "session_id": session, "task_id": "recorded_lightnav",
           "navigation_validation": False, "robot_feedback_used": False,
           "implementation": implementation, "task_mode": task_mode,
           "conditioning_mode": args.conditioning_mode, "source_manifest": str(args.manifest.resolve()),
           "instruction": instruction, "instruction_sha256": instruction_digest,
           "goal_path": str(args.goal.resolve()) if args.goal else None, "goal_jpeg_sha256": goal_digest,
           "goal_source_sha256": hashlib.sha256(args.goal.read_bytes()).hexdigest() if args.goal else None,
           "frames": lineage}
    (args.output / "run.json").write_text(json.dumps(run, indent=2) + "\n")
    try:
        latencies = []
        for end in range(args.minimum_frames, len(frames) + 1):
            observations = frames[max(0, end - args.history_frames):end]
            identity = {"session_id": session, "task_id": "recorded_lightnav",
                        "goal_id": generation_id, "goal_version": 0,
                        "frame_id": observations[-1]["frame_id"]}
            payload = {"identity": identity, "observations": observations, "task_mode": task_mode,
                       "instruction": instruction, "conditioning_mode": args.conditioning_mode}
            if imagegoal:
                payload["goal"] = {"rgb_jpeg_base64": base64.b64encode(goal_bytes).decode()}
            start = time.monotonic()
            response = post(args.server_url.rstrip("/") + "/infer", payload, args.timeout_s)
            rtt = (time.monotonic() - start) * 1000
            if response.get("identity") != identity or response.get("source_frame") != {k: observations[-1][k] for k in ("frame_id", "capture_timestamp_ns")}:
                raise ValueError("Response lineage mismatch")
            if (response.get("implementation") != implementation or response.get("goal_image_consumed") is not imagegoal
                    or response.get("official_release_commit") != OFFICIAL_RELEASE_COMMIT
                    or response.get("official_core_sha256") != OFFICIAL_CORE_DIGEST):
                raise ValueError("Server does not match the selected LightNav profile and official core")
            trajectory = response.get("trajectory_body_m_rad")
            if not isinstance(trajectory, list) or len(trajectory) != 10:
                raise ValueError("Invalid trajectory response")
            row = {"request_index": len(latencies), "round_trip_ms": rtt, "response": response,
                   "not_for_motion": True, "navigation_validation": False}
            with (args.output / "results.jsonl").open("a") as stream:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
            latencies.append(rtt)
            print(json.dumps({"requests": len(latencies), "frame_id": identity["frame_id"],
                              "round_trip_ms": rtt, "model_stop": response.get("model_stop"),
                              "raw_text": response.get("raw_text")} ), flush=True)
            if args.max_requests and len(latencies) >= args.max_requests:
                break
        summary = {"requests_completed": len(latencies), "median_round_trip_ms": statistics.median(latencies),
                   "min_round_trip_ms": min(latencies), "max_round_trip_ms": max(latencies),
                   "not_for_motion": True, "navigation_validation": False,
                   "conditioning_mode": args.conditioning_mode}
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary))
    finally:
        failed = sys.exc_info()[0] is not None
        cleanup = reset_owned_session(args.server_url, session, args.timeout_s, post)
        (args.output / "final_cleanup.json").write_text(json.dumps(cleanup, indent=2) + "\n")
        if not cleanup["ok"]:
            message = "LightNav private session cleanup failed; see final_cleanup.json"
            if failed:
                print(message, file=sys.stderr)
            else:
                raise RuntimeError(message)


if __name__ == "__main__":
    main()
