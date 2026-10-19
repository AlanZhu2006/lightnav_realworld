#!/usr/bin/env python3
"""Visualize recorded NoMaD shadow results; no inference or robot interfaces.

Only recorded images, paths and status are published under /nomad/shadow/.
All paths have the synthetic display frame nomad_shadow. No TF, odometry,
velocity commands or services are published. --validate-only needs no ROS.
"""

from __future__ import annotations

import argparse
from functools import lru_cache
import hashlib
import io
import json
import math
from pathlib import Path
from typing import Any


def load_json(text: str) -> Any:
    def reject(token: str) -> None:
        raise ValueError(f"nonfinite JSON token: {token}")
    def finite_float(token: str) -> float:
        number = float(token)
        if not math.isfinite(number):
            raise ValueError(f"nonfinite JSON number: {token}")
        return number
    return json.loads(text, parse_constant=reject, parse_float=finite_float)


def points(value: Any, columns: int, label: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) > 10000:
        raise ValueError(f"{label} must be an array of at most 10000 points")
    result = []
    for row in value:
        if not isinstance(row, list) or len(row) != columns:
            raise ValueError(f"{label} requires {columns} columns")
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in row):
            raise ValueError(f"{label} has nonfinite or nonnumeric coordinates")
        result.append([float(x) for x in row])
    return result


def file_sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_recording(manifest_path: Path, goal_path: Path, results_path: Path) -> dict[str, Any]:
    manifest_path = manifest_path.expanduser().resolve(strict=True)
    goal_path = goal_path.expanduser().resolve(strict=True)
    results_path = results_path.expanduser().resolve(strict=True)
    manifest = load_json(manifest_path.read_text())
    if not isinstance(manifest, dict) or not isinstance(manifest.get("frames"), list):
        raise ValueError("manifest must contain a frames array")
    if results_path.stat().st_size > 128 * 1024 * 1024:
        raise ValueError("results file exceeds the 128 MiB visualization limit")
    records = []
    with results_path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = load_json(line)
            if not isinstance(row, dict) or row.get("not_for_motion") is not True:
                raise ValueError(f"result line {line_number} is not a shadow record")
            records.append(row)
    if not records:
        raise ValueError("results file contains no records")
    run_path = results_path.parent / "run.json"
    run = load_json(run_path.read_text()) if run_path.is_file() else {}
    if run and (run.get("schema_version") != "nomad.offline_shadow.v1" or run.get("not_for_motion") is not True):
        raise ValueError("adjacent run.json is not a NoMaD offline shadow run")
    goal_sha = file_sha(goal_path)
    if run and run.get("goal", {}).get("original_sha256") != goal_sha:
        raise ValueError("--goal hash differs from run.json")

    # input_manifest.json is copied verbatim by shadow_client. Relative image
    # paths may therefore refer to its original manifest directory.
    original_manifest = Path(run.get("manifest_path", str(manifest_path))).expanduser()
    resolved_by_id = {}
    for record in records:
        for frame in record.get("frames", []):
            key = (frame.get("frame_id"), frame.get("capture_timestamp_ns"))
            resolved = frame.get("resolved_path")
            if isinstance(resolved, str):
                if key in resolved_by_id and resolved_by_id[key] != resolved:
                    raise ValueError("conflicting recorded image paths for one source frame")
                resolved_by_id[key] = resolved
    frame_by_id, previous_stamp = {}, -1
    for frame in manifest["frames"]:
        if not isinstance(frame, dict):
            raise ValueError("manifest frame must be an object")
        name, stamp = frame.get("frame_id"), frame.get("capture_timestamp_ns")
        if not isinstance(name, str) or not name or name in frame_by_id:
            raise ValueError("manifest frame IDs must be unique nonempty strings")
        if type(stamp) is not int or stamp <= previous_stamp or stamp <= 0:
            raise ValueError("manifest timestamps must be positive, strictly increasing integers")
        source = frame.get("timestamp_source", manifest.get("timestamp_source"))
        if not isinstance(source, str) or not source:
            raise ValueError("manifest must retain the recorded timestamp_source")
        raw_path = frame.get("path")
        if not isinstance(raw_path, str) or not raw_path:
            raise ValueError("manifest frame has no path")
        raw = Path(raw_path).expanduser()
        candidates = [raw] if raw.is_absolute() else [manifest_path.parent / raw, original_manifest.parent / raw]
        fallback = resolved_by_id.get((name, stamp))
        if fallback:
            candidates.append(Path(fallback).expanduser())
        source_path = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
        if source_path is None:
            raise ValueError(f"missing recorded RGB for frame {name}")
        frame_by_id[name] = {"frame_id": name, "capture_timestamp_ns": stamp,
                             "timestamp_source": source, "path": str(source_path)}
        previous_stamp = stamp
    if len(frame_by_id) < 4:
        raise ValueError("manifest needs at least four recorded RGB frames")

    frames_to_check = {str(goal_path)}
    for index, record in enumerate(records):
        identity = record.get("identity", {})
        name = identity.get("frame_id")
        if identity.get("goal_id") != goal_sha or name not in frame_by_id:
            raise ValueError(f"record {index}: goal/source identity differs from supplied files")
        frame = frame_by_id[name]
        history = record.get("frames")
        if not isinstance(history, list) or len(history) != 4:
            raise ValueError(f"record {index}: four source frames required")
        previous = -1
        for source_frame in history:
            source_name = source_frame.get("frame_id")
            if source_name not in frame_by_id:
                raise ValueError(f"record {index}: historical frame missing from manifest")
            source_stamp = source_frame.get("capture_timestamp_ns")
            if source_stamp != frame_by_id[source_name]["capture_timestamp_ns"] or source_stamp <= previous:
                raise ValueError(f"record {index}: history timestamp differs from manifest")
            previous = source_stamp
        if history[-1]["frame_id"] != name:
            raise ValueError(f"record {index}: newest history frame differs from identity")
        frame_path = Path(frame["path"])
        metadata = record.get("image_metadata", [])
        if metadata and metadata[-1].get("original_sha256") != file_sha(frame_path):
            raise ValueError(f"record {index}: RGB content differs from inference input")
        frames_to_check.add(str(frame_path))
        display = {"frame": frame, "raw": [], "sampled": [], "reference": [], "prediction": [],
                   "mpc_mode": "not_requested"}
        if record.get("ok") is True:
            response = record.get("response", {})
            expected_source = {key: frame[key] for key in ("frame_id", "capture_timestamp_ns")}
            if response.get("schema_version") != "nomad.inference.v1" or response.get("identity") != identity:
                raise ValueError(f"record {index}: response schema/identity mismatch")
            if response.get("source_frame") != expected_source:
                raise ValueError(f"record {index}: response source differs from recorded RGB")
            expected_history = [{key: item[key] for key in ("frame_id", "capture_timestamp_ns")} for item in history]
            if response.get("observation_frames") != expected_history:
                raise ValueError(f"record {index}: response history differs from recorded RGB")
            checks = record.get("checks")
            if not isinstance(checks, dict) or not checks or not all(value is True for value in checks.values()):
                raise ValueError(f"record {index}: inference contract checks did not pass")
            display["raw"] = points(response.get("trajectory_body_m"), 2, "trajectory_body_m")
            if len(display["raw"]) != 8:
                raise ValueError("NoMaD trajectory must contain eight cumulative waypoints")
            adaptation = record.get("adaptation", {})
            display["sampled"] = points(adaptation.get("sampled_path_body_m_rad", []), 3, "sampled path")
            mpc = adaptation.get("mpc", {})
            if mpc:
                if mpc.get("mode") != "synthetic_pose_no_robot_feedback" or mpc.get("not_for_motion") is not True:
                    raise ValueError("MPC report must explicitly identify synthetic pose/no feedback")
                if mpc.get("capture_pose_m_rad") != [0.0, 0.0, 0.0] or mpc.get("current_pose_m_rad") != [0.0, 0.0, 0.0]:
                    raise ValueError("this display profile requires the shadow client's synthetic zero poses")
                display["mpc_mode"] = mpc["mode"]
                display["reference"] = points(mpc.get("reference_m_rad", []), 3, "MPC reference")
                display["prediction"] = points(mpc.get("predicted_states_m_rad", []), 3, "MPC prediction")
        record["_display"] = display

    from PIL import Image
    for image_path in frames_to_check:
        with Image.open(image_path) as image:
            if image.width * image.height > 4096 * 2160:
                raise ValueError(f"recorded image exceeds display pixel limit: {image_path}")
            image.verify()
    return {"manifest": str(manifest_path), "results": str(results_path), "goal": str(goal_path),
            "capture_purpose": manifest.get("purpose", "recorded_shadow_diagnostic"),
            "goal_sha256": goal_sha, "records": records, "timestamp_sources": sorted({
                frame["timestamp_source"] for frame in frame_by_id.values()})}


@lru_cache(maxsize=8)
def preview_jpeg(path: str, title: str) -> bytes:
    from PIL import Image, ImageDraw
    with Image.open(path) as source:
        image = source.convert("RGB")
        image.thumbnail((640, 640))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, 23), fill="black")
    draw.text((5, 6), title, fill="yellow")
    stream = io.BytesIO()
    image.save(stream, format="JPEG", quality=80)
    return stream.getvalue()


def run_ros(recording: dict[str, Any], hz: float, loop: bool) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as ROSPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    class RecordedShadow(Node):
        def __init__(self) -> None:
            super().__init__("nomad_recorded_shadow")
            retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.outputs = {}
            for suffix, message_type in [("rgb/compressed", CompressedImage), ("goal/compressed", CompressedImage),
                                         ("path", ROSPath), ("path_sampled", ROSPath),
                                         ("mpc/reference", ROSPath), ("mpc/prediction", ROSPath), ("status", String)]:
                self.outputs[suffix] = self.create_publisher(message_type, f"/nomad/shadow/{suffix}", retained)
            self.index, self.cycle = -1, 0
            self.playback = "PLAYING"
            self.timer = self.create_timer(1.0 / hz, self.advance)
            self.status_timer = self.create_timer(1.0, self.publish_status)
            self.advance()

        def path_message(self, values: list[list[float]], stamp: Any) -> Any:
            message = ROSPath()
            message.header.frame_id, message.header.stamp = "nomad_shadow", stamp
            for index, value in enumerate(values):
                yaw = value[2] if len(value) == 3 else 0.0
                if len(value) == 2 and len(values) > 1:
                    left, right = (value, values[index + 1]) if index + 1 < len(values) else (values[index - 1], value)
                    yaw = math.atan2(right[1] - left[1], right[0] - left[0])
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y = value[:2]
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                message.poses.append(pose)
            return message

        def advance(self) -> None:
            if self.index + 1 >= len(recording["records"]):
                if not loop:
                    self.playback = "ENDED"
                    self.timer.cancel()
                    self.publish_status()
                    return
                self.index = -1
                self.cycle += 1
            self.index += 1
            display = recording["records"][self.index]["_display"]
            stamp = self.get_clock().now().to_msg()
            for suffix, path, label in [("rgb/compressed", display["frame"]["path"], "IO DIAGNOSTIC / RECORDED / NOT LIVE"),
                                        ("goal/compressed", recording["goal"], "IO DIAGNOSTIC GOAL / NOT LIVE")]:
                message = CompressedImage()
                message.header.frame_id, message.header.stamp = "nomad_shadow", stamp
                message.format, message.data = "jpeg", preview_jpeg(path, label)
                self.outputs[suffix].publish(message)
            for suffix, key in [("path", "raw"), ("path_sampled", "sampled"),
                                ("mpc/reference", "reference"), ("mpc/prediction", "prediction")]:
                self.outputs[suffix].publish(self.path_message(display[key], stamp))
            self.publish_status()

        def publish_status(self) -> None:
            record = recording["records"][self.index]
            display = record["_display"]
            payload = {
                "mode": "RECORDED_SHADOW_NOT_LIVE", "not_for_motion": True,
                "capture_purpose": recording["capture_purpose"], "navigation_validation": False,
                "playback": self.playback, "index": self.index, "count": len(recording["records"]), "cycle": self.cycle,
                "goal_sha256": recording["goal_sha256"], "identity": record["identity"],
                "source_frame": display["frame"], "recorded_result_ok": record.get("ok") is True,
                "error": record.get("error"), "mpc_mode": display["mpc_mode"],
                "mpc_solver_executed": record.get("adaptation", {}).get("mpc", {}).get("solver_executed", False),
                "recorded_rtt_ms": record.get("rtt_ms"),
                "distance_steps": record.get("response", {}).get("distance_steps"),
                "frame_notice": "nomad_shadow is a display-only capture-body origin, reset for each record; not robot odometry",
                "stamp_notice": "message headers use replay ROS clock; original timestamp and source are in source_frame",
                "robot_feedback_used": False, "command_topics_published": False,
                "evidence_notice": "Recorded camera/model/MPC IO diagnostic; does not establish navigation or obstacle avoidance",
                "manifest": recording["manifest"], "results": recording["results"],
            }
            message = String()
            message.data = json.dumps(payload, allow_nan=False, ensure_ascii=False)
            self.outputs["status"].publish(message)

    rclpy.init(args=[])
    node = None
    try:
        node = RecordedShadow()
        node.get_logger().info("RECORDED_SHADOW_NOT_LIVE: publishing display data only; no robot connection")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--goal", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--hz", type=float, default=0.5, help="visualization rate, independent of capture/inference rate")
    parser.add_argument("--loop", action="store_true", help="repeat recorded results, never run inference")
    parser.add_argument("--validate-only", action="store_true", help="validate files without importing ROS or publishing")
    args = parser.parse_args(argv)
    if not math.isfinite(args.hz) or not 0.05 <= args.hz <= 10:
        parser.error("--hz must be finite and in 0.05..10")
    recording = load_recording(args.manifest, args.goal, args.results)
    summary = {"mode": "RECORDED_SHADOW_NOT_LIVE", "not_for_motion": True,
               "records": len(recording["records"]), "timestamp_sources": recording["timestamp_sources"],
               "goal_sha256": recording["goal_sha256"], "validated_only": args.validate_only}
    print(json.dumps(summary, indent=2))
    if not args.validate_only:
        run_ros(recording, args.hz, args.loop)


if __name__ == "__main__":
    main()
