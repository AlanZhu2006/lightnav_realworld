"""Bounded read-only audit of the official RealSense ROS RGB-D topics.

Saves genuine RGB and aligned uint16 depth pairs, their ROS capture headers,
calibration, timing, and SDK metadata. Does not publish TF, odometry, or motion.
The output explicitly distinguishes sensor alignment from a robot-base mount
calibration, which this utility never invents.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import statistics
import time


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def info_dict(message):
    return {"frame_id": message.header.frame_id, "stamp_ns": stamp_ns(message.header),
            "width": message.width, "height": message.height,
            "distortion_model": message.distortion_model, "d": list(message.d),
            "k": list(message.k), "r": list(message.r), "p": list(message.p)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--pairs", type=int, default=20)
    parser.add_argument("--sample-hz", type=float, default=4.0)
    parser.add_argument("--prefix", default="/camera/camera")
    args = parser.parse_args()
    if not (0 < args.duration_s <= 120 and 1 <= args.pairs <= 1000 and 0 < args.sample_hz <= 30):
        parser.error("bounded positive duration <=120 s, pairs <=1000, and sample rate <=30 Hz required")
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error("output directory already exists")
    output.mkdir(parents=True)
    (output / "rgb").mkdir()
    (output / "depth").mkdir()

    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    try:
        from realsense2_camera_msgs.msg import Metadata
    except ImportError:
        Metadata = None

    rclpy.init()
    node = Node("dejaview_rgbd_audit")
    bridge = CvBridge()
    buffers = {"rgb": deque(maxlen=12), "depth": deque(maxlen=12)}
    calibration, metadata, received = {}, {}, {"rgb": 0, "depth": 0}
    records, ages = [], []
    prefix = args.prefix.rstrip("/")
    subscriptions = []

    def image_callback(kind, msg):
        received[kind] += 1
        now_ros = node.get_clock().now().nanoseconds
        buffers[kind].append((msg, now_ros, time.monotonic_ns()))

    for kind, suffix in (("rgb", "/color/image_raw"),
                         ("depth", "/aligned_depth_to_color/image_raw")):
        subscriptions.append(node.create_subscription(
            Image, prefix + suffix, lambda msg, kind=kind: image_callback(kind, msg),
            qos_profile_sensor_data))
    for kind, suffix in (("rgb", "/color/camera_info"),
                         ("depth", "/aligned_depth_to_color/camera_info")):
        subscriptions.append(node.create_subscription(
            CameraInfo, prefix + suffix,
            lambda msg, kind=kind: calibration.__setitem__(kind, info_dict(msg)),
            qos_profile_sensor_data))
    if Metadata is not None:
        for kind, suffix in (("rgb", "/color/metadata"), ("depth", "/depth/metadata")):
            def collect_metadata(msg, kind=kind):
                try:
                    parsed = json.loads(msg.json_data)
                except (ValueError, AttributeError):
                    parsed = {"raw": str(msg)}
                metadata[kind] = {"header_frame_id": msg.header.frame_id,
                                  "header_stamp_ns": stamp_ns(msg.header), "data": parsed}
            subscriptions.append(node.create_subscription(
                Metadata, prefix + suffix, collect_metadata, qos_profile_sensor_data))

    began = time.monotonic()
    previous_saved = -1
    last_saved_monotonic = -1_000_000_000_000
    error = None
    try:
        while time.monotonic() - began < args.duration_s and len(records) < args.pairs:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not buffers["rgb"] or not buffers["depth"]:
                continue
            rgb, rgb_received_ns, rgb_mono = buffers["rgb"][-1]
            rgb_stamp = stamp_ns(rgb.header)
            if rgb_stamp <= previous_saved or rgb_mono - last_saved_monotonic < 1e9 / args.sample_hz:
                continue
            depth, depth_received_ns, _ = min(buffers["depth"],
                key=lambda item: abs(stamp_ns(item[0].header) - rgb_stamp))
            depth_stamp = stamp_ns(depth.header)
            # SDK enable_sync aligns framesets. Still inspect actual headers;
            # a nearest depth outside 1/2 camera frame is never called a pair.
            if abs(depth_stamp - rgb_stamp) > 40_000_000:
                continue
            bgr = bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
            depth_array = bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
            if depth.encoding != "16UC1" or depth_array.dtype != np.uint16:
                raise RuntimeError(f"unexpected official aligned depth format: {depth.encoding}, {depth_array.dtype}")
            if bgr.shape[:2] != depth_array.shape:
                raise RuntimeError("aligned depth dimensions differ from RGB")
            index = len(records) + 1
            rgb_name, depth_name = f"rgb/{index:06d}.png", f"depth/{index:06d}.png"
            for relative, array in ((rgb_name, bgr), (depth_name, depth_array)):
                if not cv2.imwrite(str(output / relative), array, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
                    raise RuntimeError(f"failed writing {relative}")
            valid = depth_array[depth_array > 0].astype(np.float64) * 0.001
            age_ms = (rgb_received_ns - rgb_stamp) / 1e6
            ages.append(age_ms)
            records.append({
                "frame_id": f"realsense:{rgb_stamp}", "capture_timestamp_ns": rgb_stamp,
                "timestamp_source": "official_realsense_ros_header_system_clock_mapped_from_camera",
                "rgb_frame_name": rgb.header.frame_id, "depth_frame_name": depth.header.frame_id,
                "depth_capture_timestamp_ns": depth_stamp,
                "rgb_depth_delta_ms": (depth_stamp - rgb_stamp) / 1e6,
                "rgb_receive_age_ms": age_ms,
                "depth_receive_age_ms": (depth_received_ns - depth_stamp) / 1e6,
                "rgb": rgb_name, "depth": depth_name,
                "rgb_sha256": hashlib.sha256((output / rgb_name).read_bytes()).hexdigest(),
                "depth_sha256": hashlib.sha256((output / depth_name).read_bytes()).hexdigest(),
                "depth_encoding": depth.encoding, "depth_scale_m": 0.001,
                "depth_valid_fraction": float(np.count_nonzero(depth_array) / depth_array.size),
                "depth_valid_min_m": float(valid.min()) if valid.size else None,
                "depth_valid_median_m": float(np.median(valid)) if valid.size else None,
                "depth_valid_max_m": float(valid.max()) if valid.size else None,
            })
            previous_saved, last_saved_monotonic = rgb_stamp, rgb_mono
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        node.destroy_node()
        rclpy.shutdown()
    intrinsic_match = (all(key in calibration for key in ("rgb", "depth"))
        and calibration["rgb"]["width"] == calibration["depth"]["width"]
        and calibration["rgb"]["height"] == calibration["depth"]["height"]
        and np.allclose(calibration["rgb"]["k"], calibration["depth"]["k"], atol=1e-6)
        and calibration["rgb"]["frame_id"] == calibration["depth"]["frame_id"])
    verified = bool(records and intrinsic_match and all(
        row["rgb_frame_name"] == row["depth_frame_name"] and row["depth_valid_fraction"] > 0
        for row in records))
    report = {
        "schema_version": "dejaview.rgbd.audit.v1", "not_for_motion": True,
        "camera_alignment_verified": verified, "robot_mount_calibration_verified": False,
        "depth_scale_m": 0.001,
        "depth_scale_source": "official realsense2_camera publishes scaled depth in millimeters",
        "topics": {"rgb": prefix + "/color/image_raw",
                   "depth": prefix + "/aligned_depth_to_color/image_raw",
                   "rgb_camera_info": prefix + "/color/camera_info",
                   "depth_camera_info": prefix + "/aligned_depth_to_color/camera_info"},
        "requested_pairs": args.pairs, "received_message_counts": received,
        "pairs_saved": len(records), "elapsed_s": time.monotonic() - began,
        "calibration": calibration, "latest_sdk_metadata": metadata,
        "rgb_receive_age_ms_median": statistics.median(ages) if ages else None,
        "rgb_receive_age_ms_max": max(ages) if ages else None,
        "records": records, "error": error,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if records:
        manifest = {
            "schema_version": "dejaview.rgbd.capture.v1", "not_for_motion": True,
            "timestamp_source": "official_realsense_ros_header_system_clock_mapped_from_camera",
            "frames": [{"path": row["rgb"], "frame_id": row["frame_id"],
                        "capture_timestamp_ns": row["capture_timestamp_ns"],
                        "timestamp_source": row["timestamp_source"],
                        "sha256": row["rgb_sha256"], "depth_path": row["depth"],
                        "depth_capture_timestamp_ns": row["depth_capture_timestamp_ns"],
                        "depth_sha256": row["depth_sha256"], "depth_scale_m": 0.001,
                        "frame_name": row["rgb_frame_name"]} for row in records],
            "same_view_io_diagnostic_goal": records[-1]["rgb"],
            "goal_is_navigation_validation": False,
        }
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ("records", "latest_sdk_metadata", "calibration")}, indent=2))
    if error or not verified or len(records) < args.pairs:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
