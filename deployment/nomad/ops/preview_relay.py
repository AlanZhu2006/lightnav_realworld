#!/usr/bin/env python3
"""Read-only JPEG previews; original sensor streams remain policy inputs."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image


class PreviewRelay(Node):
    def __init__(self) -> None:
        super().__init__("nomad_preview_relay")
        defaults = {
            "rgb_topic": "/camera/camera/color/image_raw",
            "goal_topic": "/nomad/goal_image",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "depth_enabled": True,
            "goal_transient_local": False,
            "rate_hz": 5.0,
            "width": 640,
            "jpeg_quality": 75,
            "depth_u16_metres_per_unit": 0.001,
            "depth_min_metres": 0.2,
            "depth_max_metres": 4.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        value = lambda key: self.get_parameter(key).value
        self.width = int(value("width"))
        self.quality = int(value("jpeg_quality"))
        self.depth_scale = float(value("depth_u16_metres_per_unit"))
        self.depth_min = float(value("depth_min_metres"))
        self.depth_max = float(value("depth_max_metres"))
        rate = float(value("rate_hz"))
        if not (1 <= rate <= 30 and 160 <= self.width <= 1920):
            raise ValueError("rate_hz must be 1..30 and width 160..1920")
        if not (1 <= self.quality <= 100 and 0 < self.depth_scale < 1):
            raise ValueError("invalid JPEG quality or depth unit")
        if not (0 <= self.depth_min < self.depth_max < 100):
            raise ValueError("invalid depth display range")
        self.bridge = CvBridge()
        self.pending: dict[str, Image] = {}
        retained = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publishers_by_kind = {}
        self.subscriptions_by_kind = []
        for kind in ("rgb", "goal", "depth_color"):
            if kind == "depth_color" and not value("depth_enabled"):
                continue
            self.publishers_by_kind[kind] = self.create_publisher(
                CompressedImage,
                f"/nomad/foxglove/{kind}/compressed",
                retained if kind == "goal" else qos_profile_sensor_data,
            )
            input_key = "depth_topic" if kind == "depth_color" else f"{kind}_topic"
            qos = retained if kind == "goal" and value("goal_transient_local") else qos_profile_sensor_data
            self.subscriptions_by_kind.append(
                self.create_subscription(
                    Image,
                    str(value(input_key)),
                    lambda message, name=kind: self.pending.__setitem__(name, message),
                    qos,
                )
            )
        self.create_timer(1.0 / rate, self.publish_pending)
        self.get_logger().info("JPEG display only; no command publishers or services")

    def to_preview(self, kind: str, message: Image) -> np.ndarray:
        if kind != "depth_color":
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        else:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            if message.encoding.lower() in {"16uc1", "mono16"}:
                depth = raw.astype(np.float32) * self.depth_scale
            elif message.encoding.lower() == "32fc1":
                depth = raw.astype(np.float32)
            else:
                raise ValueError(f"unsupported depth encoding: {message.encoding}")
            invalid = ~np.isfinite(depth) | (depth <= 0)
            fraction = (np.nan_to_num(depth) - self.depth_min) / (self.depth_max - self.depth_min)
            grey = (255 * np.clip(fraction, 0, 1)).astype(np.uint8)
            image = cv2.applyColorMap(grey, cv2.COLORMAP_TURBO)
            image[invalid] = 0
        if image.shape[0] < 1 or image.shape[1] < 1:
            raise ValueError("empty input image")
        if image.shape[1] > self.width:
            height = max(1, round(image.shape[0] * self.width / image.shape[1]))
            image = cv2.resize(image, (self.width, height), interpolation=cv2.INTER_AREA)
        return image

    def publish_pending(self) -> None:
        pending, self.pending = self.pending, {}
        for kind, source in pending.items():
            try:
                image = self.to_preview(kind, source)
                ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    raise ValueError("JPEG encoding failed")
                preview = CompressedImage()
                preview.header = source.header
                preview.format = "jpeg"
                preview.data = data.tobytes()
                self.publishers_by_kind[kind].publish(preview)
            except (ValueError, RuntimeError, CvBridgeError, cv2.error) as error:
                self.get_logger().warning(f"{kind} preview skipped: {error}", throttle_duration_sec=5.0)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PreviewRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
