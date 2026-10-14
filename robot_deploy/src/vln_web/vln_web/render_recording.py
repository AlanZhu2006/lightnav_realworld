"""Generate both viewing videos from preserved RGB after acquisition stops."""

import bisect
import json
import os
import sys
import tempfile
from pathlib import Path


def frame_bgr(message, image_transport):
    """Decode either ROS image representation; independent of ROS message classes."""
    import cv2
    import numpy as np

    if image_transport == "compressed":
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Cannot decode recorded compressed image")
        return image
    encoding = message.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError("Unsupported raw video encoding: " + encoding)
    pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)[
        :, : message.width * channels
    ]
    pixels = (
        pixels.reshape(message.height, message.width, channels)
        if channels > 1
        else pixels.reshape(message.height, message.width)
    )
    conversion = {
        "rgb8": cv2.COLOR_RGB2BGR,
        "rgba8": cv2.COLOR_RGBA2BGR,
        "bgra8": cv2.COLOR_BGRA2BGR,
        "mono8": cv2.COLOR_GRAY2BGR,
    }.get(encoding)
    return cv2.cvtColor(pixels, conversion) if conversion is not None else pixels


def render(directory):
    import cv2
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage, Image

    from .recorder import EpisodeWriter
    from .recording_archive import atomic_json

    cv2.setNumThreads(1)
    directory = Path(directory)
    meta = json.loads((directory / "metadata.json").read_text())
    snapshots = []
    for line in (directory / "events.jsonl").read_text().splitlines():
        entry = json.loads(line)
        if entry["kind"] == "frame_snapshot":
            snapshots.append((entry["t"], entry["data"]))
    snapshots.sort(key=lambda pair: pair[0])
    times = [x[0] for x in snapshots]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(directory / "raw_ros"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    image_topic = meta.get("image_topic", "/camera/camera/color/image_raw")
    image_transport = meta.get("image_transport", "raw")
    message_class = CompressedImage if image_transport == "compressed" else Image
    reader.set_filter(rosbag2_py.StorageFilter(topics=[image_topic]))
    with tempfile.TemporaryDirectory(prefix=".render-", dir=directory) as scratch:
        writer = EpisodeWriter(scratch, fps=meta["fps"], capture_ros=False)
        writer.start(meta["instruction"], meta["mode"], 0)
        try:
            while reader.has_next():
                topic, payload, received = reader.read_next()
                m = deserialize_message(payload, message_class)
                pixels = frame_bgr(m, image_transport)
                elapsed = max(
                    0, min(meta["duration_s"], (received - meta["started_wall_ns"]) / 1e9)
                )
                index = bisect.bisect_right(times, elapsed) - 1
                snapshot = dict(snapshots[index][1]) if index >= 0 else {}
                snapshot["capture"] = {
                    "stamp_ns": m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec,
                    "frame_id": m.header.frame_id,
                    "bag_receive_ns": received,
                }
                writer.frame(pixels, snapshot, elapsed)
            if writer.source_frames == 0:
                raise RuntimeError("No RGB frames in raw archive; videos not generated")
        finally:
            writer.stop(meta["reason"], meta["duration_s"])
        metrics = {
            k: writer.metadata[k]
            for k in [
                "frames",
                "source_frames",
                "duplicated_frames",
                "observed_source_fps",
                "raw_size",
            ]
        }
        for name in ["raw.mp4", "annotated.mp4"]:
            (writer.directory / name).replace(directory / name)
        (writer.directory / "events.jsonl").replace(directory / "video_frames.jsonl")
        atomic_json(directory / "render_result.json", metrics)


if __name__ == "__main__":
    os.nice(10)
    render(sys.argv[1])
