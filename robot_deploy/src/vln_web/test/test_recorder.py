import json
from types import SimpleNamespace

import cv2
import numpy as np
from vln_web.recorder import EpisodeWriter, ExperimentRecorder, overlay

SNAPSHOT = dict(
    vln=dict(
        mode="objnav",
        apos_px=[100, 100],
        opos_px=[350, 80],
        apos_state="point",
        opos_state="point",
        last_sequence=2,
        stop=False,
    ),
    path=dict(body_waypoints=[[0.3, 0, 0], [0.8, 0, 0]]),
    control_source="auto",
    sent_command=dict(linear=0.3, angular=0),
)


def test_markers_and_trajectory_are_burned_into_image():
    frame = overlay(np.zeros((270, 480, 3), np.uint8), SNAPSHOT, 0.5)
    assert tuple(frame[100, 100]) == (84, 180, 255)
    assert tuple(frame[80, 350]) == (232, 203, 88)
    assert frame[175:230, 235:245].sum() > 0
    assert frame.shape == (360, 480, 3)


def test_episode_video_real_time_duration_and_metadata(tmp_path):
    r = EpisodeWriter(tmp_path, fps=10)
    r.start("Go to plant", "objnav", 10)
    r.frame(np.zeros((270, 480, 3), np.uint8), SNAPSHOT, 10)
    r.event("prediction", dict(apos_px=[100, 100]), 10.2)
    r.frame(np.full((270, 480, 3), 40, np.uint8), SNAPSHOT, 10.7)
    r.stop("ObjNav goal reached", 11)
    meta = json.loads((r.directory / "metadata.json").read_text())
    assert meta["frames"] == 10 and meta["duration_s"] == 1
    cap = cv2.VideoCapture(str(r.directory / "annotated.mp4"))
    assert cap.isOpened() and int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 10
    ok, frame = cap.read()
    cap.release()
    assert ok and frame[95:105, 95:105].sum() > 0
    assert meta["instruction"] == "Go to plant" and meta["reason"] == "ObjNav goal reached"
    assert "prediction" in (r.directory / "events.jsonl").read_text()


def test_repeated_start_finalizes_previous_episode(tmp_path):
    r = EpisodeWriter(tmp_path, fps=10)
    r.start("first", "track", 0)
    first = r.directory
    r.frame(np.zeros((270, 480, 3), np.uint8), SNAPSHOT, 0)
    r.start("second", "objnav", 1)
    assert r.directory != first
    assert json.loads((first / "metadata.json").read_text())["status"] == "complete"
    r.stop("stopped", 2)
    r.stop("duplicate stop", 3)


def test_background_recorder_closes_and_saves(tmp_path):
    log = SimpleNamespace(info=lambda x: None, error=lambda x: None)
    r = ExperimentRecorder(tmp_path, log, capture_ros=False)
    r.start("test", "track")
    r.frame(np.zeros((270, 480, 3), np.uint8), SNAPSHOT)
    r.close()
    assert not r.thread.is_alive() and not r.error
    folders = list(tmp_path.iterdir())
    assert len(folders) == 1
    assert json.loads((folders[0] / "metadata.json").read_text())["status"] == "complete"


def test_dual_video_keeps_native_raw_pixels_and_verified_manifest(tmp_path):
    import hashlib

    r = EpisodeWriter(tmp_path, fps=10)
    image = np.full((480, 848, 3), (30, 80, 120), dtype=np.uint8)
    r.start("dual", "track", 0)
    r.frame(image, SNAPSHOT, 0)
    r.frame(image, SNAPSHOT, 0.2)
    r.stop("stop", 0.5)
    raw = cv2.VideoCapture(str(r.directory / "raw.mp4"))
    marked = cv2.VideoCapture(str(r.directory / "annotated.mp4"))
    ok, a = raw.read()
    ok2, b = marked.read()
    assert ok and ok2 and a.shape == (480, 848, 3) and b.shape == (360, 480, 3)
    assert int(raw.get(cv2.CAP_PROP_FRAME_COUNT)) == int(marked.get(cv2.CAP_PROP_FRAME_COUNT)) == 5
    assert np.abs(a[100, 100].astype(int) - np.array([30, 80, 120])).max() < 12
    raw.release()
    marked.release()
    m = json.loads((r.directory / "manifest.json").read_text())
    assert {"raw.mp4", "annotated.mp4", "events.jsonl", "metadata.json"} <= {
        x["path"] for x in m["files"]
    }
    for e in m["files"]:
        data = (r.directory / e["path"]).read_bytes()
        assert hashlib.sha256(data).hexdigest() == e["sha256"] and len(data) == e["size"]


def test_raw_ros_archive_roundtrip(tmp_path):
    import pytest

    rosbag = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import deserialize_message, serialize_message
    from std_msgs.msg import String
    from vln_web.recording_archive import RosArchive

    archive = RosArchive(tmp_path)
    for i in range(20):
        archive.write(
            "/test", "std_msgs/msg/String", serialize_message(String(data=str(i))), 1000000000 + i
        )
    result = archive.stop()
    assert result["ok"] and result["dropped_messages"] == 0
    reader = rosbag.SequentialReader()
    reader.open(
        rosbag.StorageOptions(uri=str(tmp_path / "raw_ros"), storage_id="mcap"),
        rosbag.ConverterOptions("", ""),
    )
    actual = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        actual.append((topic, deserialize_message(data, String).data, stamp))
    assert actual == [("/test", str(i), 1000000000 + i) for i in range(20)]


def test_raw_capture_defers_both_videos_and_preserves_pixels(tmp_path, monkeypatch):
    import vln_web.recorder as module

    monkeypatch.setattr(module, "HostTelemetry", lambda *a: SimpleNamespace(close=lambda: {}))
    import time

    import pytest

    pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image

    start = time.monotonic()
    r = EpisodeWriter(tmp_path, capture_ros=True)
    r.start("deferred video", "validation", start)
    for i in range(3):
        stamp = r.metadata["started_wall_ns"] + i * 100000000
        msg = Image(height=48, width=64, encoding="rgb8", step=64 * 3)
        msg.data = np.full((48, 64, 3), (120, 80, 30), np.uint8).tobytes()
        msg.header.stamp.sec = stamp // 1000000000
        msg.header.stamp.nanosec = stamp % 1000000000
        r.raw(
            "/camera/camera/color/image_raw", "sensor_msgs/msg/Image", serialize_message(msg), stamp
        )
        r.frame(None, SNAPSHOT, start + i * 0.1)
    assert not (r.directory / "raw.mp4").exists()
    assert not (r.directory / "annotated.mp4").exists()
    r.stop("test done", start + 0.3)
    meta = json.loads((r.directory / "metadata.json").read_text())
    assert meta["status"] == "complete" and meta["source_frames"] == 3
    cap = cv2.VideoCapture(str(r.directory / "raw.mp4"))
    ok, frame = cap.read()
    cap.release()
    assert ok and frame.shape == (48, 64, 3)
    assert np.abs(frame[20, 20].astype(int) - np.array([30, 80, 120])).max() < 12
    assert (r.directory / "video_frames.jsonl").is_file()
    assert meta["archive_result"]["message_counts"]["/camera/camera/color/image_raw"] == 3


def test_start_during_finalization_is_explicitly_rejected(tmp_path):
    import pytest

    log = SimpleNamespace(info=lambda x: None, error=lambda x: None)
    r = ExperimentRecorder(tmp_path, log, capture_ros=False)
    r.saving = True
    with pytest.raises(RuntimeError, match="generating videos"):
        r.start("next", "track")
    assert not r.active
    r.close()


def test_archive_queue_drop_reports_topic_and_time():
    import queue

    from vln_web.recording_archive import RosArchive

    r = RosArchive.__new__(RosArchive)
    r.worker = SimpleNamespace(is_alive=lambda: True)
    r.jobs = queue.Queue(maxsize=1)
    r.jobs.put("full")
    r.dropped = 0
    r.accepted_by_topic = {}
    r.dropped_by_topic = {}
    r.drop_events = []
    r.write("/rgb", "sensor_msgs/msg/Image", b"x", 123)
    assert r.dropped_by_topic == {"/rgb": 1}
    assert r.drop_events == [dict(topic="/rgb", receive_ns=123, reason="archive_queue_full")]
