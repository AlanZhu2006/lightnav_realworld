"""Per-episode video recorder. Disk I/O and encoding stay off the ROS thread."""

import json
import math
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from .observability import HostTelemetry
from .recording_archive import RosArchive, atomic_json, manifest


def overlay(image, snapshot, elapsed):
    image = cv2.resize(image, (480, 270)).copy()
    vln = snapshot.get("vln", {})
    points = snapshot.get("path", {}).get("body_waypoints", [])
    focal = 240 / math.tan(math.pi / 3)
    previous = None
    for i, point in enumerate(points):
        depth = float(point[0]) + 0.65
        pixel = (
            (round(240 - focal * float(point[1]) / depth), round(135 + focal * 0.5 / depth))
            if depth > 0.05
            else None
        )
        ratio = i / max(1, len(points) - 1)
        color = (round(80 - 30 * ratio), round(230 - 170 * ratio), round(50 + 160 * ratio))
        if pixel and 0 <= pixel[0] < 480 and 0 <= pixel[1] < 270:
            if previous:
                cv2.line(image, previous, pixel, color, 2)
            cv2.circle(image, pixel, 4, color, -1)
            previous = pixel
        else:
            previous = None
    for channel, color in [("apos", (84, 180, 255)), ("opos", (232, 203, 88))]:
        point = vln.get(channel + "_px")
        if not point or len(point) != 2 or not all(math.isfinite(x) for x in point):
            continue
        x, y = map(round, point)
        if not (0 <= x < 480 and 0 <= y < 270):
            continue
        cv2.circle(image, (x, y), 6, color, 2)
        cv2.line(image, (x - 9, y), (x + 9, y), color, 2)
        cv2.line(image, (x, y - 9), (x, y + 9), color, 2)
        label = channel.upper() + ": " + str(vln.get(channel + "_state") or "")
        tx, ty = max(3, min(345, x + 9)), y + 19 if y < 19 else y - 9
        cv2.rectangle(
            image, (tx - 2, ty - 12), (min(479, tx + 133), min(269, ty + 3)), (10, 16, 12), -1
        )
        cv2.putText(image, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.34, color, 1, cv2.LINE_AA)
    canvas = np.zeros((360, 480, 3), dtype=np.uint8)
    canvas[:270] = image
    cmd = snapshot.get("sent_command", {})
    lines = [
        f"{elapsed:.1f}s  {vln.get('mode', '')}  seq={vln.get('last_sequence', 0)}  stop={vln.get('stop')}",
        f"APOS={vln.get('apos_state')} OPOS={vln.get('opos_state')} latency={vln.get('last_latency_ms')}",
        f"control={snapshot.get('control_source')} v={cmd.get('linear', 0):.2f} w={cmd.get('angular', 0):.2f}",
    ]
    for i, line in enumerate(lines):
        cv2.putText(
            canvas,
            line[:85],
            (7, 290 + i * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.36,
            (225, 235, 230),
            1,
            cv2.LINE_AA,
        )
    return canvas


class VideoSink:
    """Encode outside Python with a low-latency software encoder when available."""

    def __init__(self, path, fps, size):
        self.process = None
        self.writer = None
        if shutil.which("ffmpeg"):
            self.process = subprocess.Popen(
                [
                    "ffmpeg",
                    "-nostdin",
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-y",
                    "-f",
                    "rawvideo",
                    "-pix_fmt",
                    "bgr24",
                    "-video_size",
                    f"{size[0]}x{size[1]}",
                    "-framerate",
                    str(fps),
                    "-i",
                    "pipe:0",
                    "-an",
                    "-c:v",
                    "libx264",
                    "-preset",
                    "ultrafast",
                    "-crf",
                    "23",
                    "-pix_fmt",
                    "yuv420p",
                    "-threads",
                    "2",
                    "-movflags",
                    "+faststart",
                    str(path),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)

    def isOpened(self):
        return self.process.poll() is None if self.process else self.writer.isOpened()

    def write(self, image):
        if self.process:
            self.process.stdin.write(np.ascontiguousarray(image).tobytes())
        else:
            self.writer.write(image)

    def release(self):
        if self.process:
            self.process.stdin.close()
            try:
                code = self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
                raise RuntimeError("Video encoder did not finalize")
            if code:
                raise RuntimeError(f"Video encoder failed: {code}")
        elif self.writer:
            self.writer.release()


class EpisodeWriter:
    def __init__(
        self,
        root,
        fps=20,
        capture_ros=False,
        image_topic="/camera/camera/color/image_raw",
        image_transport="raw",
    ):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.writer = None
        self.metadata = None
        self.capture_ros = capture_ros
        self.image_topic = image_topic
        self.image_transport = image_transport
        self.raw_writer = None
        self.archive = None
        self.telemetry = None

    def start(self, instruction, mode, now):
        self.stop("new episode", now)
        if shutil.disk_usage(self.root).free < 2 * 1024**3:
            raise RuntimeError("Recording disk has less than 2 GiB free")
        name = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
        self.directory = self.root / name
        self.directory.mkdir()
        self.writer = (
            True
            if self.capture_ros
            else VideoSink(self.directory / "annotated.mp4", self.fps, (480, 360))
        )
        if not self.capture_ros and not self.writer.isOpened():
            self.writer.release()
            self.writer = None
            raise RuntimeError("Cannot open MP4 video encoder")
        self.log = (self.directory / "events.jsonl").open("w")
        self.started = now
        self.frames = 0
        self.last = None
        self.raw_writer = None
        self.last_raw = None
        self.source_frames = 0
        self.duplicates = 0
        self.first_source = None
        self.last_source = None
        self.archive = None
        self.metadata = dict(
            instruction=instruction,
            mode=mode,
            started_at=datetime.now(timezone.utc).isoformat(),
            fps=self.fps,
            video="annotated.mp4",
            status="recording",
            raw_video="raw.mp4",
            raw_ros="raw_ros" if self.capture_ros else None,
            video_codec="H264 ultrafast (mp4v fallback), lossy; raw_ros stores original ROS messages losslessly",
            overlay="Same 480x270 projection and latest APOS/OPOS as web preview; not calibrated reprojection",
        )
        self.metadata["started_wall_ns"] = time.time_ns() - int((time.monotonic() - now) * 1e9)
        self.metadata["video_generation"] = "after_stop" if self.capture_ros else "live"
        self.metadata["image_topic"] = self.image_topic
        self.metadata["image_transport"] = self.image_transport
        if self.capture_ros:
            self.telemetry = HostTelemetry(self.directory, os.environ.get("LIGHTNAV_REPO_ROOT"))
            try:
                self.archive = RosArchive(self.directory)
            except Exception as exc:
                self.metadata["archive_error"] = str(exc)
        self._save()

    def raw(self, topic, type_name, payload, received_ns):
        if self.archive is not None:
            self.archive.write(topic, type_name, payload, received_ns)

    def _save(self):
        atomic_json(self.directory / "metadata.json", self.metadata)

    def event(self, kind, data, now):
        if self.writer is not None:
            if kind == "inference_trace" and data.get("trace_id"):
                ids = self.metadata.setdefault("inference_trace_ids", [])
                if data["trace_id"] not in ids:
                    ids.append(data["trace_id"])
            if kind == "prediction" and data.get("timings", {}).get("trace_id"):
                ids = self.metadata.setdefault("inference_trace_ids", [])
                if data["timings"]["trace_id"] not in ids:
                    ids.append(data["timings"]["trace_id"])
            if kind == "topic_loss_summary":
                self.metadata["topic_loss_summary"] = data
            if kind == "recorder_counters":
                self.metadata["queue_counters"] = data
            self.log.write(
                json.dumps(dict(t=now - self.started, kind=kind, data=data), ensure_ascii=False)
                + "\n"
            )
            self.log.flush()

    def frame(self, image, snapshot, now):
        if self.writer is None:
            return
        if shutil.disk_usage(self.root).free < 1024**3:
            self.stop("disk space below 1 GiB", now)
            raise RuntimeError("Recording stopped: disk space below 1 GiB")
        if self.capture_ros:
            self.event("frame_snapshot", snapshot, now)
            return
        if self.raw_writer is None:
            height, width = image.shape[:2]
            self.raw_writer = VideoSink(self.directory / "raw.mp4", self.fps, (width, height))
            if not self.raw_writer.isOpened():
                raise RuntimeError("Cannot open raw MP4 encoder")
            self.metadata["raw_size"] = [width, height]
        self.source_frames += 1
        if self.first_source is None:
            self.first_source = now
        self.last_source = now
        self.event(
            "source_frame",
            dict(
                capture=snapshot.get("capture"),
                source_index=self.source_frames,
                video_index=self.frames,
            ),
            now,
        )
        target = max(0, int((now - self.started) * self.fps))
        if target < self.frames:
            return
        rendered = overlay(image, snapshot, now - self.started)
        # Resample to real elapsed time; slower camera FPS must not speed up video.
        while self.frames < target:
            self.writer.write(self.last if self.last is not None else rendered)
            self.raw_writer.write(self.last_raw if self.last_raw is not None else image)
            self.frames += 1
            self.duplicates += 1
        self.writer.write(rendered)
        self.raw_writer.write(image)
        self.frames += 1
        self.last = rendered
        self.last_raw = image
        self.event("frame", snapshot, now)

    def stop(self, reason, now):
        if self.writer is None:
            return
        if self.capture_ros:
            self._finish_raw_episode(reason, now)
            return
        if self.last is not None:
            while self.frames < max(1, math.ceil((now - self.started) * self.fps)):
                self.writer.write(self.last)
                self.raw_writer.write(self.last_raw)
                self.frames += 1
                self.duplicates += 1
        self.event("stop", dict(reason=reason), now)
        self.writer.release()
        self.writer = None
        self.log.close()
        if self.raw_writer is not None:
            self.raw_writer.release()
            self.raw_writer = None
        if self.archive is not None:
            self.metadata["archive_result"] = self.archive.stop()
            self.archive = None
        self.metadata.update(
            status="complete",
            reason=reason,
            frames=self.frames,
            duration_s=now - self.started,
            finished_at=datetime.now(timezone.utc).isoformat(),
            source_frames=self.source_frames,
            duplicated_frames=self.duplicates,
            observed_source_fps=(self.source_frames - 1) / (self.last_source - self.first_source)
            if self.source_frames > 1 and self.last_source > self.first_source
            else 0,
        )
        self.metadata["has_queue_drops"] = (
            any(self.metadata.get("queue_counters", {}).values())
            or self.metadata.get("archive_result", {}).get("dropped_messages", 0) > 0
            or any(self.metadata.get("topic_loss_summary", {}).get("dds_reported", {}).values())
        )
        self._save()
        manifest(self.directory)

    def _finish_raw_episode(self, reason, now):
        self.event("stop", dict(reason=reason), now)
        self.log.close()
        self.writer = None
        if self.archive is not None:
            self.metadata["archive_result"] = self.archive.stop()
            self.archive = None
        if self.telemetry is not None:
            self.metadata["telemetry_result"] = self.telemetry.close()
            self.telemetry = None
        self.metadata.update(status="rendering", reason=reason, duration_s=now - self.started)
        self._save()
        try:
            with (self.directory / "render.log").open("w") as log:
                subprocess.run(
                    [sys.executable, "-m", "vln_web.render_recording", str(self.directory)],
                    stdout=log,
                    stderr=log,
                    check=True,
                    timeout=max(120, (now - self.started) * 5),
                )
            self.metadata.update(json.loads((self.directory / "render_result.json").read_text()))
            self.metadata.update(
                status="complete", finished_at=datetime.now(timezone.utc).isoformat()
            )
        except Exception as exc:
            self.metadata.update(status="render_error", render_error=str(exc))
            self._save()
            raise
        self.metadata["has_queue_drops"] = (
            any(self.metadata.get("queue_counters", {}).values())
            or self.metadata.get("archive_result", {}).get("dropped_messages", 0) > 0
            or any(self.metadata.get("topic_loss_summary", {}).get("dds_reported", {}).values())
        )
        self._save()
        manifest(self.directory)


class ExperimentRecorder:
    def __init__(
        self,
        root,
        logger,
        capture_ros=True,
        image_topic="/camera/camera/color/image_raw",
        image_transport="raw",
    ):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.logger = logger
        self.jobs = queue.Queue()
        self.active = False
        self.closed = False
        self.warning = ""
        self.error = ""
        self.directory = ""
        self.dropped = 0
        self.dropped_events = 0
        self.dropped_raw = 0
        self.capture_ros = capture_ros
        self.saving = False
        self.latched = {}
        self.pending_drops_by_topic = {}
        self.dds_lost_by_topic = {}
        self._episode = EpisodeWriter(
            self.root,
            capture_ros=self.capture_ros,
            image_topic=image_topic,
            image_transport=image_transport,
        )
        self._raw_ready = False
        self._pending_raw = deque(maxlen=256)
        self.thread = threading.Thread(target=self._run, name="experiment-video", daemon=True)
        self.thread.start()

    def start(self, instruction, mode):
        if self.active:
            return
        if self.saving:
            raise RuntimeError(
                "Previous recording is generating videos; wait until saving finishes"
            )
        self.warning = ""
        self.error = ""
        self.dropped = 0
        self.dropped_events = 0
        self.dropped_raw = 0
        self.pending_drops_by_topic = {}
        self.dds_lost_by_topic = {}
        self._raw_ready = False
        self._pending_raw.clear()
        self.active = True
        self.jobs.put(("start", (instruction, mode, time.monotonic())))
        for topic, (type_name, payload) in self.latched.copy().items():
            self.raw(topic, type_name, payload, time.time_ns())

    def event(self, kind, data):
        if self.active:
            if self.jobs.qsize() <= 64:
                self.jobs.put(("event", (kind, data, time.monotonic())))
            else:
                self.dropped_events += 1

    def raw(self, topic, type_name, payload, received_ns):
        if topic == "/tf_static" or "/extrinsics/" in topic:
            self.latched[topic] = (type_name, payload)
        if not self.active or not self.capture_ros:
            return
        if self._raw_ready and self._episode.archive is not None:
            self._episode.archive.write(topic, type_name, payload, received_ns)
        else:
            if len(self._pending_raw) == self._pending_raw.maxlen:
                self.dropped_raw += 1
                lost_topic = self._pending_raw[0][0]
                self.pending_drops_by_topic[lost_topic] = (
                    self.pending_drops_by_topic.get(lost_topic, 0) + 1
                )
            self._pending_raw.append((topic, type_name, payload, received_ns))

    def transport_loss(self, topic, count):
        if not self.active:
            return
        self.dds_lost_by_topic[topic] = self.dds_lost_by_topic.get(topic, 0) + count
        self.event("dds_message_lost", dict(topic=topic, count=count, wall_ns=time.time_ns()))

    def frame(self, image, snapshot):
        if not self.active:
            return
        if self.jobs.qsize() > 48:
            self.dropped += 1
            return
        self.jobs.put(
            ("frame", (None if self.capture_ros else image.copy(), snapshot, time.monotonic()))
        )

    def stop(self, reason):
        if self.active:
            self.active = False
            self.saving = True
            self.jobs.put(
                (
                    "event",
                    (
                        "recorder_counters",
                        dict(
                            dropped_frames=self.dropped,
                            dropped_events=self.dropped_events,
                            dropped_raw_messages=self.dropped_raw,
                        ),
                        time.monotonic(),
                    ),
                )
            )
            self.jobs.put(
                (
                    "event",
                    (
                        "topic_loss_summary",
                        dict(
                            pending_queue=self.pending_drops_by_topic,
                            dds_reported=self.dds_lost_by_topic,
                        ),
                        time.monotonic(),
                    ),
                )
            )
            self.jobs.put(("stop", (reason, time.monotonic())))

    def status(self):
        return dict(
            active=self.active,
            saving=self.saving,
            directory=self.directory,
            error=self.error,
            warning=self.warning,
            dropped_frames=self.dropped,
            dropped_events=self.dropped_events,
            dropped_raw_messages=self.dropped_raw,
        )

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop("service shutdown")
        self.jobs.put(("close", ()))
        self.thread.join(timeout=300)

    def _run(self):
        writer = self._episode
        while True:
            kind, args = self.jobs.get()
            try:
                if kind == "close":
                    return
                getattr(writer, kind)(*args)
                if kind == "stop":
                    self.saving = False
                    if writer.metadata.get("archive_error") or not writer.metadata.get(
                        "archive_result", {"ok": True}
                    ).get("ok"):
                        self.error = "Raw ROS archive incomplete; see metadata.json"
                        self.logger.error(self.error)
                    elif writer.metadata.get("has_queue_drops"):
                        self.warning = "Saved with dropped frames/messages; see metadata.json"
                        self.logger.info(self.warning)
                if kind == "start":
                    while self._pending_raw:
                        writer.raw(*self._pending_raw.popleft())
                    self._raw_ready = True
                    self.error = ""
                    self.directory = writer.directory.name
                    self.logger.info(f"Experiment recording: {writer.directory}")
            except Exception as exc:
                self.error = str(exc)
                self.active = False
                self.saving = False
                self.logger.error(f"Experiment recording failed: {exc}")
                try:
                    writer.stop("recording error: " + str(exc), time.monotonic())
                except Exception:
                    pass
            finally:
                self.jobs.task_done()
