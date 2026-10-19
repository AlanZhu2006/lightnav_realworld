"""Read-only Unitree front-camera JPEG publisher and optional shadow ingress.

VideoClient exposes no sensor exposure timestamp. ROS headers and the legacy
ImageGoal capture_timestamp_ns field therefore carry host RPC *receive* time,
explicitly identified as such. Neither the optical frame nor capture latency is
calibrated. This module never imports an actuator client or sends motion.
"""
from __future__ import annotations

import argparse
import base64
from dataclasses import dataclass
import fcntl
import json
import math
from pathlib import Path
import signal
import threading
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid

from .preflight import load_config, preflight


TIMESTAMP_SOURCE = "go2_video_rpc_receive_host_system_time_not_sensor_source_time"
FRAME_NAME = "go2_front_camera_optical_frame"
IMAGE_TOPIC = "/go2/front_camera/image_raw/compressed"
STATUS_TOPIC = "/go2/front_camera/status"
MAX_JPEG_BYTES = 4 * 1024 * 1024


def jpeg_dimensions(data):
    """Validate bounded JPEG marker structure and dimensions without a codec.

    Entropy decoding is left to the ROS viewer / runtime's actual JPEG decoder;
    this gate rejects missing SOF/SOS/EOI, malformed segment lengths, bad sizes,
    and non-JPEG SDK responses before publication.
    """
    if not isinstance(data, (bytes, bytearray, memoryview, list, tuple)):
        raise ValueError("VideoClient image must be a byte sequence")
    if not 16 <= len(data) <= MAX_JPEG_BYTES:
        raise ValueError("JPEG size must be between 16 bytes and 4 MiB")
    blob = bytes(data)
    if blob[:2] != b"\xff\xd8" or blob[-2:] != b"\xff\xd9":
        raise ValueError("JPEG SOI/EOI markers missing")
    offset, dimensions, saw_scan = 2, None, False
    sof_markers = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                   0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
    while offset < len(blob)-2:
        if blob[offset] != 0xFF:
            raise ValueError("invalid JPEG marker boundary")
        while offset < len(blob) and blob[offset] == 0xFF:
            offset += 1
        if offset >= len(blob):
            raise ValueError("truncated JPEG marker")
        marker = blob[offset]
        offset += 1
        if marker in {0x00, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            raise ValueError("unexpected JPEG marker before scan")
        if marker == 0x01:
            continue
        if offset+2 > len(blob):
            raise ValueError("truncated JPEG segment")
        length = int.from_bytes(blob[offset:offset+2], "big")
        if length < 2 or offset+length > len(blob)-2:
            raise ValueError("invalid JPEG segment length")
        if marker in sof_markers:
            if length < 8:
                raise ValueError("short JPEG SOF")
            height = int.from_bytes(blob[offset+3:offset+5], "big")
            width = int.from_bytes(blob[offset+5:offset+7], "big")
            components = blob[offset+7]
            if (not 0 < width <= 4096 or not 0 < height <= 2160 or
                    width*height > 4096*2160 or components not in {1, 3, 4} or
                    length != 8+3*components):
                raise ValueError("invalid or excessive JPEG dimensions/components")
            dimensions = width, height
        if marker == 0xDA:
            if dimensions is None or length < 6 or offset+length >= len(blob)-2:
                raise ValueError("JPEG scan missing valid SOF or entropy payload")
            saw_scan = True
            break
        offset += length
    if not saw_scan:
        raise ValueError("JPEG scan missing")
    return blob, dimensions


@dataclass(frozen=True)
class VideoFrame:
    jpeg: bytes
    width: int
    height: int
    frame_id: str
    request_begin_timestamp_ns: int
    request_begin_monotonic_ns: int
    receive_timestamp_ns: int
    received_monotonic_ns: int
    rpc_duration_ms: float

    def payload(self):
        return {"frame_id": self.frame_id,
                "rgb_jpeg_base64": base64.b64encode(self.jpeg).decode("ascii"),
                "capture_timestamp_ns": self.receive_timestamp_ns,
                "clock_domain": "unix_epoch_ns", "timestamp_source": TIMESTAMP_SOURCE,
                "frame_name": FRAME_NAME, "synthetic": False, "commissioning": False,
                "provenance": {"source": "unitree_go2_VideoClient_GetImageSample",
                    "source_exposure_timestamp_available": False,
                    "capture_timestamp_semantics": "host_rpc_receive_not_exposure",
                    "request_begin_timestamp_ns": self.request_begin_timestamp_ns,
                    "request_begin_monotonic_ns": self.request_begin_monotonic_ns,
                    "receive_timestamp_ns": self.receive_timestamp_ns,
                    "received_monotonic_ns": self.received_monotonic_ns,
                    "rpc_duration_ms": self.rpc_duration_ms,
                    "rpc_duration_is_capture_latency": False,
                    "camera_extrinsics_verified": False,
                    "frame_name_is_uncalibrated_logical_frame": True,
                    "width": self.width, "height": self.height,
                    "motion_sent": False}}


class LatestFrame:
    """Single pending frame; HTTP stalls never block camera publication."""
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.replaced = 0

    def put(self, frame):
        with self.condition:
            if self.closed:
                return
            if self.pending is not None:
                self.replaced += 1
            self.pending = frame
            self.condition.notify()

    def take(self, timeout_s=.1):
        with self.condition:
            if self.pending is None and not self.closed:
                self.condition.wait(timeout_s)
            result, self.pending = self.pending, None
            return result

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()


class CameraState:
    def __init__(self, net_if, max_frame_age_s=1., wall_clock=time.time_ns,
                 mono_clock=time.monotonic_ns):
        self.lock = threading.RLock()
        self.net_if, self.max_frame_age_s = net_if, max_frame_age_s
        self.wall_clock, self.mono_clock = wall_clock, mono_clock
        self.active = True
        self.last_frame = None
        self.last_published_frame = None
        self.last_rpc_duration_ms = None
        self.last_error = None
        self.last_capture_error = None
        self.last_runtime_error = None
        self.runtime_session = None
        self.runtime_disabled_reason = None
        self.counters = {key: 0 for key in ("rpc_calls", "capture_successes", "capture_failures",
            "jpeg_failures", "published_frames", "publish_failures", "runtime_sent",
            "runtime_dropped", "runtime_errors", "runtime_stale_dropped")}

    def increment(self, key, amount=1):
        with self.lock:
            self.counters[key] += amount

    def error(self, message, category="capture"):
        with self.lock:
            self.last_error = str(message)[:1024]
            if category == "capture":
                self.last_capture_error = self.last_error
            else:
                self.last_runtime_error = self.last_error

    def snapshot(self, pending_replaced=0):
        with self.lock:
            frame = self.last_frame
            age = (self.mono_clock()-frame.received_monotonic_ns)/1e9 if frame else None
            published = self.last_published_frame
            published_age = ((self.mono_clock()-published.received_monotonic_ns)/1e9
                             if published else None)
            return {"schema_version": "dejaview.go2.camera.v1", "timestamp_ns": self.wall_clock(),
                "ready": bool(self.active and published and 0 <= published_age <= self.max_frame_age_s),
                "active": self.active, "net_if": self.net_if,
                "read_only": True, "synthetic": False, "motion_sent": False,
                "motion_api_imported": False, "timestamp_source": TIMESTAMP_SOURCE,
                "source_exposure_timestamp_available": False,
                "rpc_duration_is_capture_latency": False,
                "camera_extrinsics_verified": False,
                "frame_name_is_uncalibrated_logical_frame": True,
                "frame_name": FRAME_NAME, "image_topic": IMAGE_TOPIC,
                "frame_age_s": age,
                "published_frame_age_s": published_age,
                "receive_timestamp_ns": frame.receive_timestamp_ns if frame else 0,
                "received_monotonic_ns": frame.received_monotonic_ns if frame else 0,
                "last_rpc_duration_ms": self.last_rpc_duration_ms,
                "width": frame.width if frame else None, "height": frame.height if frame else None,
                "runtime_session": self.runtime_session,
                "runtime_disabled_reason": self.runtime_disabled_reason,
                "last_error": self.last_error, "last_capture_error": self.last_capture_error,
                "last_runtime_error": self.last_runtime_error,
                "http_pending_replaced": pending_replaced, **self.counters}


def capture_once(client, state, publish, handoff=None, frame_prefix="go2-video"):
    """One actual SDK response produces at most one ROS message / HTTP frame."""
    begin_wall, begin_mono = state.wall_clock(), state.mono_clock()
    state.increment("rpc_calls")
    try:
        code, data = client.GetImageSample()
        receive_wall, receive_mono = state.wall_clock(), state.mono_clock()
        with state.lock:
            state.last_rpc_duration_ms = (receive_mono-begin_mono)/1e6
        if code != 0:
            raise RuntimeError(f"VideoClient GetImageSample returned {code}")
    except Exception as exc:
        with state.lock:
            state.last_rpc_duration_ms = (state.mono_clock()-begin_mono)/1e6
        state.increment("capture_failures")
        state.error(exc)
        return None
    try:
        blob, (width, height) = jpeg_dimensions(data)
    except (ValueError, TypeError, OverflowError) as exc:
        state.increment("capture_failures")
        state.increment("jpeg_failures")
        state.error(exc)
        return None
    frame = VideoFrame(blob, width, height,
        f"{frame_prefix}-{state.counters['rpc_calls']:012d}", begin_wall, begin_mono,
        receive_wall, receive_mono, (receive_mono-begin_mono)/1e6)
    with state.lock:
        state.last_frame = frame
        state.counters["capture_successes"] += 1
        state.last_capture_error = None
    try:
        publish(frame)
        with state.lock:
            state.counters["published_frames"] += 1
            state.last_published_frame = frame
    except Exception as exc:
        state.increment("publish_failures")
        state.error(f"ROS publication failed: {exc}")
        return frame
    if handoff is not None:
        handoff.put(frame)
    return frame


def request_json(url, timeout_s, payload=None):
    blob = None if payload is None else json.dumps(payload, allow_nan=False).encode()
    request = Request(url, data=blob, headers={"Content-Type": "application/json"})
    with urlopen(request, timeout=timeout_s) as response:
        if response.status != 200:
            raise RuntimeError(f"runtime HTTP {response.status}")
        raw = response.read(1024*1024+1)
        if len(raw) > 1024*1024:
            raise ValueError("runtime JSON response exceeds 1 MiB")
        result = json.loads(raw)
        if not isinstance(result, dict):
            raise ValueError("runtime JSON must be an object")
        return result


def shadow_session(status):
    if (status.get("profile") != "external" or
            status.get("execution_capability", {}).get("operator_run_configured") is not False):
        raise ValueError("camera HTTP ingress requires external runtime configured for shadow execution")
    session = status.get("session_id")
    if not isinstance(session, str) or not session:
        raise ValueError("runtime session identity is missing")
    return session


class ShadowIngress:
    def __init__(self, url, state, max_frame_age_s=.75, timeout_s=.5, request=request_json):
        self.url, self.state, self.request = url.rstrip("/"), state, request
        self.max_frame_age_s, self.timeout_s = max_frame_age_s, timeout_s
        session = shadow_session(self.request(self.url+"/status", self.timeout_s))
        with state.lock:
            state.runtime_session = session

    def send(self, frame):
        state = self.state
        if state.runtime_disabled_reason:
            state.increment("runtime_dropped")
            return False
        try:
            status = self.request(self.url+"/status", self.timeout_s)
            try:
                session = shadow_session(status)
            except ValueError:
                with state.lock:
                    state.runtime_disabled_reason = "runtime_profile_changed_requires_camera_restart"
                raise
            if session != state.runtime_session:
                with state.lock:
                    state.runtime_disabled_reason = "runtime_session_changed_requires_camera_restart"
                raise ValueError(state.runtime_disabled_reason)
            age = (state.mono_clock()-frame.received_monotonic_ns)/1e9
            if not 0 <= age <= self.max_frame_age_s:
                state.increment("runtime_dropped")
                state.increment("runtime_stale_dropped")
                return False
            response = self.request(self.url+"/inputs/rgb", self.timeout_s, frame.payload())
            if response.get("ok") is not True:
                raise ValueError("runtime did not accept RGB input")
            state.increment("runtime_sent")
            with state.lock:
                state.last_runtime_error = None
            return True
        except Exception as exc:
            state.increment("runtime_errors")
            state.increment("runtime_dropped")
            state.error(exc, "runtime")
            return False


def atomic_status(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False)+"\n")
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--status-file", type=Path, default=Path(".runtime/go2/camera.json"))
    parser.add_argument("--sample-hz", type=float, default=4.)
    parser.add_argument("--rpc-timeout-s", type=float, default=.8)
    parser.add_argument("--max-frame-age-s", type=float, default=1.)
    parser.add_argument("--http-max-frame-age-s", type=float, default=.75)
    parser.add_argument("--http-timeout-s", type=float, default=.5)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--runtime-url", help="optional external/shadow runtime ingress; never enables motion")
    args = parser.parse_args(argv)
    for key in ("sample_hz", "rpc_timeout_s", "max_frame_age_s", "http_max_frame_age_s", "http_timeout_s", "duration_s"):
        value = getattr(args, key)
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"{key} must be positive and finite")
    if args.runtime_url:
        url = urlsplit(args.runtime_url)
        if url.scheme != "http" or not url.hostname or url.username or url.password or url.query or url.fragment:
            parser.error("runtime-url must be an explicit HTTP URL without credentials/query/fragment")
    config = load_config(args.config)
    lock_path = Path(".runtime/go2/camera.lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = lock_path.open("a+")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("another read-only Go2 camera process owns the project camera lock")
    state = CameraState(config["net_if"], args.max_frame_age_s)
    handoff = LatestFrame()
    stop = threading.Event()
    previous_signals = {}
    for signum in (signal.SIGTERM, signal.SIGINT):
        previous_signals[signum] = signal.signal(signum, lambda *_: stop.set())
    atomic_status(args.status_file, state.snapshot())
    node = None
    status_pub = None
    workers = []
    ros = None
    try:
        report = preflight(config)
        if not report["ok"]:
            print(json.dumps(report, indent=2))
            raise RuntimeError("Go2 link preflight failed before DDS initialization")
        if stop.is_set():
            return
        ingress = (ShadowIngress(args.runtime_url, state, args.http_max_frame_age_s,
                                 args.http_timeout_s) if args.runtime_url else None)
        if stop.is_set():
            return
        # Hardware/ROS imports and DDS initialization are beyond all early gates.
        import rclpy as ros
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient
        ros.init()
        node = Node("dejaview_go2_readonly_front_camera")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        image_pub = node.create_publisher(CompressedImage, IMAGE_TOPIC, qos)
        status_pub = node.create_publisher(String, STATUS_TOPIC, 1)
        ChannelFactoryInitialize(config["dds_domain_id"], config["net_if"])
        client = VideoClient()
        client.SetTimeout(args.rpc_timeout_s)
        client.Init()

        def publish(frame):
            message = CompressedImage()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(frame.receive_timestamp_ns, 1_000_000_000)
            message.header.frame_id, message.format, message.data = FRAME_NAME, "jpeg", frame.jpeg
            image_pub.publish(message)

        def status_loop():
            while not stop.is_set():
                value = state.snapshot(handoff.replaced)
                try:
                    atomic_status(args.status_file, value)
                    message = String()
                    message.data = json.dumps(value, allow_nan=False)
                    status_pub.publish(message)
                except Exception as exc:
                    state.error(f"status publication failed: {exc}")
                stop.wait(.1)

        def ingress_loop():
            while not stop.is_set():
                frame = handoff.take()
                if frame is not None:
                    ingress.send(frame)

        for name, target in (("go2-camera-status", status_loop),
                             ("go2-camera-shadow-ingress", ingress_loop)):
            if target == ingress_loop and ingress is None:
                continue
            worker = threading.Thread(target=target, name=name, daemon=True)
            worker.start()
            workers.append(worker)
        began = time.monotonic()
        next_sample = began
        prefix = "go2-video-"+uuid.uuid4().hex
        node.get_logger().info("Read-only Go2 front camera: host receive stamps; no exposure stamp, no extrinsics, no motion APIs")
        while ros.ok() and not stop.is_set():
            now = time.monotonic()
            if args.duration_s is not None and now-began >= args.duration_s:
                break
            if now < next_sample:
                stop.wait(min(next_sample-now, .05))
                continue
            capture_once(client, state, publish, handoff if ingress else None, prefix)
            next_sample = max(next_sample+1/args.sample_hz, time.monotonic())
            ros.spin_once(node, timeout_sec=0.)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        state.error(exc)
        raise
    finally:
        with state.lock:
            state.active = False
        stop.set()
        handoff.close()
        for worker in workers:
            worker.join(timeout=args.http_timeout_s*2+.2)
        final = state.snapshot(handoff.replaced)
        atomic_status(args.status_file, final)
        if node is not None:
            try:
                if status_pub is not None:
                    message = String()
                    message.data = json.dumps(final, allow_nan=False)
                    status_pub.publish(message)
            finally:
                node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
        lock_file.close()


if __name__ == "__main__":
    main()
