"""HTTP transport for native LightNav and an explicit Go2 ImageGoal extension.

No ROS, DDS, robot SDK, velocity endpoint, or automatic arrival decision.
"""
from __future__ import annotations

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import logging
from pathlib import Path
import socket
import threading
import time
import uuid

from .policy import HistoryConflictError, validate_session_identity

TRANSPORT_PROTOCOL = "lightnav.http.lifecycle.v2"
BODY_READ_TIMEOUT_S = 8.0


def decode_rgb(value):
    from PIL import Image

    if not isinstance(value, dict) or not isinstance(value.get("rgb_jpeg_base64"), str):
        raise ValueError("image requires rgb_jpeg_base64")
    if len(value["rgb_jpeg_base64"]) > 6 << 20:
        raise ValueError("Encoded image exceeds limit")
    data = base64.b64decode(value["rgb_jpeg_base64"], validate=True)
    with Image.open(io.BytesIO(data)) as image:
        if image.format != "JPEG" or image.width * image.height > 4096 * 2160:
            raise ValueError("Expected bounded RGB JPEG")
        image.load()
        return image.convert("RGB")


def parse_request(payload):
    if not isinstance(payload, dict):
        raise ValueError("Request must be an object")
    identity = payload.get("identity")
    required = {"session_id", "task_id", "goal_id", "goal_version", "frame_id"}
    if not isinstance(identity, dict) or set(identity) != required:
        raise ValueError("identity requires session_id, task_id, goal_id, goal_version, frame_id")
    for name in required - {"goal_version"}:
        if not isinstance(identity[name], str) or not identity[name] or len(identity[name]) > 256:
            raise ValueError(f"Invalid identity.{name}")
    if type(identity["goal_version"]) is not int or identity["goal_version"] < 0:
        raise ValueError("goal_version must be a nonnegative integer")
    observations = payload.get("observations")
    if not isinstance(observations, list) or not 1 <= len(observations) <= 64:
        raise ValueError("observations must contain 1..64 chronological real source frames")
    frames, ids, stamps = [], [], []
    for value in observations:
        if not isinstance(value, dict):
            raise ValueError("Observation must be an object")
        fid, stamp = value.get("frame_id"), value.get("capture_timestamp_ns")
        if not isinstance(fid, str) or not fid or len(fid) > 256 or type(stamp) is not int or stamp <= 0:
            raise ValueError("Invalid observation lineage")
        ids.append(fid); stamps.append(stamp)
        frames.append({"frame_id": fid, "capture_timestamp_ns": stamp, "image": decode_rgb(value)})
    if len(set(ids)) != len(ids) or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Observations require unique IDs and strictly increasing timestamps")
    if ids[-1] != identity["frame_id"]:
        raise ValueError("Identity must name the newest source frame")
    instruction = payload.get("instruction")
    if instruction is not None and (not isinstance(instruction, str) or len(instruction) > 8192):
        raise ValueError("instruction must be a bounded string")
    mode = payload.get("conditioning_mode", "native_language")
    if mode not in ("native_language", "native_tracking", "experimental_reference_image"):
        raise ValueError("Unknown conditioning_mode")
    imagegoal = mode == "experimental_reference_image"
    default_task = "imagegoal" if imagegoal else "tracking" if mode == "native_tracking" else "objnav"
    task_mode = payload.get("task_mode", default_task)
    allowed_tasks = {"imagegoal"} if imagegoal else {"tracking"} if mode == "native_tracking" else {"objnav", "vln"}
    if not isinstance(task_mode, str) or task_mode not in allowed_tasks:
        raise ValueError("task_mode and conditioning_mode disagree")
    if not imagegoal and not (instruction and instruction.strip()):
        raise ValueError(f"{mode} requires a nonempty instruction")
    if "goal_image" in payload:
        raise ValueError("Use the goal field only in experimental_reference_image mode, not goal_image")
    if imagegoal:
        goal = decode_rgb(payload.get("goal"))
    else:
        if "goal" in payload:
            raise ValueError("Official LightNav does not accept a separate goal image; select experimental_reference_image explicitly")
        goal = None
    return frames, goal, identity, instruction, mode, task_mode


class Server(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address, policy, record_dir):
        self.policy, self.record_dir = policy, record_dir
        self.server_boot_id = uuid.uuid4().hex
        self.slot = threading.Lock()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lightnav-model")
        self._cleanup_lock = threading.Lock()
        self._cleanup_scheduled = False
        self._closing = False
        self._connections = set()
        self._connections_lock = threading.Lock()
        super().__init__(address, Handler)

    def schedule_cleanup(self):
        if not hasattr(self.policy, "cleanup_pending") or not self.policy.cleanup_pending():
            return
        with self._cleanup_lock:
            if self._cleanup_scheduled or self._closing:
                return
            self._cleanup_scheduled = True
        try:
            self.worker.submit(self._drain_cleanup)
        except RuntimeError:
            with self._cleanup_lock:
                self._cleanup_scheduled = False
            if not self._closing:
                raise

    def _drain_cleanup(self):
        failed = False
        try:
            while self.policy.cleanup_pending():
                self.policy.cleanup_retired()
        except Exception:
            failed = True
            logging.exception("Deferred LightNav retirement cleanup failed; next control request may retry")
        finally:
            with self._cleanup_lock:
                self._cleanup_scheduled = False
            if not failed:
                self.schedule_cleanup()

    def expire_idle(self):
        if hasattr(self.policy, "expire_idle"):
            self.policy.expire_idle()
            self.schedule_cleanup()

    def predict(self, parsed):
        identity = parsed[2]
        if hasattr(self.policy, "begin_predict"):
            self.policy.begin_predict(identity)
        try:
            result = self.policy.predict(*parsed)
            if hasattr(self.policy, "assert_active"):
                self.policy.assert_active(identity)
            return result
        finally:
            if hasattr(self.policy, "end_predict"):
                self.policy.end_predict(identity)

    def get_request(self):
        connection, address = super().get_request()
        with self._connections_lock:
            self._connections.add(connection)
        return connection, address

    def shutdown_request(self, request):
        try:
            super().shutdown_request(request)
        finally:
            with self._connections_lock:
                self._connections.discard(request)

    def server_close(self):
        self._closing = True
        super().server_close()
        # HTTP/1.1 handlers can otherwise remain blocked on their next request
        # after the listener and model worker have been closed.
        with self._connections_lock:
            connections = tuple(self._connections)
        for connection in connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.worker.shutdown(wait=True, cancel_futures=True)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(30)

    def reply(self, status, payload):
        payload = {**payload, "server_boot_id": self.server.server_boot_id,
                   "transport_protocol": TRANSPORT_PROTOCOL}
        body = json.dumps(payload, allow_nan=False).encode()
        # Errors may be generated before the body is consumed. Never let
        # unread bytes become another request on a persistent connection.
        if status >= 400:
            self.close_connection = True
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if self.close_connection:
            self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def error(self, status, message, *, error_code="invalid_request", retryable=False, identity=None):
        payload = {"error": message, "error_code": error_code, "retryable": retryable}
        if identity is not None:
            payload["identity"] = identity
        self.reply(status, payload)

    def body_json(self, *, limit):
        lengths = self.headers.get_all("Content-Length", [])
        if len(lengths) != 1 or not lengths[0].isascii() or not lengths[0].isdigit():
            raise ValueError("Use one nonnegative decimal Content-Length")
        length = int(lengths[0])
        if not 0 < length <= limit:
            raise ValueError(f"Request body must be 1..{limit} bytes")
        timeout_header = self.headers.get_all("X-LightNav-Request-Timeout-Ms", [])
        timeout_s = BODY_READ_TIMEOUT_S
        if timeout_header:
            if len(timeout_header) != 1 or not timeout_header[0].isascii() or not timeout_header[0].isdigit():
                raise ValueError("X-LightNav-Request-Timeout-Ms must be one bounded integer")
            timeout_ms = int(timeout_header[0])
            if not 100 <= timeout_ms <= int(BODY_READ_TIMEOUT_S * 1000):
                raise ValueError("Request timeout must be between 100 and 8000 ms")
            timeout_s = timeout_ms / 1000.
        deadline = time.monotonic() + timeout_s
        prior_timeout = self.connection.gettimeout()
        chunks, remaining = [], length
        try:
            while remaining:
                time_left = deadline - time.monotonic()
                if time_left <= 0:
                    raise TimeoutError("Request body exceeded its total read budget")
                self.connection.settimeout(min(time_left, prior_timeout or time_left))
                chunk = self.rfile.read1(min(remaining, 65536))
                if not chunk:
                    raise ValueError("Request body ended before Content-Length")
                chunks.append(chunk)
                remaining -= len(chunk)
            if time.monotonic() > deadline:
                raise TimeoutError("Request body exceeded its total read budget")
        finally:
            self.connection.settimeout(prior_timeout)
        payload = json.loads(b"".join(chunks))
        if not isinstance(payload, dict):
            raise ValueError("Request must be an object")
        return payload

    def admission_identity(self):
        headers = [self.headers.get_all(name, []) for name in
                   ("X-LightNav-Session-Id", "X-LightNav-Task-Id")]
        if not any(headers):
            return None
        if any(len(values) != 1 for values in headers):
            raise ValueError("Session/task admission headers must appear once as a pair")
        return validate_session_identity(dict(zip(("session_id", "task_id"), (x[0] for x in headers))),
                                         task_required=True)

    def admission_boot_id(self):
        values = self.headers.get_all("X-LightNav-Server-Boot-Id", [])
        if not values:
            return None
        if len(values) != 1 or not values[0] or not values[0].isascii() or len(values[0]) > 256:
            raise ValueError("X-LightNav-Server-Boot-Id must be one bounded nonempty ASCII string")
        if values[0] != self.server.server_boot_id:
            raise HistoryConflictError("LightNav server restarted; begin a fresh task with verified history",
                                       "server_restarted")
        return values[0]

    def do_GET(self):
        if self.path != "/healthz":
            self.error(404, "Unknown endpoint", error_code="unknown_endpoint"); return
        if self.headers.get("Transfer-Encoding") is not None or self.headers.get_all("Content-Length", []) not in ([], ["0"]):
            self.error(400, "GET healthz does not accept a request body"); return
        self.server.expire_idle()
        diagnostics = self.server.policy.session_diagnostics() if hasattr(self.server.policy, "session_diagnostics") else {}
        self.reply(200, {"ok": True, "model_loaded": True, "inference_busy": self.server.slot.locked(),
                         "contract": self.server.policy.contract, **diagnostics})

    def do_POST(self):
        if self.path not in ("/infer", "/reset", "/session/status"):
            self.error(404, "Unknown endpoint", error_code="unknown_endpoint"); return
        if self.headers.get_content_type() != "application/json" or self.headers.get("Transfer-Encoding") is not None:
            self.error(415, "Use application/json with Content-Length"); return
        identity = None
        boot_id = None
        try:
            self.server.expire_idle()
            if self.path == "/infer":
                boot_id = self.admission_boot_id()
                identity = self.admission_identity()
                if identity and hasattr(self.server.policy, "assert_active"):
                    self.server.policy.assert_active(identity, check_capacity=True)
            payload = self.body_json(limit=(64 << 20) if self.path == "/infer" else 4096)
            if self.path == "/session/status":
                if set(payload) != {"identity"}:
                    raise ValueError("session/status accepts only identity")
                identity = validate_session_identity(payload["identity"], task_required=True)
                if not hasattr(self.server.policy, "session_status"):
                    self.error(501, "Session status is unavailable", error_code="unsupported_protocol"); return
                snapshot = self.server.policy.session_status(identity)
                self.reply(200, {**snapshot, "inference_busy": self.server.slot.locked()})
                return
            if self.path == "/reset":
                identity = validate_session_identity(payload.get("identity"))
                retire = payload.get("retire", False)
                if type(retire) is not bool:
                    raise ValueError("retire must be boolean")
                if retire and hasattr(self.server.policy, "retire"):
                    result = self.server.policy.retire(identity)
                    self.server.schedule_cleanup()
                    self.reply(200, result)
                    return
                if not self.server.slot.acquire(blocking=False):
                    self.error(429, "Inference busy; caller cancellation invalidates the pending result locally",
                               error_code="inference_busy", retryable=True, identity=identity)
                    return
                try:
                    args = (payload.get("identity"), True) if retire else (payload.get("identity"),)
                    self.reply(200, self.server.worker.submit(self.server.policy.reset, *args).result())
                finally:
                    self.server.slot.release()
                return
            if "server_boot_id" in payload and (boot_id is None or payload["server_boot_id"] != boot_id):
                raise ValueError("Payload server_boot_id requires the matching admission header")
            parsed = parse_request(payload)
            if identity and any(parsed[2][key] != value for key, value in identity.items()):
                raise ValueError("Admission header identity does not match the request body")
            identity = parsed[2]
            if hasattr(self.server.policy, "assert_active"):
                self.server.policy.assert_active(identity, check_capacity=True)
        except TimeoutError:
            self.error(408, "Timed out reading request body", error_code="request_read_timeout",
                       retryable=True, identity=identity); return
        except HistoryConflictError as exc:
            self.error(exc.http_status, str(exc), error_code=exc.error_code, identity=identity); return
        except Exception as exc:
            self.error(400, str(exc), identity=identity); return
        if not self.server.slot.acquire(blocking=False):
            self.error(429, "Inference busy; retry after reconciling the latest committed context",
                       error_code="inference_busy", retryable=True, identity=identity); return
        record = None
        try:
            if self.server.record_dir:
                record = self.server.record_dir / f"{time.time_ns()}_{uuid.uuid4().hex[:8]}"
                record.mkdir(parents=True)
                (record / "request.json").write_text(json.dumps(payload))
            result = self.server.worker.submit(self.server.predict, parsed).result()
            result = {**result, "server_boot_id": self.server.server_boot_id,
                      "transport_protocol": TRANSPORT_PROTOCOL}
            if record:
                (record / "response.json").write_text(json.dumps(result, allow_nan=False, indent=2) + "\n")
            if hasattr(self.server.policy, "assert_active"):
                self.server.policy.assert_active(identity)
            self.reply(200, result)
        except Exception as exc:
            status = getattr(exc, "http_status", 500)
            if status < 500:
                logging.warning("LightNav request rejected: %s", exc)
            else:
                logging.exception("LightNav inference failed")
            error = {"error": str(exc), "identity": payload.get("identity"),
                     "error_code": getattr(exc, "error_code", "inference_failed"),
                     "retryable": getattr(exc, "retryable", False),
                     "server_boot_id": self.server.server_boot_id, "transport_protocol": TRANSPORT_PROTOCOL}
            if record:
                (record / "error.json").write_text(json.dumps(error, indent=2) + "\n")
            self.reply(status, error)
        finally:
            self.server.slot.release()
            self.server.schedule_cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-path", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18884)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--record-dir", type=Path)
    parser.add_argument("--max-new-tokens", type=int, default=12)
    parser.add_argument("--max-session-frames", type=int, default=4096)
    parser.add_argument("--max-sessions", type=int, default=2)
    parser.add_argument("--session-idle-ttl-s", type=float, default=120.)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    from .policy import LightNavPolicy
    if args.max_session_frames < 64 or args.max_sessions < 1 or not 10 <= args.session_idle_ttl_s <= 86400:
        parser.error("max-session-frames >=64, max-sessions >=1, and session-idle-ttl-s in 10..86400 are required")
    policy = LightNavPolicy(args.model_path, args.device, args.max_new_tokens,
                                    args.max_session_frames, args.max_sessions, args.session_idle_ttl_s)
    if args.record_dir:
        args.record_dir.mkdir(parents=True, exist_ok=True)
    server = Server((args.host, args.port), policy, args.record_dir)
    logging.info("Loaded official LightNav core and explicit ImageGoal adapter; HTTP ready at %s:%d", args.host, args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
