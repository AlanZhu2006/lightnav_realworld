"""Local NoMaD inference HTTP transport. No ROS, DDS or movement endpoints.

POST /infer accepts identity, four chronological observations and one goal.
Each image is an object with ``rgb_jpeg_base64``; observations also require
``frame_id`` and ``capture_timestamp_ns``. GET /healthz reports the loaded model.
Run from the repository root with ``python3 -m deployment.nomad.server --help``.
"""
from __future__ import annotations

import argparse
import base64
import binascii
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import logging
from pathlib import Path
import threading
from typing import Any, Dict

from .policy import InferenceIdentity, NoMaDPolicy, RGBFrame, validate_observations

LOG = logging.getLogger("nomad.server")
MAX_IMAGE_BYTES = 4 * 1024 * 1024
MAX_IMAGE_PIXELS = 4096 * 2160


def decode_rgb_image(value: Any):
    from PIL import Image

    if not isinstance(value, dict) or not isinstance(value.get("rgb_jpeg_base64"), str):
        raise ValueError("each image requires an rgb_jpeg_base64 string")
    encoded = value["rgb_jpeg_base64"]
    if len(encoded) > 4 * ((MAX_IMAGE_BYTES + 2) // 3):
        raise ValueError("image exceeds the 4 MiB encoded-file limit")
    try:
        blob = base64.b64decode(encoded, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("invalid image base64") from exc
    if len(blob) > MAX_IMAGE_BYTES:
        raise ValueError("image exceeds the 4 MiB encoded-file limit")
    try:
        with Image.open(io.BytesIO(blob)) as image:
            if image.format != "JPEG":
                raise ValueError("rgb_jpeg_base64 must contain JPEG bytes")
            if image.width * image.height > MAX_IMAGE_PIXELS:
                raise ValueError("decoded image dimensions exceed the limit")
            image.load()
            return image.convert("RGB")
    except (OSError, Image.DecompressionBombError) as exc:
        raise ValueError("JPEG decoding failed") from exc


def parse_request(payload: Any):
    if not isinstance(payload, dict):
        raise ValueError("request must be a JSON object")
    metadata = payload.get("identity")
    names = {"session_id", "task_id", "goal_id", "goal_version", "frame_id"}
    if not isinstance(metadata, dict) or set(metadata) != names:
        raise ValueError("identity requires session_id, task_id, goal_id, goal_version and frame_id")
    identity = InferenceIdentity(**metadata)
    values = payload.get("observations")
    if not isinstance(values, list) or len(values) != 4:
        raise ValueError("observations must contain exactly four chronological frames")
    frames = []
    for value in values:
        if not isinstance(value, dict):
            raise ValueError("each observation must be an object")
        frames.append(RGBFrame(image=None, frame_id=value.get("frame_id"),
                               capture_timestamp_ns=value.get("capture_timestamp_ns")))
    validate_observations(frames, identity)
    decoded = [RGBFrame(image=decode_rgb_image(value), frame_id=frame.frame_id,
                        capture_timestamp_ns=frame.capture_timestamp_ns)
               for value, frame in zip(values, frames)]
    return decoded, decode_rgb_image(payload.get("goal")), identity


class InferenceServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, policy: NoMaDPolicy, max_body_bytes: int = 24 * 1024 * 1024):
        self.policy = policy
        self.max_body_bytes = max_body_bytes
        self.inference_slot = threading.Lock()
        # CUDA/cuDNN cache initialization is thread-local. HTTP request threads
        # are short lived, whereas inference always uses this persistent worker.
        # The slot below still rejects concurrent requests, so no old-frame
        # inference queue is introduced by the executor.
        self.inference_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="nomad-inference")
        super().__init__(address, InferenceHandler)

    def server_close(self):
        super().server_close()
        self.inference_worker.shutdown(wait=True, cancel_futures=True)


class InferenceHandler(BaseHTTPRequestHandler):
    server: InferenceServer

    def setup(self):
        super().setup()
        self.connection.settimeout(15)

    def _reply(self, status: int, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path != "/healthz":
            self._reply(404, {"error": "unknown endpoint"})
            return
        self._reply(200, {"ok": True, "model_loaded": True,
                          "inference_busy": self.server.inference_slot.locked(),
                          "inference_execution": "single_persistent_worker_no_request_queue",
                          "contract": self.server.policy.contract})

    def do_POST(self):
        if self.path != "/infer":
            self._reply(404, {"error": "unknown endpoint"})
            return
        if self.headers.get_content_type() != "application/json":
            self._reply(415, {"error": "Content-Type must be application/json"})
            return
        if self.headers.get("Transfer-Encoding"):
            self._reply(400, {"error": "chunked requests are unsupported; send Content-Length"})
            return
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._reply(411, {"error": "a numeric Content-Length is required"})
            return
        if length <= 0 or length > self.server.max_body_bytes:
            self._reply(413, {"error": "request body exceeds the configured size limit"})
            return
        if not self.server.inference_slot.acquire(blocking=False):
            self._reply(503, {"error": "inference busy; send the newest complete observation later"})
            return
        try:
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise ValueError("incomplete request body")
            payload = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(
                ValueError(f"nonfinite JSON value: {value}")))
            observations, goal, identity = parse_request(payload)
            result = self.server.inference_worker.submit(
                self.server.policy.predict, observations, goal, identity).result()
            self._reply(200, result)
        except (ValueError, TypeError, UnicodeDecodeError) as exc:
            self._reply(400, {"error": str(exc)})
        except TimeoutError:
            self._reply(408, {"error": "request body read timed out"})
        except (BrokenPipeError, ConnectionResetError):
            LOG.info("inference client disconnected")
        except Exception:
            LOG.exception("NoMaD inference failed")
            self._reply(500, {"error": "NoMaD inference failed; inspect server log"})
        finally:
            self.server.inference_slot.release()

    def log_message(self, format, *args):
        LOG.info("%s %s", self.client_address[0], format % args)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, required=True, help="pinned official visualnav-transformer checkout")
    parser.add_argument("--checkpoint", type=Path, required=True, help="local official nomad.pth; never downloaded")
    parser.add_argument("--diffusion-policy", type=Path, required=True, help="vendor root containing diffusion_policy/")
    parser.add_argument("--checkpoint-sha256", help="optional expected checkpoint SHA256")
    parser.add_argument("--device", default="cpu", help="cpu, cuda or cuda:N")
    parser.add_argument("--seed", type=int, default=260806)
    parser.add_argument("--model-max-v", type=float, default=0.2, help="model action calibration, independent of Go2 speed limit")
    parser.add_argument("--model-frame-rate", type=float, default=4.0, help="model action calibration, independent of inference frequency")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8091)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    policy = NoMaDPolicy(
        upstream=args.upstream, checkpoint=args.checkpoint, diffusion_policy=args.diffusion_policy,
        device=args.device, seed=args.seed, model_max_v=args.model_max_v,
        model_frame_rate=args.model_frame_rate, expected_checkpoint_sha256=args.checkpoint_sha256)
    server = InferenceServer((args.host, args.port), policy)
    LOG.info("NoMaD inference listening on http://%s:%s; robot transport absent", args.host, args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
