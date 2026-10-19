"""Official LightNav robot-client JPEG transform with bounded prefetch/cache.

The complete upstream client is hash locked. Only its CameraFrame definition,
literal encoder constants and original VlnClient._encode_image AST execute:
compressed camera JPEG -> OpenCV BGR decode -> INTER_AREA 480x270 -> JPEG 80.
The model performs its own checkpoint preprocessing after this wire transform.
Original source bytes, identities, timestamps and provenance remain unchanged.
No networking client instance, model, ROS or robot SDK is created here.
"""
from __future__ import annotations

import ast
import base64
from collections import OrderedDict
import copy
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import io
import json
from pathlib import Path
import queue
import threading
import time
from typing import Any

import numpy as np
from PIL import Image, UnidentifiedImageError


SCHEMA = "lightnav.observation_transport.v2"
OFFICIAL_COMMIT = "3015508b70fb6e30bbd66701e94c9b916dcb8df8"
OFFICIAL_SOURCE_SHA256 = "3598531abd85327714c8607e56bf7412a1085b77b437ff7f8b7d86deb78ce278"
OFFICIAL_VENDOR = Path(__file__).parent / "vendor/lightnav_official_client"
MAX_JPEG_BYTES = 4 << 20
MAX_SOURCE_PIXELS = 4096 * 2160


@dataclass(frozen=True)
class OfficialImageEncoder:
    encode: Any
    frame_type: type
    width: int
    height: int
    jpeg_quality: int
    opencv_version: str
    source_sha256: str


@lru_cache(maxsize=4)
def load_official_image_encoder(source_path=OFFICIAL_VENDOR / "vln_client.py",
                                manifest_path=OFFICIAL_VENDOR / "SOURCE.json") -> OfficialImageEncoder:
    """Reuse the verified original method without importing its network client."""
    import cv2

    source_path, manifest_path = Path(source_path), Path(manifest_path)
    source = source_path.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    manifest = json.loads(manifest_path.read_text())
    if (digest != OFFICIAL_SOURCE_SHA256 or manifest.get("source_sha256") != digest or
            manifest.get("commit") != OFFICIAL_COMMIT or manifest.get("unmodified") is not True):
        raise ValueError("Official LightNav client source/manifest integrity mismatch")
    tree = ast.parse(source, filename=str(source_path))
    constants = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in {"OUTPUT_WIDTH", "OUTPUT_HEIGHT", "JPEG_QUALITY"}:
                constants[name] = ast.literal_eval(node.value)
    if constants != manifest.get("constants"):
        raise ValueError("Official LightNav encoder constants differ from source manifest")
    frame = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "CameraFrame")
    client = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "VlnClient")
    encode = next(node for node in client.body if isinstance(node, ast.FunctionDef) and node.name == "_encode_image")
    encode.decorator_list = []  # Original static method body, loaded as a function.
    namespace = {"__name__": __name__, "dataclass": dataclass, "cv2": cv2,
                 "np": np, "base64": base64, **constants}
    exec(compile(ast.Module(body=[frame, encode], type_ignores=[]), str(source_path), "exec"), namespace)
    return OfficialImageEncoder(namespace["_encode_image"], namespace["CameraFrame"],
                                constants["OUTPUT_WIDTH"], constants["OUTPUT_HEIGHT"],
                                constants["JPEG_QUALITY"], cv2.__version__, digest)


class LightNavObservationTransport:
    """Deterministic fixed transform with a bounded content-addressed cache.

    Overlapping histories must reproduce identical pixels for a source frame,
    including after cache eviction: the model rejects changed overlap pixels.
    Cache entries depend on source JPEG SHA, never frame ID alone.
    """

    def __init__(self, cache_frames: int = 64, prefetch_queue_frames: int = 16):
        if type(cache_frames) is not int or not 1 <= cache_frames <= 256:
            raise ValueError("cache_frames must be 1..256")
        if type(prefetch_queue_frames) is not int or not 1 <= prefetch_queue_frames <= 64:
            raise ValueError("prefetch_queue_frames must be 1..64")
        self.cache_frames = cache_frames
        self._cache: OrderedDict[str, tuple[bytes, dict[str, Any]]] = OrderedDict()
        self._cache_lock = threading.RLock()
        # Cache lookups stay available during expensive encoding. A second
        # check under this lock makes foreground/background misses single-flight.
        self._encode_lock = threading.Lock()
        # Producer/lifecycle lock never protects encoding or waits on it.
        self._prefetch_lock = threading.Lock()
        self._prefetch_queue = queue.Queue(maxsize=prefetch_queue_frames)
        self._prefetch_pending: set[str] = set()
        self._prefetch_stop = threading.Event()
        self._prefetch_worker: threading.Thread | None = None
        self._prefetch_closed = False
        self._prefetch_counters = {name: 0 for name in
                                  ("accepted", "completed", "failed", "cancelled", "duplicate", "queue_full")}
        self._prefetch_last_error = None

    def prefetch(self, frame) -> bool:
        """Queue compression only, without waiting for encoding or queue space.

        One lazy daemon worker and at most ``prefetch_queue_frames`` queued
        immutable JPEG references are owned by this helper. No HTTP/model work
        runs here. False means closed, duplicate, busy producer, or full queue;
        ordinary observations() remains the authoritative fallback.
        """
        jpeg = frame.jpeg
        if not isinstance(jpeg, bytes) or not 0 < len(jpeg) <= MAX_JPEG_BYTES:
            return False
        if not self._prefetch_lock.acquire(blocking=False):
            return False
        try:
            if self._prefetch_closed:
                return False
            digest = hashlib.sha256(jpeg).hexdigest()
            if digest in self._prefetch_pending:
                self._prefetch_counters["duplicate"] += 1
                return False
            # Looking up a cached frame is optional. Never wait for even a
            # short cache lock on the ingestion/producer path.
            if self._cache_lock.acquire(blocking=False):
                try:
                    if digest in self._cache:
                        self._prefetch_counters["duplicate"] += 1
                        return False
                finally:
                    self._cache_lock.release()
            try:
                self._prefetch_queue.put_nowait((digest, str(frame.frame_id), jpeg))
            except queue.Full:
                self._prefetch_counters["queue_full"] += 1
                return False
            self._prefetch_pending.add(digest)
            self._prefetch_counters["accepted"] += 1
            if self._prefetch_worker is None:
                self._prefetch_worker = threading.Thread(
                    target=self._prefetch_loop, name=f"lightnav-observation-prefetch-{id(self):x}", daemon=True)
                self._prefetch_worker.start()
            return True
        finally:
            self._prefetch_lock.release()

    def _prefetch_loop(self) -> None:
        while not self._prefetch_stop.is_set():
            try:
                digest, frame_id, jpeg = self._prefetch_queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                if self._prefetch_stop.is_set():
                    with self._prefetch_lock:
                        self._prefetch_counters["cancelled"] += 1
                    continue
                try:
                    self._encode(jpeg)
                except Exception as exc:
                    with self._prefetch_lock:
                        self._prefetch_counters["failed"] += 1
                        self._prefetch_last_error = {
                            "frame_id": frame_id, "source_jpeg_sha256": digest,
                            "error": f"{type(exc).__name__}: {exc}"[:240]}
                else:
                    with self._prefetch_lock:
                        self._prefetch_counters["completed"] += 1
            finally:
                with self._prefetch_lock:
                    self._prefetch_pending.discard(digest)
                self._prefetch_queue.task_done()

    def prefetch_status(self) -> dict[str, Any]:
        with self._prefetch_lock:
            return {"closed": self._prefetch_closed,
                    "worker_alive": self._prefetch_worker is not None and self._prefetch_worker.is_alive(),
                    "queued": self._prefetch_queue.qsize(),
                    "queue_capacity": self._prefetch_queue.maxsize,
                    "pending": len(self._prefetch_pending),
                    **self._prefetch_counters,
                    "last_error": copy.deepcopy(self._prefetch_last_error)}

    def close(self, timeout_s: float = 1.) -> None:
        """Stop accepting prefetch, discard queued work, and join for a bound.

        An already decoding JPEG may finish after a short timeout; no new work
        will start. Calling close again can wait for that same owned worker.
        """
        if not isinstance(timeout_s, (int, float)) or not 0 <= timeout_s <= 5:
            raise ValueError("close timeout_s must be 0..5")
        with self._prefetch_lock:
            self._prefetch_closed = True
            self._prefetch_stop.set()
            while True:
                try:
                    digest, _, _ = self._prefetch_queue.get_nowait()
                except queue.Empty:
                    break
                self._prefetch_pending.discard(digest)
                self._prefetch_counters["cancelled"] += 1
                self._prefetch_queue.task_done()
            worker = self._prefetch_worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=timeout_s)

    def _cached(self, digest):
        with self._cache_lock:
            cached = self._cache.get(digest)
            if cached is None:
                return None
            self._cache.move_to_end(digest)
            return cached[0], copy.deepcopy(cached[1]), True

    def _encode(self, jpeg: bytes) -> tuple[bytes, dict[str, Any], bool]:
        if not isinstance(jpeg, bytes) or not 0 < len(jpeg) <= MAX_JPEG_BYTES:
            raise ValueError("Expected a JPEG smaller than 4 MiB")
        digest = hashlib.sha256(jpeg).hexdigest()
        cached = self._cached(digest)
        if cached is not None:
            return cached
        with self._encode_lock:
            # Foreground dispatch and the prefetch worker can miss together.
            # The winner encodes once; the other reuses the resulting cache.
            cached = self._cached(digest)
            if cached is not None:
                return cached
            encoded, lineage = self._encode_uncached(jpeg, digest)
            with self._cache_lock:
                self._cache[digest] = (encoded, copy.deepcopy(lineage))
                while len(self._cache) > self.cache_frames:
                    self._cache.popitem(last=False)
            return encoded, lineage, False

    def _encode_uncached(self, jpeg, digest):
        try:
            with Image.open(io.BytesIO(jpeg)) as image:
                if image.format != "JPEG" or image.width * image.height > MAX_SOURCE_PIXELS:
                    raise ValueError("Expected bounded source JPEG")
                source_size = [image.width, image.height]
                image.load()
            official = load_official_image_encoder()
            # Compressed input takes exactly the upstream cv2.imdecode branch;
            # the raw-RGB fields are unused. Encoder cache identity is pixels,
            # so source timestamps are retained only in the outer wire record.
            camera_frame = official.frame_type(stamp_sec=0, stamp_nanosec=0,
                          width=source_size[0], height=source_size[1], step=0,
                          rgb=b"", jpeg=jpeg)
            encoded = base64.b64decode(official.encode(camera_frame), validate=True)
        except (OSError, UnidentifiedImageError, Image.DecompressionBombError) as exc:
            raise ValueError("Invalid observation JPEG") from exc
        except RuntimeError as exc:
            raise ValueError("Official LightNav observation encoding failed") from exc
        lineage = {
            "schema_version": SCHEMA,
            "source_jpeg_sha256": digest, "source_jpeg_bytes": len(jpeg),
            "source_size_wh": source_size,
            "wire_jpeg_sha256": hashlib.sha256(encoded).hexdigest(),
            "wire_jpeg_bytes": len(encoded), "wire_size_wh": [official.width, official.height],
            "resize": "official_cv2_INTER_AREA",
            "jpeg": {"quality": official.jpeg_quality, "encoder": "cv2.imencode",
                     "other_parameters": "OpenCV defaults"},
            "opencv_version": official.opencv_version,
            "official_source": {"repository": "https://github.com/lightorigins/LightNav-0",
                "commit": OFFICIAL_COMMIT, "source_sha256": official.source_sha256,
                "path": "robot_deploy/src/vln_client/vln_client/vln_client.py",
                "entrypoint": "VlnClient._encode_image"},
            "lossy": True, "original_rgb_preserved_by_runtime": True,
        }
        return encoded, lineage

    def observations(self, frames) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        """Return LightNav-only observations plus auditable wire/source mapping.

        ``frames`` follows runtime.Frame's read-only attributes.  There is no
        goal or backend parameter: caller must use this only in the LightNav
        branch, leaving Frame.wire() and the NoMaD/goal payload unchanged.
        """
        frames = list(frames)
        if not 1 <= len(frames) <= 64:
            raise ValueError("LightNav requires 1..64 observation frames")
        result, records, cache_hits = [], [], 0
        started = time.monotonic_ns()
        for frame in frames:
            encoded, lineage, cached = self._encode(frame.jpeg)
            cache_hits += int(cached)
            result.append({"frame_id": frame.frame_id,
                           "capture_timestamp_ns": frame.stamp,
                           "rgb_jpeg_base64": base64.b64encode(encoded).decode("ascii"),
                           "rgb_transport": copy.deepcopy(lineage)})
            records.append({"frame_id": frame.frame_id,
                            "capture_timestamp_ns": frame.stamp,
                            "source_metadata": copy.deepcopy(frame.metadata),
                            "source_provenance": copy.deepcopy(frame.provenance), **lineage})
        return result, {"schema_version": SCHEMA, "frames": records,
                        "source_jpeg_bytes": sum(row["source_jpeg_bytes"] for row in records),
                        "wire_jpeg_bytes": sum(row["wire_jpeg_bytes"] for row in records),
                        "encode_ms": (time.monotonic_ns() - started) / 1e6,
                        "cache_hits": cache_hits, "prefetch": self.prefetch_status()}
