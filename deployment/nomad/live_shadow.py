"""Bounded live RGB -> NoMaD HTTP shadow capture, with no movement interfaces.

The camera thread continuously dequeues V4L2 frames and selects distinct real
frames at 4 Hz. The main thread sends one request at a time using the latest
four samples. Queueing old inference requests is intentionally impossible.
Host dequeue timestamps are proxies, not sensor exposure times. All images,
predictions and age measurements are ``not_for_motion``.

Use --goal for a fixed RGB goal. Without it, the first recorded RGB becomes a
same-view IO diagnostic goal, explicitly not a navigation trial. There is no
odometry, MPC, ROS, DDS, SDK, robot command or automatic reconnection here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import statistics
import threading
import time
from typing import Any
from urllib.parse import urlsplit
import uuid

from .shadow_client import (
    _dump_json, adapt_response, encode_rgb_jpeg, inspect_response, request_json,
)


TIMESTAMP_SOURCE = "host_wall_clock_after_v4l2_dequeue_not_sensor_exposure"
AGE_SOURCE = "host_monotonic_after_v4l2_dequeue_proxy_not_sensor_exposure"


class CameraSampler:
    """Own the only camera reader; publish a bounded latest-window snapshot."""

    def __init__(self, args: argparse.Namespace, output: Path, deadline_ns: int):
        self.args, self.output, self.deadline_ns = args, output, deadline_ns
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.frames: list[dict[str, Any]] = []
        self.properties: dict[str, Any] = {}
        self.error: str | None = None
        self.camera: Any = None
        self.read_count = 0
        self.release_called = False
        self.forced_release_called = False
        self.finished = False
        self.thread = threading.Thread(target=self._run, name="nomad-rgb-dequeue", daemon=True)

    def _run(self) -> None:
        camera = None
        try:
            import cv2
            camera = cv2.VideoCapture(self.args.device, cv2.CAP_V4L2)
            self.camera = camera
            if not camera.isOpened():
                raise RuntimeError(f"cannot open camera {self.args.device}")
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
            camera.set(cv2.CAP_PROP_FPS, self.args.camera_fps)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # Some V4L2 backends ignore this; shutdown state is reported below.
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                camera.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)
            self.properties = {
                "width": camera.get(cv2.CAP_PROP_FRAME_WIDTH),
                "height": camera.get(cv2.CAP_PROP_FRAME_HEIGHT),
                "fps_reported": camera.get(cv2.CAP_PROP_FPS),
                "backend": "OpenCV CAP_V4L2", "buffer_size_requested": 1,
            }
            next_sample_ns = 0
            period_ns = int(1e9 / self.args.sample_hz)
            previous_wall_ns = -1
            while not self.stop_event.is_set() and time.monotonic_ns() < self.deadline_ns:
                ok, image_bgr = camera.read()
                dequeue_monotonic_ns = time.monotonic_ns()
                dequeue_wall_ns = time.time_ns()
                if self.stop_event.is_set() or dequeue_monotonic_ns >= self.deadline_ns:
                    break
                if not ok or image_bgr is None:
                    raise RuntimeError("camera frame read failed; no reconnect attempted")
                self.read_count += 1
                if self.read_count <= self.args.warmup_frames or dequeue_monotonic_ns < next_sample_ns:
                    continue
                if dequeue_wall_ns <= previous_wall_ns:
                    raise RuntimeError("host wall clock moved backwards; original timestamps were not modified")
                index = len(self.frames) + 1
                relative_path = f"rgb/{index:06d}.png"
                if not cv2.imwrite(str(self.output / relative_path), image_bgr):
                    raise RuntimeError("failed to save original camera RGB")
                frame = {
                    "frame_id": f"live-rgb-{index:06d}",
                    "capture_timestamp_ns": dequeue_wall_ns,
                    "host_monotonic_ns": dequeue_monotonic_ns,
                    "timestamp_source": TIMESTAMP_SOURCE,
                    "path": relative_path,
                    "shape_hwc": list(image_bgr.shape),
                    "camera_dequeue_index": self.read_count,
                    "stored_encoding": "PNG_RGB_decoded_by_PIL",
                }
                with self.condition:
                    self.frames.append(frame)
                    self.condition.notify_all()
                previous_wall_ns = dequeue_wall_ns
                # Keep the 4 Hz schedule while selecting real 15 Hz frames
                # (intervals naturally alternate near that rate). Never catch
                # up by duplicating a read after a long camera/disk stall.
                next_sample_ns = next_sample_ns + period_ns if next_sample_ns else dequeue_monotonic_ns + period_ns
                if next_sample_ns <= dequeue_monotonic_ns:
                    next_sample_ns = dequeue_monotonic_ns + period_ns
        except Exception as exc:
            with self.condition:
                self.error = f"{type(exc).__name__}: {exc}"
                self.condition.notify_all()
        finally:
            if camera is not None:
                try:
                    camera.release()
                    self.release_called = True
                except Exception as exc:
                    self.error = self.error or f"camera release failed: {exc}"
            with self.condition:
                self.finished = True
                self.condition.notify_all()

    def latest_window(self, previous_frame_id: str | None) -> list[dict[str, Any]] | None:
        """Wait for a new complete window; never return the same newest frame."""
        with self.condition:
            while time.monotonic_ns() < self.deadline_ns and not self.stop_event.is_set():
                if self.error is not None:
                    raise RuntimeError(self.error)
                if len(self.frames) >= 4 and self.frames[-1]["frame_id"] != previous_frame_id:
                    return [dict(frame) for frame in self.frames[-4:]]
                if self.finished:
                    return None
                remaining_s = (self.deadline_ns - time.monotonic_ns()) / 1e9
                self.condition.wait(timeout=min(0.2, max(0.0, remaining_s)))
            return None

    def stop(self) -> None:
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread.ident is None:
            return
        self.thread.join(timeout=3.0)
        if self.thread.is_alive() and self.camera is not None:
            # Normally read returns at the requested camera frame rate. This
            # bounded fallback attempts to unblock an unplugged/stalled source.
            self.forced_release_called = True
            try:
                self.camera.release()
            except Exception as exc:
                self.error = self.error or f"forced camera release failed: {exc}"
            self.thread.join(timeout=1.0)

    def state(self) -> dict[str, Any]:
        with self.condition:
            return {
                "frames_sampled": len(self.frames), "frames_dequeued": self.read_count,
                "thread_exited": not self.thread.is_alive(), "camera_release_called": self.release_called,
                "forced_release_called": self.forced_release_called, "camera_error": self.error,
                "camera_properties": dict(self.properties),
            }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="explicit V4L2 RGB node, e.g. /dev/video4")
    parser.add_argument("--server-url", required=True)
    parser.add_argument("--output", type=Path, required=True, help="new run directory")
    bounds = parser.add_mutually_exclusive_group(required=True)
    bounds.add_argument("--duration-s", type=float, help="bounded capture/inference duration after health check")
    bounds.add_argument("--requests", type=int, help="maximum attempted requests, with derived finite wall-time bound")
    parser.add_argument("--goal", type=Path, help="fixed goal RGB; default is first frame, same-view IO diagnostic only")
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument("--camera-fps", type=float, default=15.0)
    parser.add_argument("--sample-hz", type=float, default=4.0)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--v-ref-mps", type=float, default=0.2, help="spatial resampling configuration only")
    parser.add_argument("--mpc-dt-s", type=float, default=0.1, help="spatial resampling configuration only; no MPC is run")
    args = parser.parse_args(argv)
    for name in ("timeout_s", "camera_fps", "sample_hz", "v_ref_mps", "mpc_dt_s"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.duration_s is not None and (not math.isfinite(args.duration_s) or args.duration_s <= 0):
        parser.error("--duration-s must be finite and positive")
    if args.requests is not None and args.requests <= 0:
        parser.error("--requests must be positive")
    if args.camera_fps < args.sample_hz or args.warmup_frames < 0 or args.width <= 0 or args.height <= 0:
        parser.error("invalid sampling, warmup or image size")
    base = args.server_url.rstrip("/")
    parsed = urlsplit(base)
    if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.query or parsed.fragment:
        parser.error("--server-url must be an HTTP(S) base URL")
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error("--output must be a new run directory")
    goal_source = args.goal.expanduser().resolve(strict=True) if args.goal is not None else None
    output.mkdir(parents=True)
    (output / "rgb").mkdir()
    session_id = "live-shadow-" + uuid.uuid4().hex
    manifest_path = output / "manifest.json"
    manifest: dict[str, Any] = {
        "schema_version": "nomad.rgb_capture.v1", "purpose": "live_rgb_http_shadow",
        "not_for_motion": True, "timestamp_source": TIMESTAMP_SOURCE, "device": args.device,
        "sample_hz_requested": args.sample_hz, "camera_fps_requested": args.camera_fps,
        "frames": [],
    }
    run: dict[str, Any] = {
        # Keep the recorded-shadow schema for the existing replay viewer.
        "schema_version": "nomad.offline_shadow.v1", "acquisition_mode": "live_camera_http_shadow",
        "not_for_motion": True, "session_id": session_id, "manifest_path": str(manifest_path),
        "server_url": base, "goal_mode": "fixed_image" if goal_source else "same_view_io_diagnostic",
        "timestamp_notice": TIMESTAMP_SOURCE, "source_age_timestamp_source": AGE_SOURCE,
        "robot_feedback_used": False, "mpc_mode": "not_requested", "max_inflight_requests": 1,
        "adapter_configuration": {"v_ref_mps": args.v_ref_mps, "mpc_dt_s": args.mpc_dt_s},
    }
    camera = None
    attempts, completed, failures = 0, 0, 0
    rtts: list[float] = []
    source_ages: list[float] = []
    error: str | None = None
    reason = "not_started"
    try:
        health, health_rtt_ms = request_json(base + "/healthz", timeout_s=args.timeout_s)
        if not isinstance(health, dict) or health.get("ok") is not True or health.get("model_loaded") is not True:
            raise ValueError("server health check did not confirm a loaded model")
        run.update({"server_healthz": health, "healthz_rtt_ms": health_rtt_ms})
        bound_s = args.duration_s if args.duration_s is not None else (
            args.warmup_frames / args.camera_fps + 4 / args.sample_hz + args.requests * (args.timeout_s + 1) + 5)
        started_ns = time.monotonic_ns()
        deadline_ns = started_ns + int(bound_s * 1e9)
        run.update({"duration_bound_s": bound_s, "request_count_bound": args.requests,
                    "started_host_monotonic_ns": started_ns})
        camera = CameraSampler(args, output, deadline_ns)
        camera.thread.start()
        goal_encoded = None
        goal_meta = None
        if goal_source is not None:
            goal_copy = output / ("goal" + goal_source.suffix)
            shutil.copyfile(goal_source, goal_copy)
            goal_encoded, goal_meta = encode_rgb_jpeg(goal_copy)
            run["goal"] = goal_meta
        previous_frame_id = None
        with (output / "results.jsonl").open("x") as stream:
            while time.monotonic_ns() < deadline_ns and (args.requests is None or attempts < args.requests):
                window = camera.latest_window(previous_frame_id)
                if window is None:
                    reason = "duration_bound"
                    break
                previous_frame_id = window[-1]["frame_id"]
                if goal_encoded is None:
                    # The first actual sampled frame, never a fabricated RGB.
                    with camera.condition:
                        first_frame = dict(camera.frames[0])
                    goal_copy = output / "goal.png"
                    shutil.copyfile(output / first_frame["path"], goal_copy)
                    goal_encoded, goal_meta = encode_rgb_jpeg(goal_copy)
                    run["goal"] = goal_meta
                frames = [{**frame, "resolved_path": str(output / frame["path"]), "manifest_frame": frame}
                          for frame in window]
                identity = {"session_id": session_id, "task_id": "live-shadow-no-motion",
                            "goal_id": goal_meta["original_sha256"], "goal_version": 0,
                            "frame_id": window[-1]["frame_id"]}
                record: dict[str, Any] = {
                    "window_index": attempts, "identity": identity, "frames": frames,
                    "not_for_motion": True, "source_age_timestamp_source": AGE_SOURCE,
                    "recorded_frame_intervals_ms": [
                        (b["capture_timestamp_ns"] - a["capture_timestamp_ns"]) / 1e6
                        for a, b in zip(window, window[1:])],
                }
                attempts += 1
                sent_ns = None
                try:
                    observations, image_metadata = [], []
                    for frame in window:
                        encoded, metadata = encode_rgb_jpeg(output / frame["path"])
                        observations.append({"frame_id": frame["frame_id"],
                                             "capture_timestamp_ns": frame["capture_timestamp_ns"],
                                             "rgb_jpeg_base64": encoded})
                        image_metadata.append(metadata)
                    payload = {"identity": identity, "observations": observations,
                               "goal": {"rgb_jpeg_base64": goal_encoded}}
                    sent_ns = time.monotonic_ns()
                    remaining_s = (deadline_ns - sent_ns) / 1e9
                    if remaining_s <= 0:
                        raise TimeoutError("duration bound reached before HTTP submission")
                    record["source_age_at_submission_ms"] = (sent_ns - window[-1]["host_monotonic_ns"]) / 1e6
                    response, rtt = request_json(base + "/infer", timeout_s=min(args.timeout_s, remaining_s), payload=payload)
                    received_ns = time.monotonic_ns()
                    source_age_ms = (received_ns - window[-1]["host_monotonic_ns"]) / 1e6
                    record.update({"rtt_ms": rtt, "source_age_at_response_ms": source_age_ms,
                                   "request_started_host_monotonic_ns": sent_ns,
                                   "response_received_host_monotonic_ns": received_ns,
                                   "image_metadata": image_metadata, "response": response})
                    rtts.append(rtt)
                    source_ages.append(source_age_ms)
                    checks = inspect_response(response, payload)
                    record["checks"] = checks
                    if not all(checks.values()):
                        raise ValueError("response contract failed: " + ", ".join(key for key, ok in checks.items() if not ok))
                    record["adaptation"] = adapt_response(
                        response, source_stamp_ns=window[-1]["capture_timestamp_ns"],
                        v_ref_mps=args.v_ref_mps, mpc_dt_s=args.mpc_dt_s,
                        core=None, simulate_mpc=False, horizon=5)
                    record["ok"] = True
                    completed += 1
                except Exception as exc:
                    failures += 1
                    error = f"{type(exc).__name__}: {exc}"
                    record.update({"ok": False, "error_type": type(exc).__name__, "error": str(exc)})
                    if sent_ns is not None:
                        failed_ns = time.monotonic_ns()
                        record["request_elapsed_ms_at_failure"] = (failed_ns - sent_ns) / 1e6
                        record["source_age_at_failure_ms"] = (failed_ns - window[-1]["host_monotonic_ns"]) / 1e6
                    reason = "request_failed_no_retry"
                stream.write(json.dumps(record, allow_nan=False, ensure_ascii=False) + "\n")
                stream.flush()
                if error is not None:
                    break
            if reason == "not_started":
                reason = "request_count_bound" if args.requests is not None and attempts >= args.requests else "duration_bound"
    except KeyboardInterrupt:
        reason = "interrupted"
        error = "KeyboardInterrupt"
    except Exception as exc:
        reason = "failed"
        error = f"{type(exc).__name__}: {exc}"
    finally:
        if camera is not None:
            camera.stop()
            state = camera.state()
            with camera.condition:
                manifest["frames"] = list(camera.frames)
            manifest["camera_properties"] = state["camera_properties"]
            if state["camera_error"] is not None and error is None:
                error = state["camera_error"]
                reason = "camera_failed_no_reconnect"
            if not state["thread_exited"] and error is None:
                error = "camera thread failed to exit within bounded shutdown"
                reason = "camera_shutdown_failed"
        else:
            state = {"thread_exited": True, "camera_release_called": False,
                     "forced_release_called": False, "camera_error": "not_started"}
        if completed == 0 and error is None:
            error, reason = "no complete inference window", "no_completed_request"
        manifest.update({"status": "failed" if error else "complete", "termination_reason": reason,
                         "error": error, "camera_shutdown": state})
        run.update({"termination_reason": reason, "error": error, "camera_shutdown": state})
        _dump_json(manifest_path, manifest)
        _dump_json(output / "input_manifest.json", manifest)
        _dump_json(output / "run.json", run)
        summary = {
            "schema_version": "nomad.offline_shadow.v1", "acquisition_mode": "live_camera_http_shadow",
            "not_for_motion": True, "attempted": attempts, "completed": completed, "failed": failures,
            "termination_reason": reason, "error": error, "camera_shutdown": state,
            "timestamp_source": TIMESTAMP_SOURCE, "source_age_timestamp_source": AGE_SOURCE,
            "robot_feedback_used": False, "mpc_mode": "not_requested",
            "rtt_ms": {"count": len(rtts), "median": statistics.median(rtts) if rtts else None,
                       "max": max(rtts) if rtts else None},
            "source_age_at_response_ms": {"count": len(source_ages),
                                          "median": statistics.median(source_ages) if source_ages else None,
                                          "max": max(source_ages) if source_ages else None},
            "manifest": str(manifest_path), "results": str(output / "results.jsonl"),
            "goal": run.get("goal", {}).get("path"),
        }
        _dump_json(output / "summary.json", summary)
        print(json.dumps(summary, allow_nan=False, ensure_ascii=False, indent=2))
    return 1 if error is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
