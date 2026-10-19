"""Offline transport checks for the read-only Go2 camera; no ROS, DDS or robot.

Uses a real 8x8 JPEG, a virtual receive clock, fake VideoClient / ROS publisher,
and fake HTTP requests. Results are synthetic diagnostics, not field evidence.
"""
from __future__ import annotations

import base64
import json
from pathlib import Path
import sys
import tempfile
import threading

from .camera import (CameraState, LatestFrame, ShadowIngress, TIMESTAMP_SOURCE,
                     atomic_status, capture_once, jpeg_dimensions)


JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/"
    "2wBDAQkJCQwLDBgNDRgyIRwhMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjL/"
    "wAARCAAIAAgDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/"
    "8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDiKKKK+mPDP//Z")


class Clock:
    def __init__(self):
        self.wall = 1_800_000_000_000_000_000
        self.mono = 10_000_000_000

    def advance(self, seconds):
        delta = int(seconds*1e9)
        self.wall += delta
        self.mono += delta


class FakeVideoClient:
    def __init__(self, clock):
        self.clock, self.code, self.data = clock, 0, JPEG

    def GetImageSample(self):
        self.clock.advance(.025)
        return self.code, self.data


class FakeRosPublisher:
    def __init__(self):
        self.frames = []

    def publish(self, frame):
        self.frames.append(frame)


class FakeHttp:
    def __init__(self):
        self.status = {"profile": "external", "session_id": "fake-runtime-one",
                       "execution_capability": {"operator_run_configured": False}}
        self.payloads = []
        self.fail = False
        self.before_status = None

    def __call__(self, url, timeout, payload=None):
        if self.fail:
            raise TimeoutError("synthetic HTTP timeout")
        if url.endswith("/status"):
            if self.before_status:
                self.before_status()
            return dict(self.status)
        self.payloads.append(payload)
        return {"ok": True}


def audit():
    checks = {}
    clock = Clock()
    state = CameraState("fake-interface", wall_clock=lambda: clock.wall,
                        mono_clock=lambda: clock.mono)
    client, ros, slot = FakeVideoClient(clock), FakeRosPublisher(), LatestFrame()
    begin = clock.wall
    frame = capture_once(client, state, ros.publish, slot)
    checks["valid_jpeg_dimensions"] = jpeg_dimensions(JPEG)[1] == (8, 8)
    checks["real_rpc_receive_semantics_in_fake_transport"] = bool(
        frame and frame.request_begin_timestamp_ns == begin and
        frame.receive_timestamp_ns == begin+25_000_000 and frame.rpc_duration_ms == 25.)
    payload = frame.payload()
    checks["exposure_and_extrinsics_explicitly_unavailable"] = (
        payload["timestamp_source"] == TIMESTAMP_SOURCE and
        payload["capture_timestamp_ns"] == frame.receive_timestamp_ns and
        payload["provenance"]["source_exposure_timestamp_available"] is False and
        payload["provenance"]["camera_extrinsics_verified"] is False and
        payload["provenance"]["rpc_duration_is_capture_latency"] is False)
    checks["published_and_recent_ready"] = len(ros.frames) == 1 and state.snapshot()["ready"]
    client.code = 42
    failed = capture_once(client, state, ros.publish, slot)
    checks["failed_rpc_never_republishes_old_frame"] = failed is None and len(ros.frames) == 1
    client.code, client.data = 0, JPEG[:-2]
    failed = capture_once(client, state, ros.publish, slot)
    checks["truncated_jpeg_rejected_before_publish"] = failed is None and len(ros.frames) == 1
    try:
        jpeg_dimensions(b"x"*(4*1024*1024+1))
        checks["oversized_jpeg_rejected"] = False
    except ValueError:
        checks["oversized_jpeg_rejected"] = True
    clock.advance(2.)
    checks["heartbeats_do_not_refresh_stale_image"] = not state.snapshot()["ready"]
    client.data = JPEG
    def fail_publish(_):
        raise RuntimeError("fake ROS failure")
    capture_once(client, state, fail_publish, slot)
    checks["failed_publication_cannot_refresh_ready"] = not state.snapshot()["ready"]
    frame = capture_once(client, state, ros.publish, slot)
    http = FakeHttp()
    ingress = ShadowIngress("http://fake", state, request=http)
    checks["shadow_only_ingress_accepts_measured_rgb"] = ingress.send(frame) and len(http.payloads) == 1
    http.before_status = lambda: clock.advance(1.)
    checks["frame_age_checked_after_slow_status_request"] = not ingress.send(frame) and len(http.payloads) == 1
    http.before_status = None
    frame = capture_once(client, state, ros.publish, slot)
    http.fail = True
    checks["http_failure_drops_frame_without_republish"] = not ingress.send(frame) and len(http.payloads) == 1
    http.fail = False
    http.status["session_id"] = "fake-runtime-two"
    checks["runtime_restart_latches_ingress_until_camera_restart"] = (
        not ingress.send(frame) and bool(state.runtime_disabled_reason))
    http.status["session_id"] = "fake-runtime-one"
    checks["restored_session_does_not_unlatch"] = not ingress.send(frame)
    http.status["execution_capability"]["operator_run_configured"] = True
    try:
        ShadowIngress("http://fake", state, request=http)
        checks["physical_runtime_rejected_before_ingress"] = False
    except ValueError:
        checks["physical_runtime_rejected_before_ingress"] = True
    http.status["execution_capability"]["operator_run_configured"] = False
    state.runtime_disabled_reason = None
    ingress = ShadowIngress("http://fake", state, request=http)
    http.status["profile"] = "simulated-replay"
    checks["runtime_profile_change_latches_ingress"] = not ingress.send(frame) and bool(state.runtime_disabled_reason)

    # One HTTP POST is held in flight while real capture/publish code continues.
    # Pending handoff must contain only the newest sample on release.
    latest = LatestFrame()
    stalled, release, completed = threading.Event(), threading.Event(), threading.Event()
    blocked_http = FakeHttp()
    def request(url, timeout, payload=None):
        if payload is not None:
            stalled.set()
            if not release.wait(2.):
                raise TimeoutError("offline test did not release HTTP")
        return blocked_http(url, timeout, payload)
    state.runtime_disabled_reason = None
    blocked_ingress = ShadowIngress("http://fake", state, request=request)
    first = capture_once(client, state, ros.publish, latest)
    def one_send():
        blocked_ingress.send(latest.take())
        completed.set()
    worker = threading.Thread(target=one_send, daemon=True)
    worker.start()
    entered = stalled.wait(1.)
    published_before = len(ros.frames)
    newest = None
    for _ in range(6):
        newest = capture_once(client, state, ros.publish, latest)
    release.set()
    finished = completed.wait(1.)
    pending = latest.take()
    checks["slow_http_does_not_block_ros_camera_publication"] = entered and finished and len(ros.frames) == published_before+6
    checks["http_handoff_is_latest_only_bounded"] = pending is newest and pending is not first and latest.replaced == 5 and latest.take(0) is None
    worker.join(timeout=.1)
    latest.close()
    latest.put(newest)
    checks["closed_handoff_discards_pending_and_future_frames"] = latest.take(0) is None
    state.active = False
    checks["shutdown_clears_ready"] = not state.snapshot()["ready"]
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/"camera.json"
        atomic_status(path, state.snapshot())
        checks["atomic_final_status_has_no_motion"] = (
            json.loads(path.read_text())["motion_sent"] is False and
            not list(path.parent.glob("*.tmp")))
    checks["no_sdk_or_ros_imported_in_offline_audit"] = not any(
        name == "rclpy" or name.startswith("unitree_sdk2py") for name in sys.modules)
    return {"schema_version": "dejaview.go2.camera.offline_audit.v1",
            "offline_diagnostic": True, "synthetic": True,
            "dds_initialized": False, "motion_sent": False,
            "checks": checks, "ok": all(checks.values())}


def main():
    report = audit()
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
