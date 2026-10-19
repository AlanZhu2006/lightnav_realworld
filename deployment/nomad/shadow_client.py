"""Replay recorded RGB manifests through NoMaD HTTP and save offline reports.

This program has no ROS, DDS, SDK, robot-feedback or movement interface.
Everything it writes is ``not_for_motion``. Camera-host dequeue timestamps
must retain their real ``timestamp_source``; they are not promoted to sensor
capture times by their wire field name.

Example manifest (relative paths are resolved against the manifest directory)::

    {"timestamp_source": "host_dequeue_monotonic_ns_proxy_not_sensor_capture",
     "frames": [{"frame_id": "rgb_0001", "capture_timestamp_ns": 123456789,
                 "path": "rgb_0001.jpg"}, ...]}

Four chronological frames are required. ``--mode latest4`` sends the final
four; ``--mode windows`` replays every consecutive four-frame window. The
optional MPC mode uses explicitly synthetic zero poses, never robot odometry.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import io
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid


def _json_loads(value: str | bytes) -> Any:
    def reject_nonfinite(token: str) -> None:
        raise ValueError(f"nonfinite JSON token: {token}")
    def finite_float(token: str) -> float:
        result = float(token)
        if not math.isfinite(result):
            raise ValueError(f"nonfinite JSON number: {token}")
        return result
    return json.loads(value, parse_constant=reject_nonfinite, parse_float=finite_float)


def _dump_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n")


def load_manifest(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Validate actual metadata; do not sort, synthesize timestamps or pad RGB."""
    path = path.resolve(strict=True)
    manifest = _json_loads(path.read_text())
    if not isinstance(manifest, dict) or not isinstance(manifest.get("frames"), list):
        raise ValueError("manifest must be an object containing a frames array")
    if len(manifest["frames"]) < 4:
        raise ValueError("at least four real chronological frames are required")
    frames, seen, previous_stamp = [], set(), -1
    for index, frame in enumerate(manifest["frames"]):
        if not isinstance(frame, dict):
            raise ValueError(f"frames[{index}] must be an object")
        frame_id, stamp = frame.get("frame_id"), frame.get("capture_timestamp_ns")
        if not isinstance(frame_id, str) or not frame_id.strip() or len(frame_id) > 256 or frame_id in seen:
            raise ValueError(f"frames[{index}] frame_id must be unique and nonempty")
        if type(stamp) is not int or stamp <= 0 or stamp <= previous_stamp:
            raise ValueError("capture_timestamp_ns must be positive and strictly increasing as recorded")
        source = frame.get("timestamp_source", manifest.get("timestamp_source"))
        if not isinstance(source, str) or not source.strip():
            raise ValueError(f"frames[{index}] needs its real timestamp_source, locally or at manifest root")
        raw_path = frame.get("path")
        if not isinstance(raw_path, str) or not raw_path:
            raise ValueError(f"frames[{index}].path must name an existing RGB image")
        image_path = Path(raw_path).expanduser()
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        image_path = image_path.resolve(strict=True)
        if not image_path.is_file():
            raise ValueError(f"not an image file: {image_path}")
        frames.append({"frame_id": frame_id, "capture_timestamp_ns": stamp,
                       "timestamp_source": source, "resolved_path": str(image_path),
                       "manifest_frame": frame})
        seen.add(frame_id)
        previous_stamp = stamp
    return manifest, frames


def encode_rgb_jpeg(path: Path) -> tuple[str, dict[str, Any]]:
    """Use one RGB/PIL/JPEG path for observations and the goal."""
    from PIL import Image
    original = path.read_bytes()
    with Image.open(io.BytesIO(original)) as image:
        if image.width * image.height > 4096 * 2160:
            raise ValueError(f"image exceeds server pixel limit: {path}")
        rgb = image.convert("RGB")
        buffer = io.BytesIO()
        rgb.save(buffer, format="JPEG", quality=95, subsampling=0)
        jpeg = buffer.getvalue()
        size = list(rgb.size)
    if len(jpeg) > 4 * 1024 * 1024:
        raise ValueError(f"wire JPEG exceeds server 4 MiB limit: {path}")
    return base64.b64encode(jpeg).decode("ascii"), {
        "path": str(path), "original_sha256": hashlib.sha256(original).hexdigest(),
        "wire_jpeg_sha256": hashlib.sha256(jpeg).hexdigest(), "size_wh": size,
        "wire_jpeg_bytes": len(jpeg), "encoding": "PIL RGB JPEG quality95 subsampling0",
    }


def request_json(url: str, *, timeout_s: float, payload: dict[str, Any] | None = None) -> tuple[Any, float]:
    body = None if payload is None else json.dumps(payload, allow_nan=False, separators=(",", ":")).encode()
    request = Request(url, data=body, headers={"Content-Type": "application/json"},
                      method="GET" if body is None else "POST")
    started = time.perf_counter()
    try:
        with urlopen(request, timeout=timeout_s) as response:
            result = response.read(32 * 1024 * 1024 + 1)
            if len(result) > 32 * 1024 * 1024:
                raise ValueError("server response exceeds 32 MiB")
    except HTTPError as exc:
        detail = exc.read(4096).decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code}: {detail}") from exc
    return _json_loads(result), (time.perf_counter() - started) * 1000.0


def inspect_response(response: Any, payload: dict[str, Any]) -> dict[str, bool]:
    """Check identity, source lineage, finite output and the exact scaling contract."""
    import numpy as np
    if not isinstance(response, dict):
        raise ValueError("inference response must be an object")
    newest = payload["observations"][-1]
    expected_source = {key: newest[key] for key in ("frame_id", "capture_timestamp_ns")}
    expected_frames = [{key: frame[key] for key in ("frame_id", "capture_timestamp_ns")}
                       for frame in payload["observations"]]
    raw = np.asarray(response.get("first_sample_normalized"), dtype=np.float64)
    metric = np.asarray(response.get("trajectory_body_m"), dtype=np.float64)
    scale = response.get("action_scale_m")
    scale_valid = (isinstance(scale, (int, float)) and not isinstance(scale, bool)
                   and math.isfinite(scale) and scale > 0)
    shape_valid = raw.shape == (8, 2) and metric.shape == (8, 2)
    finite = bool(shape_valid and np.all(np.isfinite(raw)) and np.all(np.isfinite(metric)))
    for name in ("distance_steps", "inference_ms"):
        value = response.get(name)
        finite = finite and isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    checks = {
        "schema": response.get("schema_version") == "nomad.inference.v1",
        "identity_echo": response.get("identity") == payload["identity"],
        "source_frame_match": response.get("source_frame") == expected_source,
        "observation_frames_match": response.get("observation_frames") == expected_frames,
        "finite_numeric_output": bool(finite), "scale_positive_finite": bool(scale_valid),
        "metric_equals_raw_times_scale": bool(finite and scale_valid and np.allclose(metric, raw * scale, rtol=1e-7, atol=1e-9)),
        "scale_alias_matches": response.get("scale_m_per_model_unit") == scale,
    }
    return checks


def adapt_response(response: dict[str, Any], *, source_stamp_ns: int,
                   v_ref_mps: float, mpc_dt_s: float, core: Any, simulate_mpc: bool,
                   horizon: int) -> dict[str, Any]:
    import numpy as np
    from .trajectory_adapter import raw_nomad_to_metric_path, resample_metric_path
    from .mpc_reference import build_reference
    metric = raw_nomad_to_metric_path(
        response["first_sample_normalized"], action_scale_m=response["action_scale_m"],
        capture_stamp_ns=source_stamp_ns, frame_id="base_link")
    sampled = resample_metric_path(metric, v_ref_mps=v_ref_mps, mpc_dt_s=mpc_dt_s)
    report: dict[str, Any] = {
        "not_for_motion": True, "capture_stamp_ns": sampled.capture_stamp_ns,
        "body_frame": "capture body assumed +x forward/+y left/yaw CCW; not verified on robot",
        "yaw_source": "derived polyline tangent, not NoMaD prediction",
        "metric_polyline_xy_m": metric.xy_m.tolist(), "polyline_length_m": metric.length_m,
        "sampled_path_body_m_rad": sampled.points_body_m_rad.tolist(),
        "sampled_arc_length_m": sampled.arc_length_m.tolist(),
        "reference_spacing_m": sampled.spacing_m, "has_motion": sampled.has_motion,
        "v_ref_mps": v_ref_mps, "mpc_dt_s": mpc_dt_s,
    }
    if core is not None:
        # These are explicitly synthetic coordinates. There is no feedback
        # source and no pose inferred from an image or recorded timestamp.
        synthetic_pose = np.zeros(3, dtype=np.float64)
        packet = build_reference(
            sampled, capture_pose_odom_m_rad=synthetic_pose,
            current_pose_odom_m_rad=synthetic_pose, horizon=horizon,
            weights=(1.0, 1.0, 1.0), core=core, odom_frame_id="synthetic_odom_not_robot")
        mpc: dict[str, Any] = {
            "mode": "synthetic_pose_no_robot_feedback", "not_for_motion": True,
            "capture_pose_m_rad": synthetic_pose.tolist(), "current_pose_m_rad": synthetic_pose.tolist(),
            "vendor_source_sha256": packet.vendor_source_sha256,
            "reference_m_rad": packet.reference_odom_m_rad.tolist(),
            "stop_required": packet.stop_required, "solver_executed": False,
        }
        if simulate_mpc and not packet.stop_required:
            controller = core.controller_type(
                horizon=horizon, dt_s=mpc_dt_s, w_max=0.4, a_max_v=0.5, a_max_w=1.0,
                q_weights=(1.0, 1.0, 1.0), r_weights=(0.01, 0.01))
            control, prediction = controller.solve(synthetic_pose, packet.reference_odom_m_rad,
                                                   previous=(0.0, 0.0), v_max=v_ref_mps)
            if not np.all(np.isfinite(prediction)) or not np.all(np.isfinite(control)):
                raise ValueError("synthetic MPC returned nonfinite output")
            mpc.update({"solver_executed": True, "synthetic_previous_control": [0.0, 0.0],
                        "simulated_control_only_v_w": list(control),
                        "predicted_states_m_rad": prediction.tolist()})
        report["mpc"] = mpc
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--goal", type=Path, required=True)
    parser.add_argument("--server-url", required=True, help="base URL, e.g. http://127.0.0.1:8091")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("latest4", "windows"), default="latest4")
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--v-ref-mps", type=float, default=0.2)
    parser.add_argument("--mpc-dt-s", type=float, default=0.1)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--mpc-source", type=Path, help="hash-locked LightNav vendor mpc.py; optional reference generation")
    parser.add_argument("--simulate-mpc", action="store_true", help="also solve MPC at a synthetic zero pose; never robot feedback")
    args = parser.parse_args(argv)
    for name in ("timeout_s", "v_ref_mps", "mpc_dt_s"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.horizon <= 0:
        parser.error("--horizon must be positive")
    if args.simulate_mpc and args.mpc_source is None:
        parser.error("--simulate-mpc requires --mpc-source")
    base = args.server_url.rstrip("/")
    parsed = urlsplit(base)
    if parsed.scheme not in ("http", "https") or not parsed.netloc or parsed.query or parsed.fragment:
        parser.error("--server-url must be an HTTP(S) base URL")
    manifest_path = args.manifest.expanduser().resolve(strict=True)
    manifest, frames = load_manifest(manifest_path)
    goal_path = args.goal.expanduser().resolve(strict=True)
    goal_encoded, goal_meta = encode_rgb_jpeg(goal_path)
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    for name in ("results.jsonl", "summary.json", "input_manifest.json", "run.json"):
        if (output / name).exists():
            parser.error(f"refusing to overwrite an existing run file: {output / name}")
    _dump_json(output / "input_manifest.json", manifest)
    health, health_rtt = request_json(base + "/healthz", timeout_s=args.timeout_s)
    if not isinstance(health, dict) or health.get("ok") is not True or health.get("model_loaded") is not True:
        raise ValueError("server /healthz did not confirm a loaded model")
    core = None
    if args.mpc_source is not None:
        from .mpc_reference import load_vendor_mpc
        core = load_vendor_mpc(args.mpc_source.expanduser(), with_solver=args.simulate_mpc)
    session_id = "offline-shadow-" + uuid.uuid4().hex
    _dump_json(output / "run.json", {
        "schema_version": "nomad.offline_shadow.v1", "not_for_motion": True,
        "session_id": session_id, "manifest_path": str(manifest_path),
        "server_url": base, "healthz_rtt_ms": health_rtt, "server_healthz": health,
        "goal": goal_meta, "mode": args.mode,
        "adapter_configuration": {"v_ref_mps": args.v_ref_mps, "mpc_dt_s": args.mpc_dt_s,
                                  "horizon": args.horizon, "simulate_mpc": args.simulate_mpc},
        "timestamp_notice": "preserved manifest sources; wire capture field may be host dequeue proxy, never validated sensor capture",
        "mpc_mode": "synthetic_pose_no_robot_feedback" if core is not None else "not_requested",
    })
    windows = [frames[-4:]] if args.mode == "latest4" else [frames[i:i + 4] for i in range(len(frames) - 3)]
    cache: dict[str, tuple[str, dict[str, Any]]] = {}
    rtts, failures, completed = [], 0, 0
    with (output / "results.jsonl").open("x") as results:
        for index, window in enumerate(windows):
            identity = {"session_id": session_id, "task_id": "offline-manifest-replay",
                        "goal_id": goal_meta["original_sha256"], "goal_version": 0,
                        "frame_id": window[-1]["frame_id"]}
            record: dict[str, Any] = {"window_index": index, "identity": identity,
                                      "not_for_motion": True, "frames": window,
                                      "recorded_frame_intervals_ms": [
                                          (right["capture_timestamp_ns"] - left["capture_timestamp_ns"]) / 1e6
                                          for left, right in zip(window, window[1:])]}
            try:
                observations, image_metadata = [], []
                for frame in window:
                    image_path = frame["resolved_path"]
                    if image_path not in cache:
                        cache[image_path] = encode_rgb_jpeg(Path(image_path))
                    encoded, metadata = cache[image_path]
                    observations.append({"frame_id": frame["frame_id"],
                                         "capture_timestamp_ns": frame["capture_timestamp_ns"],
                                         "rgb_jpeg_base64": encoded})
                    image_metadata.append(metadata)
                payload = {"identity": identity, "observations": observations,
                           "goal": {"rgb_jpeg_base64": goal_encoded}}
                response, rtt = request_json(base + "/infer", timeout_s=args.timeout_s, payload=payload)
                rtts.append(rtt)
                record.update({"rtt_ms": rtt, "image_metadata": image_metadata, "response": response})
                checks = inspect_response(response, payload)
                record["checks"] = checks
                if not all(checks.values()):
                    raise ValueError("response contract failed: " + ", ".join(k for k, value in checks.items() if not value))
                record["adaptation"] = adapt_response(
                    response, source_stamp_ns=window[-1]["capture_timestamp_ns"],
                    v_ref_mps=args.v_ref_mps, mpc_dt_s=args.mpc_dt_s, core=core,
                    simulate_mpc=args.simulate_mpc, horizon=args.horizon)
                record["ok"] = True
                completed += 1
            except Exception as exc:
                failures += 1
                record.update({"ok": False, "error_type": type(exc).__name__, "error": str(exc)})
            results.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            results.flush()
    summary = {"schema_version": "nomad.offline_shadow.v1", "not_for_motion": True,
               "windows": len(windows), "completed": completed, "failed": failures,
               "rtt_ms": {"count": len(rtts), "min": min(rtts) if rtts else None,
                          "median": statistics.median(rtts) if rtts else None,
                          "max": max(rtts) if rtts else None},
               "timestamp_sources": sorted({frame["timestamp_source"] for frame in frames}),
               "robot_feedback_used": False, "mpc_mode": "synthetic_pose_no_robot_feedback" if core is not None else "not_requested",
               "results": str(output / "results.jsonl")}
    _dump_json(output / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
