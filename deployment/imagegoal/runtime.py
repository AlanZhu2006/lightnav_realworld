"""ImageGoal scheduler, feedback tracker, and explicit executor ownership.

Shadow mode only produces proposals. The supervised-native Start endpoint
owns a separate guarded Go2 executor; this process never imports the SDK.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass, field
import base64
import hashlib
import io
import json
import math
from pathlib import Path
import queue
import threading
import time
from typing import Any
import uuid

import numpy as np

from deployment.nomad.mpc_reference import build_reference, load_vendor_mpc
from deployment.nomad.trajectory_adapter import raw_nomad_to_metric_path, resample_metric_path
from deployment.nomad.vendor.lightnav.geometry import TimedPose, pose_at_stamp
from deployment.nomad.shadow_client import inspect_response, request_json
from .guards import load_gem_guards
from deployment.lightnav.official_control import OFFICIAL_COMMIT, OfficialLightNavController, validate_waypoints
from deployment.lightnav.policy import OFFICIAL_CORE_DIGEST
from deployment.lightnav.go2_adaptation import prepare_path, postprocess_command
from .calibration import load_calibration
from .lightnav_transport import LightNavObservationTransport
from .lightnav_http import LightNavHTTPClient


TERMINAL = {"IDLE", "SUCCEEDED", "CANCELLED", "FAILED", "TIMED_OUT"}


def json_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode()


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(json_bytes(value))
    temporary.replace(path)


def native_speed_gate(vx: float, wz: float, heading_error: float, config) -> tuple[float,float]:
    """TinyNav-style effective-speed gate for the explicit supervised profile.

    Small forward requests decay to zero; engaged forward motion has a 0.10
    minimum. Straight references suppress yaw hunting inside eight degrees.
    These are controller settings, not a claimed measured firmware threshold.
    """
    if not all(math.isfinite(float(value)) for value in (vx,wz,heading_error)):
        raise ValueError("nonfinite native controller command")
    vx = min(config.max_v_mps,max(0.,float(vx)))
    vx = 0. if vx < .04 else min(config.max_v_mps,max(vx,config.min_effective_v_mps))
    wz = min(config.max_w_rad_s,max(-config.max_w_rad_s,float(wz)))
    if abs(heading_error) < math.radians(8.) or abs(wz) < .04:
        wz = 0.
    else:
        wz = math.copysign(min(config.max_w_rad_s,max(abs(wz),config.min_effective_w_rad_s)),wz)
    return vx,wz


def decode_jpeg(encoded: Any) -> bytes:
    from PIL import Image
    if not isinstance(encoded, str) or len(encoded) > 6 * 1024 * 1024:
        raise ValueError("rgb_jpeg_base64 must be a JPEG smaller than 4 MiB")
    blob = base64.b64decode(encoded, validate=True)
    if len(blob) > 4 * 1024 * 1024:
        raise ValueError("JPEG exceeds 4 MiB")
    try:
        with Image.open(io.BytesIO(blob)) as image:
            if image.format != "JPEG" or image.width * image.height > 4096 * 2160:
                raise ValueError("RGB must be JPEG with at most 4096x2160 pixels")
            image.load()
    except (OSError,Image.DecompressionBombError) as exc:
        raise ValueError("RGB must be a valid, complete JPEG") from exc
    return blob


def input_metadata(value: dict[str, Any]) -> dict[str, Any]:
    stamp = value.get("capture_timestamp_ns")
    if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
        raise ValueError("capture_timestamp_ns must be a positive integer")
    if value.get("clock_domain") != "unix_epoch_ns":
        raise ValueError("clock_domain must explicitly be unix_epoch_ns")
    for key in ("timestamp_source", "frame_name"):
        if not isinstance(value.get(key), str) or not value[key].strip():
            raise ValueError(f"{key} is required")
    if not isinstance(value.get("synthetic"), bool):
        raise ValueError("synthetic must explicitly be true or false")
    if "commissioning" in value and not isinstance(value["commissioning"],bool):
        raise ValueError("commissioning must be boolean")
    age = (time.time_ns() - stamp) / 1e9
    if age < -0.1:
        raise ValueError("source timestamp is more than 100 ms in the future")
    metadata = {key: value[key] for key in ("capture_timestamp_ns", "clock_domain", "timestamp_source", "frame_name", "synthetic")}
    metadata["commissioning"] = value.get("commissioning",False)
    if "body_frame_name" in value:
        metadata["body_frame_name"] = value["body_frame_name"]
    return metadata


@dataclass
class RuntimeConfig:
    profile: str = "external"
    nomad_url: str = "http://127.0.0.1:18882"
    lightnav_url: str = "http://127.0.0.1:18884"
    request_timeout_s: float = 8.0
    max_rgb_age_s: float = 3.0
    max_plan_age_s: float = 3.0
    max_odom_age_s: float = 0.3
    max_depth_age_s: float = 0.5
    max_rgb_depth_skew_s: float = 0.35
    max_capture_odom_gap_s: float = 0.15
    max_history_gap_s: float = 0.6
    history_frames: int = 4
    tick_hz: float = 10.0
    v_ref_mps: float = 0.2
    max_v_mps: float = 0.2
    max_w_rad_s: float = 0.4
    hard_stop_m: float = 0.35
    horizon: int = 5
    max_inference_errors: int = 5
    lightnav_max_pending_frames: int = 8
    lightnav_max_request_bytes: int = 384 * 1024
    lightnav_max_pending_age_s: float = 2.0
    execution_profile: str = "shadow"
    robot_id: str = ""
    mount_receipt: str | None = None
    timing_receipt: str | None = None
    commissioning_real_inputs: bool = False
    execution_lease_s: float = 0.25
    nomad_stop_distance_steps: float | None = None
    nomad_stop_consecutive: int = 3
    operator_supervised: bool = False
    min_effective_v_mps: float = 0.0
    min_effective_w_rad_s: float = 0.0
    lightnav_adapt_resample: bool = True
    lightnav_adapt_speed_floor: bool = True
    lightnav_adapt_heading_filter: bool = True
    lightnav_adapt_short_actions: bool = True

    def validate(self) -> None:
        if self.profile not in {"external", "simulated-replay"}:
            raise ValueError("profile must be external or simulated-replay")
        if self.execution_profile not in {"shadow","operator-run","supervised-native"}:
            raise ValueError("unexpected execution_profile")
        if type(self.operator_supervised) is not bool:
            raise ValueError("operator_supervised must be boolean")
        for name in ("lightnav_adapt_resample", "lightnav_adapt_speed_floor", "lightnav_adapt_heading_filter", "lightnav_adapt_short_actions"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be boolean")
        if self.execution_profile == "supervised-native":
            if self.profile != "external" or not self.operator_supervised or self.commissioning_real_inputs:
                raise ValueError("supervised-native requires real external inputs and an explicit onsite operator")
            if self.max_v_mps > .3 or self.max_w_rad_s > .55 or self.max_plan_age_s > 2 or self.max_rgb_age_s > 2 or self.max_odom_age_s > .3:
                raise ValueError("supervised-native exceeds the .3/.55 velocity or source-age contract")
        for name, limit in (("min_effective_v_mps",self.max_v_mps),("min_effective_w_rad_s",self.max_w_rad_s)):
            value = getattr(self,name)
            if type(value) not in (int,float) or not math.isfinite(value) or not 0 <= value <= limit:
                raise ValueError(f"{name} must be finite, nonnegative and within the configured maximum")
        if self.execution_profile=="operator-run" and self.profile!="external":
            raise ValueError("operator-run requires external measured inputs")
        if self.commissioning_real_inputs and self.profile!="external":
            raise ValueError("commissioning real-input fixtures require external profile")
        for name, value in asdict(self).items():
            if name.startswith("lightnav_adapt_"):
                continue
            if name in {"profile", "nomad_url", "lightnav_url","execution_profile","robot_id","mount_receipt","timing_receipt","commissioning_real_inputs","nomad_stop_distance_steps","operator_supervised","min_effective_v_mps","min_effective_w_rad_s"}:
                continue
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.history_frames != 4:
            raise ValueError("initial shared contract requires four real chronological frames")
        for name in ("lightnav_max_pending_frames", "lightnav_max_request_bytes"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if self.lightnav_max_pending_frames > 64:
            raise ValueError("LightNav pending history must not exceed 64 frames")
        if self.execution_lease_s>.25:
            raise ValueError("execution lease must not exceed 250 ms")
        if type(self.nomad_stop_consecutive) is not int or self.nomad_stop_consecutive<2:
            raise ValueError("nomad_stop_consecutive must be an integer of at least two")
        if self.nomad_stop_distance_steps is not None and (type(self.nomad_stop_distance_steps) not in (int,float) or
            not math.isfinite(self.nomad_stop_distance_steps) or self.nomad_stop_distance_steps<0):
            raise ValueError("NoMaD stop threshold must be a finite nonnegative temporal score")
        if self.execution_profile=="operator-run" and (self.max_plan_age_s>3 or self.max_rgb_age_s>3 or
            self.max_odom_age_s>.3 or self.max_depth_age_s>.5 or self.max_v_mps>.2 or self.max_w_rad_s>.4):
            raise ValueError("operator-run limits cannot exceed the physical envelope contract")


@dataclass
class Frame:
    frame_id: str
    jpeg: bytes
    metadata: dict[str, Any]
    received_monotonic_ns: int
    provenance: dict[str, Any] = field(default_factory=dict)

    @property
    def stamp(self) -> int:
        return self.metadata["capture_timestamp_ns"]

    def wire(self) -> dict[str, Any]:
        return {"frame_id": self.frame_id, "capture_timestamp_ns": self.stamp,
                "rgb_jpeg_base64": base64.b64encode(self.jpeg).decode()}


class ImageGoalRuntime:
    def __init__(self, output: Path, config: RuntimeConfig):
        config.validate()
        self.config = config
        self.output = output.expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output / "goals").mkdir()
        (self.output / "requests").mkdir()
        (self.output / "rgb").mkdir()
        self.session_id = "imagegoal-" + uuid.uuid4().hex
        self.calibration = load_calibration(config.mount_receipt,config.timing_receipt,config.robot_id,
                                            allow_commissioning=config.commissioning_real_inputs)
        self.proposal_evaluated_ns = time.time_ns()
        self.control_sequence = 0
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        # NoMaD selects exactly four. LightNav sends overlapping chronological
        # blocks to its session history rather than silently truncating the
        # policy's native long context to a four-frame NoMaD FIFO.
        self.frames: deque[Frame] = deque(maxlen=512)
        self.lightnav_transport = LightNavObservationTransport()
        self.lightnav_http = LightNavHTTPClient()
        self.seen_rgb_ids: set[str] = set()
        self.odom: deque[tuple[TimedPose, dict[str, Any], int, list[float]]] = deque(maxlen=1000)
        self.depth: dict[str, Any] | None = None
        self.task: dict[str, Any] | None = None
        self.goal_jpeg: bytes | None = None
        # Selection is independent of a navigation task. A live preview is
        # never implicitly a goal, and restarting a runtime never rearms one.
        self.goal_selection = self._live_goal_selection()
        self.epoch = 0
        self.plan: dict[str, Any] | None = None
        self.preview: dict[str, Any] | None = None
        self.proposal = self._zero("idle")
        self.gates: list[str] = ["no_active_task"]
        self.state = "IDLE"
        self.events: deque[dict[str, Any]] = deque(maxlen=4000)
        self.event_sequence = 0
        self.last_frame_requested: tuple[int, str] | None = None
        self.hold_after_stamp_ns = 0
        self.inflight: dict[str, Any] | None = None
        self.completed: queue.SimpleQueue = queue.SimpleQueue()
        self.faults: dict[str, Any] = {}
        self.pending_session_cleanup: set[str] = set()
        self.metrics = {key: 0 for key in ("rgb_received", "depth_received", "odom_received", "inference_started", "inference_accepted", "inference_rejected", "inference_held", "inference_errors", "mpc_solves", "mpc_errors", "commands_nonzero", "invalidations")}
        self.native_executor = None
        self.control_module, latency = load_gem_guards()
        self.depth_config = self.control_module.DepthSafetyConfig(hard_stop_m=config.hard_stop_m)
        self.latency_guard = latency.LatencyMotionGuard(latency.LatencyMotionGuardConfig(max_plan_input_age_s=config.max_plan_age_s))
        source = Path(__file__).parents[1] / "nomad/vendor/lightnav/mpc.py"
        self.core = load_vendor_mpc(source, with_solver=True)
        self.controller = self.core.controller_type(
            horizon=config.horizon, dt_s=1.0/config.tick_hz, w_max=config.max_w_rad_s,
            a_max_v=0.5, a_max_w=1.0, q_weights=(1.0, 1.0, 1.0), r_weights=(0.01, 0.01))
        self.lightnav_controller = OfficialLightNavController(self.core)
        self.thread = threading.Thread(target=self._loop, name="imagegoal-feedback", daemon=True)
        atomic_json(self.output / "run.json", {"schema_version": "imagegoal.run.v1", "session_id": self.session_id,
                    "started_timestamp_ns": time.time_ns(), "config": asdict(config), "not_for_motion": True,
                    "hardware_transport": "absent", "synthetic_feedback": config.profile == "simulated-replay",
                    "mpc_source_sha256": self.core.source_sha256,"calibration":self.calibration.summary(),
                    "lightnav_control": self.lightnav_controller.contract(config.max_v_mps, config.max_w_rad_s),
                    "lightnav_imagegoal_adaptation": self._adaptation_contract(),
                    "execution_profile":config.execution_profile,"commissioning":config.commissioning_real_inputs})
        if self.calibration.mount or self.calibration.timing:
            directory = self.output/"calibration"
            directory.mkdir()
            for name,path in (("mount",config.mount_receipt),("timing",config.timing_receipt)):
                if path is not None and Path(path).is_file():
                    (directory/f"{name}.json").write_bytes(Path(path).read_bytes())
        self._event("runtime_started", profile=config.profile, hardware_transport="absent")

    @staticmethod
    def _zero(reason: str) -> dict[str, Any]:
        return {"vx_m_s": 0.0, "vy_m_s": 0.0, "wz_rad_s": 0.0, "reason": reason,
                "not_for_motion": True, "published_to_robot": False}

    def _adaptation_contract(self) -> dict[str, Any]:
        return {"profile":"go2_imagegoal_adaptation", "official_release_commit":OFFICIAL_COMMIT,
                "reference_image_conditioning":"experimental_reference_image", "goal_image_consumed":True,
                "trajectory_resampling":self.config.lightnav_adapt_resample,
                "minimum_speed_boost":self.config.lightnav_adapt_speed_floor,
                "heading_deadband":self.config.lightnav_adapt_heading_filter,
                "short_action_softening":self.config.lightnav_adapt_short_actions,
                "base_v_ref_mps":self.config.v_ref_mps,"base_w_ref_rad_s":self.config.max_w_rad_s,
                "min_effective_v_mps":self.config.min_effective_v_mps,"min_effective_w_rad_s":self.config.min_effective_w_rad_s,
                "hardware_output_limits":{"vx_m_s":self.config.max_v_mps,"wz_rad_s":self.config.max_w_rad_s},
                "short_action_semantics":"predicted_local_action_amplitude; not_goal_distance",
                "goal_arrival_verifier":False}

    def _lightnav_control_contract(self) -> dict[str, Any]:
        official = self.lightnav_controller.contract(self.config.max_v_mps,self.config.max_w_rad_s)
        adapted = self._adaptation_contract()
        active = adapted if self.task and self.task["backend"]=="lightnav" and self.task.get("task_mode")=="imagegoal" else official
        return {**active,"available_profiles":{"official_release":official,"go2_imagegoal_adaptation":adapted}}

    def _event(self, kind: str, **fields: Any) -> None:
        self.event_sequence += 1
        item = {"sequence": self.event_sequence, "timestamp_ns": time.time_ns(), "kind": kind,
                "epoch": self.epoch, "task_id": self.task["task_id"] if self.task else None, **fields}
        self.events.append(item)
        with (self.output / "events.jsonl").open("ab") as stream:
            stream.write(json_bytes(item) + b"\n")

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        with self.lock:
            if self.native_executor is not None:
                self.native_executor.stop("runtime_shutdown")
            if self.task and self.state not in TERMINAL:
                self._finish("CANCELLED", "runtime_shutdown")
            self.proposal = self._zero("runtime_shutdown")
            self.stop_event.set()
        self.lightnav_http.close()
        if self.thread.ident is not None:
            self.thread.join(timeout=3)
        self.lightnav_transport.close()
        with self.lock:
            self._event("runtime_closed", inference_still_finishing=self.inflight is not None)
            atomic_json(self.output / "status.json", self.status())

    def _check_stream(self, metadata: dict[str, Any], previous: dict[str, Any] | None) -> None:
        if metadata["synthetic"] != (self.config.profile == "simulated-replay"):
            raise ValueError("source synthetic flag disagrees with runtime profile")
        if metadata["commissioning"] != self.config.commissioning_real_inputs:
            raise ValueError("source commissioning flag disagrees with runtime profile")
        if previous:
            if metadata["capture_timestamp_ns"] <= previous["capture_timestamp_ns"]:
                raise ValueError("source timestamps must strictly increase")
            if any(metadata[key] != previous[key] for key in ("clock_domain", "frame_name", "synthetic", "timestamp_source")):
                raise ValueError("source frame/clock/provenance changed within the run")

    def ingest_rgb(self, value: dict[str, Any]) -> dict[str, Any]:
        meta = self.calibration.correct_metadata("rgb",input_metadata(value))
        jpeg = decode_jpeg(value.get("rgb_jpeg_base64"))
        identifier = value.get("frame_id")
        if not isinstance(identifier, str) or not identifier or len(identifier) > 256:
            raise ValueError("frame_id must be a nonempty source identity")
        with self.lock:
            self._check_stream(meta, self.frames[-1].metadata if self.frames else None)
            if identifier in self.seen_rgb_ids:
                raise ValueError("duplicate RGB source identity")
            frame = Frame(identifier, jpeg, meta, time.monotonic_ns(), value.get("provenance", {}))
            self.frames.append(frame)
            self.seen_rgb_ids.add(identifier)
            self.metrics["rgb_received"] += 1
            result = {"ok": True, "frame_id": identifier, "history_size": len(self.frames)}
            warm_lightnav = bool(self.thread.is_alive() and self.task and self.state not in TERMINAL
                                 and self.task["backend"] == "lightnav" and not self.task["paused"]
                                 and not self.task["model_stop_latched"])
        if warm_lightnav:
            # Encode new RGB while the current model request is in flight.
            # The bounded worker never owns task state or sends a request,
            # and enqueueing does not hold the feedback/task lock.
            self.lightnav_transport.prefetch(frame)
        return result

    def ingest_odom(self, value: dict[str, Any]) -> dict[str, Any]:
        meta = self.calibration.correct_metadata("odom",input_metadata(value))
        pose, velocity = np.asarray(value.get("pose_m_rad"), dtype=float), np.asarray(value.get("velocity_v_w", [0,0]), dtype=float)
        if pose.shape != (3,) or velocity.shape != (2,) or not np.isfinite(pose).all() or not np.isfinite(velocity).all():
            raise ValueError("odom requires finite pose_m_rad[3] and velocity_v_w[2]")
        with self.lock:
            self._check_stream(meta, self.odom[-1][1] if self.odom else None)
            self.odom.append((TimedPose(meta["capture_timestamp_ns"], *map(float, pose)), meta,
                              time.monotonic_ns(), velocity.tolist()))
            self.metrics["odom_received"] += 1
            return {"ok": True}

    def ingest_depth(self, value: dict[str, Any]) -> dict[str, Any]:
        from PIL import Image
        meta = self.calibration.correct_metadata("depth",input_metadata(value))
        encoded = value.get("depth_png_base64")
        if not isinstance(encoded, str) or len(encoded) > 12*1024*1024:
            raise ValueError("depth_png_base64 must be bounded uint16 PNG")
        blob = base64.b64decode(encoded, validate=True)
        with Image.open(io.BytesIO(blob)) as image:
            if image.format != "PNG" or image.width*image.height > 4096*2160:
                raise ValueError("depth must be a bounded PNG")
            if image.mode not in {"I", "I;16", "I;16L", "I;16B"}:
                raise ValueError("depth PNG must encode 16-bit integer samples")
            depth_raw = np.asarray(image)
            if depth_raw.ndim != 2 or depth_raw.dtype.kind not in "ui" or depth_raw.max(initial=0) > 65535:
                raise ValueError("depth must be a single uint16 channel")
        scale = value.get("scale_m")
        if isinstance(scale, bool) or not isinstance(scale, (int,float)) or not math.isfinite(scale) or not 0 < scale <= .1:
            raise ValueError("scale_m must be finite and in (0,0.1]")
        depth_m = depth_raw.astype(np.float32)*float(scale)
        clearance = self.control_module.front_clearance(depth_m, self.depth_config)
        with self.lock:
            self._check_stream(meta, self.depth["metadata"] if self.depth else None)
            self.depth = {"metadata": meta, "depth_m": depth_m, "clearance_m": clearance,
                          "received_monotonic_ns": time.monotonic_ns(), "aligned_to_rgb": value.get("aligned_to_rgb") is True,
                          "scale_m": scale, "shape_hw": list(depth_raw.shape)}
            self.metrics["depth_received"] += 1
            return {"ok": True, "clearance_m": clearance}

    def _seal_goal(self, jpeg: bytes) -> None:
        self.goal_jpeg = jpeg
        digest = hashlib.sha256(jpeg).hexdigest()
        version = self.task["goal_version"]
        relative = f"goals/{self.task['task_id']}-v{version}.jpg"
        (self.output / relative).write_bytes(jpeg)
        self.task.update(goal_id=f"rgb-sha256:{digest}", goal_sha256=digest, goal_path=relative)

    def _seal_tracking_target(self, instruction: str) -> None:
        """Bind a language target without manufacturing a reference image.

        The existing goal generation identifies the tracking intent in requests
        and executor leases. A separate task ID keeps two identical instructions
        in independent model episodes. Frozen ImageGoal selections are preserved.
        """
        digest = hashlib.sha256(instruction.encode("utf-8")).hexdigest()
        relative = f"goals/{self.task['task_id']}-v{self.task['goal_version']}.tracking.json"
        atomic_json(self.output / relative, {"task_mode":"tracking", "instruction":instruction,
                    "instruction_sha256":digest, "goal_image_consumed":False})
        self.task.update(goal_id=f"tracking-instruction-sha256:{digest}", goal_sha256=None,
                         goal_path=None, tracking_target_path=relative,
                         tracking_visible=None, tracking_hold_reason=None,
                         tracking_source_frame_id=None, tracking_source_stamp_ns=None)

    def _seal_navigation_target(self, instruction: str) -> None:
        digest = hashlib.sha256(instruction.encode("utf-8")).hexdigest()
        relative = f"goals/{self.task['task_id']}-v{self.task['goal_version']}.instruction.json"
        atomic_json(self.output / relative, {"task_mode": self.task["task_mode"],
                    "instruction": instruction, "instruction_sha256": digest,
                    "goal_image_consumed": False, "implementation": "official_release"})
        self.task.update(goal_id=f"navigation-instruction-sha256:{digest}", goal_sha256=None,
                         goal_path=None, instruction_path=relative)

    def _live_goal_selection(self) -> dict[str, Any]:
        return {"schema_version":"imagegoal.goal_selection.v1","session_id":self.session_id,
                "mode":"live","ready":False,"selection_id":None,"sha256":None,
                "source":None,"source_frame_id":None,"source_metadata":None,
                "selected_timestamp_ns":None,"selection_path":None}

    def _check_selection_request(self, value: dict[str, Any]) -> None:
        if value.get("expected_session_id") != self.session_id:
            raise ValueError("expected_session_id mismatch or missing: refresh runtime status")
        self._check_expected_epoch(value,required=True)

    def _capture_goal_frame(self) -> Frame:
        if not self.frames:
            raise ValueError("no current RGB to capture")
        frame = self.frames[-1]
        age = (time.time_ns()-frame.stamp)/1e9
        if not -.1 <= age <= min(1.0,self.config.max_rgb_age_s):
            raise ValueError("current RGB is stale or future; a fresh frame is required to capture a goal")
        return frame

    def _prepare_selection(self, jpeg: bytes, source: str, frame: Frame | None = None) -> dict[str, Any]:
        """Persist validated candidate bytes before changing task authority."""
        identifier = uuid.uuid4().hex
        relative = f"goals/selection-{identifier}.jpg"
        selected = {"schema_version":"imagegoal.goal_selection.v1","session_id":self.session_id,
                    "mode":"frozen","ready":True,"selection_id":identifier,
                    "sha256":hashlib.sha256(jpeg).hexdigest(),"source":source,
                    "source_frame_id":frame.frame_id if frame else None,
                    "source_metadata":deepcopy(frame.metadata) if frame else None,
                    "selected_timestamp_ns":time.time_ns(),"selection_path":relative}
        (self.output/relative).write_bytes(jpeg)
        atomic_json(self.output/relative.replace(".jpg",".json"),selected)
        return selected

    def _selection_barrier(self, reason: str) -> None:
        if self.task and self.state not in TERMINAL:
            self._finish("CANCELLED",reason)
        else:
            self._invalidate(reason,require_post_stop_rgb=True)
        self.hold_after_stamp_ns = time.time_ns()
        self.preview = None
        self.last_frame_requested = None
        self.gates = ["no_active_task"]

    def select_goal(self, value: dict[str, Any], *, source: str) -> dict[str, Any]:
        """Freeze a camera/import goal without starting a task or inference."""
        if source not in {"camera","import"}:
            raise ValueError("goal source must be camera or import")
        # Invalid imports must leave the previous selection and task intact.
        goal = value.get("goal")
        jpeg = (decode_jpeg(goal.get("rgb_jpeg_base64") if isinstance(goal,dict) else None)
                if source=="import" else None)
        with self.lock:
            self._check_selection_request(value)
            frame = self._capture_goal_frame() if source=="camera" else None
            if frame is not None:
                jpeg = frame.jpeg
            selected = self._prepare_selection(jpeg,source,frame)
            self._selection_barrier("goal_selection_changed")
            self.goal_selection,self.goal_jpeg = selected,jpeg
            self._event("goal_selected",**{key:selected[key] for key in
                        ("selection_id","sha256","source","source_frame_id","selection_path")})
            return self.status()

    def clear_goal(self, value: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self._check_selection_request(value)
            self._selection_barrier("goal_selection_cleared")
            self.goal_selection,self.goal_jpeg = self._live_goal_selection(),None
            self._event("goal_selection_cleared")
            return self.status()

    def _check_expected_session(self, value: dict[str, Any]) -> None:
        # Optional for legacy HTTP clients; Foxglove always supplies it.
        # Validate under the same lock as mutation to reject commands that
        # crossed a runtime restart between the adapter's GET and POST.
        if "expected_session_id" in value and value["expected_session_id"] != self.session_id:
            raise ValueError("expected_session_id mismatch: runtime restarted; refresh status before sending a command")

    def _check_expected_epoch(self,value: dict[str,Any],*,required: bool = False) -> None:
        self._check_expected_session(value)
        expected=value.get("expected_epoch")
        if expected is None and not required:
            return
        if type(expected) is not int or expected!=self.epoch:
            raise ValueError(f"expected_epoch mismatch: requested {expected!r}, current {self.epoch}; task/goal request was superseded or stopped")

    def create_task(self, value: dict[str, Any], *, capture_current: bool = False) -> dict[str, Any]:
        backend = value.get("backend", "nomad")
        if backend not in {"nomad", "lightnav"}:
            raise ValueError("backend must be nomad or lightnav")
        task_mode = value.get("task_mode", "objnav" if backend == "lightnav" else "imagegoal")
        if not isinstance(task_mode,str) or task_mode not in {"imagegoal", "objnav", "vln", "tracking"}:
            raise ValueError("task_mode must be imagegoal, objnav, vln or tracking")
        tracking = task_mode == "tracking"
        lightnav = backend == "lightnav"
        imagegoal = task_mode == "imagegoal"
        if not lightnav and not imagegoal:
            raise ValueError("Official LightNav uses objnav/vln/tracking instructions; NoMaD uses imagegoal")
        if "goal_image" in value:
            raise ValueError("Use the goal object or a frozen selection_id, not goal_image")
        if lightnav and not imagegoal and (capture_current or "selection_id" in value or "goal" in value):
            raise ValueError("Official LightNav requires a language instruction, without goal image or selection_id")
        conditioning = "experimental_reference_image" if imagegoal else "native_tracking" if tracking else "native_language"
        if lightnav and value.get("conditioning_mode", conditioning) != conditioning:
            raise ValueError("LightNav conditioning_mode and task_mode disagree")
        if lightnav and (self.config.horizon != 5 or self.config.tick_hz != 10):
            raise ValueError("Official LightNav requires a five-step MPC horizon at 10 Hz")
        execution_mode = value.get("execution_mode","shadow")
        if execution_mode not in {"shadow","operator_run","supervised_native"}:
            raise ValueError("unexpected execution_mode")
        if execution_mode=="operator_run" and self.config.execution_profile!="operator-run":
            raise ValueError("runtime was not configured for operator-run; task cannot elevate authority")
        if execution_mode=="supervised_native" and (self.config.execution_profile!="supervised-native" or not self.config.operator_supervised):
            raise ValueError("runtime was not configured for onsite supervised native avoidance")
        timeout = value.get("timeout_s", 120.0)
        if isinstance(timeout, bool) or not isinstance(timeout, (int,float)) or not math.isfinite(timeout) or not 0 < timeout <= 3600:
            raise ValueError("timeout_s must be in (0,3600]")
        instruction = value.get("instruction", "")
        if not isinstance(instruction, str) or len(instruction) > 4096:
            raise ValueError("instruction must be a string of at most 4096 characters")
        if lightnav and not imagegoal:
            instruction = instruction.strip()
            if not instruction:
                raise ValueError("Official LightNav requires a nonempty task instruction")
        with self.lock:
            selected = None
            jpeg = None
            if lightnav and not imagegoal:
                self._check_expected_epoch(value,required=True)
            elif "selection_id" in value:
                self._check_selection_request(value)
                if capture_current or "goal" in value:
                    raise ValueError("selection_id cannot be combined with an inline goal or capture_current")
                if (not self.goal_selection["ready"] or not isinstance(value["selection_id"],str)
                    or value["selection_id"] != self.goal_selection["selection_id"] or self.goal_jpeg is None):
                    raise ValueError("selection_id mismatch or no frozen goal; capture or import a goal first")
                jpeg = self.goal_jpeg
                selected = self.goal_selection
            else:
                self._check_expected_epoch(value,required=execution_mode in {"operator_run","supervised_native"})
                if capture_current:
                    frame = self._capture_goal_frame()
                    jpeg = frame.jpeg
                    selected = self._prepare_selection(jpeg,"camera",frame)
                else:
                    goal = value.get("goal")
                    jpeg = decode_jpeg(goal.get("rgb_jpeg_base64") if isinstance(goal,dict) else None)
                    selected = self._prepare_selection(jpeg,"import")
            if self.task and self.state not in TERMINAL:
                self._finish("CANCELLED", "superseded_by_new_task")
            if lightnav:
                self.lightnav_http.cancel_inflight()
                self.lightnav_http.close()
                self.lightnav_http = LightNavHTTPClient()
            self.epoch += 1
            if selected is not None:
                self.goal_selection = selected
            self.task = {"task_id": uuid.uuid4().hex, "backend": backend, "task_mode":task_mode,
                         "control_profile":"go2_imagegoal_adaptation" if lightnav and imagegoal else "official_release" if lightnav else "nomad_imagegoal",
                         "goal_version": 1,"execution_mode":execution_mode,
                         "selection_id":selected["selection_id"] if selected else None,
                         "created_timestamp_ns": time.time_ns(), "deadline_monotonic_ns": time.monotonic_ns()+int(timeout*1e9),
                         "timeout_s": timeout, "instruction": instruction, "paused": False,
                         "model_stop_latched": False, "outcome": None, "consecutive_inference_errors": 0,
                         "nomad_stop_streak":0,"nomad_stop_source_frame_id":None,"nomad_stop_epoch":self.epoch,
                         "goal_started_timestamp_ns":time.time_ns(), "last_lightnav_ack_stamp_ns":None}
            if tracking:
                self._seal_tracking_target(instruction)
            elif lightnav and not imagegoal:
                self._seal_navigation_target(instruction)
            else:
                self._seal_goal(jpeg)
            self.lightnav_controller.reset()
            self.plan = self.preview = None
            self.last_frame_requested = None
            self.hold_after_stamp_ns = time.time_ns()
            self.proposal = self._zero("new_task_waiting_inputs")
            self.state = "WAITING_INPUT"
            self._event("task_started", backend=backend, task_mode=task_mode,
                        goal_id=self.task["goal_id"], goal_version=1,
                        goal_sha256=self.task["goal_sha256"], selection_id=self.task["selection_id"],
                        captured_current=capture_current,execution_mode=execution_mode)
            return self.status()

    def replace_goal(self, value: dict[str, Any]) -> dict[str, Any]:
        goal = value.get("goal")
        jpeg = decode_jpeg(goal.get("rgb_jpeg_base64") if isinstance(goal,dict) else None)
        with self.lock:
            self._require_active()
            if self.task["backend"] == "lightnav" and self.task.get("task_mode") != "imagegoal":
                raise ValueError("Official LightNav instruction changes require a new task, not an image goal")
            self._check_expected_epoch(value,required=self.task.get("execution_mode") in {"operator_run","supervised_native"})
            selected = self._prepare_selection(jpeg,"import")
            self._invalidate("goal_changed", require_post_stop_rgb=True)
            self.task["goal_version"] += 1
            self.task["model_stop_latched"] = False
            self.task["goal_started_timestamp_ns"] = time.time_ns()
            self.task["last_lightnav_ack_stamp_ns"] = None
            self.goal_selection = selected
            self.task["selection_id"] = selected["selection_id"]
            self._seal_goal(jpeg)
            self.state="PAUSED" if self.task["paused"] else "WAITING_INPUT"
            self.gates=["operator_paused","fresh_plan_required"] if self.task["paused"] else ["fresh_plan_required"]
            self._event("goal_changed", goal_id=self.task["goal_id"], goal_version=self.task["goal_version"],
                        selection_id=selected["selection_id"])
            return self.status()

    def _require_active(self) -> None:
        if not self.task or self.state in TERMINAL:
            raise ValueError("no active task")

    def task_action(self, action: str, value: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self._check_expected_session(value)
            if "expected_task_id" in value and value["expected_task_id"] != (self.task or {}).get("task_id"):
                raise ValueError("expected_task_id mismatch: the active task changed")
            if action == "cancel":
                if self.task and self.state not in TERMINAL:
                    self._finish("CANCELLED", str(value.get("reason", "operator_stop")))
                else:
                    # Even IDLE/terminal Stop is a cancellation barrier for
                    # a target POST whose body has not finished arriving.
                    self._invalidate(str(value.get("reason","operator_stop")),require_post_stop_rgb=True)
                    self.gates=["no_active_task"]
                return self.status()
            self._check_expected_epoch(value)
            self._require_active()
            if action == "pause":
                self.task["paused"] = True
                self._invalidate("operator_pause", require_post_stop_rgb=True)
                self.state = "PAUSED"
                self._event("task_paused")
            elif action == "resume":
                if self.task["model_stop_latched"]:
                    raise ValueError("model STOP is latched; choose a new goal or record an outcome")
                self.task["paused"] = False
                self._invalidate("operator_resume_waiting_fresh_rgb", require_post_stop_rgb=True)
                self.state = "WAITING_INPUT"
                self._event("task_resumed")
            elif action == "complete":
                if self.task.get("task_mode") == "tracking":
                    raise ValueError("tracking is continuous; stop the task rather than confirming arrival")
                if value.get("evidence") != "operator_confirmed":
                    raise ValueError("success requires evidence=operator_confirmed; model scores are not arrival evidence")
                self._finish("SUCCEEDED", "operator_confirmed")
            else:
                raise ValueError("unknown task action")
            return self.status()

    def _invalidate(self, reason: str, *, require_post_stop_rgb: bool = False) -> None:
        self.epoch += 1
        self.lightnav_controller.reset()
        self.plan = None
        self.proposal = self._zero(reason)
        self.proposal_evaluated_ns = time.time_ns()
        self.control_sequence += 1
        self.metrics["invalidations"] += 1
        if self.task and reason!="nomad_distance_stop_not_arrival":
            self.task["nomad_stop_streak"]=0
            self.task["nomad_stop_source_frame_id"]=None
            self.task["nomad_stop_epoch"]=self.epoch
        if require_post_stop_rgb:
            self.hold_after_stamp_ns = time.time_ns()
        self._event("execution_invalidated", reason=reason)

    def _finish(self, state: str, reason: str) -> None:
        self._invalidate(reason)
        self.state = state
        if self.native_executor is not None:
            self.native_executor.stop(reason)
        self.task["outcome"] = {"state": state, "reason": reason, "timestamp_ns": time.time_ns(),
                                "arrival_independently_validated": False}
        self._event("task_finished", state=state, reason=reason)
        atomic_json(self.output / f"task-{self.task['task_id']}.json", self.task)
        if self.task["backend"]=="lightnav":
            self.lightnav_http.cancel_inflight()
            self.lightnav_http.close()
            self._schedule_lightnav_cleanup(self.task["task_id"])

    def _schedule_lightnav_cleanup(self, task_id: str) -> None:
        """Release only this finished task's server FIFO after its request."""
        if task_id in self.pending_session_cleanup:
            return
        self.pending_session_cleanup.add(task_id)
        identity = {"session_id":self.session_id,"task_id":task_id}
        def worker():
            attempts, last_report = 0, 0.
            # A timed-out HTTP caller does not cancel GPU inference. Keep this
            # exact owned task pending until the model slot is actually free;
            # abandoning it after 15 s can permanently fill session capacity.
            while not self.stop_event.is_set():
                try:
                    response,_ = request_json(self.config.lightnav_url.rstrip("/")+"/reset",timeout_s=3.,payload={"identity":identity,"retire":True})
                    if response.get("retired") is not True:
                        raise ValueError("Model service did not retire the completed task identity")
                    if response.get("cleanup_pending"):
                        remote,_ = request_json(self.config.lightnav_url.rstrip("/")+"/session/status",
                                                timeout_s=3.,payload={"identity":identity})
                        if not remote.get("retired") or remote.get("cleanup_pending") or remote.get("exists"):
                            raise RuntimeError("Task is retired; waiting for its in-flight model resources to be released")
                    with self.lock:
                        self.pending_session_cleanup.discard(task_id)
                        self._event("lightnav_session_released",released_task_id=task_id,response=response)
                    return
                except Exception as exc:
                    attempts += 1
                    if attempts == 1 or time.monotonic()-last_report >= 15.:
                        with self.lock:
                            self._event("lightnav_session_cleanup_retrying",released_task_id=task_id,
                                        attempts=attempts,detail=f"{type(exc).__name__}: {exc}")
                        last_report = time.monotonic()
                    self.stop_event.wait(min(3., .25 * 2**min(attempts,4)))
            with self.lock:
                self._event("lightnav_session_cleanup_interrupted",released_task_id=task_id,
                            attempts=attempts,reason="runtime_shutdown; release_not_confirmed")
        threading.Thread(target=worker,name="imagegoal-lightnav-session-cleanup",daemon=True).start()

    @staticmethod
    def _age(meta: dict[str,Any] | None, now: int) -> float | None:
        return None if meta is None else (now-meta["capture_timestamp_ns"])/1e9

    def _sensor_gates(self, now: int) -> list[str]:
        result = []
        if not self.frames:
            result.append("rgb_missing")
        elif not -.1 <= self._age(self.frames[-1].metadata, now) <= self.config.max_rgb_age_s:
            result.append("rgb_stale_or_future")
        if not self.odom:
            result.append("odom_missing")
        elif not -.1 <= self._age(self.odom[-1][1], now) <= self.config.max_odom_age_s:
            result.append("odom_stale_or_future")
        if self.config.execution_profile == "supervised-native":
            # An explicit alternative safety architecture: the separate
            # executor verifies the native avoidance service and owns every
            # nonzero RPC. No dummy depth or calibrated timestamp is inserted.
            sources = self._sensors_snapshot(now)
            expected = {"rgb":("go2_front_camera_optical_frame","go2_video_rpc_receive_host_system_time_not_sensor_source_time"),
                        "odom":("go2_odom","go2_dds_receive_host_system_time_not_sensor_source_time")}
            for name,(frame,provenance) in expected.items():
                item = sources[name]
                if item is None:
                    continue
                if (item.get("synthetic") is not False or item.get("commissioning") is not False
                    or item.get("clock_domain") != "unix_epoch_ns"):
                    result.append(f"native_{name}_not_real")
                if item.get("frame_name") != frame or item.get("timestamp_source") != provenance:
                    result.append(f"native_{name}_source_mismatch")
                if item.get("age_s",-1) < 0:
                    result.append(f"native_{name}_future")
            if sources["odom"] and sources["odom"].get("body_frame_name") != "base_link":
                result.append("native_odom_body_frame_mismatch")
            return result
        if self.depth is None:
            result.append("depth_missing")
        elif not -.1 <= self._age(self.depth["metadata"], now) <= self.config.max_depth_age_s:
            result.append("depth_stale_or_future")
        elif not self.depth["aligned_to_rgb"]:
            result.append("depth_not_aligned_to_rgb")
        elif self.depth["clearance_m"] is None:
            result.append("depth_unavailable_stop")
        elif self.depth["clearance_m"] <= self.config.hard_stop_m:
            result.append("obstacle_stop")
        if self.depth is not None and self.frames:
            if self.depth["metadata"]["frame_name"] != self.frames[-1].metadata["frame_name"]:
                result.append("rgb_depth_frame_mismatch")
            if abs(self.depth["metadata"]["capture_timestamp_ns"]-self.frames[-1].stamp)/1e9 > self.config.max_rgb_depth_skew_s:
                result.append("rgb_depth_timestamp_skew")
        return result

    def _sensors_snapshot(self, now: int) -> dict[str,Any]:
        latest = self.frames[-1] if self.frames else None
        odom = self.odom[-1] if self.odom else None
        return {"rgb":None if latest is None else {**latest.metadata,"frame_id":latest.frame_id,"age_s":self._age(latest.metadata,now),"history_size":len(self.frames)},
                "odom":None if odom is None else {**odom[1],"age_s":self._age(odom[1],now),"pose_m_rad":[odom[0].x,odom[0].y,odom[0].yaw],"velocity_v_w":odom[3]},
                "depth":None if self.depth is None else {**self.depth["metadata"],"age_s":self._age(self.depth["metadata"],now),"clearance_m":self.depth["clearance_m"],"aligned_to_rgb":self.depth["aligned_to_rgb"],"shape_hw":self.depth["shape_hw"]}}

    def _operator_source_reasons(self, now: int, *, allow_commissioning: bool = False) -> list[str]:
        reasons = []
        if self.config.execution_profile!="operator-run":
            reasons.append("runtime_execution_profile_shadow")
        if self.config.profile!="external":
            reasons.append("operator_run_requires_measured_inputs")
        reasons.extend(self.calibration.blocking_reasons)
        if self.calibration.commissioning and not allow_commissioning:
            reasons.append("commissioning_calibration_cannot_authorize_robot")
        sensors = self._sensors_snapshot(now)
        reasons.extend(self.calibration.source_reasons(sensors))
        reasons.extend(self._sensor_gates(now))
        uncertainty = (self.calibration.timing or {}).get("max_alignment_error_s",.05)
        if type(uncertainty) not in (int,float) or not math.isfinite(uncertainty):
            uncertainty = .05
        for name,max_age in (("rgb",self.config.max_rgb_age_s),("depth",self.config.max_depth_age_s),("odom",self.config.max_odom_age_s)):
            item = sensors[name]
            if item is None:
                continue
            if item["synthetic"] is not False:
                reasons.append(f"operator_{name}_synthetic")
            if item["commissioning"] and not allow_commissioning:
                reasons.append(f"operator_{name}_commissioning")
            if not 0<=item["age_s"]<=max_age-uncertainty:
                reasons.append(f"operator_{name}_age_or_calibration_uncertainty")
        return list(dict.fromkeys(reasons))

    def _task_execution_reasons(self, now: int) -> list[str]:
        if self.task and self.task.get("execution_mode")=="operator_run":
            return self._operator_source_reasons(now,allow_commissioning=self.config.commissioning_real_inputs)
        if self.task and self.task.get("execution_mode")=="supervised_native":
            return self._native_source_reasons(now)
        return []

    def _native_source_reasons(self, now: int) -> list[str]:
        reasons = []
        if self.config.execution_profile != "supervised-native" or not self.config.operator_supervised:
            reasons.append("native_supervision_not_configured")
        if self.config.profile != "external" or self.config.commissioning_real_inputs:
            reasons.append("native_requires_real_external_inputs")
        reasons.extend(self._sensor_gates(now))
        return list(dict.fromkeys(reasons))

    def start_native(self, value: dict[str,Any], *, runtime_url: str) -> dict[str,Any]:
        with self.lock:
            if self.config.execution_profile!="supervised-native" or not self.config.operator_supervised:
                raise ValueError("prepare the supervised-native observation profile before native Start")
            if value.get("operator_present") is not True:
                raise ValueError("native Start requires an onsite operator")
            if "expected_session_id" not in value:
                raise ValueError("native Start requires expected_session_id")
            self._check_expected_epoch(value,required=True)
            backend = value.get("backend", "nomad")
            if backend not in {"nomad", "lightnav"}:
                raise ValueError("native Start backend must be nomad or lightnav")
            timeout = value.get("timeout_s",60)
            if type(timeout) not in (float,int) or not math.isfinite(timeout) or not 5 <= timeout <= 120:
                raise ValueError("supervised native duration must be 5–120 seconds")
            if self.native_executor is None:
                from .native_control import NativeExecutorManager
                self.native_executor = NativeExecutorManager(self)
            if self.native_executor.status()["running"]:
                raise ValueError("stop the current native executor before starting a new task")
            task_value = dict(value,backend=backend,execution_mode="supervised_native",timeout_s=timeout)
            self.create_task(task_value)
            try:
                self.native_executor.start(self.task["task_id"],runtime_url,timeout)
            except Exception:
                self._finish("FAILED","native_executor_start_failed")
                raise
            self._event("native_executor_started",task_id=self.task["task_id"],duration_s=timeout)
            return self.status()

    def stop_native(self, value: dict[str,Any]) -> dict[str,Any]:
        with self.lock:
            self._check_expected_session(value)
            if "expected_task_id" in value and value["expected_task_id"] != (self.task or {}).get("task_id"):
                raise ValueError("expected_task_id mismatch: the active task changed")
            if self.native_executor is not None:
                self.native_executor.stop("operator_native_stop")
            self.task_action("cancel",dict(value,reason="operator_native_stop"))
            return self.status()

    def _window(self, now: int) -> list[Frame] | None:
        required = 1 if self.task["backend"] == "lightnav" else self.config.history_frames
        if len(self.frames) < required:
            return None
        frames = list(self.frames)[-self.config.history_frames:]
        if self.task["backend"]=="lightnav":
            available = [frame for frame in self.frames if frame.stamp>=self.task["goal_started_timestamp_ns"]]
            if len(available)<required:
                return None
            previous = self.task["last_lightnav_ack_stamp_ns"]
            if previous is None:
                frames = available
            else:
                new_frames = [frame for frame in available if frame.stamp>previous]
                overlap = [frame for frame in available if frame.stamp<=previous]
                if not new_frames:
                    return None
                frames = overlap[-1:]+new_frames
                if not overlap and new_frames[0].stamp>previous+int(self.config.max_history_gap_s*1e9):
                    self._finish("FAILED","lightnav_native_history_gap_exceeded")
                    return None
            if len(frames)>64:
                self._finish("FAILED","lightnav_native_history_gap_exceeded")
                return None
            pending = [frame for frame in frames if previous is None or frame.stamp > previous]
            if (len(frames) > self.config.lightnav_max_pending_frames or
                    pending and (now-pending[0].stamp)/1e9 > self.config.lightnav_max_pending_age_s):
                self._event("lightnav_upload_budget_exceeded", frames=len(frames),
                            oldest_pending_age_s=(now-pending[0].stamp)/1e9 if pending else None)
                self._finish("FAILED", "lightnav_upload_backlog_exceeded")
                return None
        if frames[-1].stamp < self.hold_after_stamp_ns:
            return None
        if (now-frames[-1].stamp)/1e9 > self.config.max_rgb_age_s:
            return None
        if any((right.stamp-left.stamp)/1e9 > self.config.max_history_gap_s for left,right in zip(frames,frames[1:])):
            return None
        return frames

    def _dispatch(self, frames: list[Frame]) -> None:
        identity = {"session_id": self.session_id, "task_id": self.task["task_id"], "goal_id": self.task["goal_id"],
                    "goal_version": self.task["goal_version"], "frame_id": frames[-1].frame_id}
        observation_transport = None
        if self.task["backend"] == "lightnav":
            observations, observation_transport = self.lightnav_transport.observations(frames)
        else:
            observations = [frame.wire() for frame in frames]
        tracking = self.task.get("task_mode") == "tracking"
        payload = {"identity": identity, "observations": observations}
        if self.task.get("task_mode") == "imagegoal":
            payload["goal"] = {"rgb_jpeg_base64": base64.b64encode(self.goal_jpeg).decode()}
        if self.task["backend"] == "lightnav":
            payload["instruction"] = self.task["instruction"]
            if tracking:
                payload.update(conditioning_mode="native_tracking",task_mode="tracking")
            elif self.task["task_mode"] == "imagegoal":
                payload.update(conditioning_mode="experimental_reference_image",task_mode="imagegoal")
            else:
                payload.update(conditioning_mode="native_language", task_mode=self.task["task_mode"])
            wire_bytes = len(json.dumps(payload,allow_nan=False,separators=(",",":")).encode())
            if wire_bytes > self.config.lightnav_max_request_bytes:
                self._event("lightnav_upload_budget_exceeded", frames=len(frames),wire_bytes=wire_bytes)
                self._finish("FAILED", "lightnav_upload_bytes_exceeded")
                return
        capture = pose_at_stamp([item[0] for item in self.odom], frames[-1].stamp, self.config.max_capture_odom_gap_s)
        request_id = f"{self.metrics['inference_started']+1:06d}"
        for frame in frames:
            name = hashlib.sha256(frame.jpeg).hexdigest()+".jpg"
            path = self.output / "rgb" / name
            if not path.exists():
                path.write_bytes(frame.jpeg)
        metadata = {"request_id": request_id, "identity": identity, "epoch": self.epoch,
                    "backend": self.task["backend"], "task_mode":self.task.get("task_mode","imagegoal"),
                    "source_stamp_ns": frames[-1].stamp,
                    "source_metadata": frames[-1].metadata, "created_monotonic_ns": time.monotonic_ns(),
                    "capture_pose": [capture[0].x,capture[0].y,capture[0].yaw] if capture else None,
                    "capture_odom_gap_s": capture[1] if capture else None,
                    "frames": [{"frame_id": f.frame_id, **f.metadata, "provenance": f.provenance,
                                "stored_path": "rgb/"+hashlib.sha256(f.jpeg).hexdigest()+".jpg"} for f in frames]}
        if observation_transport is not None:
            metadata["observation_transport"] = observation_transport
            metadata["transport"] = {"serialized_bytes":wire_bytes,
                                     "max_request_bytes":self.config.lightnav_max_request_bytes,
                                     "automatic_image_retry":False,"images_attempted":False}
            metadata["session_preflight_required"] = self.task.get("lightnav_server_boot_id") is None
        self.inflight = metadata
        self.last_frame_requested = (self.epoch, frames[-1].frame_id)
        self.metrics["inference_started"] += 1
        atomic_json(self.output / "requests" / f"{request_id}.request.json",
                    {**metadata, "goal_path": self.task["goal_path"],
                     "instruction":payload.get("instruction"),
                     "conditioning_mode":payload.get("conditioning_mode")})
        self._event("inference_started", request_id=request_id, source_frame_id=identity["frame_id"], goal_version=identity["goal_version"])
        url = getattr(self.config, self.task["backend"]+"_url").rstrip("/")+"/infer"
        delay = float(self.faults.get("response_delay_s", 0))
        model_client = self.lightnav_http if self.task["backend"] == "lightnav" else None
        model_request = model_client.request_json if model_client is not None else request_json

        def worker():
            try:
                if metadata["backend"] == "lightnav" and metadata["session_preflight_required"]:
                    # A metadata-only request establishes that capacity and
                    # lifecycle are usable before any image upload. It is not
                    # an atomic reservation: a racing refusal still terminates
                    # this episode without retransmitting its JPEG history.
                    control = LightNavHTTPClient()
                    try:
                        snapshot,_ = control.request_json(
                            self.config.lightnav_url.rstrip("/")+"/session/status",
                            timeout_s=min(2.,self.config.request_timeout_s),
                            payload={"identity":{key:identity[key] for key in ("session_id","task_id")}})
                    finally:
                        control.close()
                    metadata["session_preflight"] = snapshot
                    if snapshot.get("transport_protocol") != "lightnav.http.lifecycle.v2":
                        raise RuntimeError("LightNav lifecycle protocol mismatch")
                    if (snapshot.get("identity") != {key:identity[key] for key in ("session_id","task_id")} or
                            not isinstance(snapshot.get("server_boot_id"),str) or not snapshot["server_boot_id"]):
                        raise RuntimeError("LightNav session preflight identity mismatch")
                    if snapshot.get("retired"):
                        metadata["preflight_error_code"] = "task_retired"
                        raise RuntimeError("LightNav task_retired before image upload")
                    if snapshot.get("inference_busy"):
                        metadata["preflight_error_code"] = "inference_busy"
                        raise RuntimeError("LightNav inference_busy before image upload")
                    if (not snapshot.get("exists") and
                            snapshot.get("active_session_count",0) >= snapshot.get("max_sessions",0)):
                        metadata["preflight_error_code"] = "session_capacity"
                        raise RuntimeError("LightNav session_capacity before image upload")
                    with self.lock:
                        if (not self.task or self.state in TERMINAL or self.task.get("paused") or
                                self.task["task_id"] != identity["task_id"] or
                                self.task["goal_version"] != identity["goal_version"] or
                                self.epoch != metadata["epoch"]):
                            raise RuntimeError("LightNav task changed before image upload")
                        self.task["lightnav_server_boot_id"] = snapshot["server_boot_id"]
                request_timeout = self.config.request_timeout_s
                if metadata["backend"] == "lightnav":
                    with self.lock:
                        if (not self.task or self.state in TERMINAL or self.task.get("paused") or
                                self.task["task_id"] != identity["task_id"] or
                                self.task["goal_version"] != identity["goal_version"] or
                                self.epoch != metadata["epoch"]):
                            raise RuntimeError("LightNav task changed before image upload")
                        metadata["expected_server_boot_id"] = self.task["lightnav_server_boot_id"]
                    payload["server_boot_id"] = metadata["expected_server_boot_id"]
                    wire_bytes_final = len(json.dumps(payload,allow_nan=False,separators=(",",":")).encode())
                    metadata["transport"]["serialized_bytes"] = wire_bytes_final
                    if wire_bytes_final > self.config.lightnav_max_request_bytes:
                        raise ValueError("LightNav image upload exceeds its byte budget")
                    if (time.time_ns()-frames[0].stamp)/1e9 > self.config.lightnav_max_pending_age_s:
                        raise TimeoutError("LightNav pending history aged out before image upload")
                    request_timeout -= (time.monotonic_ns()-metadata["created_monotonic_ns"])/1e9
                    if request_timeout <= 0:
                        raise TimeoutError("LightNav preflight exhausted the request deadline")
                    metadata["transport"]["images_attempted"] = True
                response, rtt = model_request(url, timeout_s=request_timeout, payload=payload)
                if (metadata["backend"] == "lightnav" and
                        response.get("server_boot_id") != metadata["expected_server_boot_id"]):
                    raise RuntimeError("LightNav server restarted; old history cannot resume implicitly")
                if delay:
                    self.stop_event.wait(delay)
                if metadata["backend"] == "lightnav":
                    metadata["transport_http"] = model_client.last_request_stats
                self.completed.put((metadata,payload,response,rtt,None))
            except Exception as exc:
                if metadata["backend"] == "lightnav":
                    metadata["transport_error"] = (exc.as_dict() if hasattr(exc,"as_dict") else
                        {"type":type(exc).__name__,"code":metadata.get("preflight_error_code","transport_failure"),"detail":str(exc)})
                    metadata["transport_http"] = (model_client.last_request_stats
                        if metadata["transport"]["images_attempted"] else None)
                self.completed.put((metadata,payload,None,None,f"{type(exc).__name__}: {exc}"))
        threading.Thread(target=worker,name=f"imagegoal-inference-{request_id}",daemon=True).start()

    def _ack_lightnav_observations(self, metadata, payload, response) -> None:
        """A verified history commit is independent of motion-plan eligibility."""
        if (metadata["backend"] != "lightnav" or not self.task or self.state in TERMINAL or
                not isinstance(response,dict)):
            return
        identity = payload["identity"]
        if any(identity.get(key) != value for key,value in
               (("session_id",self.session_id),("task_id",self.task["task_id"]),
                ("goal_id",self.task["goal_id"]),("goal_version",self.task["goal_version"]))):
            return
        tracking = self.task.get("task_mode") == "tracking"
        adapted = self.task.get("task_mode") == "imagegoal"
        expected = [{key:f[key] for key in ("frame_id","capture_timestamp_ns")}
                    for f in payload["observations"]]
        checks = {"schema_version":"lightnav.imagegoal.inference.v1", "identity":identity,
                  "observation_frames":expected,"source_frame":expected[-1],
                  "conditioning_mode":"experimental_reference_image" if adapted else "native_tracking" if tracking else "native_language",
                  "implementation":"go2_imagegoal_adaptation" if adapted else "official_release",
                  "official_release_commit":OFFICIAL_COMMIT,"official_core_sha256":OFFICIAL_CORE_DIGEST,
                  "eval_task":"trackvla" if tracking else "vlnce", "task_mode":self.task["task_mode"]}
        if (any(response.get(key) != value for key,value in checks.items()) or
                response.get("goal_image_consumed") is not adapted):
            return
        stamp = expected[-1]["capture_timestamp_ns"]
        previous = self.task["last_lightnav_ack_stamp_ns"]
        if previous is None or stamp > previous:
            self.task["last_lightnav_ack_stamp_ns"] = stamp
            self._event("lightnav_history_acknowledged",request_id=metadata["request_id"],
                        source_stamp_ns=stamp,motion_authorized_by_ack=False)

    def _accept_completed(self) -> None:
        while not self.completed.empty():
            metadata,payload,response,rtt,error = self.completed.get()
            superseded = bool(self.inflight is not None and any(
                self.inflight.get(key) != metadata.get(key) for key in ("request_id","identity","epoch")))
            if not superseded:
                self.inflight = None
            if rtt is not None:
                self.metrics["last_rtt_ms"] = rtt
            if isinstance(response,dict) and isinstance(response.get("inference_ms"),(int,float)) and math.isfinite(response["inference_ms"]):
                self.metrics["last_inference_ms"] = response["inference_ms"]
            request_id = metadata["request_id"]
            atomic_json(self.output / "requests" / f"{request_id}.response.json", {
                "response":response,"error":error,"round_trip_ms":rtt,
                "transport":metadata.get("transport"),"transport_http":metadata.get("transport_http"),
                "transport_error":metadata.get("transport_error"),"session_preflight":metadata.get("session_preflight")})
            if not error:
                self._ack_lightnav_observations(metadata,payload,response)
            reasons = ["inference_request_superseded"] if superseded else []
            if not self.task or self.state in TERMINAL or self.task["paused"] or metadata["epoch"] != self.epoch:
                reasons.append("execution_epoch_invalidated")
            elif payload["identity"]["task_id"] != self.task["task_id"] or payload["identity"]["goal_version"] != self.task["goal_version"]:
                reasons.append("task_or_goal_changed")
            elif time.monotonic_ns() >= self.task["deadline_monotonic_ns"]:
                self._finish("TIMED_OUT", "task_budget_exhausted")
                reasons.append("task_budget_exhausted")
            if error:
                self.metrics["inference_errors"] += 1
                reasons.append("inference_error")
                same_task = bool(self.task and payload["identity"]["task_id"] == self.task["task_id"] and
                                 payload["identity"]["goal_version"] == self.task["goal_version"])
                if metadata["backend"] == "lightnav" and same_task and not superseded and self.state not in TERMINAL:
                    # A timeout is an unknown remote commit, not permission to
                    # resend a growing history. Stop this episode immediately;
                    # the independent retirement path resolves remote cleanup.
                    self.task["transport_failure"] = metadata.get("transport_error",{"detail":error})
                    self._event("lightnav_image_upload_stopped",request_id=request_id,
                                detail=self.task["transport_failure"],automatic_retry=False)
                    self._finish("FAILED", "lightnav_transport_failed_no_image_retry")
                elif self.task and metadata["epoch"] == self.epoch and self.state not in TERMINAL:
                    self.task["consecutive_inference_errors"] += 1
                    if self.task["consecutive_inference_errors"] >= self.config.max_inference_errors:
                        self._finish("FAILED", "repeated_inference_errors")
            if not reasons:
                try:
                    if metadata["backend"] == "nomad":
                        checks = inspect_response(response,payload)
                        if not all(checks.values()):
                            raise ValueError(f"NoMaD contract checks failed: {checks}")
                        raw,scale = response["first_sample_normalized"],response["action_scale_m"]
                        if not math.isclose(scale,0.05,rel_tol=1e-7,abs_tol=1e-9):
                            raise ValueError("NoMaD action scale differs from the pinned 0.2/4=0.05 calibration")
                    else:
                        expected = [{key:f[key] for key in ("frame_id","capture_timestamp_ns")} for f in payload["observations"]]
                        if response.get("schema_version") != "lightnav.imagegoal.inference.v1" or response.get("identity") != payload["identity"] or response.get("observation_frames") != expected or response.get("source_frame") != expected[-1]:
                            raise ValueError("LightNav lineage/schema mismatch")
                        tracking = self.task.get("task_mode") == "tracking"
                        adapted_imagegoal = self.task.get("task_mode") == "imagegoal"
                        expected_mode = "experimental_reference_image" if adapted_imagegoal else "native_tracking" if tracking else "native_language"
                        if response.get("conditioning_mode") != expected_mode:
                            raise ValueError(f"LightNav must declare {expected_mode} conditioning")
                        if (response.get("implementation") != ("go2_imagegoal_adaptation" if adapted_imagegoal else "official_release") or
                            response.get("official_release_commit") != OFFICIAL_COMMIT or
                            response.get("official_core_sha256") != OFFICIAL_CORE_DIGEST or
                            response.get("eval_task") != ("trackvla" if tracking else "vlnce") or
                            response.get("goal_image_consumed") is not adapted_imagegoal or
                            response.get("task_mode") != self.task["task_mode"]):
                            raise ValueError("LightNav response does not match the requested model/profile/goal conditioning")
                        if tracking:
                            if response.get("task_mode") != "tracking" or response.get("goal_image_consumed") is not False:
                                raise ValueError("LightNav tracking requires native tracking without a goal image")
                            visible = response.get("model_visible")
                            if visible is not None and type(visible) is not bool:
                                raise ValueError("LightNav target visibility must be boolean or null")
                        raw,scale = response.get("trajectory_body_m"),1.0
                        array = np.asarray(raw,dtype=float)
                        if array.ndim != 2 or array.shape[1] != 2 or len(array)==0 or not np.isfinite(array).all():
                            raise ValueError("LightNav requires finite metric Nx2 trajectory")
                        if not isinstance(response.get("model_stop"),bool) or response.get("arrival_confirmed") is not False:
                            raise ValueError("LightNav STOP/arrival contract invalid")
                        self.task["last_lightnav_ack_stamp_ns"] = metadata["source_stamp_ns"]
                        self.metrics["lightnav_policy_history_count"] = response.get("policy_history_count",response.get("history_frames_count",len(expected)))
                    if metadata["backend"] == "lightnav":
                        native_path = validate_waypoints(response.get("trajectory_body_m_rad"))
                        if not np.allclose(native_path[:,:2],np.asarray(raw),rtol=1e-7,atol=1e-9):
                            raise ValueError("LightNav XY and SE(2) trajectory disagree")
                        if adapted_imagegoal:
                            sampled, adaptation = prepare_path(native_path,capture_stamp_ns=metadata["source_stamp_ns"],config=self.config)
                        else:
                            sampled, adaptation = None, None
                        preview_xy = native_path[:, :2].tolist()
                        has_motion = bool(np.any(native_path))
                        if response["model_stop"] != bool(np.allclose(native_path, 0.0)):
                            raise ValueError("LightNav STOP must agree with its decoded zero trajectory")
                    else:
                        native_path = None
                        adaptation = None
                        metric = raw_nomad_to_metric_path(np.asarray(raw),action_scale_m=scale,capture_stamp_ns=metadata["source_stamp_ns"])
                        sampled = resample_metric_path(metric,v_ref_mps=self.config.v_ref_mps,mpc_dt_s=1/self.config.tick_hz)
                        preview_xy = metric.xy_m.tolist()
                        has_motion = sampled.has_motion
                    self.preview = {"request_id":request_id,"backend":metadata["backend"],"trajectory_body_m":preview_xy,
                                    "source_stamp_ns":metadata["source_stamp_ns"],"accepted_for_tracking":False,
                                    "distance_steps":response.get("distance_steps"),"conditioning_mode":response.get("conditioning_mode"),
                                    "task_mode":self.task.get("task_mode","imagegoal"),
                                    "control_profile":self.task["control_profile"],"adaptation":adaptation,
                                    "model_visible":response.get("model_visible"),"model_stop":response.get("model_stop"),
                                    "round_trip_ms":rtt,"model_inference_ms":response.get("inference_ms"),
                                    "goal_version":metadata["identity"]["goal_version"],"identity":metadata["identity"],
                                    "epoch":metadata["epoch"],"frame":"body_at_newest_capture_unanchored_preview",
                                    "units":"meters; +x forward, +y left", "arrival_confirmed":False}
                    age = (time.time_ns()-metadata["source_stamp_ns"])/1e9
                    elapsed = (time.monotonic_ns()-metadata["created_monotonic_ns"])/1e9
                    if elapsed > self.config.request_timeout_s:
                        reasons.append("inference_deadline_exceeded")
                    if not -.1 <= age <= self.config.max_plan_age_s:
                        reasons.append("plan_source_stale_or_future")
                    if metadata["capture_pose"] is None:
                        reasons.append("capture_odom_unavailable")
                    reasons.extend(self._sensor_gates(time.time_ns()))
                    reasons.extend(self._task_execution_reasons(time.time_ns()))
                    if self.task.get("task_mode") == "tracking" and not reasons:
                        self.task.update(tracking_visible=response.get("model_visible"),
                                         tracking_source_frame_id=metadata["identity"]["frame_id"],
                                         tracking_source_stamp_ns=metadata["source_stamp_ns"])
                        hold_reason = ("tracking_model_stop" if response.get("model_stop") else
                                       "tracking_zero_path" if not has_motion else None)
                        if hold_reason:
                            # A valid tracking observation can ask to wait without
                            # completing or permanently latching its episode. Never
                            # project a zero body path back toward an old capture pose.
                            self._invalidate(hold_reason,require_post_stop_rgb=True)
                            self.task["tracking_hold_reason"] = hold_reason
                            self.task["consecutive_inference_errors"] = 0
                            self.preview["epoch"] = self.epoch
                            self.metrics["inference_held"] += 1
                            self._event("tracking_hold",request_id=request_id,reason=hold_reason,
                                        model_visible=response.get("model_visible"),
                                        model_stop=response.get("model_stop"),source_age_s=age)
                            continue
                        self.task["tracking_hold_reason"] = None
                    if metadata["backend"] == "lightnav" and response.get("model_stop") and not tracking:
                        # A rejected completion cannot change execution state.
                        # In particular, a stale STOP must not freeze future
                        # planning after its source image has already expired.
                        if not reasons:
                            self.task["model_stop_latched"] = True
                            self.task["consecutive_inference_errors"] = 0
                            self.metrics["inference_accepted"] += 1
                            self._finish("SUCCEEDED", "go2_imagegoal_model_stop" if adapted_imagegoal else "official_lightnav_model_stop")
                            continue
                        else:
                            self._event("model_stop_rejected", request_id=request_id,
                                        source_frame_id=metadata["identity"]["frame_id"],
                                        source_age_s=age, reasons=list(dict.fromkeys(reasons)))
                    if metadata["backend"]=="nomad" and not reasons and self.config.nomad_stop_distance_steps is not None:
                        if self.task["nomad_stop_epoch"]!=self.epoch:
                            self.task["nomad_stop_streak"]=0
                            self.task["nomad_stop_source_frame_id"]=None
                            self.task["nomad_stop_epoch"]=self.epoch
                        frame_id=metadata["identity"]["frame_id"]
                        if frame_id!=self.task["nomad_stop_source_frame_id"]:
                            self.task["nomad_stop_source_frame_id"]=frame_id
                            self.task["nomad_stop_streak"]=(self.task["nomad_stop_streak"]+1 if
                                response["distance_steps"]<=self.config.nomad_stop_distance_steps else 0)
                            self._event("nomad_stop_score",source_frame_id=frame_id,distance_steps=response["distance_steps"],
                                        threshold_steps=self.config.nomad_stop_distance_steps,streak=self.task["nomad_stop_streak"],goal_version=self.task["goal_version"])
                        if self.task["nomad_stop_streak"]>=self.config.nomad_stop_consecutive:
                            self.task["model_stop_latched"]=True
                            self._invalidate("nomad_distance_stop_not_arrival",require_post_stop_rgb=True)
                            reasons.append("nomad_distance_stop_not_arrival")
                    if not has_motion:
                        reasons.append("zero_path_not_arrival")
                    if not reasons:
                        self.plan = {"request_id":request_id,"epoch":self.epoch,"sampled":sampled,
                                     "native_path": native_path,
                                     "adaptation":adaptation,"control_profile":self.task["control_profile"],
                                     "capture_pose":np.asarray(metadata["capture_pose"]),"source_stamp_ns":metadata["source_stamp_ns"],
                                     "goal_version":self.task["goal_version"],"round_trip_ms":rtt,
                                     "trajectory_odom_m_rad":[],"reference_odom_m_rad":[],"prediction_odom_m_rad":[]}
                        self.preview["accepted_for_tracking"] = True
                        self.task["consecutive_inference_errors"] = 0
                        self.metrics["inference_accepted"] += 1
                        self._event("inference_accepted", request_id=request_id, round_trip_ms=rtt, source_age_s=age,
                                    capture_odom_gap_s=metadata["capture_odom_gap_s"], goal_version=self.task["goal_version"])
                except Exception as exc:
                    reasons.append(f"invalid_inference_response:{type(exc).__name__}:{exc}")
            if reasons:
                if (self.task and metadata["backend"]=="nomad" and metadata["epoch"]==self.epoch and
                    not self.task["model_stop_latched"]):
                    self.task["nomad_stop_streak"]=0
                    self.task["nomad_stop_source_frame_id"]=None
                self.metrics["inference_rejected"] += 1
                self._event("inference_rejected", request_id=request_id, reasons=list(dict.fromkeys(reasons)), detail=error)

    def tick(self) -> None:
        with self.lock:
            self._accept_completed()
            now = time.time_ns()
            self.proposal_evaluated_ns = now
            self.control_sequence += 1
            if not self.task or self.state in TERMINAL:
                self.gates = ["no_active_task"]
                self.proposal = self._zero(self.state.lower())
                return
            if time.monotonic_ns() >= self.task["deadline_monotonic_ns"]:
                self._finish("TIMED_OUT", "task_budget_exhausted")
                self.gates = ["task_budget_exhausted"]
                return
            gates = self._sensor_gates(now)
            gates.extend(self._task_execution_reasons(now))
            if self.task["paused"]:
                gates.append("operator_paused")
            if self.task["model_stop_latched"]:
                gates.append("model_stop_not_arrival")
            if self.task.get("task_mode") == "tracking" and self.task.get("tracking_hold_reason"):
                gates.append(self.task["tracking_hold_reason"])
            if self.plan and (self.plan["epoch"] != self.epoch or self.plan["goal_version"] != self.task["goal_version"]):
                gates.append("plan_generation_mismatch")
            if self.plan and not -.1 <= (now-self.plan["source_stamp_ns"])/1e9 <= self.config.max_plan_age_s:
                gates.append("plan_source_stale_or_future")
            if gates and self.plan:
                self._invalidate(";".join(gates), require_post_stop_rgb=True)
            if not self.plan:
                gates.append("fresh_plan_required")
            self.gates = list(dict.fromkeys(gates))
            if self.gates:
                self.lightnav_controller.reset()
            self.proposal = self._zero(";".join(self.gates) if self.gates else "planning")
            if self.plan and not self.gates:
                try:
                    current = self.odom[-1][0]
                    pose = np.array([current.x,current.y,current.yaw])
                    adapted_lightnav = self.task["backend"] == "lightnav" and self.task["task_mode"] == "imagegoal"
                    official_lightnav = self.task["backend"] == "lightnav" and not adapted_lightnav
                    if official_lightnav or (adapted_lightnav and self.plan["sampled"] is None):
                        packet = self.lightnav_controller.reference(
                            self.plan["native_path"], self.plan["capture_pose"], pose,
                            self.plan["source_stamp_ns"], self.odom[-1][1]["frame_name"],
                            weights=(1.,1.,1.) if adapted_lightnav else (10.,10.,1.))
                    else:
                        packet = build_reference(self.plan["sampled"], capture_pose_odom_m_rad=self.plan["capture_pose"],
                                                 current_pose_odom_m_rad=pose,horizon=self.config.horizon,weights=(1.,1.,1.),
                                                 core=self.core,odom_frame_id=self.odom[-1][1]["frame_name"])
                    if packet.stop_required:
                        self._invalidate("zero_path_not_arrival")
                        self.gates = ["zero_path_not_arrival"]
                    else:
                        if official_lightnav:
                            velocity, prediction = self.lightnav_controller.solve(
                                pose, packet.reference_odom_m_rad, self.task["task_mode"])
                            # Final device envelope only. Preserve arbitrarily
                            # small valid MPC commands, with no boost/deadband.
                            velocity = (float(np.clip(velocity[0], 0., self.config.max_v_mps)),
                                        float(np.clip(velocity[1], -self.config.max_w_rad_s, self.config.max_w_rad_s)))
                        else:
                            velocity,prediction = self.controller.solve(pose,packet.reference_odom_m_rad,
                                                                       tuple(self.odom[-1][3]),self.config.max_v_mps)
                        cmd = self.control_module.VelocityCommand(linear_x=velocity[0],angular_z=velocity[1])
                        guard = self.latency_guard.apply(cmd,plan_input_age_s=(time.time_ns()-self.plan["source_stamp_ns"])/1e9)
                        control_command = guard.command
                        if adapted_lightnav:
                            target = packet.reference_odom_m_rad[-1]
                            dx,dy = target[0]-pose[0],target[1]-pose[1]
                            angle = math.atan2(dy,dx)-pose[2] if math.hypot(dx,dy)>.01 else target[2]-pose[2]
                            angle = math.atan2(math.sin(angle),math.cos(angle))
                            yaw_error = math.atan2(math.sin(target[2]-pose[2]),math.cos(target[2]-pose[2]))
                            last = packet.trajectory_odom_m_rad[-1]
                            remaining_distance = float(np.linalg.norm(last[:2]-pose[:2]))
                            remaining_yaw = math.atan2(math.sin(last[2]-pose[2]),math.cos(last[2]-pose[2]))
                            vx,wz,compensation = postprocess_command(
                                control_command.linear_x,control_command.angular_z,angle,yaw_error,
                                self.plan["native_path"],remaining_distance,remaining_yaw,self.config)
                            control_command = self.control_module.VelocityCommand(linear_x=vx,angular_z=wz)
                            self.plan["command_adaptation"] = compensation
                        native = self.config.execution_profile == "supervised-native"
                        if native and (official_lightnav or adapted_lightnav):
                            guarded = self.control_module.SafetyResult(
                                command=control_command, clearance_m=None,
                                reason="native_avoidance_executor_required")
                        elif native:
                            target = packet.reference_odom_m_rad[-1]
                            dx,dy = target[0]-pose[0],target[1]-pose[1]
                            angle = math.atan2(dy,dx)-pose[2] if math.hypot(dx,dy)>.01 else target[2]-pose[2]
                            angle = math.atan2(math.sin(angle),math.cos(angle))
                            vx,wz = native_speed_gate(guard.command.linear_x,guard.command.angular_z,angle,self.config)
                            guarded = self.control_module.SafetyResult(
                                command=self.control_module.VelocityCommand(linear_x=vx,angular_z=wz),
                                clearance_m=None,reason="native_avoidance_executor_required")
                        else:
                            guarded = self.control_module.apply_depth_safety(control_command,self.depth["depth_m"],self.depth_config)
                        # Recheck mutable task identity and wall ages after a
                        # solve: solving does not freshen the input plan.
                        post_gates = self._sensor_gates(time.time_ns())
                        post_gates.extend(self._task_execution_reasons(time.time_ns()))
                        failed_guards = ([guard.reason] if guard.reason != "pass" else [])
                        if guarded.reason not in {"clear","native_avoidance_executor_required"}:
                            failed_guards.append(guarded.reason)
                        failed_guards = list(dict.fromkeys(failed_guards + post_gates))
                        if failed_guards:
                            self._invalidate("post_solver_gate:"+";".join(failed_guards),require_post_stop_rgb=True)
                            self.gates = failed_guards
                        else:
                            self.proposal = {"vx_m_s":float(guarded.command.linear_x),"vy_m_s":0.0,"wz_rad_s":float(guarded.command.angular_z),
                                             "reason":"native_supervised_tracking" if native else "shadow_tracking","not_for_motion":True,"published_to_robot":False,
                                             "task_id":self.task["task_id"],"goal_version":self.task["goal_version"],"epoch":self.epoch,
                                             "source_stamp_ns":self.plan["source_stamp_ns"],"expires_timestamp_ns":self.plan["source_stamp_ns"]+int(self.config.max_plan_age_s*1e9)}
                            self.plan.update(trajectory_odom_m_rad=packet.trajectory_odom_m_rad.tolist(),reference_odom_m_rad=packet.reference_odom_m_rad.tolist(),
                                             prediction_odom_m_rad=prediction.tolist())
                            self.proposal_evaluated_ns = time.time_ns()
                            self.metrics["mpc_solves"] += 1
                            if abs(self.proposal["vx_m_s"])+abs(self.proposal["wz_rad_s"])>1e-8:
                                self.metrics["commands_nonzero"] += 1
                except Exception as exc:
                    self.metrics["mpc_errors"] += 1
                    self._invalidate("mpc_error",require_post_stop_rgb=True)
                    self.gates = ["mpc_error"]
                    self._event("mpc_error",detail=f"{type(exc).__name__}: {exc}")
            can_plan = not self.task["paused"] and not self.task["model_stop_latched"]
            # LightNav window validation can finish an episode whose unsent
            # history no longer fits its native contract. Only validate when
            # a new request can actually be sent: a deliberate pause, model
            # STOP, or outstanding request must not trigger that side effect.
            if can_plan and self.inflight is None:
                window = self._window(now)
                if self.state in TERMINAL:
                    return
                if window and self.last_frame_requested != (self.epoch,window[-1].frame_id):
                    self._dispatch(window)
            self.state = "PAUSED" if self.task["paused"] else ("MODEL_STOP" if self.task["model_stop_latched"] else
                         ("TRACKING" if self.plan and not self.gates else
                          ("TRACK_HOLD" if self.task.get("tracking_hold_reason") else
                           ("PLANNING" if self.inflight else "WAITING_INPUT"))))

    def _loop(self) -> None:
        period = 1/self.config.tick_hz
        while not self.stop_event.is_set():
            start = time.monotonic()
            try:
                self.tick()
                with self.lock:
                    status = self.status()
                    snapshot=json_bytes(status)
                # Checkpoint transfers must not hold the input/task lock
                # through artifact IO and age otherwise fresh RGB-D.
                with (self.output / "status.jsonl").open("ab") as stream:
                    stream.write(snapshot+b"\n")
                temporary=self.output/"status.json.tmp"
                temporary.write_bytes(snapshot)
                temporary.replace(self.output/"status.json")
            except Exception as exc:
                with self.lock:
                    self.proposal = self._zero("runtime_fault")
                    self._event("runtime_fault",detail=f"{type(exc).__name__}: {exc}")
                    if self.task and self.state not in TERMINAL:
                        self._finish("FAILED","runtime_fault")
            self.stop_event.wait(max(0,period-(time.monotonic()-start)))

    def status(self) -> dict[str, Any]:
        with self.lock:
            now = time.time_ns()
            odom = self.odom[-1] if self.odom else None
            task = dict(self.task) if self.task else None
            if task:
                task["remaining_s"] = max(0,(task["deadline_monotonic_ns"]-time.monotonic_ns())/1e9)
                task.pop("deadline_monotonic_ns",None)
            plan = None if self.plan is None else {key:value for key,value in self.plan.items() if key not in {"sampled","native_path","capture_pose"}}
            if plan:
                plan["capture_pose_odom_m_rad"] = self.plan["capture_pose"].tolist()
                plan["source_age_s"] = (now-plan["source_stamp_ns"])/1e9
                plan["odom_frame_id"] = odom[1]["frame_name"] if odom else None
            sensors = self._sensors_snapshot(now)
            deadlines = [self.proposal_evaluated_ns+int(self.config.execution_lease_s*1e9)]
            if plan and odom and self.depth:
                deadlines.extend([plan["source_stamp_ns"]+int(self.config.max_plan_age_s*1e9),
                                  odom[1]["capture_timestamp_ns"]+int(self.config.max_odom_age_s*1e9),
                                  self.depth["metadata"]["capture_timestamp_ns"]+int(self.config.max_depth_age_s*1e9)])
            current_gates = list(self.gates)
            if task and self.state not in TERMINAL:
                current_gates.extend(self._sensor_gates(now))
                current_gates.extend(self._task_execution_reasons(now))
            if plan and not 0<=(now-plan["source_stamp_ns"])/1e9<=self.config.max_plan_age_s:
                current_gates.append("plan_source_stale_or_future")
            if now-self.proposal_evaluated_ns>int(self.config.execution_lease_s*1e9):
                current_gates.append("controller_evaluation_stale")
            current_gates = list(dict.fromkeys(current_gates))
            tracked = bool(plan and not current_gates and self.state=="TRACKING")
            physical_reasons = self._operator_source_reasons(now)
            candidate_reasons = self._operator_source_reasons(now,allow_commissioning=True)
            requested = bool(task and task.get("execution_mode")=="operator_run")
            physical = bool(requested and tracked and not physical_reasons and not self.config.commissioning_real_inputs)
            candidate = bool(requested and tracked and not candidate_reasons and self.config.commissioning_real_inputs)
            command = dict(self.proposal)
            command["not_for_motion"] = not physical
            if requested and not (physical or candidate):
                command = self._zero("operator_run_not_eligible")
            if physical or candidate:
                uncertainty = float(self.calibration.timing["max_alignment_error_s"])
                deadlines.extend([odom[1]["capture_timestamp_ns"]+int((self.config.max_odom_age_s-uncertainty)*1e9),
                                  self.depth["metadata"]["capture_timestamp_ns"]+int((self.config.max_depth_age_s-uncertainty)*1e9)])
            authority = "operator_run" if physical else ("commissioning_only" if self.config.commissioning_real_inputs else "shadow_only_no_robot_transport")
            native_configured = self.config.execution_profile == "supervised-native"
            native_reasons = self._native_source_reasons(now) if native_configured else ["native_supervision_not_configured"]
            native_requested = bool(task and task.get("execution_mode") == "supervised_native")
            native_authorized = bool(native_configured and native_requested and tracked and not native_reasons)
            native_deadlines = [self.proposal_evaluated_ns + int(self.config.execution_lease_s*1e9)]
            if plan:
                native_deadlines.append(plan["source_stamp_ns"]+int(self.config.max_plan_age_s*1e9))
            for name,max_age in (("rgb",self.config.max_rgb_age_s),("odom",self.config.max_odom_age_s)):
                if sensors[name]:
                    native_deadlines.append(sensors[name]["capture_timestamp_ns"]+int(max_age*1e9))
            native_command = {key:float(self.proposal.get(key,0.)) if native_authorized else 0.
                              for key in ("vx_m_s","vy_m_s","wz_rad_s")}
            native_execution = {"schema_version":"imagegoal.native_execution.v1", "authorized":native_authorized,
                                "session_id":self.session_id,"task_id":task["task_id"] if task else None,
                                "backend":task["backend"] if task else None,
                                "control_profile":task.get("control_profile") if task else None,
                                "task_mode":task.get("task_mode","imagegoal") if task else None,
                                "target_visible":task.get("tracking_visible") if task and task.get("task_mode")=="tracking" else None,
                                "goal_id":task["goal_id"] if task else None,"goal_version":task["goal_version"] if task else None,
                                "epoch":self.epoch,"sequence":self.control_sequence,
                                "evaluated_timestamp_ns":self.proposal_evaluated_ns,"expires_timestamp_ns":min(native_deadlines),
                                "source_stamp_ns":plan["source_stamp_ns"] if plan else None,
                                "request_id":plan["request_id"] if plan else None,"command":native_command,
                                "sensors":{"rgb":sensors["rgb"],"odom":sensors["odom"]},
                                "supervised":native_configured and self.config.operator_supervised,
                                "safety_backend":"unitree_obstacles_avoid_only",
                                "calibration_claim":"factory_front_assumed_receive_time_unverified",
                                "blocking_reasons":list(dict.fromkeys(native_reasons+current_gates)),
                                "camera_exposure_time_verified":False,"camera_extrinsic_matrix_verified":False}
            command.update(expires_timestamp_ns=min(deadlines),sequence=self.control_sequence,
                           evaluated_timestamp_ns=self.proposal_evaluated_ns)
            envelope = {"schema_version":"imagegoal.execution_envelope.v1","session_id":self.session_id,
                        "task_id":task["task_id"] if task else None,"goal_id":task["goal_id"] if task else None,
                        "goal_version":task["goal_version"] if task else None,"epoch":self.epoch,
                        "evaluated_timestamp_ns":self.proposal_evaluated_ns,"expires_timestamp_ns":min(deadlines),"sequence":self.control_sequence,
                        "eligible_for_shadow_tracking":tracked,"eligible_for_robot_tracking":physical,
                        "eligible_for_commissioning_tracking":candidate,"candidate_transport_authority":"operator_run" if candidate else None,
                        "gate_snapshot":current_gates,"source_stamp_ns":plan["source_stamp_ns"] if plan else None,
                        "request_id":plan["request_id"] if plan else None,"sensors":sensors,
                        "command":command,"transport_authority":authority,"authority_schema":"imagegoal.robot_execution_authorized.v1",
                        "execution_mode":task.get("execution_mode","shadow") if task else "shadow","robot_id":self.config.robot_id,
                        "calibration":self.calibration.summary(),"commissioning":self.config.commissioning_real_inputs,
                        "synthetic":self.config.profile=="simulated-replay","not_for_motion":not physical}
            return {"schema_version":"imagegoal.runtime.v1","session_id":self.session_id,"timestamp_ns":now,
                    "profile":self.config.profile,"not_for_motion":not (physical or native_authorized),"hardware_transport":"absent",
                    "commissioning":self.config.commissioning_real_inputs,
                    "execution_capability":{"operator_run_configured":self.config.execution_profile=="operator-run",
                                            "calibration_verified":self.calibration.verified and not self.calibration.commissioning,
                                            "commissioning":self.config.commissioning_real_inputs,"blocking_reasons":native_reasons if native_configured else physical_reasons,
                                            "supervised_native_configured":native_configured,"operator_supervised":self.config.operator_supervised,
                                            "supervised_native_backends":["nomad","lightnav"],
                                            "lightnav_imagegoal_adapter":True,
                                            "lightnav_profiles":["official_release","go2_imagegoal_adaptation"],
                                            "task_modes":["imagegoal","objnav","vln","tracking"],
                                            "supervised_native_task_modes":["imagegoal","objnav","vln","tracking"],
                                            "calibration":self.calibration.summary()},
                    "stop_policy":{"mode":("tracking_observation_hold" if task and task.get("task_mode")=="tracking" else
                                            "go2_imagegoal_model_stop_completes_task" if task and task["backend"]=="lightnav" and task.get("task_mode")=="imagegoal" else
                                            "official_lightnav_model_stop_completes_task" if task and task["backend"]=="lightnav" else
                                            ("nomad_temporal_threshold_hold" if self.config.nomad_stop_distance_steps is not None else "operator_only_unconfigured")),
                                   "threshold_steps":self.config.nomad_stop_distance_steps,"consecutive_required":self.config.nomad_stop_consecutive,
                                   "streak":task.get("nomad_stop_streak",0) if task else 0,
                                   "last_source_frame_id":task.get("nomad_stop_source_frame_id") if task else None,
                                   "latched":task.get("model_stop_latched",False) if task else False,
                                   "automatic_success":bool(task and task["backend"]=="lightnav" and task.get("task_mode")!="tracking"),
                                   "arrival_independently_validated":False,"units":"RVQ STOP for LightNav; learned temporal steps for NoMaD"},
                    "lightnav_control":self._lightnav_control_contract(),
                    "lightnav_transport":{"protocol":"lightnav.http.lifecycle.v2",
                                          "automatic_image_retry":False,
                                          "failure_policy":"finish_episode_then_retire; explicit_new_start_required",
                                          "max_pending_frames":self.config.lightnav_max_pending_frames,
                                          "max_request_bytes":self.config.lightnav_max_request_bytes,
                                          "max_pending_age_s":self.config.lightnav_max_pending_age_s},
                    "synthetic_feedback":self.config.profile=="simulated-replay","state":self.state,"epoch":self.epoch,"task":task,
                    "tracking":self._tracking_status(now),
                    "goal_selection":deepcopy(self.goal_selection),
                    "gates":list(self.gates),"proposal":dict(self.proposal),"plan":plan,"preview":self.preview,
                    "sensors":sensors,"execution_envelope":envelope,"native_execution":native_execution,
                    "native_executor":self.native_executor.status() if self.native_executor is not None else {"running":False,"state":"not_started","task_id":None,"error":None},
                    "inflight":None if self.inflight is None else {key:self.inflight[key] for key in ("request_id","epoch","identity","source_stamp_ns")},
                    "metrics":dict(self.metrics),"pending_session_cleanup":len(self.pending_session_cleanup),
                    "latest_event_sequence":self.event_sequence,"run_directory":str(self.output)}

    def _tracking_status(self, now: int) -> dict[str,Any]:
        task = self.task or {}
        enabled = task.get("task_mode") == "tracking"
        stamp = task.get("tracking_source_stamp_ns") if enabled else None
        fresh = bool(enabled and self.state not in TERMINAL and type(stamp) is int
                     and 0 <= now-stamp <= self.config.max_plan_age_s*1e9)
        visible = task.get("tracking_visible") if fresh else None
        hold = task.get("tracking_hold_reason") if enabled else None
        target_state = ("unknown" if not fresh else "not_visible" if visible is False else
                        "holding" if hold else "visible" if visible is True else "unknown")
        return {"enabled":enabled,"visible":visible,"fresh":fresh,"target_state":target_state,
                "hold_reason":hold,"instruction":task.get("instruction") if enabled else None,
                "source_frame_id":task.get("tracking_source_frame_id") if enabled else None,
                "source_stamp_ns":stamp,"source_age_s":(now-stamp)/1e9 if type(stamp) is int else None,
                "identity_semantics":"language_and_visual_history; no_explicit_reidentification"}

    def set_fault(self, name: str, value: Any) -> dict[str, Any]:
        if self.config.profile != "simulated-replay":
            raise ValueError("fault injection is available only in explicitly simulated replay")
        allowed = {"rgb_paused","odom_paused","depth_paused","obstacle","response_delay_s"}
        if name not in allowed:
            raise ValueError(f"fault must be one of {sorted(allowed)}")
        if name == "response_delay_s":
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0 <= value <= 60:
                raise ValueError("response_delay_s must be in [0,60]")
        elif not isinstance(value,bool):
            raise ValueError("fault value must be boolean")
        with self.lock:
            self.faults[name] = value
            self._event("simulation_fault_changed",name=name,value=value)
            return {"ok":True,"faults":dict(self.faults),"synthetic":True}
