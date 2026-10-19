"""ROS-independent bridge from NoMaD spatial paths to the existing MPC core.

No optimizer is reimplemented here. ``load_vendor_mpc`` verifies the pinned
LightNav source before loading its original reference builder; ``with_solver``
also loads the unchanged CasADi/IPOPT controller. The reference-only mode uses
the original function AST so NumPy-only machines can inspect paths without
installing CasADi or importing robot libraries.

Capture odometry and current odometry must share the same continuous odom
frame. The caller must match capture odometry to the image timestamp, reject
stale inputs/goal generations, and stop after task cancellation. Neither
rebuilding a reference nor solving MPC refreshes ``capture_stamp_ns``.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
import hashlib
import importlib.util
import math
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .trajectory_adapter import SampledPath


LIGHTNAV_COMMIT = "ddfd37059c49e8090217e11c4ffa6f6b3ff9d554"
LIGHTNAV_MPC_SHA256 = "fe97c24c6eb5ec443892ea8e6719493390a448ed036cebac1fcbd213c795c0a8"
LIGHTNAV_MPC_SOURCE = (
    "https://github.com/AlanZhu2006/lightnav_realworld/blob/"
    f"{LIGHTNAV_COMMIT}/robot_deploy/src/vln_mpc/vln_mpc/mpc.py"
)


@dataclass(frozen=True)
class VendorMPCCore:
    reference_builder: Callable[..., np.ndarray]
    controller_type: type | None
    source_path: str
    source_sha256: str


@dataclass(frozen=True)
class ReferencePacket:
    """MPC and visualization data, all positions in the supplied odom frame.

    On a zero path ``stop_required=True`` and the reference is the *current*
    pose, never a command to return to the old capture pose. Callers must stop
    directly rather than using the optimizer to infer stop/goal completion.
    """

    trajectory_odom_m_rad: np.ndarray
    reference_odom_m_rad: np.ndarray
    capture_pose_odom_m_rad: np.ndarray
    current_pose_odom_m_rad: np.ndarray
    capture_stamp_ns: int
    body_frame_id: str
    odom_frame_id: str
    stop_required: bool
    vendor_source_sha256: str


def load_vendor_mpc(
    source_path: str | Path,
    *,
    expected_sha256: str = LIGHTNAV_MPC_SHA256,
    with_solver: bool = False,
) -> VendorMPCCore:
    """Load the hash-locked upstream MPC file, without editing its source.

    ``with_solver=False`` executes only upstream ``_wrap_angle`` and
    ``build_pose_aligned_reference`` function bodies with math/NumPy globals.
    ``with_solver=True`` imports the full upstream file and requires CasADi.
    Passing an empty/incorrect hash fails; a changed upstream source needs an
    explicit reviewed pin update rather than silently loading new code.
    """
    path = Path(source_path).resolve(strict=True)
    source = path.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    if not isinstance(expected_sha256, str) or digest != expected_sha256:
        raise ValueError(f"vendor MPC SHA256 mismatch: {path} has {digest}")
    if with_solver:
        spec = importlib.util.spec_from_file_location(f"_dejaview_lightnav_mpc_{digest}", path)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load vendor MPC: {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        builder = module.build_pose_aligned_reference
        controller_type = module.MPCController
    else:
        tree = ast.parse(source, filename=str(path))
        names = {"_wrap_angle", "build_pose_aligned_reference"}
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
        if {node.name for node in functions} != names:
            raise ValueError("pinned MPC source lacks its reference builder")
        namespace: dict[str, Any] = {"math": math, "np": np}
        # Bodies are taken verbatim from the verified source; no fallback
        # reference implementation can drift from the robot's MPC version.
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
        builder = namespace["build_pose_aligned_reference"]
        controller_type = None
    return VendorMPCCore(builder, controller_type, str(path), digest)


def _pose(values: np.ndarray, name: str) -> np.ndarray:
    pose = np.asarray(values, dtype=np.float64)
    if pose.shape != (3,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"{name} must be a finite [x_m, y_m, yaw_rad] vector")
    return pose


def _readonly(values: np.ndarray) -> np.ndarray:
    result = np.array(values, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


def body_path_to_odom(points_body_m_rad: np.ndarray, capture_pose_odom_m_rad: np.ndarray) -> np.ndarray:
    """SE(2) transform for +x forward/+y left/yaw CCW capture-body paths."""
    points = np.asarray(points_body_m_rad, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or not np.all(np.isfinite(points)):
        raise ValueError("points_body_m_rad must be a non-empty finite Nx3 array")
    capture = _pose(capture_pose_odom_m_rad, "capture_pose_odom_m_rad")
    cosine, sine = math.cos(capture[2]), math.sin(capture[2])
    result = np.empty_like(points)
    result[:, 0] = capture[0] + cosine * points[:, 0] - sine * points[:, 1]
    result[:, 1] = capture[1] + sine * points[:, 0] + cosine * points[:, 1]
    result[:, 2] = capture[2] + points[:, 2]
    result[:, 2] = np.arctan2(np.sin(result[:, 2]), np.cos(result[:, 2]))
    return result


def build_reference(
    path: SampledPath,
    *,
    capture_pose_odom_m_rad: np.ndarray,
    current_pose_odom_m_rad: np.ndarray,
    horizon: int,
    weights: tuple[float, float, float],
    core: VendorMPCCore,
    odom_frame_id: str = "go2_odom",
) -> ReferencePacket:
    """Anchor once at image capture and reuse upstream nearest-pose selection.

    Source samples already have metric spatial spacing. This function does not
    scale/resample them or assign timestamps. Original endpoint clamping and
    nearest weighted-pose selection come directly from LightNav's pinned core.
    The same ``mpc_dt_s`` used for sampling must configure ``MPCController``.
    """
    if not isinstance(path, SampledPath):
        raise TypeError("path must be a SampledPath from resample_metric_path")
    if isinstance(horizon, bool) or not isinstance(horizon, int) or horizon <= 0:
        raise ValueError("horizon must be a positive integer")
    if not isinstance(odom_frame_id, str) or not odom_frame_id.strip():
        raise ValueError("odom_frame_id must be non-empty")
    cost_weights = np.asarray(weights, dtype=np.float64)
    if cost_weights.shape != (3,) or not np.all(np.isfinite(cost_weights)) or np.any(cost_weights <= 0.0):
        raise ValueError("weights must contain three finite positive values")
    capture = _pose(capture_pose_odom_m_rad, "capture_pose_odom_m_rad")
    current = _pose(current_pose_odom_m_rad, "current_pose_odom_m_rad")
    trajectory = body_path_to_odom(path.points_body_m_rad, capture)
    stop_required = not path.has_motion
    if stop_required:
        reference = np.repeat(current[None, :], horizon, axis=0)
    else:
        reference = np.asarray(
            core.reference_builder(trajectory, current, horizon=horizon, weights=tuple(cost_weights)),
            dtype=np.float64,
        )
        if reference.shape != (horizon, 3) or not np.all(np.isfinite(reference)):
            raise ValueError("vendor reference builder returned an invalid reference")
    return ReferencePacket(
        trajectory_odom_m_rad=_readonly(trajectory),
        reference_odom_m_rad=_readonly(reference),
        capture_pose_odom_m_rad=_readonly(capture),
        current_pose_odom_m_rad=_readonly(current),
        capture_stamp_ns=path.capture_stamp_ns,
        body_frame_id=path.frame_id,
        odom_frame_id=odom_frame_id,
        stop_required=stop_required,
        vendor_source_sha256=core.source_sha256,
    )
