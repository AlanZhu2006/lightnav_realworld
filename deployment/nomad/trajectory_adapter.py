"""Convert NoMaD's cumulative planar predictions into a metric spatial path.

The input is the *unscaled* ``[N, 2]`` first diffusion sample after NoMaD's
``get_action`` cumulative-sum conversion. It is neither a list of increments
nor the old wrapper's partly scaled ``first_sample`` NumPy view. Scale exactly
once here. The compatible initial scale is 0.05 m per model action unit; it is
an explicit configuration value, independent of robot speed limits.

All output positions use the body frame at ``capture_stamp_ns``: +x forward,
+y left, yaw positive counterclockwise. Yaw is derived from the polyline's
direction and is not predicted by NoMaD. Waypoint indices have no verified
physical duration. Resampling below assigns a *reference* spacing of
``v_ref_mps * mpc_dt_s``; it does not recover model timestamps.

These functions know nothing about ROS, odometry freshness, goal arrival, or
robot commands. A zero path means stop/replan, never goal arrival.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


BODY_FRAME_CONVENTION = "capture body: +x forward, +y left, yaw CCW"
YAW_SOURCE = "derived polyline tangent; not a model output"


def _positive_finite(value: float, name: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a finite positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return result


def _readonly(values: np.ndarray) -> np.ndarray:
    result = np.array(values, dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


def _validate_capture(capture_stamp_ns: int, frame_id: str) -> None:
    if (
        isinstance(capture_stamp_ns, (bool, np.bool_))
        or not isinstance(capture_stamp_ns, (int, np.integer))
        or capture_stamp_ns <= 0
    ):
        raise ValueError("capture_stamp_ns must be a positive integer")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a non-empty body-frame name")


@dataclass(frozen=True)
class MetricPath:
    """Un-timed metric polyline, including the capture-frame origin.

    Instances are returned by :func:`raw_nomad_to_metric_path`; arrays are
    read-only. ``action_scale_m`` records provenance, not a pending operation.
    ``arc_length_m`` is spatial distance along the path, never elapsed time.
    """

    xy_m: np.ndarray
    arc_length_m: np.ndarray
    capture_stamp_ns: int
    frame_id: str
    action_scale_m: float
    raw_waypoint_count: int

    @property
    def length_m(self) -> float:
        return float(self.arc_length_m[-1])

    @property
    def has_motion(self) -> bool:
        return len(self.xy_m) > 1 and self.length_m > 0.0


@dataclass(frozen=True)
class SampledPath:
    """Spatial samples for MPC; yaw is a derived tangent in radians.

    ``points_body_m_rad`` includes the origin and the exact final polyline
    endpoint, without extension beyond that endpoint. Short paths may have
    fewer samples than an MPC horizon; horizon padding belongs to the tracker.
    ``has_motion=False`` requires stopping/replanning, not position tracking.
    """

    points_body_m_rad: np.ndarray
    arc_length_m: np.ndarray
    capture_stamp_ns: int
    frame_id: str
    spacing_m: float
    v_ref_mps: float
    mpc_dt_s: float
    action_scale_m: float
    raw_waypoint_count: int

    @property
    def length_m(self) -> float:
        return float(self.arc_length_m[-1])

    @property
    def has_motion(self) -> bool:
        return len(self.points_body_m_rad) > 1 and self.length_m > 0.0


def raw_nomad_to_metric_path(
    raw_xy: np.ndarray,
    *,
    action_scale_m: float,
    capture_stamp_ns: int,
    frame_id: str = "base_link",
) -> MetricPath:
    """Scale unscaled cumulative NoMaD positions once and prepend the origin.

    Standard NoMaD emits 8 points; any nonempty finite Nx2 array is accepted
    for diagnostics. Only consecutive exactly coincident points are removed.
    Backtracking, lateral motion and final endpoints are preserved rather than
    being silently clipped into a forward path. Invalid inputs fail explicitly.
    Passing an existing :class:`MetricPath` is rejected to catch double scaling.
    """
    if isinstance(raw_xy, (MetricPath, SampledPath)):
        raise TypeError("raw_xy must contain unscaled model positions, not a metric path")
    _validate_capture(capture_stamp_ns, frame_id)
    scale = _positive_finite(action_scale_m, "action_scale_m")
    raw = np.asarray(raw_xy)
    if raw.dtype.kind not in "iuf" or raw.ndim != 2 or raw.shape[1] != 2 or len(raw) == 0:
        raise ValueError("raw_xy must be a non-empty numeric Nx2 array")
    raw = np.asarray(raw, dtype=np.float64)
    if not np.all(np.isfinite(raw)):
        raise ValueError("raw_xy must contain only finite values")
    with np.errstate(over="ignore", invalid="ignore"):
        scaled = raw * scale
    if not np.all(np.isfinite(scaled)):
        raise ValueError("scaled path contains non-finite values")
    points = np.vstack((np.zeros((1, 2), dtype=np.float64), scaled))
    deltas = np.diff(points, axis=0)
    distances = np.hypot(deltas[:, 0], deltas[:, 1])
    if not np.all(np.isfinite(distances)):
        raise ValueError("path segment length overflow")
    points = points[np.concatenate(([True], distances > 0.0))]
    deltas = np.diff(points, axis=0)
    arc = np.concatenate(([0.0], np.cumsum(np.hypot(deltas[:, 0], deltas[:, 1]))))
    if not np.all(np.isfinite(arc)):
        raise ValueError("path arc length overflow")
    return MetricPath(
        xy_m=_readonly(points),
        arc_length_m=_readonly(arc),
        capture_stamp_ns=int(capture_stamp_ns),
        frame_id=frame_id,
        action_scale_m=scale,
        raw_waypoint_count=len(raw),
    )


def resample_metric_path(
    path: MetricPath,
    *,
    v_ref_mps: float,
    mpc_dt_s: float,
    max_samples: int = 100_000,
) -> SampledPath:
    """Resample a metric spatial polyline; never multiply action scale again.

    Samples are ``0, spacing, 2*spacing, ...`` plus the exact endpoint. Yaw is
    the outgoing segment tangent (the last sample uses the final tangent),
    unwrapped along the path. This is a deterministic derived reference, not
    a model-predicted heading or duration. A one-point zero path gets yaw zero
    solely for display and returns ``has_motion=False``.
    """
    if not isinstance(path, MetricPath):
        raise TypeError("path must be a MetricPath; convert raw NoMaD positions first")
    speed = _positive_finite(v_ref_mps, "v_ref_mps")
    dt = _positive_finite(mpc_dt_s, "mpc_dt_s")
    spacing = _positive_finite(speed * dt, "reference spacing")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
        raise ValueError("max_samples must be an integer of at least 2")
    length = path.length_m
    # Endpoint addition and origin are both accounted for before allocating.
    sample_ratio = length / spacing
    if not math.isfinite(sample_ratio) or sample_ratio > max_samples - 1:
        raise ValueError("reference spacing would exceed max_samples")
    if not path.has_motion:
        samples = np.zeros((1, 3), dtype=np.float64)
        sample_arc = np.zeros(1, dtype=np.float64)
    else:
        count = int(math.floor(sample_ratio))
        sample_arc = np.arange(count + 1, dtype=np.float64) * spacing
        # Floating point arithmetic can straddle the endpoint. Clamp rather
        # than extrapolate, then append it if no existing sample reaches it.
        sample_arc = sample_arc[sample_arc < length]
        sample_arc = np.concatenate((sample_arc, [length]))
        samples = np.empty((len(sample_arc), 3), dtype=np.float64)
        samples[:, 0] = np.interp(sample_arc, path.arc_length_m, path.xy_m[:, 0])
        samples[:, 1] = np.interp(sample_arc, path.arc_length_m, path.xy_m[:, 1])
        segments = np.diff(path.xy_m, axis=0)
        headings = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
        segment_indices = np.clip(
            np.searchsorted(path.arc_length_m, sample_arc, side="right") - 1,
            0,
            len(headings) - 1,
        )
        samples[:, 2] = headings[segment_indices]
        samples[-1, :2] = path.xy_m[-1]
    return SampledPath(
        points_body_m_rad=_readonly(samples),
        arc_length_m=_readonly(sample_arc),
        capture_stamp_ns=path.capture_stamp_ns,
        frame_id=path.frame_id,
        spacing_m=spacing,
        v_ref_mps=speed,
        mpc_dt_s=dt,
        action_scale_m=path.action_scale_m,
        raw_waypoint_count=path.raw_waypoint_count,
    )
