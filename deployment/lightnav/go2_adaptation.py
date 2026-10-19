"""Explicit Go2 ImageGoal controller adaptations, separate from official mode.

All budgets describe the model's predicted LOCAL ACTION and its current local
reference. They are not distance to the image goal, confidence, or arrival
tests. Floors only amplify nonzero MPC commands; budgets limit that added
amplification, never truncate an otherwise valid MPC command.
"""
from __future__ import annotations

import math

import numpy as np

from deployment.imagegoal.se2_adapter import resample_se2


SCHEMA = "lightnav.go2_imagegoal_adaptation.v1"
HEADING_DEADBAND_RAD = math.radians(8.)
COMMAND_DEADBAND = .04
NUMERICAL_EPSILON = 1e-9
DIRECTION_MIN_DISTANCE_M = .001
FLAGS = ("lightnav_adapt_resample", "lightnav_adapt_speed_floor",
         "lightnav_adapt_heading_filter", "lightnav_adapt_short_actions")


def _flag_values(config):
    values = {name: getattr(config, name, True) for name in FLAGS}
    if any(type(value) is not bool for value in values.values()):
        raise ValueError("LightNav adaptation flags must be boolean")
    return values


def _positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _settings(config):
    settings = {"v_ref": _positive(config.v_ref_mps, "v_ref_mps"),
                "w_ref": _positive(config.max_w_rad_s, "max_w_rad_s"),
                "v_max": _positive(config.max_v_mps, "max_v_mps"),
                "dt": 1 / _positive(config.tick_hz, "tick_hz"),
                "horizon": _positive(config.horizon, "horizon")}
    settings["duration"] = settings["dt"] * settings["horizon"]
    for key, name in (("v_floor", "min_effective_v_mps"), ("w_floor", "min_effective_w_rad_s")):
        value = getattr(config, name)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
        settings[key] = float(value)
    return settings


def _extent(raw):
    path = np.asarray(raw, dtype=np.float64)
    if path.shape != (10, 3) or not np.isfinite(path).all():
        raise ValueError("LightNav adaptation requires finite raw 10x3 metric SE(2) waypoints")
    points = np.vstack((np.zeros((1, 3)), path))
    deltas = np.diff(points, axis=0)
    length = float(np.hypot(deltas[:, 0], deltas[:, 1]).sum())
    angular = float(np.abs(np.arctan2(np.sin(deltas[:, 2]), np.cos(deltas[:, 2]))).sum())
    endpoint_distance = float(np.hypot(path[-1, 0], path[-1, 1]))
    bearing = (abs(math.atan2(float(path[-1, 1]), float(path[-1, 0])))
               if endpoint_distance >= DIRECTION_MIN_DISTANCE_M else 0.)
    return length, angular, max(angular, bearing)


def _telemetry(flags, settings, length, angular):
    return {"schema_version": SCHEMA, "profile": "go2_imagegoal_adaptation",
            "official_controller_equivalence_claimed": False,
            "metric_semantics": "predicted_local_action_extent_not_image_goal_distance",
            "arrival_evaluated": False, "flags": flags,
            "raw_path_length_m": length, "raw_angular_travel_rad": angular,
            "mpc_horizon_s": settings["duration"]}


def prepare_path(raw, capture_stamp_ns, config):
    """Resample locally at nominal rates, bounded for short predicted actions.

    Returns None when resampling is disabled, leaving the caller to use the
    original poses. Pure rotations retain a positive unused translation rate.
    """
    flags, settings = _flag_values(config), _settings(config)
    length, angular, _ = _extent(raw)
    telemetry = _telemetry(flags, settings, length, angular)
    v_ref, w_ref = settings["v_ref"], settings["w_ref"]
    if flags["lightnav_adapt_short_actions"]:
        if length > NUMERICAL_EPSILON:
            v_ref = min(v_ref, length / settings["duration"])
        if angular > NUMERICAL_EPSILON:
            w_ref = min(w_ref, angular / settings["duration"])
    telemetry.update(resampled=flags["lightnav_adapt_resample"],
                     nominal_v_ref_mps=settings["v_ref"], effective_v_ref_mps=v_ref,
                     nominal_w_ref_rad_s=settings["w_ref"], effective_w_ref_rad_s=w_ref)
    if not flags["lightnav_adapt_resample"]:
        return None, telemetry
    sampled = resample_se2(raw, capture_stamp_ns=capture_stamp_ns, v_ref_mps=v_ref,
                           w_ref_rad_s=w_ref, mpc_dt_s=settings["dt"])
    return sampled, telemetry


def postprocess_command(vx, wz, heading_error, reference_yaw_error, raw,
                        remaining_distance_m, remaining_yaw_rad, config):
    """Apply named filters/floors without inventing motion or asserting arrival.

    A short action with explicit angular/lateral intent gets an angle deadband
    no larger than one third of that intent and bypasses the magnitude deadband.
    Long straight predictions keep the nominal 8-degree/.04 filtering even if
    their RVQ residual contains a tiny nonzero yaw. Both reference bearing AND
    reference yaw must be small before the angular filter suppresses a command.
    """
    values = (vx, wz, heading_error, reference_yaw_error,
              remaining_distance_m, remaining_yaw_rad)
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError("Command, local reference errors and remaining extents must be finite")
    if remaining_distance_m < 0:
        raise ValueError("remaining_distance_m must be nonnegative")
    flags, settings = _flag_values(config), _settings(config)
    length, angular, intent = _extent(raw)
    telemetry = _telemetry(flags, settings, length, angular)
    vx = min(settings["v_max"], max(0., float(vx)))
    wz = min(settings["w_ref"], max(-settings["w_ref"], float(wz)))
    original_v, original_w = vx, wz
    short = flags["lightnav_adapt_short_actions"]
    duration = settings["duration"]
    v_budget = min(length, float(remaining_distance_m)) / duration
    # Turning toward a lateral local reference can be necessary even when
    # the model's yaw samples are all zero. These are local errors, not goals.
    angular_extent = max(angular, abs(heading_error), abs(reference_yaw_error))
    remaining_angle = max(abs(remaining_yaw_rad), abs(heading_error), abs(reference_yaw_error))
    w_budget = min(angular_extent, remaining_angle) / duration
    v_scale = min(1., v_budget / settings["v_ref"]) if short else 1.
    w_scale = min(1., w_budget / settings["w_ref"]) if short else 1.
    v_floor, w_floor = settings["v_floor"] * v_scale, settings["w_floor"] * w_scale
    if short:
        v_floor, w_floor = min(v_floor, v_budget), min(w_floor, w_budget)
    protected_turn = bool(short and length < settings["v_ref"] * duration
                          and intent > NUMERICAL_EPSILON)
    heading_deadband = min(HEADING_DEADBAND_RAD, intent/3.) if protected_turn else HEADING_DEADBAND_RAD
    w_deadband = 0. if protected_turn else COMMAND_DEADBAND
    v_deadband = COMMAND_DEADBAND * v_scale
    angular_suppressed = linear_suppressed = False
    if flags["lightnav_adapt_heading_filter"] and wz != 0.:
        if ((abs(heading_error) < heading_deadband and abs(reference_yaw_error) < heading_deadband)
                or abs(wz) < w_deadband):
            wz, angular_suppressed = 0., True
    if flags["lightnav_adapt_speed_floor"]:
        if vx != 0.:
            if vx < v_deadband:
                vx, linear_suppressed = 0., True
            else:
                vx = min(settings["v_max"], max(vx, v_floor))
        if wz != 0.:
            wz = math.copysign(min(settings["w_ref"], max(abs(wz), w_floor)), wz)
    telemetry.update(input_command={"vx_m_s": original_v, "wz_rad_s": original_w},
                     output_command={"vx_m_s": vx, "wz_rad_s": wz},
                     remaining_local_distance_m=float(remaining_distance_m),
                     remaining_local_yaw_rad=abs(float(remaining_yaw_rad)),
                     linear_amplification_budget_mps=v_budget,
                     angular_amplification_budget_rad_s=w_budget,
                     effective_v_floor_mps=v_floor, effective_w_floor_rad_s=w_floor,
                     effective_v_deadband_mps=v_deadband,
                     effective_w_deadband_rad_s=w_deadband,
                     effective_heading_deadband_rad=heading_deadband,
                     protected_short_turn=protected_turn,
                     linear_suppressed=linear_suppressed, angular_suppressed=angular_suppressed,
                     original_mpc_command_clipped_by_local_budget=False)
    return vx, wz, telemetry
