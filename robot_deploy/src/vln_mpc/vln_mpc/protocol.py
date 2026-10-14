"""ROS-independent response validation, task modes, and controller settings."""

from __future__ import annotations

import json
import math

OBJNAV_MODE = "objnav"
TRACK_MODE = "track"
VALID_TASK_MODES = {OBJNAV_MODE, TRACK_MODE}
CONTROL_RATE_HZ = 10.0
HORIZON = 5
MPC_DT_S = 0.1
WAYPOINT_DT_S = 0.1
MPC_CONFIG_NAMES = (
    "track_v_max",
    "objnav_v_max",
    "w_max",
    "a_max_v",
    "a_max_w",
    "q_x",
    "q_y",
    "q_yaw",
    "r_v",
    "r_w",
    "v_output_scale",
    "w_output_scale",
)
POSITIVE_MPC_CONFIG_NAMES = {
    "track_v_max",
    "objnav_v_max",
    "w_max",
    "a_max_v",
    "a_max_w",
}
NONNEGATIVE_MPC_CONFIG_NAMES = set(MPC_CONFIG_NAMES) - POSITIVE_MPC_CONFIG_NAMES
CONTROLLER_CONFIG_NAMES = {
    "w_max",
    "a_max_v",
    "a_max_w",
    "q_x",
    "q_y",
    "q_yaw",
    "r_v",
    "r_w",
}


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def parse_vln_response(
    raw: str,
    expected_frame: str,
) -> tuple[int, int, int, bool | None, list[tuple[float, float, float]]]:
    try:
        response = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid VLN response JSON: {exc}") from exc
    if not isinstance(response, dict):
        raise ValueError("VLN response must be a JSON object")

    episode = response.get("episode")
    sequence = response.get("seq")
    stamp_ns = response.get("capture_stamp_ns")
    for name, value in (
        ("episode", episode),
        ("seq", sequence),
        ("capture_stamp_ns", stamp_ns),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"VLN response {name} must be a positive integer")
    if response.get("frame_id") != expected_frame:
        raise ValueError(
            f"VLN response frame is {response.get('frame_id')!r}, expected {expected_frame!r}"
        )
    stop = response.get("stop")
    for name in ("stop", "visible"):
        value = response.get(name)
        if value is not None and not isinstance(value, bool):
            raise ValueError(f"VLN response {name} must be boolean or null")

    raw_waypoints = response.get("waypoints")
    if not isinstance(raw_waypoints, list):
        raise ValueError("VLN response waypoints must be an array")
    waypoints = []
    for point in raw_waypoints:
        if not isinstance(point, list) or len(point) != 3:
            raise ValueError("each VLN waypoint must be [x, y, yaw]")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in point):
            raise ValueError("VLN waypoint values must be numbers")
        values = tuple(float(value) for value in point)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("VLN waypoint contains non-finite values")
        waypoints.append(values)
    return episode, sequence, stamp_ns, stop, waypoints


def should_complete_task(mode: str, stop: bool | None) -> bool:
    return mode == OBJNAV_MODE and stop is True


def validate_mpc_config(config: dict[str, float]) -> str:
    if set(config) != set(MPC_CONFIG_NAMES):
        return "MPC configuration fields are incomplete"
    if not all(math.isfinite(value) for value in config.values()):
        return "MPC configuration values must be finite"
    if any(config[name] <= 0.0 for name in POSITIVE_MPC_CONFIG_NAMES):
        return "MPC limits must be positive"
    if any(config[name] < 0.0 for name in NONNEGATIVE_MPC_CONFIG_NAMES):
        return "MPC weights and output scales must be nonnegative"
    if not any(config[name] > 0.0 for name in ("q_x", "q_y", "q_yaw")):
        return "at least one MPC state weight must be positive"
    return ""


def scale_command(
    command: tuple[float, float],
    v_scale: float,
    w_scale: float,
) -> tuple[float, float]:
    return command[0] * v_scale, command[1] * w_scale
