"""Transport-independent execution of the released LightNav Go2 MPC path.

The MPC and geometry modules are byte-identical to LightNav-0 commit
3015508b70fb6e30bbd66701e94c9b916dcb8df8. Reference selection, coordinate
transforms and optimization call those original functions. Only the final
hardware command envelope is supplied by this deployment.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from deployment.nomad.mpc_reference import ReferencePacket
from deployment.nomad.vendor.lightnav import geometry


OFFICIAL_COMMIT = "3015508b70fb6e30bbd66701e94c9b916dcb8df8"
GEOMETRY_SHA256 = "af72a6ebca4ff18b3d1a09b69583c431d6cbdd071c478945cbe9aa45690d3a14"
GO2_MPC = {
    "horizon": 5, "dt_s": 0.1, "w_max": 3.0,
    "a_max_v": 10.0, "a_max_w": 10.0,
    "q_weights": (10.0, 10.0, 1.0), "r_weights": (0.1, 0.1),
}


def validate_waypoints(value) -> np.ndarray:
    """Keep the decoded ten cumulative poses, including their original scale."""
    points = np.asarray(value, dtype=np.float64)
    if points.shape != (10, 3) or not np.isfinite(points).all():
        raise ValueError("Official LightNav requires exactly ten finite [x,y,yaw] poses")
    points = points.copy()
    points.setflags(write=False)
    return points


class OfficialLightNavController:
    def __init__(self, core):
        if hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest() != GEOMETRY_SHA256:
            raise ValueError("Official LightNav geometry source integrity mismatch")
        self.core = core
        self.controller = core.controller_type(**GO2_MPC)
        self.previous_control = (0.0, 0.0)

    def reset(self):
        self.previous_control = (0.0, 0.0)

    def reference(self, points, capture_pose, current_pose, capture_stamp_ns, odom_frame_id, *, weights=GO2_MPC["q_weights"]):
        points = validate_waypoints(points)
        capture = geometry.TimedPose(capture_stamp_ns, *map(float, capture_pose))
        trajectory = np.asarray(geometry.project_body_to_odom(points, capture), dtype=np.float64)
        reference = self.core.reference_builder(
            trajectory, current_pose, horizon=GO2_MPC["horizon"], weights=weights)
        return ReferencePacket(
            trajectory, reference, np.asarray(capture_pose), np.asarray(current_pose),
            capture_stamp_ns, "base_link", odom_frame_id,
            not bool(np.any(points)), self.core.source_sha256)

    def solve(self, pose, reference, task_mode):
        # This is the same local-frame solve as MpcNode._solve. The previous
        # control is the last unscaled solver output, not measured odometry.
        origin = geometry.TimedPose(0, *map(float, pose))
        local_reference = np.asarray(geometry.project_odom_to_local(reference, origin))
        command, local_prediction = self.controller.solve(
            np.zeros(3), local_reference, self.previous_control,
            1.5 if task_mode == "tracking" else 0.8)
        self.previous_control = command
        prediction = np.asarray(geometry.project_local_to_odom(local_prediction, origin))
        return (command[0] * 2.0, command[1]), prediction

    def contract(self, max_v_mps, max_w_rad_s):
        return {
            "source_repository": "https://github.com/lightorigins/LightNav-0",
            "source_commit": OFFICIAL_COMMIT, "profile": "official_go2_launch",
            **GO2_MPC, "objnav_v_max": 0.8, "track_v_max": 1.5,
            "v_output_scale": 2.0, "w_output_scale": 1.0,
            "mpc_source_sha256": self.core.source_sha256,
            "geometry_source_sha256": GEOMETRY_SHA256,
            "trajectory_resampling": False, "prepend_origin": False,
            "minimum_speed_boost": False, "heading_deadband": False,
            "previous_control": "previous_unscaled_solver_command",
            "hardware_output_limits": {"vx_m_s": max_v_mps, "wz_rad_s": max_w_rad_s},
            "deployment_adaptations": ["HTTP orchestration", "native camera and odometry transport",
                "hardware output limits", "remote priority and stale-input watchdog"],
        }
