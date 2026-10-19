"""Preserve LightNav metric SE(2) headings, including in-place rotations.

LightNav waypoint rows have no trained time base. Reference time below is our
controller configuration: each segment receives max(distance/v_ref,
abs(shortest_yaw_delta)/w_ref), then samples at the MPC dt. NoMaD continues to
use its separate XY/tangent-yaw adapter.
"""
from __future__ import annotations

import math
import numpy as np

from deployment.nomad.trajectory_adapter import SampledPath


class SE2SampledPath(SampledPath):
    @property
    def has_motion(self) -> bool:
        delta = np.diff(self.points_body_m_rad,axis=0)
        return bool(len(delta) and np.any(np.abs(delta)>1e-12))


def resample_se2(trajectory, *, capture_stamp_ns: int, v_ref_mps: float,
                 w_ref_rad_s: float, mpc_dt_s: float, max_samples: int = 10000) -> SE2SampledPath:
    path = np.asarray(trajectory,dtype=np.float64)
    if path.ndim!=2 or path.shape[1]!=3 or not len(path) or not np.isfinite(path).all():
        raise ValueError("LightNav trajectory must be a finite nonempty Nx3 metric SE(2) array")
    if capture_stamp_ns<=0 or min(v_ref_mps,w_ref_rad_s,mpc_dt_s)<=0:
        raise ValueError("capture timestamp and execution reference rates must be positive")
    points = np.vstack((np.zeros((1,3)),path))
    output = [points[0].copy()]
    arcs = [0.]
    total = 0.
    for target in points[1:]:
        start = output[-1]
        delta = target-start
        delta[2] = math.atan2(math.sin(delta[2]),math.cos(delta[2]))
        distance = float(np.hypot(delta[0],delta[1]))
        duration = max(distance/v_ref_mps,abs(delta[2])/w_ref_rad_s)
        if duration<=1e-12:
            continue
        count = max(1,math.ceil(duration/mpc_dt_s))
        if len(output)+count>max_samples:
            raise ValueError("SE(2) reference exceeds bounded sample count")
        for index in range(1,count+1):
            ratio = index/count
            output.append(start+ratio*delta)
            arcs.append(total+ratio*distance)
        total += distance
    array,arc = np.asarray(output),np.asarray(arcs)
    array.setflags(write=False)
    arc.setflags(write=False)
    return SE2SampledPath(array,arc,capture_stamp_ns,"base_link",v_ref_mps*mpc_dt_s,
                          v_ref_mps,mpc_dt_s,1.0,len(path))
