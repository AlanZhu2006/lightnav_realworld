"""Strict boundary validation; current shadow envelopes are always rejected.

This module never imports ROS or a robot SDK. A physical executor must have an
independent operator binding and physical preflight evidence. Merely toggling
one field in a runtime status never turns its shadow proposal into authority.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time


@dataclass(frozen=True)
class ExecutionDecision:
    accepted: bool
    reasons: tuple[str, ...]
    command_vx_vy_wz: tuple[float,float,float]
    expires_timestamp_ns: int


def _validate(envelope, *, binding, config, now_ns=None, commissioning=False):
    now = time.time_ns() if now_ns is None else now_ns
    reasons = []
    if not isinstance(envelope, dict):
        return ExecutionDecision(False,("envelope_not_object",),(0.,0.,0.),0)
    if envelope.get("schema_version") != "imagegoal.execution_envelope.v1":
        reasons.append("schema_mismatch")
    expected_authority = "commissioning_only" if commissioning else "operator_run"
    if envelope.get("transport_authority") != expected_authority:
        reasons.append("transport_authority_not_physical")
    if envelope.get("authority_schema") != "imagegoal.robot_execution_authorized.v1":
        reasons.append("authority_schema_mismatch")
    if envelope.get("execution_mode") != "operator_run":
        reasons.append("execution_mode_not_operator_run")
    if envelope.get("not_for_motion") is not commissioning:
        reasons.append("not_for_motion")
    if (not isinstance(envelope.get("synthetic"),bool) if commissioning else envelope.get("synthetic") is not False):
        reasons.append("synthetic_or_unspecified")
    if envelope.get("commissioning") is not commissioning:
        reasons.append("commissioning_boundary_mismatch")
    if commissioning and envelope.get("candidate_transport_authority") != "operator_run":
        reasons.append("commissioning_candidate_not_operator_run")
    eligibility = "eligible_for_commissioning_tracking" if commissioning else "eligible_for_robot_tracking"
    if envelope.get(eligibility) is not True:
        reasons.append("not_eligible_for_robot_tracking")
    if envelope.get("gate_snapshot") != []:
        reasons.append("runtime_gates_not_clear")
    binding_key = "commissioning_fixture_verified" if commissioning else "physical_preflight_verified"
    if not isinstance(binding,dict) or binding.get(binding_key) is not True:
        reasons.append("physical_operator_binding_missing")
        binding = binding if isinstance(binding,dict) else {}
    for key in ("session_id","task_id","goal_id","goal_version","epoch"):
        if key not in binding or type(envelope.get(key)) is not type(binding.get(key)) or envelope.get(key) != binding.get(key):
            reasons.append(f"binding_{key}_mismatch")
    calibration = envelope.get("calibration")
    expected_calibration = binding.get("calibration")
    if not commissioning and not isinstance(expected_calibration,dict):
        reasons.append("independent_calibration_binding_missing")
    if isinstance(expected_calibration,dict):
        if not isinstance(calibration,dict) or any(calibration.get(key)!=expected_calibration.get(key)
                for key in ("robot_id","mount_sha256","timing_sha256","commissioning")):
            reasons.append("calibration_receipt_binding_mismatch")
        if not isinstance(calibration,dict) or calibration.get("verified") is not True:
            reasons.append("calibration_not_verified")
        if envelope.get("robot_id") != expected_calibration.get("robot_id"):
            reasons.append("binding_robot_id_mismatch")
    evaluated,expires = envelope.get("evaluated_timestamp_ns"),envelope.get("expires_timestamp_ns")
    if type(evaluated) is not int or not -50_000_000 <= now-evaluated <= 250_000_000:
        reasons.append("evaluation_stale_or_future")
    if (type(expires) is not int or expires <= now or type(evaluated) is not int
            or expires > evaluated+350_000_000):
        reasons.append("execution_lease_invalid")
    source = envelope.get("source_stamp_ns")
    if type(source) is not int or not 0 <= now-source <= 3_000_000_000:
        reasons.append("plan_source_stale_or_future")
    sensors = envelope.get("sensors")
    sensors = sensors if isinstance(sensors,dict) else {}
    for name,max_age_ns in (("rgb",3_000_000_000),("depth",500_000_000),("odom",300_000_000)):
        item = sensors.get(name)
        if not isinstance(item,dict):
            reasons.append(f"{name}_missing")
            continue
        stamp = item.get("capture_timestamp_ns")
        if type(stamp) is not int or not 0 <= now-stamp <= max_age_ns:
            reasons.append(f"{name}_stale_or_future")
        if ((not isinstance(item.get("synthetic"),bool) if commissioning else item.get("synthetic") is not False)
                or item.get("commissioning") is not commissioning
                or item.get("clock_domain") != "unix_epoch_ns"):
            reasons.append(f"{name}_clock_or_source_invalid")
        if not item.get("timestamp_source") or not item.get("frame_name"):
            reasons.append(f"{name}_provenance_missing")
    command = envelope.get("command")
    values = (0.,0.,0.)
    if not isinstance(command,dict) or command.get("not_for_motion") is not commissioning:
        reasons.append("command_not_authorized")
    else:
        raw = tuple(command.get(key) for key in ("vx_m_s","vy_m_s","wz_rad_s"))
        if not all(type(value) in (int,float) and math.isfinite(value) for value in raw):
            reasons.append("command_nonfinite")
        else:
            values = tuple(map(float,raw))
            limits = (config["max_vx_m_s"],config["max_vy_m_s"],config["max_wz_rad_s"])
            if any(abs(value)>limit+1e-12 for value,limit in zip(values,limits)):
                reasons.append("command_exceeds_limits")
    return ExecutionDecision(not reasons,tuple(reasons),values if not reasons else (0.,0.,0.),
                             expires if type(expires) is int else 0)


def validate_envelope(envelope, *, binding, config, now_ns=None):
    """Physical path; commissioning inputs can never pass."""
    return _validate(envelope,binding=binding,config=config,now_ns=now_ns,commissioning=False)


def validate_commissioning_candidate(envelope, *, binding, config, now_ns=None):
    """Offline fake transport only; this never grants physical authority."""
    return _validate(envelope,binding=binding,config=config,now_ns=now_ns,commissioning=True)


def main():
    from .preflight import load_config
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--status",type=Path,required=True)
    parser.add_argument("--binding",type=Path)
    parser.add_argument("--config",type=Path)
    args = parser.parse_args()
    value = json.loads(args.status.read_text())
    envelope = value.get("execution_envelope",value)
    binding = json.loads(args.binding.read_text()) if args.binding else None
    result = validate_envelope(envelope,binding=binding,config=load_config(args.config))
    print(json.dumps({**asdict(result),"audit_only":True,"dds_initialized":False,
                      "motion_sent":False},indent=2))


if __name__ == "__main__":
    main()
