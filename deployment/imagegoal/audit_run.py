"""Audit recorded task/feedback invariants; never connects to hardware."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def audit_run(directory: Path) -> dict:
    run = json.loads((directory/"run.json").read_text())
    statuses = [json.loads(line) for line in (directory/"status.jsonl").read_text().splitlines()]
    events = [json.loads(line) for line in (directory/"events.jsonl").read_text().splitlines()]
    errors = []
    nonzero = 0
    stopped_after_terminal = 0
    positions = []
    previous_sequence = 0
    for event in events:
        if event["sequence"]<=previous_sequence:
            errors.append("event sequence is not strictly increasing")
        previous_sequence = event["sequence"]
    for index,status in enumerate(statuses):
        command = status["proposal"]
        moving = abs(command["vx_m_s"])+abs(command["vy_m_s"])+abs(command["wz_rad_s"])>1e-8
        if status["hardware_transport"]!="absent" or command["published_to_robot"] is not False:
            errors.append(f"status {index}: hardware boundary changed")
        if status["not_for_motion"] is not True:
            envelope=status["execution_envelope"]
            if not (run["config"].get("execution_profile")=="operator-run" and
                    status["task"].get("execution_mode")=="operator_run" and
                    envelope.get("transport_authority")=="operator_run" and envelope.get("eligible_for_robot_tracking") is True and
                    envelope.get("commissioning") is False and envelope.get("synthetic") is False and
                    envelope.get("calibration",{}).get("verified") is True and envelope.get("gate_snapshot")==[]):
                errors.append(f"status {index}: physical authority without explicit profile/task/verified real-input gates")
        if status["profile"]=="simulated-replay" and not status["synthetic_feedback"]:
            errors.append(f"status {index}: simulation label missing")
        if status["sensors"]["odom"]:
            positions.append(status["sensors"]["odom"]["pose_m_rad"])
        if moving:
            nonzero += 1
            plan = status["plan"]
            if status["state"]!="TRACKING" or status["gates"] or plan is None:
                errors.append(f"status {index}: nonzero proposal without ungated TRACKING")
                continue
            if command["epoch"]!=status["epoch"] or plan["epoch"]!=status["epoch"] or plan["goal_version"]!=status["task"]["goal_version"]:
                errors.append(f"status {index}: stale task/goal/epoch control")
            if plan["source_age_s"]>run["config"]["max_plan_age_s"]+.15:
                errors.append(f"status {index}: stale source produced a nonzero proposal")
            if abs(command["vx_m_s"])>run["config"]["max_v_mps"]+1e-8 or abs(command["wz_rad_s"])>run["config"]["max_w_rad_s"]+1e-8:
                errors.append(f"status {index}: MPC velocity limit exceeded")
            if not status["execution_envelope"]["eligible_for_shadow_tracking"]:
                errors.append(f"status {index}: tracking envelope is not eligible")
            request = json.loads((directory/"requests"/f"{plan['request_id']}.request.json").read_text())
            if request["capture_pose"]!=plan["capture_pose_odom_m_rad"]:
                errors.append(f"status {index}: capture pose was re-anchored after inference")
        if status["state"] in {"CANCELLED","TIMED_OUT","FAILED","SUCCEEDED","PAUSED","MODEL_STOP"}:
            stopped_after_terminal += 1
            if moving:
                errors.append(f"status {index}: task stop/terminal state produced motion")
    displacement = 0.
    if positions:
        displacement = max(((pose[0]-positions[0][0])**2+(pose[1]-positions[0][1])**2)**.5 for pose in positions)
    final = json.loads((directory/"status.json").read_text())
    rejected = [event for event in events if event["kind"]=="inference_rejected"]
    return {"schema_version":"imagegoal.audit.v1","ok":not errors,"errors":list(dict.fromkeys(errors)),
            "run_directory":str(directory.resolve()),"not_for_motion":True,"navigation_success_validated":False,
            "status_count":len(statuses),"event_count":len(events),"nonzero_shadow_proposals":nonzero,
            "stopped_or_terminal_statuses":stopped_after_terminal,"synthetic_pose_displacement_m":displacement if run["config"]["profile"]=="simulated-replay" else None,
            "final_state":final["state"],"metrics":final["metrics"],
            "invalidation_reasons":[event["reason"] for event in events if event["kind"]=="execution_invalidated"],
            "rejection_reasons":sorted({reason for event in rejected for reason in event["reasons"]}),
            "limitations":["Recorded images do not change with simulated pose.","Synthetic odom/depth never establishes real navigation performance.","No robot command transport was present."]}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory",type=Path)
    parser.add_argument("--output",type=Path)
    args = parser.parse_args(argv)
    report = audit_run(args.run_directory)
    if args.output:
        args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
    return 0 if report["ok"] else 1


if __name__=="__main__":
    raise SystemExit(main())
