"""Bounded real-NoMaD temporal stop commissioning; never an arrival evaluator."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from deployment.nomad.shadow_client import encode_rgb_jpeg
from .runtime import ImageGoalRuntime, RuntimeConfig, atomic_json
from .sources import ReplaySimulator


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest",type=Path,required=True)
    parser.add_argument("--goal",type=Path,required=True)
    parser.add_argument("--alternate-goal",type=Path,required=True)
    parser.add_argument("--server-url",default="http://127.0.0.1:18882")
    parser.add_argument("--threshold-steps",type=float,required=True,help="explicit commissioning threshold; not a physically calibrated default")
    parser.add_argument("--consecutive",type=int,default=3)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--duration-s",type=float,default=8)
    args=parser.parse_args(argv)
    if args.duration_s<=0 or args.duration_s>30:
        parser.error("commissioning duration must be in (0,30]")
    runtime=ImageGoalRuntime(args.output,RuntimeConfig(profile="simulated-replay",nomad_url=args.server_url,
                              nomad_stop_distance_steps=args.threshold_steps,nomad_stop_consecutive=args.consecutive))
    simulator=ReplaySimulator(runtime,args.manifest)
    report={"schema_version":"imagegoal.nomad_stop_commissioning.v1","not_for_motion":True,
            "navigation_success_validated":False,"threshold_physically_calibrated":False,"checks":{},"snapshots":[]}
    deadline=time.monotonic()+args.duration_s
    def stopped(status):
        return (status["state"]=="MODEL_STOP" and status["stop_policy"]["latched"] and
                status["stop_policy"]["streak"]>=args.consecutive and
                all(status["proposal"][key]==0 for key in ("vx_m_s","vy_m_s","wz_rad_s")))
    def wait_stop():
        while time.monotonic()<deadline:
            status=runtime.status()
            if stopped(status): return status
            time.sleep(.03)
        raise TimeoutError("NoMaD stop commissioning exceeded its bound")
    goal,_=encode_rgb_jpeg(args.goal)
    other,_=encode_rgb_jpeg(args.alternate_goal)
    runtime.start();simulator.start()
    try:
        runtime.create_task({"goal":{"rgb_jpeg_base64":goal},"backend":"nomad","timeout_s":args.duration_s+2,"expected_epoch":runtime.status()["epoch"]})
        first=wait_stop()
        report["snapshots"].append(first)
        report["checks"]["threshold_latches_stop_not_success"]=first["stop_policy"]["streak"]>=args.consecutive and first["task"]["outcome"] is None
        changed=runtime.replace_goal({"goal":{"rgb_jpeg_base64":other},"expected_epoch":runtime.status()["epoch"]})
        report["checks"]["goal_change_clears_streak_and_latch"]=(changed["task"]["goal_version"]==2 and changed["stop_policy"]["streak"]==0 and not changed["stop_policy"]["latched"])
        second=wait_stop()
        report["snapshots"].append(second)
        scores=[event for event in runtime.events if event["kind"]=="nomad_stop_score"]
        report["checks"]["consecutive_frames_are_distinct"]=all(len({event["source_frame_id"] for event in scores if event["goal_version"]==version})>=args.consecutive for version in (1,2))
        report["checks"]["second_goal_latches_independently"]=second["stop_policy"]["streak"]>=args.consecutive
        cancelled=runtime.task_action("cancel",{"reason":"nomad_stop_commissioning_complete"})
        report["snapshots"].append(cancelled)
        report["checks"]["cancel_keeps_zero"]=cancelled["state"]=="CANCELLED" and all(cancelled["proposal"][key]==0 for key in ("vx_m_s","vy_m_s","wz_rad_s"))
        report["checks"]["never_auto_succeeded"]=not any(event["kind"]=="task_finished" and event.get("state")=="SUCCEEDED" for event in runtime.events)
        report["ok"]=all(report["checks"].values())
    finally:
        simulator.close();runtime.close()
        atomic_json(args.output/"stop_commissioning_report.json",report)
    print(json.dumps({"ok":report.get("ok",False),"checks":report["checks"],"output":str(args.output)},indent=2))
    return 0 if report.get("ok") else 1


if __name__=="__main__":
    raise SystemExit(main())
