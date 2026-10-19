"""Run the full task API with external sensors or explicit recorded simulation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import threading
import time

from deployment.nomad.shadow_client import encode_rgb_jpeg
from .http_api import RuntimeHTTPServer
from .runtime import ImageGoalRuntime, RuntimeConfig
from .sources import ReplaySimulator


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile",choices=("external","simulated-replay"),default="external")
    parser.add_argument("--output",type=Path,required=True,help="new artifact directory")
    parser.add_argument("--host",default="127.0.0.1")
    parser.add_argument("--port",type=int,default=8093)
    parser.add_argument("--static-dir",type=Path)
    parser.add_argument("--nomad-url",default="http://127.0.0.1:18882")
    parser.add_argument("--lightnav-url",default="http://127.0.0.1:18884")
    parser.add_argument("--request-timeout-s",type=float,default=8)
    parser.add_argument("--lightnav-max-pending-frames",type=int,default=8)
    parser.add_argument("--lightnav-max-request-bytes",type=int,default=384*1024)
    parser.add_argument("--lightnav-max-pending-age-s",type=float,default=2.)
    parser.add_argument("--max-plan-age-s",type=float,default=3)
    parser.add_argument("--execution-profile",choices=("shadow","operator-run","supervised-native"),default="shadow",
                        help="operator-run additionally requires a per-task explicit execution_mode and verified physical receipts")
    parser.add_argument("--operator-supervised",action="store_true",help="explicit onsite operator for the native avoidance test profile")
    parser.add_argument("--max-rgb-age-s",type=float,default=3.)
    parser.add_argument("--max-v-mps",type=float,default=.2)
    parser.add_argument("--max-w-rad-s",type=float,default=.4)
    parser.add_argument("--min-effective-v-mps",type=float,default=0.)
    parser.add_argument("--min-effective-w-rad-s",type=float,default=0.)
    for feature in ("resample", "speed-floor", "heading-filter", "short-actions"):
        parser.add_argument("--lightnav-adapt-"+feature,action=argparse.BooleanOptionalAction,default=True,
                            help="Go2 LightNav ImageGoal adaptation only; official navigation/tracking unchanged")
    parser.add_argument("--robot-id",default="")
    parser.add_argument("--mount-receipt",type=Path)
    parser.add_argument("--timing-receipt","--time-alignment-receipt",dest="timing_receipt",type=Path)
    parser.add_argument("--commissioning-real-inputs",action="store_true",
                        help="offline real-schema fixture only; all sources and envelopes are commissioning, physical adapters must reject")
    parser.add_argument("--execution-mode",choices=("shadow","operator_run","supervised_native"),default="shadow",
                        help="mode only for the optional --goal autostart task; API tasks specify their own mode")
    parser.add_argument("--nomad-stop-distance-steps",type=float,help="optional calibrated learned temporal-score stop threshold; never meters or automatic success")
    parser.add_argument("--nomad-stop-consecutive",type=int,default=3,help="distinct current-goal source frames required for NoMaD stop hold, at least two")
    parser.add_argument("--manifest",type=Path,help="required only for explicit simulated-replay")
    parser.add_argument("--goal",type=Path,help="optionally start a task using this full image")
    parser.add_argument("--backend",choices=("nomad","lightnav"),default="nomad")
    parser.add_argument("--instruction",default="")
    parser.add_argument("--task-timeout-s",type=float,default=120)
    parser.add_argument("--duration-s",type=float,help="optional finite service bound")
    parser.add_argument("--scenario",type=Path,help="explicit simulation-only scheduled fault/action JSON")
    args = parser.parse_args(argv)
    if args.profile=="simulated-replay" and args.manifest is None:
        parser.error("simulated-replay requires --manifest")
    if args.profile!="simulated-replay" and (args.manifest or args.scenario):
        parser.error("replay manifests and fault scenarios require simulated-replay")
    if args.duration_s is not None and args.duration_s<=0:
        parser.error("duration must be positive")
    config = RuntimeConfig(profile=args.profile,nomad_url=args.nomad_url,lightnav_url=args.lightnav_url,
                           request_timeout_s=args.request_timeout_s,max_plan_age_s=args.max_plan_age_s,max_rgb_age_s=args.max_rgb_age_s,
                           lightnav_max_pending_frames=args.lightnav_max_pending_frames,
                           lightnav_max_request_bytes=args.lightnav_max_request_bytes,
                           lightnav_max_pending_age_s=args.lightnav_max_pending_age_s,
                           max_v_mps=args.max_v_mps,max_w_rad_s=args.max_w_rad_s,
                           min_effective_v_mps=args.min_effective_v_mps,min_effective_w_rad_s=args.min_effective_w_rad_s,
                           lightnav_adapt_resample=args.lightnav_adapt_resample,
                           lightnav_adapt_speed_floor=args.lightnav_adapt_speed_floor,
                           lightnav_adapt_heading_filter=args.lightnav_adapt_heading_filter,
                           lightnav_adapt_short_actions=args.lightnav_adapt_short_actions,
                           operator_supervised=args.operator_supervised,
                           execution_profile=args.execution_profile,robot_id=args.robot_id,
                           mount_receipt=str(args.mount_receipt.expanduser().resolve()) if args.mount_receipt else None,
                           timing_receipt=str(args.timing_receipt.expanduser().resolve()) if args.timing_receipt else None,
                           commissioning_real_inputs=args.commissioning_real_inputs,
                           nomad_stop_distance_steps=args.nomad_stop_distance_steps,nomad_stop_consecutive=args.nomad_stop_consecutive)
    runtime = ImageGoalRuntime(args.output,config)
    server = RuntimeHTTPServer((args.host,args.port),runtime,args.static_dir)
    simulator = ReplaySimulator(runtime,args.manifest) if args.profile=="simulated-replay" else None
    shutdown = threading.Event()
    old_handlers = {}
    for signum in (signal.SIGTERM,signal.SIGINT):
        old_handlers[signum] = signal.signal(signum,lambda *_:shutdown.set())
    thread = threading.Thread(target=server.serve_forever,name="imagegoal-http",daemon=True)
    runtime.start()
    thread.start()
    if simulator:
        simulator.start()
    if args.goal:
        encoded,_ = encode_rgb_jpeg(args.goal.expanduser().resolve(strict=True))
        runtime.create_task({"goal":{"rgb_jpeg_base64":encoded},"backend":args.backend,
                             "task_mode":"imagegoal",
                             "instruction":args.instruction,"timeout_s":args.task_timeout_s,"execution_mode":args.execution_mode,
                             "expected_epoch":runtime.status()["epoch"]})
    scenario = []
    if args.scenario:
        scenario = json.loads(args.scenario.read_text())
        if not isinstance(scenario,list) or any(not isinstance(item,dict) or not isinstance(item.get("after_s"),(int,float)) or item["after_s"]<0 for item in scenario):
            raise ValueError("scenario must be a list of actions with nonnegative after_s")
        scenario = sorted(scenario,key=lambda item:item["after_s"])
        (runtime.output/"scenario.json").write_text(json.dumps(scenario,indent=2))
    print(json.dumps({"url":f"http://{args.host}:{server.server_port}","profile":args.profile,
                      "not_for_motion":True,"hardware_transport":"absent","output":str(runtime.output)}),flush=True)
    start = time.monotonic()
    try:
        while not shutdown.wait(.05):
            elapsed = time.monotonic()-start
            while scenario and scenario[0]["after_s"]<=elapsed:
                action = scenario.pop(0)
                kind = action["action"]
                if kind=="fault":
                    runtime.set_fault(action["name"],action["value"])
                elif kind=="goal":
                    encoded,_ = encode_rgb_jpeg(Path(action["path"]).expanduser().resolve(strict=True))
                    runtime.replace_goal({"goal":{"rgb_jpeg_base64":encoded},"expected_epoch":runtime.status()["epoch"]})
                else:
                    runtime.task_action(kind,action.get("payload",{}))
            if args.duration_s is not None and elapsed>=args.duration_s:
                break
    finally:
        if simulator:
            simulator.close()
        runtime.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        for signum,handler in old_handlers.items():
            signal.signal(signum,handler)
    print(json.dumps({"finished":True,"state":runtime.state,"metrics":runtime.metrics,"output":str(runtime.output)}),flush=True)
    return 0


if __name__=="__main__":
    raise SystemExit(main())
