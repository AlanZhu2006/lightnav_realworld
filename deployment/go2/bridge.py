"""Guarded entry point around the unchanged, pinned GEM Go2 bridge.

Default disabled mode does not import ROS, initialize DDS, construct a
SportClient, or send a shutdown zero. Hardware-locked is an explicit operator
mode which requires real fresh telemetry and still leaves enabled=false.
There is no automatic path from a shadow runtime proposal to this bridge.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from urllib.request import urlopen

from .preflight import load_config, preflight, check_feedback


def verify_vendor():
    root = Path(__file__).parent / "vendor/gem"
    record = json.loads((root/"SOURCE.json").read_text())
    for name,item in record["files"].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != item["sha256"]:
            raise RuntimeError(f"GEM source hash mismatch: {name}")
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode",choices=("disabled","hardware-locked","execute"),default="disabled")
    parser.add_argument("--execute",action="store_true",help="explicit operator-run mode; preflight remains mandatory")
    parser.add_argument("--config",type=Path)
    parser.add_argument("--feedback-status",type=Path,default=Path(".runtime/go2/telemetry.json"))
    parser.add_argument("--runtime-url",default="http://127.0.0.1:8093")
    parser.add_argument("--task-id")
    parser.add_argument("--robot-id")
    parser.add_argument("--mount-receipt",type=Path)
    parser.add_argument("--timing-receipt",type=Path)
    args = parser.parse_args()
    if args.execute: args.mode = "execute"
    config = load_config(args.config)
    vendor = verify_vendor()
    if args.mode == "disabled":
        print(json.dumps({"mode":"disabled","enabled":False,"dds_initialized":False,
                          "sport_client_initialized":False,"motion_sent":False,
                          "gem_source_hashes_verified":True},indent=2))
        return
    report = preflight(config,args.feedback_status)
    if not report["ok"]:
        print(json.dumps(report,indent=2))
        raise SystemExit(2)
    calibration = None
    binding = {}
    if args.mode == "execute":
        if not all((args.task_id,args.robot_id,args.mount_receipt,args.timing_receipt)):
            parser.error("--execute requires task-id, robot-id, mount-receipt and timing-receipt")
        from deployment.imagegoal.calibration import load_calibration
        from .execution_envelope import validate_envelope
        calibration = load_calibration(args.mount_receipt,args.timing_receipt,args.robot_id)
        if not calibration.verified:
            print(json.dumps({"blocked":calibration.blocking_reasons,"dds_initialized":False,
                              "sport_client_initialized":False},indent=2))
            raise SystemExit(2)
        try:
            with urlopen(args.runtime_url.rstrip("/")+"/status",timeout=.3) as response:
                initial = json.load(response)
            envelope = initial["execution_envelope"]
            if initial.get("state") in {"CANCELLED","FAILED","TIMED_OUT","SUCCEEDED"}:
                raise ValueError("operator task is terminal")
            if envelope.get("task_id") != args.task_id:
                raise ValueError("explicit task-id differs from active task")
            source_reasons = calibration.source_reasons(envelope.get("sensors",{}))
            if source_reasons: raise ValueError(";".join(source_reasons))
            binding = {key:envelope[key] for key in ("session_id","task_id","goal_id","goal_version","epoch")}
            binding.update(physical_preflight_verified=True,calibration=calibration.summary())
            decision = validate_envelope(envelope,binding=binding,config=config)
            if not decision.accepted: raise ValueError(";".join(decision.reasons))
        except Exception as exc:
            print(json.dumps({"blocked":[f"runtime_operator_run_preflight:{exc}"],
                              "dds_initialized":False,"sport_client_initialized":False},indent=2))
            raise SystemExit(2)
    # No SDK/SportClient import or initialization exists before preflight.
    sys.path.insert(0,str(vendor))
    spec = importlib.util.spec_from_file_location("_dejaview_gem_go2_bridge",vendor/"go2_cmd_bridge.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    from .authority import CommandLease,CheckedSportTransport,make_validated_gem_bridge
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Empty
    ChannelFactoryInitialize(config["dds_domain_id"],config["net_if"])
    client = SportClient(); client.SetTimeout(.20); client.Init()
    lease = CommandLease(binding,config)
    transport = CheckedSportTransport(client,lease)
    rclpy.init(args=["--ros-args","--params-file",str(Path(__file__).with_name("gem_bridge.yaml"))])
    node = make_validated_gem_bridge(module)(transport,lease)
    node.create_subscription(Empty,"/imagegoal/operator/stop_motion",node.on_operator_stop,1)
    node.set_parameters([Parameter("enabled",value=args.mode=="execute")])
    transport.start_watchdog()
    stop_poll = threading.Event()
    def poll_runtime():
        while not stop_poll.is_set():
            started = time.monotonic()
            try:
                feedback_checks,_ = check_feedback(args.feedback_status,config)
                failures = [key for key,ok in feedback_checks.items() if not ok]
                if failures: raise ValueError("live_feedback:"+";".join(failures))
                with urlopen(args.runtime_url.rstrip("/")+"/status",timeout=.2) as response:
                    value = json.load(response)
                envelope = value["execution_envelope"]
                reasons = calibration.source_reasons(envelope.get("sensors",{}))
                if reasons: raise ValueError(";".join(reasons))
                lease.submit(envelope,runtime_state=value.get("state"))
            except Exception as exc:
                lease.invalidate(f"runtime_source_unavailable:{type(exc).__name__}:{exc}")
            stop_poll.wait(max(0,.05-(time.monotonic()-started)))
    poller = None
    if args.mode == "execute":
        poller = threading.Thread(target=poll_runtime,name="operator-envelope-reader",daemon=True)
        poller.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_poll.set()
        lease.invalidate("bridge_shutdown",latch=True)
        transport.close()
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if poller: poller.join(timeout=1)


if __name__ == "__main__":
    main()
