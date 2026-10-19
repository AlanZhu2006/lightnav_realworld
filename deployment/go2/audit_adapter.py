"""Offline commissioning audit of the real GEM methods with fake Sport RPCs.

No DDS or SportClient is initialized. A virtual playback clock is explicit;
fixture authority remains commissioning_only and not_for_motion=true. The
physical validator independently rejects every fixture. This is a deployment
diagnostic, not navigation evaluation or measured hardware stop latency.
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

from .authority import CommandLease,CheckedSportTransport,make_validated_gem_bridge
from .bridge import verify_vendor
from .execution_envelope import validate_envelope
from .preflight import load_config


def fixture(now):
    sensor = {"capture_timestamp_ns":now-10_000_000,"clock_domain":"unix_epoch_ns",
              "timestamp_source":"commissioning_virtual_playback","frame_name":"fixture_frame",
              "synthetic":True,"commissioning":True}
    return {"schema_version":"imagegoal.execution_envelope.v1","session_id":"offline-fixture",
            "task_id":"fake-sport-task","goal_id":"full-image-fixture","goal_version":0,"epoch":1,
            "sequence":1,
            "evaluated_timestamp_ns":now,"expires_timestamp_ns":now+250_000_000,
            "transport_authority":"commissioning_only","authority_schema":"imagegoal.robot_execution_authorized.v1",
            "candidate_transport_authority":"operator_run","not_for_motion":True,"synthetic":True,"commissioning":True,
            "execution_mode":"operator_run",
            "eligible_for_commissioning_tracking":True,"eligible_for_robot_tracking":False,
            "gate_snapshot":[],"source_stamp_ns":now-20_000_000,
            "sensors":{name:dict(sensor) for name in ("rgb","depth","odom")},
            "command":{"vx_m_s":.08,"vy_m_s":0.,"wz_rad_s":.1,"not_for_motion":True}}


class FakeSportTransport:
    is_fake_transport = True
    def __init__(self): self.calls=[]; self.fail_next=False
    def Move(self,vx,vy,wz):
        self.calls.append({"method":"Move","values":[vx,vy,wz]})
        code = 1 if self.fail_next else 0; self.fail_next=False; return code
    def StopMove(self): self.calls.append({"method":"StopMove"}); return 0


def audit(envelope=None):
    root = verify_vendor(); sys.path.insert(0,str(root))
    spec = importlib.util.spec_from_file_location("_go2_audit_original_gem",root/"go2_cmd_bridge.py")
    gem = importlib.util.module_from_spec(spec); sys.modules[spec.name]=gem; spec.loader.exec_module(gem)
    config = load_config(); clock=[time.time_ns(),1_000_000_000]
    initial = copy.deepcopy(envelope) if envelope else fixture(clock[0])
    # A producer can snapshot a received sample a few ms ahead of its earlier
    # evaluation timestamp. Playback begins after the latest recorded sample;
    # source/evaluation/expiry stamps and the physical future gate stay intact.
    latest_source=max(item["capture_timestamp_ns"] for item in initial["sensors"].values())
    playback_offset=max(1_000_000,latest_source-initial["evaluated_timestamp_ns"]+1_000_000)
    clock[0]=initial["evaluated_timestamp_ns"]+playback_offset
    binding={key:initial[key] for key in ("session_id","task_id","goal_id","goal_version","epoch")}
    binding["commissioning_fixture_verified"]=True
    if initial.get("calibration"): binding["calibration"]=initial["calibration"]
    lease=CommandLease(binding,config,commissioning=True,wall_clock=lambda:clock[0],mono_clock=lambda:clock[1])
    fake=FakeSportTransport(); transport=CheckedSportTransport(fake,lease)
    Base=make_validated_gem_bridge(gem)
    class FakeNode(Base):
        def __init__(self):
            self.lease=lease; self.sport_client=transport; self.remote=False
            self.enabled=True; self.latest_cmd=None; self.last_sent=(None,None,None); self.command_active=False
            self.operator_stop_latched=False; self._operator_stop_sent=False
            self.max_vx=.2; self.max_vy=0.; self.max_wz=.4
            self.x_sign=self.y_sign=self.wz_sign=1.; self.swap_xy=False
            self.deadband_v=.01; self.deadband_w=.02; self.min_cmd_v=self.min_cmd_w=0.
            self.timeout_sec=.25; self.send_zero_when_idle=False; self.stop_once_on_release=True
            self.remote_priority=True; self.remote_takeover_logged=False; self.log_commands=False
            self._stop_status_pub=NS(publish=lambda msg:None)
            transport.remote_active=self.remote_is_active
        def get_logger(self): return NS(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
        def get_parameter(self,name): return NS(value=self.enabled)
        def now_sec(self): return clock[0]/1e9
        def get_clock(self): return NS(now=lambda:NS(nanoseconds=clock[0]))
        def remote_is_active(self): return self.remote
    node=FakeNode()
    checks={}
    physical_binding={**binding,"physical_preflight_verified":True}
    checks["physical_validator_rejects_commissioning"]=not validate_envelope(
        initial,binding=physical_binding,config=config,now_ns=clock[0]).accepted
    checks["commissioning_candidate_accepted"]=lease.submit(initial)
    node.publish_to_go2()
    checks["original_gem_moves_fake_only"]=bool(fake.calls and fake.calls[-1].get("values",[0])[0]!=0)
    before=len(fake.calls); node.on_cmd_vel(gem.Twist())
    checks["raw_twist_inert"]=len(fake.calls)==before
    # Actual adapter polling is 20 Hz while the producer evaluates MPC at
    # 10 Hz. Polling the same evaluation must not stop or renew that command.
    sequence_counter=[initial["sequence"]]
    polling_packet=copy.deepcopy(initial)
    polling_ok=True; duplicates_preserved=True
    for poll in range(8):
        clock[0]+=50_000_000; clock[1]+=50_000_000
        if poll%2:
            sequence_counter[0]+=1
            for key in ("evaluated_timestamp_ns","expires_timestamp_ns","source_stamp_ns"):
                polling_packet[key]+=100_000_000
            polling_packet["sequence"]=sequence_counter[0]
            for item in polling_packet["sensors"].values(): item["capture_timestamp_ns"]+=100_000_000
            for key in ("evaluated_timestamp_ns","expires_timestamp_ns","source_stamp_ns"):
                if key in polling_packet["command"]: polling_packet["command"][key]+=100_000_000
            if "sequence" in polling_packet["command"]: polling_packet["command"]["sequence"]=sequence_counter[0]
        old_received,old_fence=lease.received_monotonic_ns,lease.minimum_post_stop_source_ns
        accepted=lease.submit(polling_packet)
        node.publish_to_go2(); transport.watchdog_once()
        polling_ok=polling_ok and accepted and transport.command_active
        if not poll%2:
            duplicates_preserved=duplicates_preserved and lease.received_monotonic_ns==old_received and lease.minimum_post_stop_source_ns==old_fence
    checks["twenty_hz_poll_ten_hz_controller_continuous"]=polling_ok
    checks["duplicate_poll_does_not_renew_or_raise_stop_fence"]=duplicates_preserved
    accepted_decision=lease.current(); before_received=lease.received_monotonic_ns
    changed_age=copy.deepcopy(polling_packet)
    for item in changed_age["sensors"].values(): item["age_s"]=.05
    checks["duplicate_live_age_does_not_change_evaluation"]=lease.submit(changed_age) and lease.current() is accepted_decision and lease.received_monotonic_ns==before_received
    tampered=copy.deepcopy(polling_packet); tampered["command"]["vx_m_s"]+=.01
    checks["same_sequence_changed_command_rejected_without_rollback"]=not lease.submit(tampered) and lease.current() is accepted_decision
    old_evaluation=copy.deepcopy(polling_packet)
    old_evaluation["evaluated_timestamp_ns"]-=1; old_evaluation["sequence"]-=1
    checks["old_evaluation_cannot_interrupt_current_lease"]=not lease.submit(old_evaluation) and lease.current() is accepted_decision
    for _ in range(4):
        clock[0]+=50_000_000; clock[1]+=50_000_000
        lease.submit(polling_packet); node.publish_to_go2(); transport.watchdog_once()
    clock[0]+=50_000_000; clock[1]+=50_000_000
    repeated_after_expiry=lease.submit(polling_packet); transport.watchdog_once()
    checks["repeated_poll_cannot_extend_real_expiry"]=not repeated_after_expiry and lease.current() is None and not transport.command_active
    clock[0]+=300_000_000; clock[1]+=300_000_000; transport.watchdog_once()
    checks["independent_watchdog_zero_and_clear"]=not transport.command_active and lease.current() is None and any(
        item.get("values")==[0.,0.,0.] for item in fake.calls)
    old_plan=copy.deepcopy(initial)
    old_plan["evaluated_timestamp_ns"]=clock[0]; old_plan["expires_timestamp_ns"]=clock[0]+250_000_000
    sequence_counter[0]+=1; old_plan["sequence"]=sequence_counter[0]
    checks["new_lease_cannot_revive_pre_stop_plan"]=not lease.submit(old_plan)
    def renew():
        step=max(100_000_000,initial["evaluated_timestamp_ns"]-initial["source_stamp_ns"]+playback_offset+20_000_000)
        packet=copy.deepcopy(initial); delta=clock[0]-packet["evaluated_timestamp_ns"]+step-playback_offset
        for key in ("evaluated_timestamp_ns","expires_timestamp_ns","source_stamp_ns"): packet[key]+=delta
        for item in packet["sensors"].values(): item["capture_timestamp_ns"]+=delta
        sequence_counter[0]+=1; packet["sequence"]=sequence_counter[0]
        for key in ("evaluated_timestamp_ns","expires_timestamp_ns","source_stamp_ns"):
            if key in packet["command"]: packet["command"][key]+=delta
        if "sequence" in packet["command"]: packet["command"]["sequence"]=sequence_counter[0]
        clock[0]+=step; clock[1]+=step
        return packet
    packet=renew(); lease.submit(packet); node.remote=True; transport.watchdog_once(); node.publish_to_go2()
    checks["gem_remote_priority_yields"]=not node.command_active
    node.remote=False; packet=renew(); lease.submit(packet); node.publish_to_go2()
    advanced=copy.deepcopy(packet); advanced["epoch"]+=1; lease.submit(advanced); transport.watchdog_once()
    checks["epoch_change_zero_and_clear"]=not transport.command_active and lease.current() is None
    checks["epoch_fence_cannot_reaccept_old_task"]=not lease.submit(renew())
    fresh_epoch=renew(); fresh_epoch["epoch"]=advanced["epoch"]
    accepted=lease.submit(fresh_epoch); node.publish_to_go2()
    checks["same_task_new_epoch_resumes_with_post_stop_source"]=accepted and transport.command_active
    late=renew(); checks["late_old_epoch_cannot_interrupt_new_plan"]=(not lease.submit(late) and lease.current() is not None)
    next_goal=renew(); next_goal.update(epoch=advanced["epoch"]+1,goal_version=initial["goal_version"]+1,
                                      goal_id=initial["goal_id"]+"-next-keyframe")
    lease.submit(next_goal); transport.watchdog_once()
    checks["new_keyframe_goal_stops_before_rebinding"]=not transport.command_active
    fresh_goal=renew(); fresh_goal.update({key:next_goal[key] for key in ("epoch","goal_version","goal_id")})
    accepted=lease.submit(fresh_goal); node.publish_to_go2()
    checks["same_task_new_keyframe_resumes_without_restart"]=accepted and transport.command_active
    lease.submit(fresh_goal,runtime_state="CANCELLED"); transport.watchdog_once()
    checks["terminal_task_latches_run"]=bool(lease.latched_fault) and not transport.command_active
    # Independent terminal/RPC/STOP scenarios each use a new fake adapter,
    # representing a new explicit operator run. Goal changes above did not.
    transport.close()
    lease=CommandLease(binding,config,commissioning=True,wall_clock=lambda:clock[0],mono_clock=lambda:clock[1])
    transport=CheckedSportTransport(fake,lease); node=FakeNode()
    packet=renew(); lease.submit(packet); node.publish_to_go2(); lease.invalidate("http_source_disconnected"); transport.watchdog_once()
    checks["source_disconnect_zero_and_clear"]=not transport.command_active
    packet=renew(); lease.submit(packet); fake.fail_next=True; node.publish_to_go2(); transport.watchdog_once()
    checks["rpc_failure_zero_and_latch"]=bool(lease.latched_fault) and not transport.command_active
    checks["new_packet_cannot_clear_rpc_fault"]=not lease.submit(renew())
    transport.close()
    lease=CommandLease(binding,config,commissioning=True,wall_clock=lambda:clock[0],mono_clock=lambda:clock[1])
    transport=CheckedSportTransport(fake,lease); node=FakeNode()
    packet=renew(); lease.submit(packet); node.publish_to_go2(); node.on_operator_stop(gem.Empty())
    checks["operator_stop_latched"]=node.operator_stop_latched and bool(lease.latched_fault)
    checks["new_packet_cannot_clear_stop"]=not lease.submit(renew())
    result={"schema_version":"dejaview.go2.adapter.offline-audit.v1","offline_diagnostic":True,
            "commissioning":True,"not_for_motion":True,"clock":"explicit_virtual_fixture_playback",
            "playback_start_after_evaluation_ns":playback_offset,
            "dds_initialized":False,"sport_client_initialized":False,"physical_motion_sent":False,
            "original_gem_methods_used":True,"checks":checks,"fake_rpc_calls":fake.calls}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture",type=Path,help="actual commissioning runtime status JSON; physical envelopes are rejected")
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    value=json.loads(args.fixture.read_text()) if args.fixture else None
    envelope=value.get("execution_envelope",value) if value else None
    result=audit(envelope)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if all(result["checks"].values()) else 1)


if __name__=="__main__": main()
