"""Bounded real-model authority commissioning with fake external-schema inputs.

All fixtures, receipts and envelopes are marked commissioning. They can only
drive FakeSportTransport; a physical adapter must reject them. The module
never initializes ROS, DDS, a robot SDK or any physical sensor.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image

from deployment.nomad.shadow_client import encode_rgb_jpeg
from .calibration import host_boot_id, load_calibration, sha256_file
from .http_api import RuntimeHTTPServer
from .runtime import ImageGoalRuntime, RuntimeConfig, atomic_json
from .sources import read_replay_manifest


def create_fixture_receipts(directory: Path, robot_id: str = "commissioning-go2-not-physical") -> tuple[Path,Path]:
    directory.mkdir(parents=True,exist_ok=False)
    evidence = directory/"commissioning_measurement.json"
    atomic_json(evidence,{"commissioning":True,"physical_measurement":False,"note":"Exact fake clock/frame fixture; not a physical calibration."})
    common = {"robot_id":robot_id,"verified":True,"verified_by":"offline commissioning fixture, not an operator measurement",
              "verified_at_timestamp_ns":time.time_ns(),"commissioning":True,
              "evidence":[{"path":evidence.name,"sha256":sha256_file(evidence)}]}
    mount = {**common,"schema_version":"imagegoal.mount_receipt.v1","body_frame":"base_link",
             "rgb_frame":"camera_color_optical_frame","odom_frame":"go2_odom",
             "camera_heading_aligned_to_body":True,
             "T_body_rgb":[[0,0,1,0],[ -1,0,0,0],[0,-1,0,0],[0,0,0,1]]}
    timing = {**common,"schema_version":"imagegoal.time_alignment_receipt.v1","clock_domain":"unix_epoch_ns",
              "host_boot_id":host_boot_id(),"max_alignment_error_s":0.,
              "sources":{name:{"frame_name":"go2_odom" if name=="odom" else "camera_color_optical_frame",
                                "timestamp_source":f"commissioning_{name}_header_not_hardware","stamp_correction_s":0.}
                         for name in ("rgb","depth","odom")}}
    mount_path,timing_path = directory/"mount.json",directory/"timing.json"
    atomic_json(mount_path,mount)
    atomic_json(timing_path,timing)
    return mount_path,timing_path


class CommissioningInputs:
    """Fake moving state using real input schemas, always commissioning=true."""
    def __init__(self,runtime: ImageGoalRuntime,manifest: Path):
        if not runtime.config.commissioning_real_inputs:
            raise ValueError("fake real-schema source requires the explicit commissioning profile")
        self.runtime,self.frames = runtime,read_replay_manifest(manifest)
        self.stop = threading.Event()
        self.pause_odom = False
        self.pose = np.zeros(3)
        self.thread = threading.Thread(target=self._run,name="imagegoal-commissioning-inputs",daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)

    def _run(self):
        last,next_rgb,next_depth,index = time.monotonic(),0.,0.,0
        while not self.stop.is_set():
            now,stamp = time.monotonic(),time.time_ns()
            dt = min(.1,max(0,now-last));last=now
            with self.runtime.lock:
                command = dict(self.runtime.proposal)
            velocity = [command["vx_m_s"],command["wz_rad_s"]]
            self.pose += dt*np.array([velocity[0]*np.cos(self.pose[2]),velocity[0]*np.sin(self.pose[2]),velocity[1]])
            common = {"capture_timestamp_ns":stamp,"clock_domain":"unix_epoch_ns","synthetic":False,"commissioning":True}
            try:
                if not self.pause_odom:
                    self.runtime.ingest_odom({**common,"timestamp_source":"commissioning_odom_header_not_hardware","frame_name":"go2_odom",
                                              "body_frame_name":"base_link","pose_m_rad":self.pose.tolist(),"velocity_v_w":velocity})
                if now>=next_rgb:
                    frame = self.frames[index%len(self.frames)]
                    self.runtime.ingest_rgb({**common,"timestamp_source":"commissioning_rgb_header_not_hardware","frame_name":"camera_color_optical_frame",
                                             "frame_id":f"commissioning-rgb-{index+1:08d}","rgb_jpeg_base64":frame["rgb_jpeg_base64"],
                                             "provenance":{"commissioning":True,"original_capture_timestamp_ns":frame["original_capture_timestamp_ns"]}})
                    index+=1;next_rgb=now+.25
                if now>=next_depth:
                    buffer=io.BytesIO();Image.fromarray(np.full((48,64),4000,dtype=np.uint16)).save(buffer,format="PNG")
                    self.runtime.ingest_depth({**common,"timestamp_source":"commissioning_depth_header_not_hardware","frame_name":"camera_color_optical_frame",
                                               "depth_png_base64":base64.b64encode(buffer.getvalue()).decode(),"scale_m":.001,"aligned_to_rgb":True})
                    next_depth=now+.1
            except Exception as exc:
                with self.runtime.lock:
                    self.runtime._event("commissioning_source_error",detail=f"{type(exc).__name__}: {exc}")
                return
            self.stop.wait(.02)


def main(argv=None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest",type=Path,required=True)
    parser.add_argument("--goal",type=Path,required=True)
    parser.add_argument("--server-url",default="http://127.0.0.1:18882")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--port",type=int,default=0)
    parser.add_argument("--hold-s",type=float,default=0,help="bounded candidate hold for a separate FakeSport audit client")
    args=parser.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    mount,timing=create_fixture_receipts(args.output/"fixture_calibration")
    robot_id="commissioning-go2-not-physical"
    report={"commissioning":True,"physical_execution":False,"checks":{}}
    runtime=ImageGoalRuntime(args.output/"runtime",RuntimeConfig(profile="external",execution_profile="operator-run",
                            nomad_url=args.server_url,robot_id=robot_id,mount_receipt=str(mount),timing_receipt=str(timing),commissioning_real_inputs=True))
    source=CommissioningInputs(runtime,args.manifest)
    server=RuntimeHTTPServer(("127.0.0.1",args.port),runtime)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    def wait_for(predicate,seconds=8):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            status=runtime.status()
            if predicate(status): return status
            time.sleep(.03)
        raise RuntimeError("commissioning condition timed out: "+json.dumps(runtime.status()["gates"]))
    def nonzero(status):
        return abs(status["proposal"]["vx_m_s"])+abs(status["proposal"]["wz_rad_s"])>1e-8
    encoded,_=encode_rgb_jpeg(args.goal)
    task={"goal":{"rgb_jpeg_base64":encoded},"backend":"nomad","timeout_s":90}
    runtime.start();source.start()
    try:
        runtime.create_task({**task,"expected_epoch":runtime.status()["epoch"]})
        shadow=wait_for(lambda s:s["state"]=="TRACKING" and nonzero(s))
        report["checks"]["default_task_stays_shadow"]=not shadow["execution_envelope"]["eligible_for_commissioning_tracking"]
        runtime.task_action("cancel",{"reason":"commissioning_default_mode_checked"})
        runtime.create_task({**task,"execution_mode":"operator_run","expected_epoch":runtime.status()["epoch"]})
        candidate=wait_for(lambda s:s["execution_envelope"]["eligible_for_commissioning_tracking"] and nonzero(s))
        atomic_json(args.output/"candidate_status.json",candidate)
        report["checks"]["explicit_operator_candidate"]=True
        report["checks"]["physical_authority_stays_blocked"]=(candidate["execution_envelope"]["transport_authority"]=="commissioning_only" and
                       candidate["execution_envelope"]["not_for_motion"] is True and candidate["execution_envelope"]["eligible_for_robot_tracking"] is False)
        print(json.dumps({"url":f"http://127.0.0.1:{server.server_port}","candidate":str(args.output/"candidate_status.json"),
                          "commissioning":True,"physical_execution":False}),flush=True)
        if args.hold_s:
            time.sleep(min(30,max(0,args.hold_s)))
        source.pause_odom=True
        stale=wait_for(lambda s:"odom_stale_or_future" in s["gates"] and not s["execution_envelope"]["eligible_for_commissioning_tracking"],seconds=2)
        report["checks"]["stale_odom_revokes_candidate"]=not nonzero(stale)
        source.pause_odom=False
        restored=wait_for(lambda s:s["execution_envelope"]["eligible_for_commissioning_tracking"] and nonzero(s))
        report["checks"]["fresh_inputs_require_new_plan"]=restored["execution_envelope"]["epoch"]>candidate["execution_envelope"]["epoch"]
        # A stopped controller cannot renew a lease merely through status GETs;
        # the source continues to emit fresh timestamps during this check.
        runtime.stop_event.set();runtime.thread.join(timeout=2);time.sleep(.3)
        expired=runtime.status()
        report["checks"]["status_poll_does_not_renew_control_lease"]=(not expired["execution_envelope"]["eligible_for_commissioning_tracking"] and
                           "controller_evaluation_stale" in expired["execution_envelope"]["gate_snapshot"])
        cancelled=runtime.task_action("cancel",{"reason":"commissioning_complete"})
        report["checks"]["cancel_revokes_synchronously"]=not cancelled["execution_envelope"]["eligible_for_commissioning_tracking"] and not nonzero(cancelled)
        physical=load_calibration(mount,timing,robot_id)
        report["checks"]["physical_receipt_loader_rejects_fixtures"]=not physical.verified
        report["checks"]["calibration_hash_traceable"]=candidate["execution_envelope"]["calibration"]["mount_sha256"]==sha256_file(mount)
        guard=ImageGoalRuntime(args.output/"default_shadow_guard",RuntimeConfig(profile="external",commissioning_real_inputs=True))
        try:
            try:
                guard.create_task({**task,"execution_mode":"operator_run","expected_epoch":guard.status()["epoch"]})
                report["checks"]["default_runtime_rejects_operator_task"]=False
            except ValueError:
                report["checks"]["default_runtime_rejects_operator_task"]=True
        finally:
            guard.close()
        missing=ImageGoalRuntime(args.output/"missing_calibration_guard",RuntimeConfig(profile="external",execution_profile="operator-run",
                                robot_id=robot_id,commissioning_real_inputs=True))
        try:
            unavailable=missing.create_task({**task,"execution_mode":"operator_run","expected_epoch":missing.status()["epoch"]})
            report["checks"]["missing_receipts_never_elevate"]=(not unavailable["execution_envelope"]["eligible_for_commissioning_tracking"] and
                        all(reason in unavailable["execution_capability"]["blocking_reasons"] for reason in ("mount_receipt_missing","timing_receipt_missing")))
        finally:
            missing.close()
        mismatched=json.loads(json.dumps(candidate["sensors"]))
        mismatched["odom"]["timestamp_source"]="unmeasured_other_odom_source"
        report["checks"]["source_provenance_mismatch_blocks"]=bool(runtime.calibration.source_reasons(mismatched))
        tampered=json.loads(timing.read_text())
        tampered["evidence"][0]["sha256"]="0"*64
        tampered_path=timing.parent/"tampered_timing.json"
        atomic_json(tampered_path,tampered)
        rejected=load_calibration(mount,tampered_path,robot_id,allow_commissioning=True)
        report["checks"]["tampered_evidence_blocks"]=not rejected.verified and any("sha256_mismatch" in reason for reason in rejected.blocking_reasons)
        report["ok"]=all(report["checks"].values())
    finally:
        source.close();runtime.close();server.shutdown();server.server_close()
        atomic_json(args.output/"report.json",report)
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report.get("ok") else 1


if __name__=="__main__":
    raise SystemExit(main())
