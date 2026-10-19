"""Offline LightNav response -> shared MPC -> native executor integration.

Only the recorded model's numeric output is reused. Test RGB, odometry,
identity and time are fixtures, and the executor/HTTP/process are fakes.
No model is started and no robot SDK, socket or subprocess may be used.
"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
from unittest.mock import patch

from PIL import Image

from . import runtime as runtime_module
from .runtime import ImageGoalRuntime, RuntimeConfig
from .audit_native_runtime import FakeManager, RecordedClock
from deployment.go2.native_imagegoal import NativeImageGoalExecutor, NATIVE_SOURCES, validate_native_status
from deployment.go2.audit_native_imagegoal import FakeClient


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESPONSE = ROOT/"data/realworld/nomad_commissioning_20261017/lightnav_full_model_002/reference_image.response.json"


def _goal():
    buffer = io.BytesIO()
    Image.new("RGB", (32, 24), (24, 64, 96)).save(buffer, format="JPEG")
    return {"rgb_jpeg_base64": base64.b64encode(buffer.getvalue()).decode("ascii")}


def _config(profile="supervised-native"):
    return RuntimeConfig(profile="external", execution_profile=profile,
                         operator_supervised=profile == "supervised-native",
                         max_v_mps=.3, max_w_rad_s=.55, max_rgb_age_s=2., max_plan_age_s=2.,
                         min_effective_v_mps=.1, min_effective_w_rad_s=.2)


def _request(runtime, goal, backend="lightnav"):
    return {"expected_session_id": runtime.session_id, "expected_epoch": runtime.epoch,
            **({"task_mode":"objnav"} if backend=="lightnav" else {"goal":goal}),
            "backend": backend, "timeout_s":60, "operator_present":True,
            "instruction":"Go to the coat rack by the windows and stop."}


def _seed(runtime, clock, goal):
    start = clock.time_ns()
    for index in range(4):
        clock.wall_ns = start + (index+1)*250_000_000
        common = {"capture_timestamp_ns":clock.time_ns(), "clock_domain":"unix_epoch_ns",
                  "synthetic":False, "commissioning":False}
        frame, source = NATIVE_SOURCES["rgb"]
        runtime.ingest_rgb({**common, **goal, "frame_id":f"offline-rgb-{index}",
                            "frame_name":frame, "timestamp_source":source,
                            "provenance":{"offline_fixture":True}})
        frame, source = NATIVE_SOURCES["odom"]
        runtime.ingest_odom({**common, "frame_name":frame, "timestamp_source":source,
                            "body_frame_name":"base_link", "pose_m_rad":[1.,2.,.3],
                            "velocity_v_w":[0.,0.]})
    clock.wall_ns += 50_000_000
    frame, source = NATIVE_SOURCES["odom"]
    runtime.ingest_odom({"capture_timestamp_ns":clock.time_ns(), "clock_domain":"unix_epoch_ns",
                        "synthetic":False, "frame_name":frame, "timestamp_source":source,
                        "body_frame_name":"base_link", "pose_m_rad":[1.,2.,.3], "velocity_v_w":[0.,0.]})


def _inject(runtime, clock, source_response, *, request_id, model_stop=False):
    frames = list(runtime.frames)
    identity = {"session_id":runtime.session_id,"task_id":runtime.task["task_id"],
                "goal_id":runtime.task["goal_id"],"goal_version":runtime.task["goal_version"],
                "frame_id":frames[-1].frame_id}
    payload = {"identity":identity,"observations":[frame.wire() for frame in frames]}
    metadata = {"request_id":request_id,"identity":identity,"epoch":runtime.epoch,
                "backend":"lightnav","source_stamp_ns":frames[-1].stamp,
                "created_monotonic_ns":clock.monotonic_ns()-50_000_000,
                "capture_pose":[1.,2.,.3],"capture_odom_gap_s":0.,"offline_fixture":True}
    response = copy.deepcopy(source_response)
    observation_frames = [{"frame_id":frame.frame_id,"capture_timestamp_ns":frame.stamp} for frame in frames]
    response.update(identity=identity,observation_frames=observation_frames,
                    source_frame=observation_frames[-1],model_stop=model_stop,
                    conditioning_mode="experimental_reference_image" if runtime.task["task_mode"]=="imagegoal" else "native_language",
                    task_mode=runtime.task["task_mode"],
                    implementation="go2_imagegoal_adaptation" if runtime.task["task_mode"]=="imagegoal" else "official_release",
                    goal_image_consumed=runtime.task["task_mode"]=="imagegoal",
                    official_release_commit=runtime_module.OFFICIAL_COMMIT,
                    official_core_sha256=runtime_module.OFFICIAL_CORE_DIGEST,eval_task="vlnce")
    if model_stop:
        response.update(trajectory_body_m_rad=[[0.,0.,0.]]*10,trajectory_body_m=[[0.,0.]]*10)
    runtime.inflight = metadata
    runtime.completed.put((metadata,payload,response,50.,None))
    runtime.tick()
    return runtime.status()


def audit(response_path=DEFAULT_RESPONSE):
    response_path = Path(response_path).resolve()
    recorded = json.loads(response_path.read_text())
    response = recorded["response"]
    assert response["model_stop"] is False and len(response["trajectory_body_m_rad"]) == 10
    goal, checks = _goal(), {}
    clock = RecordedClock(1_800_000_000_000_000_000, source_ns=0, source_mono_ns=10_000_000_000)
    before_sdk = {name for name in sys.modules if name.startswith("unitree_sdk2py")}
    runtimes = []
    with tempfile.TemporaryDirectory(prefix="lightnav-native-offline-") as temporary, \
         patch.object(runtime_module,"time",clock), \
         patch.object(runtime_module,"request_json",side_effect=AssertionError("offline audit must not use HTTP")), \
         patch.object(ImageGoalRuntime,"_schedule_lightnav_cleanup"), \
         patch.object(socket,"socket",side_effect=AssertionError("offline audit must not open sockets")), \
         patch("subprocess.Popen",side_effect=AssertionError("offline audit must not start a subprocess")):
        try:
            runtime = ImageGoalRuntime(Path(temporary)/"native",_config())
            runtimes.append(runtime)
            runtime._dispatch = lambda frames: None
            manager = FakeManager()
            runtime.native_executor = manager
            selected = runtime.select_goal({"expected_session_id":runtime.session_id,"expected_epoch":runtime.epoch,
                                            "goal":goal},source="import")
            request = _request(runtime,goal)
            initial = runtime.start_native(request,runtime_url="http://offline.invalid:8093")
            assert initial["task"]["backend"] == "lightnav" and initial["task"]["instruction"] == request["instruction"]
            assert initial["task"]["selection_id"] is None
            assert initial["goal_selection"]["selection_id"] == selected["goal_selection"]["selection_id"]
            assert initial["task"]["execution_mode"] == "supervised_native" and len(manager.starts)==1
            assert initial["execution_capability"]["supervised_native_backends"] == ["nomad","lightnav"]
            checks["lightnav_native_start_preserves_backend_instruction_and_manager_binding"] = True

            # A second Start must not silently take over the owned executor.
            try:
                runtime.start_native(_request(runtime,goal,"nomad"),runtime_url="http://offline.invalid:8093")
                raise AssertionError("second Start replaced a running executor")
            except ValueError as exc:
                assert "stop the current native executor" in str(exc)
            assert len(manager.starts)==1 and runtime.task["backend"] == "lightnav"
            checks["running_executor_cannot_be_replaced_by_another_backend"] = True

            _seed(runtime,clock,goal)
            tracked = _inject(runtime,clock,response,request_id="offline-recorded-lightnav")
            assert runtime.metrics["inference_accepted"]==1 and runtime.metrics["mpc_solves"]==1
            assert runtime.metrics["mpc_errors"]==0 and runtime.depth is None
            assert tracked["state"] == "TRACKING" and tracked["native_execution"]["backend"] == "lightnav"
            assert tracked["native_execution"]["authorized"], tracked["native_execution"]["blocking_reasons"]
            decision = validate_native_status(tracked,session_id=runtime.session_id,
                                              task_id=runtime.task["task_id"],now_ns=clock.time_ns())
            assert decision["accepted"],decision["reasons"]
            vx,vy,wz = decision["command"]
            assert 0 < vx <= .3 and vy==0 and abs(wz)<=.55
            assert runtime.plan["capture_pose"].tolist()==[1.,2.,.3]
            assert tracked["plan"]["trajectory_odom_m_rad"] and tracked["plan"]["prediction_odom_m_rad"]
            checks["recorded_lightnav_se2_passes_real_mpc_and_native_contract_without_depth"] = True

            client = FakeClient()
            executor = NativeImageGoalExecutor(client,session_id=runtime.session_id,task_id=runtime.task["task_id"],
                        fetch_status=runtime.status,check_feedback=lambda:None,
                        wall_clock=clock.time_ns,mono_clock=clock.monotonic_ns)
            executor.arm(tracked)
            assert executor.send_once() and client.calls[-1]==("move",decision["command"])
            checks["lightnav_mpc_command_reaches_fake_native_transport"] = True

            held = _inject(runtime,clock,response,request_id="offline-lightnav-stop",model_stop=True)
            assert held["task"]["model_stop_latched"] and not held["native_execution"]["authorized"]
            assert held["state"] == "SUCCEEDED" and not manager.running
            assert held["task"]["outcome"]["reason"] == "official_lightnav_model_stop"
            assert held["task"]["outcome"]["arrival_independently_validated"] is False
            assert not executor.submit_status(held) and not executor.send_once()
            checks["official_lightnav_stop_completes_task_releases_executor_without_independent_arrival_claim"] = True

            epoch = runtime.epoch
            stopped = runtime.stop_native({"expected_session_id":runtime.session_id,"expected_task_id":runtime.task["task_id"]})
            assert stopped["state"] == "SUCCEEDED" and runtime.epoch>epoch and not manager.running
            assert not executor.submit_status(stopped)
            assert client.calls[-1] == ("authority",False) and not executor.send_once()
            executor.close()
            checks["explicit_stop_cancels_lightnav_signals_owned_manager_and_releases_executor"] = True

            # A queued Start from before Stop cannot create another process.
            try:
                runtime.start_native(request,runtime_url="http://offline.invalid:8093")
                raise AssertionError("stale LightNav Start crossed Stop's epoch barrier")
            except ValueError as exc:
                assert "expected_epoch" in str(exc)
            assert len(manager.starts)==1 and runtime.state == "SUCCEEDED"
            checks["stale_lightnav_start_cannot_cross_stop_epoch"] = True
            epoch = runtime.epoch
            try:
                runtime.start_native(_request(runtime,goal,"unregistered"),runtime_url="http://offline.invalid:8093")
                raise AssertionError("unknown backend created a native task")
            except ValueError as exc:
                assert "backend" in str(exc)
            assert len(manager.starts)==1 and runtime.epoch==epoch
            checks["unknown_native_backend_rejected_before_task_or_process_change"] = True

            for backend in ("nomad","lightnav"):
                shadow = ImageGoalRuntime(Path(temporary)/("shadow-"+backend),_config())
                runtimes.append(shadow)
                shadow.create_task({**_request(shadow,goal,backend),"execution_mode":"shadow"})
                assert not shadow.status()["native_execution"]["authorized"] and shadow.native_executor is None
            checks["both_backends_remain_shadow_without_explicit_native_start"] = True
            default = ImageGoalRuntime(Path(temporary)/"legacy-nomad",_config())
            runtimes.append(default)
            default.native_executor = FakeManager()
            request = _request(default,goal,"nomad")
            del request["backend"]
            assert default.start_native(request,runtime_url="http://offline.invalid:8093")["task"]["backend"]=="nomad"
            checks["legacy_native_start_defaults_to_nomad"] = True
        finally:
            for runtime in runtimes:
                runtime.close()
    assert {name for name in sys.modules if name.startswith("unitree_sdk2py")} == before_sdk
    checks["no_model_http_socket_subprocess_or_robot_sdk"] = True
    return {"schema_version":"imagegoal.lightnav_native_audit.v1","ok":all(checks.values()),"checks":checks,
            "offline_fixture":True,"synthetic_sensor_and_identity_fixture":True,
            "recorded_model_response":str(response_path),"recorded_model_response_sha256":hashlib.sha256(response_path.read_bytes()).hexdigest(),
            "gpu_inference_executed":False,"sdk_initialized":False,"hardware_motion_sent":False,
            "native_candidate_command":{"vx_m_s":vx,"vy_m_s":vy,"wz_rad_s":wz},
            "navigation_success_validated":False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--response-fixture",type=Path,default=DEFAULT_RESPONSE)
    parser.add_argument("--output",type=Path)
    args = parser.parse_args(argv)
    report = audit(args.response_fixture)
    if args.output:
        args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
