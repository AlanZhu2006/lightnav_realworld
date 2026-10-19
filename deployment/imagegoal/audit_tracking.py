"""CPU integration checks for language-conditioned LightNav tracking.

RGB, odometry, model outputs, clocks and robot transport are explicit fixtures.
The real runtime, original raw-waypoint MPC and native execution validator run;
no model, HTTP service, subprocess, robot SDK or physical motion is started.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import socket
import sys
import tempfile
from unittest.mock import patch

from . import runtime as runtime_module
from .runtime import ImageGoalRuntime
from .audit_lightnav_native import _config, _goal, _seed
from .audit_lightnav_dispatch import session_status_fixture
from .audit_native_runtime import FakeManager, RecordedClock
from deployment.go2.audit_native_imagegoal import FakeClient
from deployment.go2.native_imagegoal import (
    NATIVE_SOURCES, NativeImageGoalExecutor, validate_native_status,
)


INSTRUCTION = "Follow the person wearing a white T-shirt."


def _request(runtime, **changes):
    return {
        "expected_session_id": runtime.session_id,
        "expected_epoch": runtime.epoch,
        "backend": "lightnav", "task_mode": "tracking",
        "instruction": INSTRUCTION, "timeout_s": 60, "operator_present": True,
        **changes,
    }


def _fresh(runtime, clock, image, index):
    clock.wall_ns += 250_000_000
    common = {"capture_timestamp_ns": clock.time_ns(), "clock_domain": "unix_epoch_ns",
              "synthetic": False, "commissioning": False}
    frame, source = NATIVE_SOURCES["rgb"]
    runtime.ingest_rgb({**common, **image, "frame_id": f"tracking-fixture-{index}",
                        "frame_name": frame, "timestamp_source": source,
                        "provenance": {"offline_fixture": True}})
    frame, source = NATIVE_SOURCES["odom"]
    runtime.ingest_odom({**common, "frame_name": frame, "timestamp_source": source,
                        "body_frame_name": "base_link", "pose_m_rad": [1., 2., .3],
                        "velocity_v_w": [0., 0.]})
    clock.wall_ns += 20_000_000


def _completion(runtime, clock, *, request_id, visible=True, model_stop=False,
                zero=False, mode="native_tracking"):
    frames = list(runtime.frames)
    identity = {"session_id": runtime.session_id, "task_id": runtime.task["task_id"],
                "goal_id": runtime.task["goal_id"], "goal_version": runtime.task["goal_version"],
                "frame_id": frames[-1].frame_id}
    observations = [frame.wire() for frame in frames]
    payload = {"identity": identity, "observations": observations,
               "task_mode": "tracking", "conditioning_mode": mode,
               "instruction": INSTRUCTION}
    metadata = {"request_id": request_id, "identity": identity, "epoch": runtime.epoch,
                "backend": "lightnav", "task_mode": "tracking",
                "source_stamp_ns": frames[-1].stamp,
                "created_monotonic_ns": clock.monotonic_ns()-50_000_000,
                "capture_pose": [1., 2., .3], "capture_odom_gap_s": 0.,
                "offline_fixture": True}
    trajectory = [[0. if zero or model_stop else .1*(index+1), 0., 0.] for index in range(10)]
    source_frames = [{"frame_id": frame.frame_id, "capture_timestamp_ns": frame.stamp}
                     for frame in frames]
    response = {"schema_version": "lightnav.imagegoal.inference.v1", "identity": identity,
                "observation_frames": source_frames, "source_frame": source_frames[-1],
                "conditioning_mode": mode, "task_mode": "tracking", "goal_image_consumed": False,
                "implementation": "official_release",
                "official_release_commit":runtime_module.OFFICIAL_COMMIT,"eval_task":"trackvla",
                "official_core_sha256":runtime_module.OFFICIAL_CORE_DIGEST,
                "trajectory_body_m_rad": trajectory,
                "trajectory_body_m": [point[:2] for point in trajectory],
                "model_stop": model_stop or zero, "model_visible": visible,
                "arrival_confirmed": False, "policy_history_count": len(frames),
                "inference_ms": 20., "instruction": INSTRUCTION}
    return metadata, payload, response, 50., None


def _inject(runtime, completion):
    runtime.inflight = completion[0]
    runtime.completed.put(completion)
    runtime.tick()
    return runtime.status()


def audit():
    checks, runtimes = {}, []
    clock = RecordedClock(1_800_000_000_000_000_000, source_ns=0, source_mono_ns=10_000_000_000)
    before_sdk = {name for name in sys.modules if name.startswith("unitree_sdk2py")}
    image = _goal()
    with tempfile.TemporaryDirectory(prefix="lightnav-tracking-offline-") as temporary, \
         patch.object(runtime_module, "time", clock), \
         patch.object(runtime_module, "request_json", side_effect=AssertionError("offline audit must not use HTTP")), \
         patch.object(ImageGoalRuntime, "_schedule_lightnav_cleanup"), \
         patch.object(socket, "socket", side_effect=AssertionError("offline audit must not open sockets")), \
         patch("subprocess.Popen", side_effect=AssertionError("offline audit must not start a subprocess")):
        try:
            runtime = ImageGoalRuntime(Path(temporary)/"tracking", _config())
            runtimes.append(runtime)
            dispatched = []
            runtime._dispatch = lambda frames: dispatched.append([frame.frame_id for frame in frames])
            runtime.native_executor = FakeManager()
            initial = runtime.start_native(_request(runtime), runtime_url="http://offline.invalid:8093")
            task_id = initial["task"]["task_id"]
            goal_id = initial["task"]["goal_id"]
            assert initial["task"]["task_mode"] == "tracking"
            assert initial["task"]["instruction"] == INSTRUCTION
            assert runtime.goal_jpeg is None and initial["tracking"]["enabled"]
            assert initial["tracking"]["visible"] is None
            assert goal_id and len(runtime.native_executor.starts) == 1
            checks["native_tracking_starts_without_goal_image_or_frozen_selection"] = True

            _seed(runtime, clock, image)
            tracked = _inject(runtime, _completion(runtime, clock, request_id="visible-motion"))
            assert tracked["state"] == "TRACKING" and tracked["tracking"]["visible"] is True
            assert tracked["native_execution"]["authorized"] and runtime.metrics["mpc_errors"] == 0
            decision = validate_native_status(tracked, session_id=runtime.session_id,
                                              task_id=task_id, now_ns=clock.time_ns())
            assert decision["accepted"], decision["reasons"]
            assert 0 < decision["command"][0] <= .3
            checks["visible_tracking_uses_real_mpc_and_existing_native_execution_contract"] = True

            # Exercise the original dispatch method with a fake HTTP boundary.
            # This catches accidental goal-image conditioning before a GPU run.
            _fresh(runtime, clock, image, "wire")
            captured = []

            def fake_http(url, *, timeout_s, payload):
                captured.append(copy.deepcopy(payload))
                response = _completion(runtime, clock, request_id="wire")[2]
                response["server_boot_id"]="fixture-server"
                response["identity"] = payload["identity"]
                source = [{"frame_id": frame["frame_id"], "capture_timestamp_ns": frame["capture_timestamp_ns"]}
                          for frame in payload["observations"]]
                response.update(observation_frames=source, source_frame=source[-1])
                return response, 50.

            with patch.object(runtime_module.LightNavHTTPClient, "request_json", side_effect=session_status_fixture), \
                 patch.object(runtime.lightnav_http, "request_json", side_effect=fake_http):
                ImageGoalRuntime._dispatch(runtime, list(runtime.frames))
                completed = runtime.completed.get(timeout=2.)
            runtime.completed.put(completed)
            runtime.tick()
            tracked = runtime.status()
            assert len(captured) == 1 and "goal" not in captured[0]
            assert captured[0]["conditioning_mode"] == "native_tracking" and captured[0]["task_mode"] == "tracking"
            assert captured[0]["instruction"] == INSTRUCTION and tracked["native_execution"]["authorized"]
            checks["real_dispatch_sends_native_tracking_instruction_and_observations_without_goal_image"] = True

            forged = copy.deepcopy(tracked)
            forged["native_execution"]["target_visible"] = False
            assert validate_native_status(forged, session_id=runtime.session_id,
                                              task_id=task_id, now_ns=clock.time_ns())["accepted"]
            forged["native_execution"]["target_visible"] = True
            forged["native_execution"]["task_mode"] = "imagegoal"
            assert not validate_native_status(forged, session_id=runtime.session_id,
                                              task_id=task_id, now_ns=clock.time_ns())["accepted"]
            forgery_client = FakeClient()
            forgery_executor = NativeImageGoalExecutor(forgery_client, session_id=runtime.session_id, task_id=task_id,
                        fetch_status=runtime.status, check_feedback=lambda: None,
                        wall_clock=clock.time_ns, mono_clock=clock.monotonic_ns)
            forgery_executor.arm(tracked)
            forged["task"]["task_mode"] = "imagegoal"
            assert not forgery_executor.submit_status(forged) and not forgery_executor.send_once()
            forgery_executor.close()
            checks["official_pointing_visibility_is_diagnostic_but_task_mode_mutation_is_rejected"] = True

            client = FakeClient()
            executor = NativeImageGoalExecutor(client, session_id=runtime.session_id, task_id=task_id,
                        fetch_status=runtime.status, check_feedback=lambda: None,
                        wall_clock=clock.time_ns, mono_clock=clock.monotonic_ns)
            executor.arm(tracked)
            assert executor.send_once() and client.calls[-1][0] == "move"

            hold_cases = (
                ({"model_stop": True}, "tracking_model_stop"),
                ({"zero": True}, "tracking_model_stop"),
            )
            for index, (changes, reason) in enumerate(hold_cases):
                _fresh(runtime, clock, image, f"hold-{index}")
                old_epoch = runtime.epoch
                held = _inject(runtime, _completion(runtime, clock, request_id=f"hold-{index}", **changes))
                assert held["state"] == "TRACK_HOLD" and held["tracking"]["hold_reason"] == reason
                assert runtime.plan is None and not any(held["proposal"].get(key, 0) for key in ("vx_m_s", "vy_m_s", "wz_rad_s"))
                assert not held["native_execution"]["authorized"] and not runtime.task["model_stop_latched"]
                assert runtime.epoch > old_epoch and runtime.task["task_id"] == task_id and runtime.task["goal_id"] == goal_id
                assert runtime.task["last_lightnav_ack_stamp_ns"] == runtime.frames[-1].stamp
                assert not executor.submit_status(held) and not executor.send_once()
                assert client.calls[-1] == ("move", (0., 0., 0.))
                before = len(dispatched)
                _fresh(runtime, clock, image, f"observe-{index}")
                runtime.tick()
                assert len(dispatched) > before, "held tracking stopped observing"
                resumed = _inject(runtime, _completion(runtime, clock, request_id=f"resume-{index}"))
                assert resumed["state"] == "TRACKING" and resumed["tracking"]["hold_reason"] is None
                assert resumed["tracking"]["visible"] is True and resumed["native_execution"]["authorized"]
                assert executor.submit_status(resumed) and executor.send_once()
                assert runtime.task["task_id"] == task_id and runtime.task["goal_id"] == goal_id
                checks[reason+"_holds_zero_preserves_history_and_resumes_on_fresh_visible_motion"] = True

            for index, visible in enumerate((False, None)):
                _fresh(runtime, clock, image, f"visibility-{index}")
                observed = _inject(runtime, _completion(runtime, clock, request_id=f"visibility-{index}", visible=visible))
                assert observed["state"] == "TRACKING" and observed["native_execution"]["authorized"]
                assert observed["tracking"]["visible"] is visible
            checks["official_trajectory_execution_does_not_use_extra_pointing_visibility_gates"] = True

            # A stale STOP must not change task state or invalidate a newer plan.
            _fresh(runtime, clock, image, "old-stop")
            old_stop = _completion(runtime, clock, request_id="obsolete-stop", model_stop=True)
            runtime.task_action("pause", {"expected_session_id": runtime.session_id})
            runtime.task_action("resume", {"expected_session_id": runtime.session_id})
            _fresh(runtime, clock, image, "new-after-resume")
            _inject(runtime, _completion(runtime, clock, request_id="new-after-resume"))
            current_plan = runtime.plan
            _inject(runtime, old_stop)
            assert runtime.plan is current_plan and runtime.task["tracking_hold_reason"] is None
            checks["obsolete_tracking_stop_cannot_hold_a_new_generation"] = True

            # Mode mismatches fail closed rather than being interpreted as tracking.
            _fresh(runtime, clock, image, "bad-mode")
            before = runtime.metrics["inference_accepted"]
            _inject(runtime, _completion(runtime, clock, request_id="bad-mode", mode="experimental_reference_image"))
            assert runtime.metrics["inference_accepted"] == before
            checks["reference_image_response_cannot_be_accepted_as_native_tracking"] = True

            # Explicit Stop still cancels and fences all previous tracking results.
            obsolete = _completion(runtime, clock, request_id="before-stop")
            request_before_stop = _request(runtime)
            stopped = runtime.stop_native({"expected_session_id": runtime.session_id, "expected_task_id": task_id})
            assert stopped["state"] == "CANCELLED" and not runtime.native_executor.running
            assert not executor.submit_status(stopped) and not executor.send_once()
            executor.close()
            _inject(runtime, obsolete)
            assert runtime.state == "CANCELLED" and runtime.plan is None
            try:
                runtime.start_native(request_before_stop, runtime_url="http://offline.invalid:8093")
                raise AssertionError("stale tracking Start crossed Stop epoch")
            except ValueError as exc:
                assert "expected_epoch" in str(exc)
            checks["operator_stop_cancels_tracking_and_fences_old_results_and_queued_start"] = True

            switched = runtime.start_native(
                _request(runtime, backend="nomad", task_mode="imagegoal", goal=image),
                runtime_url="http://offline.invalid:8093")
            replacement_id = switched["task"]["task_id"]
            assert replacement_id != task_id and switched["task"]["task_mode"] == "imagegoal"
            accepted_before = runtime.metrics["inference_accepted"]
            _inject(runtime, obsolete)
            assert runtime.task["task_id"] == replacement_id and runtime.plan is None
            assert runtime.metrics["inference_accepted"] == accepted_before
            checks["switch_to_imagegoal_creates_new_task_and_rejects_old_tracking_response"] = True

            # Invalid new-task requests must never supersede an active task.
            validation = ImageGoalRuntime(Path(temporary)/"validation", _config())
            runtimes.append(validation)
            validation._dispatch = lambda frames: None
            validation.create_task({**_request(validation), "execution_mode": "shadow"})
            bound, epoch = validation.task["task_id"], validation.epoch
            invalid = (
                {"instruction": "   "}, {"backend": "nomad"}, {"task_mode": "unsupported"},
                {"goal": image}, {"selection_id": "stale-selection"},
            )
            for changes in invalid:
                try:
                    validation.create_task({**_request(validation, **changes), "execution_mode": "shadow"})
                    raise AssertionError(f"invalid tracking task accepted: {changes.keys()}")
                except ValueError:
                    pass
                assert validation.task["task_id"] == bound and validation.epoch == epoch
            checks["invalid_tracking_requests_do_not_mutate_existing_task_or_generation"] = True

            # NoMaD ImageGoal keeps its goal requirement. Official LightNav
            # defaults to ObjNav language conditioning and rejects image goals.
            legacy = ImageGoalRuntime(Path(temporary)/"legacy", _config())
            runtimes.append(legacy)
            legacy._dispatch = lambda frames: None
            try:
                legacy.create_task({"backend": "nomad"})
                raise AssertionError("ImageGoal accepted no goal image")
            except ValueError:
                pass
            legacy.create_task({"backend": "nomad", "goal": image})
            assert legacy.task["task_mode"] == "imagegoal" and legacy.goal_jpeg is not None
            legacy.create_task({"backend":"lightnav", "instruction":INSTRUCTION, "expected_epoch":legacy.epoch})
            assert legacy.task["task_mode"] == "objnav" and legacy.task["goal_path"] is None
            try:
                legacy.create_task({"backend":"lightnav", "instruction":INSTRUCTION, "goal":image})
                raise AssertionError("Official LightNav accepted a custom image goal")
            except ValueError:
                pass
            checks["nomad_imagegoal_preserved_and_official_lightnav_requires_language_without_goal_image"] = True
        finally:
            for runtime in runtimes:
                runtime.close()
    assert {name for name in sys.modules if name.startswith("unitree_sdk2py")} == before_sdk
    checks["no_gpu_model_http_socket_subprocess_robot_sdk_or_hardware_motion"] = True
    return {"schema_version": "imagegoal.tracking_audit.v1", "ok": all(checks.values()),
            "checks": checks, "offline_fixture": True, "synthetic_sensor_and_model_output_fixture": True,
            "real_runtime_mpc_and_native_validator": True, "gpu_inference_executed": False,
            "sdk_initialized": False, "hardware_motion_sent": False,
            "navigation_or_tracking_success_validated": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = audit()
    text = json.dumps(result, indent=2, ensure_ascii=False)+"\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end="")


if __name__ == "__main__":
    main()
