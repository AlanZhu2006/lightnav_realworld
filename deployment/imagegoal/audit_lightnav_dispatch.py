"""CPU-only actual runtime dispatch checks with fake HTTP, no robot or model."""
from __future__ import annotations

import base64
import hashlib
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from PIL import Image

from . import runtime as runtime_module
from .runtime import ImageGoalRuntime, RuntimeConfig


def jpeg(color, size=(1920, 1080)):
    output = io.BytesIO()
    Image.new("RGB", size, color).save(output, "JPEG", quality=95)
    return output.getvalue()


def session_status_fixture(url, *, timeout_s, payload):
    """Fake only the metadata admission boundary; image calls are separately audited."""
    if not url.endswith("/session/status") or set(payload) != {"identity"}:
        raise AssertionError("Expected an image-free session preflight")
    return {"transport_protocol":"lightnav.http.lifecycle.v2", "server_boot_id":"fixture-server",
            "identity":dict(payload["identity"]), "retired":False, "exists":False,
            "inference_busy":False, "active_session_count":0, "max_sessions":2},1.


class RuntimeDispatchTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.runtimes = []
        self.sdk_before = {name for name in sys.modules if name.startswith("unitree_sdk")}
        self.sockets = patch.object(socket,"socket",side_effect=AssertionError("offline audit must not use sockets"))
        self.processes = patch("subprocess.Popen",side_effect=AssertionError("offline audit must not start a process"))
        self.sockets.start()
        self.processes.start()

    def tearDown(self):
        try:
            for runtime in self.runtimes:
                runtime.close()
            self.assertEqual(self.sdk_before,{name for name in sys.modules if name.startswith("unitree_sdk")})
        finally:
            self.processes.stop()
            self.sockets.stop()
            self.directory.cleanup()

    def build(self, backend):
        runtime = ImageGoalRuntime(Path(self.directory.name)/backend,
            RuntimeConfig(profile="simulated-replay", nomad_url="http://offline.invalid:18882",
                          lightnav_url="http://offline.invalid:18884"))
        self.runtimes.append(runtime)
        runtime._schedule_lightnav_cleanup = Mock()
        now = time.time_ns()
        original_goal = jpeg((120,80,30))
        goal = {"rgb_jpeg_base64":base64.b64encode(original_goal).decode()}
        with patch.object(runtime_module.time,"time_ns",return_value=now-2_000_000_000):
            runtime.create_task({"backend":backend,"timeout_s":60,
                **({"task_mode":"objnav","instruction":"Go to the door and stop.","expected_epoch":runtime.epoch}
                   if backend=="lightnav" else {"goal":goal})})
        with patch.object(runtime_module.time,"time_ns",return_value=now):
            for index in range(4):
                stamp = now-(3-index)*250_000_000
                runtime.ingest_rgb({"frame_id":f"synthetic-{index}",
                    "rgb_jpeg_base64":base64.b64encode(jpeg((30+index*20,80,120))).decode(),
                    "capture_timestamp_ns":stamp,"clock_domain":"unix_epoch_ns",
                    "timestamp_source":"synthetic_dispatch_source","frame_name":"synthetic_rgb",
                    "synthetic":True,"provenance":{"recorded_for":"offline audit","ordinal":index}})
            runtime.ingest_odom({"capture_timestamp_ns":now,"clock_domain":"unix_epoch_ns",
                "timestamp_source":"synthetic_dispatch_odom","frame_name":"synthetic_odom",
                "synthetic":True,"pose_m_rad":[1.,2.,.3],"velocity_v_w":[0.,0.]})
        return runtime, original_goal

    def dispatch(self, runtime, frames=None):
        # The actual dispatcher builds, records, and submits the payload. Only
        # the HTTP boundary is faked; the model result is never accepted here.
        boundary = runtime.lightnav_http if runtime.task["backend"]=="lightnav" else runtime_module
        expected={"offline_response":True}
        if runtime.task["backend"]=="lightnav":
            expected["server_boot_id"]="fixture-server"
        with patch.object(runtime_module.LightNavHTTPClient,"request_json",side_effect=session_status_fixture), \
             patch.object(boundary,"request_json",return_value=(expected,1.)) as http:
            runtime._dispatch(list(runtime.frames) if frames is None else frames)
            metadata,payload,response,rtt,error = runtime.completed.get(timeout=2)
            http.assert_called_once()
            called_url = http.call_args.args[0]
        self.assertIsNone(error)
        self.assertEqual(response,expected)
        record = json.loads((runtime.output/"requests"/(metadata["request_id"]+".request.json")).read_text())
        return payload, record, called_url

    def assert_original_recordings(self, runtime, request, goal):
        if runtime.task["backend"] == "lightnav":
            self.assertIsNone(request["goal_path"])
            self.assertIsNone(runtime.task["selection_id"])
            instruction = json.loads((runtime.output/runtime.task["instruction_path"]).read_text())
            self.assertEqual(instruction["instruction"],runtime.task["instruction"])
            self.assertFalse(instruction["goal_image_consumed"])
        else:
            self.assertEqual(runtime.goal_jpeg,goal)
            self.assertEqual((runtime.output/request["goal_path"]).read_bytes(),goal)
        self.assertEqual(request["capture_pose"],[1.,2.,.3])
        for source, saved in zip(runtime.frames,request["frames"]):
            self.assertEqual((runtime.output/saved["stored_path"]).read_bytes(),source.jpeg)
            self.assertEqual(Path(saved["stored_path"]).name,hashlib.sha256(source.jpeg).hexdigest()+".jpg")
            self.assertEqual(saved["frame_id"],source.frame_id)
            self.assertEqual(saved["capture_timestamp_ns"],source.stamp)
            self.assertEqual(saved["provenance"],source.provenance)
            self.assertEqual(saved["timestamp_source"],source.metadata["timestamp_source"])

    def test_lightnav_dispatch_compresses_only_observations_and_records_both_lineages(self):
        runtime, goal = self.build("lightnav")
        original_wires = [source.wire() for source in runtime.frames]
        payload, request, url = self.dispatch(runtime)
        self.assertEqual(url,"http://offline.invalid:18884/infer")
        self.assertNotIn("goal",payload)
        self.assertEqual(payload["conditioning_mode"],"native_language")
        self.assertEqual(payload["task_mode"],"objnav")
        self.assertEqual([source.wire() for source in runtime.frames],original_wires)
        self.assert_original_recordings(runtime,request,goal)
        transport = request["observation_transport"]
        self.assertEqual(transport["schema_version"],"lightnav.observation_transport.v2")
        self.assertEqual(transport["cache_hits"],0)
        self.assertGreater(transport["source_jpeg_bytes"],transport["wire_jpeg_bytes"])
        for source, wire, audit in zip(runtime.frames,payload["observations"],transport["frames"]):
            encoded = base64.b64decode(wire["rgb_jpeg_base64"])
            with Image.open(io.BytesIO(encoded)) as image:
                self.assertEqual(image.size,(480,270))
            self.assertEqual(wire["frame_id"],source.frame_id)
            self.assertEqual(wire["capture_timestamp_ns"],source.stamp)
            self.assertEqual(audit["source_jpeg_sha256"],hashlib.sha256(source.jpeg).hexdigest())
            self.assertEqual(audit["wire_jpeg_sha256"],hashlib.sha256(encoded).hexdigest())
            self.assertEqual(audit["source_metadata"],source.metadata)
            self.assertEqual(audit["source_provenance"],source.provenance)
            self.assertTrue(wire["rgb_transport"]["lossy"])

    def test_lightnav_repeated_dispatch_preserves_overlap_wire_and_goal(self):
        runtime, goal = self.build("lightnav")
        first, _, _ = self.dispatch(runtime)
        second, request, _ = self.dispatch(runtime,list(runtime.frames)[-2:])
        self.assertEqual(second["observations"],first["observations"][-2:])
        self.assertNotIn("goal",second)
        self.assertEqual(second["instruction"],first["instruction"])
        self.assertEqual(request["observation_transport"]["cache_hits"],2)
        self.assertEqual(second["conditioning_mode"],"native_language")

    def test_nomad_dispatch_preserves_all_original_observation_and_goal_bytes(self):
        runtime, goal = self.build("nomad")
        original_wires = [source.wire() for source in runtime.frames]
        # NoMaD must never enter the LightNav derivative encoder.
        runtime.lightnav_transport.observations = Mock(side_effect=AssertionError("NoMaD entered LightNav transport"))
        payload, request, url = self.dispatch(runtime)
        self.assertEqual(url,"http://offline.invalid:18882/infer")
        self.assertEqual(payload["observations"],original_wires)
        self.assertNotIn("observation_transport",request)
        self.assertNotIn("instruction",payload)
        self.assertEqual(base64.b64decode(payload["goal"]["rgb_jpeg_base64"]),goal)
        self.assert_original_recordings(runtime,request,goal)

    def test_ingress_prefetch_only_runs_for_active_running_lightnav_outside_runtime_lock(self):
        runtime, _ = self.build("lightnav")
        newest = runtime.frames[-1]
        def assert_unlocked(source):
            self.assertFalse(runtime.lock._is_owned(),"prefetch retained the runtime/task lock")
            self.assertIs(source,runtime.frames[-1])
            return True
        runtime.lightnav_transport.prefetch = Mock(side_effect=assert_unlocked)
        task = runtime.task
        cases = [
            ("active_lightnav",True,"lightnav",False,False,"WAITING_INPUT",True),
            ("nomad",True,"nomad",False,False,"WAITING_INPUT",False),
            ("paused",True,"lightnav",True,False,"PAUSED",False),
            ("model_stop",True,"lightnav",False,True,"MODEL_STOP",False),
            ("finished",True,"lightnav",False,False,"CANCELLED",False),
            ("thread_not_started",False,"lightnav",False,False,"WAITING_INPUT",False),
            ("no_task",True,"lightnav",False,False,"IDLE",False),
        ]
        for index,(name,alive,backend,paused,stopped,state,expected) in enumerate(cases,1):
            with self.subTest(name=name):
                runtime.task = None if name=="no_task" else task
                task.update(backend=backend,paused=paused,model_stop_latched=stopped)
                runtime.state = state
                runtime.lightnav_transport.prefetch.reset_mock()
                stamp = newest.stamp+index*1_000_000
                with patch.object(runtime.thread,"is_alive",return_value=alive), \
                     patch.object(runtime_module.time,"time_ns",return_value=stamp):
                    runtime.ingest_rgb({**newest.metadata,"frame_id":"guard-"+name,
                        "capture_timestamp_ns":stamp,
                        "rgb_jpeg_base64":base64.b64encode(newest.jpeg).decode(),
                        "provenance":newest.provenance})
                self.assertEqual(runtime.lightnav_transport.prefetch.call_count,int(expected))


if __name__ == "__main__":
    unittest.main()
