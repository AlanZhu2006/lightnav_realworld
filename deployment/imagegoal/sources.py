"""Explicit recorded-image replay and moving synthetic feedback source."""
from __future__ import annotations

import base64
import io
import json
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image

from deployment.nomad.shadow_client import encode_rgb_jpeg
from .runtime import ImageGoalRuntime


def read_replay_manifest(path: Path) -> list[dict]:
    value = json.loads(path.read_text())
    frames = value.get("frames")
    if not isinstance(frames,list) or len(frames)<4:
        raise ValueError("replay manifest requires at least four distinct real source frames")
    result,seen,previous = [],set(),0
    for frame in frames:
        identifier,stamp = frame.get("frame_id"),frame.get("capture_timestamp_ns")
        if not isinstance(identifier,str) or not identifier or identifier in seen or not isinstance(stamp,int) or isinstance(stamp,bool) or stamp<=previous:
            raise ValueError("replay source identity and timestamps must be unique and chronological")
        filename = (path.parent/frame["path"]).resolve(strict=True)
        encoded,info = encode_rgb_jpeg(filename)
        result.append({"original_frame_id":identifier,"original_capture_timestamp_ns":stamp,
                       "original_timestamp_source":frame.get("timestamp_source",value.get("timestamp_source")),
                       "original_path":str(filename),"original_sha256":info["original_sha256"],"rgb_jpeg_base64":encoded})
        seen.add(identifier)
        previous = stamp
    return result


class ReplaySimulator:
    """Dynamic ideal unicycle; recorded images do not react to simulated pose.

    This exercises scheduler, capture anchoring and controller feedback only.
    It does not create a photorealistic environment or navigation success.
    """
    def __init__(self, runtime: ImageGoalRuntime, manifest: Path, sample_hz: float = 4):
        if runtime.config.profile != "simulated-replay":
            raise ValueError("simulator requires simulated-replay runtime")
        self.runtime,self.frames,self.sample_hz = runtime,read_replay_manifest(manifest),sample_hz
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run,name="imagegoal-explicit-simulator",daemon=True)
        self.pose = np.zeros(3)
        self.velocity = [0.,0.]

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def _run(self):
        last,next_rgb,next_depth,index = time.monotonic(),0.,0.,0
        source = "synthetic_playback_clock_not_sensor_exposure"
        while not self.stop_event.is_set() and not self.runtime.stop_event.is_set():
            now,stamp = time.monotonic(),time.time_ns()
            dt = min(.1,max(0,now-last))
            last = now
            with self.runtime.lock:
                command,faults = dict(self.runtime.proposal),dict(self.runtime.faults)
            self.velocity = [command["vx_m_s"],command["wz_rad_s"]]
            self.pose[0] += dt*self.velocity[0]*np.cos(self.pose[2])
            self.pose[1] += dt*self.velocity[0]*np.sin(self.pose[2])
            self.pose[2] += dt*self.velocity[1]
            common = {"capture_timestamp_ns":stamp,"clock_domain":"unix_epoch_ns","timestamp_source":source,"synthetic":True}
            try:
                if not faults.get("odom_paused"):
                    self.runtime.ingest_odom({**common,"frame_name":"imagegoal_sim_odom","pose_m_rad":self.pose.tolist(),"velocity_v_w":self.velocity})
                if now>=next_rgb:
                    if not faults.get("rgb_paused"):
                        original = self.frames[index%len(self.frames)]
                        self.runtime.ingest_rgb({**common,"frame_name":"imagegoal_sim_camera","frame_id":f"simulation-{index+1:08d}",
                                                 "rgb_jpeg_base64":original["rgb_jpeg_base64"],
                                                 "provenance":{key:value for key,value in original.items() if key!="rgb_jpeg_base64"}})
                        index += 1
                    next_rgb = now+1/self.sample_hz
                if now>=next_depth:
                    if not faults.get("depth_paused"):
                        raw = np.full((48,64),200 if faults.get("obstacle") else 4000,dtype=np.uint16)
                        buffer = io.BytesIO()
                        Image.fromarray(raw).save(buffer,format="PNG")
                        self.runtime.ingest_depth({**common,"frame_name":"imagegoal_sim_camera","depth_png_base64":base64.b64encode(buffer.getvalue()).decode(),
                                                   "scale_m":.001,"aligned_to_rgb":True})
                    next_depth=now+.1
            except Exception as exc:
                with self.runtime.lock:
                    self.runtime._event("simulator_fault",detail=f"{type(exc).__name__}: {exc}")
                    if self.runtime.task:
                        self.runtime.task_action("cancel",{"reason":"simulator_fault"})
                break
            self.stop_event.wait(.02)
