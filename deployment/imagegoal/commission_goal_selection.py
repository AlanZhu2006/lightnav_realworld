"""Exercise staged ImageGoal selection and HTTP races without a model or SDK.

The real runtime, HTTP server and pinned MPC dependency are used. Images are
explicitly synthetic software fixtures; no robot, ROS, camera or GPU is opened.
"""
from __future__ import annotations

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
from http.client import HTTPConnection
import io
import json
from pathlib import Path
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from PIL import Image

from .http_api import RuntimeHTTPServer
from .runtime import ImageGoalRuntime, RuntimeConfig, atomic_json, json_bytes


def jpeg(colour: tuple[int,int,int]) -> bytes:
    stream = io.BytesIO()
    Image.new("RGB",(48,32),colour).save(stream,format="JPEG",quality=93)
    return stream.getvalue()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args(argv)
    runtime = ImageGoalRuntime(args.output,RuntimeConfig(profile="simulated-replay"))
    server = RuntimeHTTPServer(("127.0.0.1",0),runtime)
    thread = threading.Thread(target=server.serve_forever,daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    first,second = jpeg((20,70,130)),jpeg((160,90,20))
    encoded = lambda value: base64.b64encode(value).decode("ascii")
    report = {"schema_version":"imagegoal.goal_selection_commissioning.v1",
              "not_for_motion":True,"synthetic_images":True,"sdk_initialized":False,
              "model_initialized":False,"checks":{}}
    checks = report["checks"]
    other = None

    def http(path,value=None):
        data = None if value is None else json_bytes(value)
        request = Request(base+path,data=data,headers={"Content-Type":"application/json"})
        try:
            response = urlopen(request,timeout=3)
        except HTTPError as exc:
            response = exc
        with response:
            body = response.read()
            result = json.loads(body) if response.headers.get_content_type()=="application/json" else body
            return response.status,result,dict(response.headers)

    def mutation():
        status = runtime.status()
        return {"expected_session_id":status["session_id"],"expected_epoch":status["epoch"]}

    frame_number = 0
    def ingest(blob=first,*,stamp=None):
        nonlocal frame_number
        frame_number += 1
        runtime.ingest_rgb({"rgb_jpeg_base64":encoded(blob),"frame_id":f"software-fixture-{frame_number}",
                            "capture_timestamp_ns":stamp or time.time_ns(),"clock_domain":"unix_epoch_ns",
                            "timestamp_source":"synthetic_selection_commissioning","frame_name":"fixture_rgb",
                            "synthetic":True})

    def slow_post(path,value):
        body = json_bytes(value)
        connection = HTTPConnection("127.0.0.1",server.server_port,timeout=3)
        connection.putrequest("POST",path)
        connection.putheader("Content-Type","application/json")
        connection.putheader("Content-Length",str(len(body)))
        connection.endheaders()
        cut = min(32,len(body)-1)
        connection.send(body[:cut])
        return connection,body[cut:]

    try:
        initial = runtime.status()
        checks["initial_live_not_ready"] = initial["goal_selection"]["mode"]=="live" and not initial["goal_selection"]["ready"]
        checks["no_goal_image_before_selection"] = http("/goal.jpg")[0]==404
        checks["start_without_frozen_goal_rejected"] = http("/tasks",{**mutation(),"selection_id":"missing"})[0]==400
        checks["selection_requires_session_and_epoch"] = all(
            http("/goal/clear",value)[0]==400 for value in ({},{"expected_epoch":runtime.epoch},{"expected_session_id":runtime.session_id}))

        # Four chronological frames would permit inference in an active task.
        stamp = time.time_ns()-750_000_000
        for index in range(4):
            ingest(stamp=stamp+index*250_000_000)
        before = runtime.status()
        code,captured,_ = http("/goal/capture",mutation())
        selection = captured["goal_selection"]
        runtime.tick()
        checks["capture_freezes_without_starting"] = (code==200 and captured["task"] is None and captured["state"]=="IDLE"
            and runtime.metrics["inference_started"]==0 and captured["epoch"]>before["epoch"])
        checks["capture_preserves_source_provenance"] = (selection["source"]=="camera" and
            selection["source_frame_id"]=="software-fixture-4" and selection["source_metadata"]["synthetic"] is True)
        code,blob,headers = http("/goal.jpg?selection_id="+selection["selection_id"])
        checks["capture_bytes_and_id_exact"] = (code==200 and blob==first and
            headers.get("X-ImageGoal-Selection-Id")==selection["selection_id"] and selection["sha256"]==hashlib.sha256(first).hexdigest())
        ingest(second)
        checks["new_rgb_never_changes_frozen_goal"] = http("/goal.jpg")[1]==first and http("/frame/latest.jpg")[1]==second
        checks["mismatched_image_id_rejected"] = http("/goal.jpg?selection_id=old-selection")[0]==400
        checks["empty_or_repeated_image_id_rejected"] = all(http(path)[0]==400 for path in
            ("/goal.jpg?selection_id=","/goal.jpg?selection_id="+selection["selection_id"]+"&selection_id=other"))
        checks["selection_persisted_for_export"] = (runtime.output/selection["selection_path"]).read_bytes()==first

        old_epoch = runtime.epoch
        invalid = http("/goal/import",{**mutation(),"goal":{"rgb_jpeg_base64":"invalid!"}})
        checks["invalid_import_preserves_selection"] = (invalid[0]==400 and runtime.epoch==old_epoch and
            runtime.status()["goal_selection"]["selection_id"]==selection["selection_id"])
        invalid = http("/goal/import",{**mutation(),"goal":{"rgb_jpeg_base64":encoded(b"not a JPEG")}})
        checks["non_jpeg_import_is_client_error_and_preserves_selection"] = (invalid[0]==400 and runtime.epoch==old_epoch and
            runtime.status()["goal_selection"]["selection_id"]==selection["selection_id"])
        code,imported,_ = http("/goal/import",{**mutation(),"goal":{"rgb_jpeg_base64":encoded(second)}})
        selection = imported["goal_selection"]
        runtime.tick()
        checks["import_freezes_without_starting"] = (code==200 and selection["source"]=="import" and
            selection["source_metadata"] is None and imported["task"] is None and runtime.metrics["inference_started"]==0)
        code,started,_ = http("/tasks",{**mutation(),"selection_id":selection["selection_id"],"backend":"nomad"})
        task = started["task"]
        checks["start_uses_exact_frozen_content"] = (code==200 and task["selection_id"]==selection["selection_id"] and
            task["goal_sha256"]==selection["sha256"] and (runtime.output/task["goal_path"]).read_bytes()==second)
        checks["selection_and_inline_goal_are_unambiguous"] = http("/tasks",{**mutation(),"selection_id":selection["selection_id"],
            "goal":{"rgb_jpeg_base64":encoded(first)}})[0]==400
        code,stopped,_ = http("/task/cancel",{"reason":"selection_commissioning_stop"})
        checks["stop_preserves_frozen_selection"] = (code==200 and stopped["state"]=="CANCELLED" and
            stopped["goal_selection"]["selection_id"]==selection["selection_id"] and stopped["epoch"]>started["epoch"])

        # Returning LIVE cancels the active task and fences its pending result.
        code,started,_ = http("/tasks",{**mutation(),"selection_id":selection["selection_id"]})
        old_identity = {key:started["task"][key] for key in ("task_id","goal_id","goal_version")}
        metadata = {"request_id":"late-selection-fixture","backend":"nomad","epoch":started["epoch"],
                    "identity":old_identity,"source_stamp_ns":time.time_ns()}
        runtime.inflight = metadata
        code,cleared,_ = http("/goal/clear",mutation())
        runtime.completed.put((metadata,{"identity":old_identity},{},1.,None))
        runtime.tick()
        checks["clear_cancels_and_returns_live"] = (code==200 and cleared["state"]=="CANCELLED" and
            not cleared["goal_selection"]["ready"] and cleared["goal_selection"]["mode"]=="live" and http("/goal.jpg")[0]==404)
        checks["late_inference_cannot_revive_cleared_goal"] = (runtime.metrics["inference_rejected"]==1 and runtime.plan is None and
            runtime.preview is None and all(runtime.proposal[key]==0 for key in ("vx_m_s","vy_m_s","wz_rad_s")))
        checks["cleared_selection_cannot_start"] = http("/tasks",{**mutation(),"selection_id":selection["selection_id"]})[0]==400

        # Stop is independently serviced while the other HTTP body is incomplete.
        for path in ("/goal/import","/goal/capture","/tasks","/goal/clear"):
            ingest(second)
            _,current,_ = http("/goal/import",{**mutation(),"goal":{"rgb_jpeg_base64":encoded(first)}})
            payload = mutation()
            if path=="/goal/import":
                payload["goal"] = {"rgb_jpeg_base64":encoded(second)}
            if path=="/tasks":
                payload["selection_id"] = current["goal_selection"]["selection_id"]
            connection,remaining = slow_post(path,payload)
            _,stopped,_ = http("/task/cancel",{"reason":"stop_while_body_pending"})
            connection.send(remaining)
            response = connection.getresponse()
            detail = json.loads(response.read())
            status = response.status
            connection.close()
            after = runtime.status()
            checks["slow_"+path.strip("/").replace("/","_")+"_cannot_cross_stop"] = (status==400 and "expected_epoch" in detail["error"] and
                after["epoch"]==stopped["epoch"] and after["goal_selection"]["selection_id"]==current["goal_selection"]["selection_id"] and
                after["state"] in {"IDLE","CANCELLED"})

        # Independent panels issuing different choices against one epoch get one winner.
        ingest(second)
        compare = mutation()
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(http,"/goal/capture",compare),pool.submit(http,"/goal/import",
                {**compare,"goal":{"rgb_jpeg_base64":encoded(first)}})]
            outcomes = [future.result() for future in futures]
        checks["concurrent_selection_has_one_cas_winner"] = sorted(item[0] for item in outcomes)==[200,400]
        winner = next(item[1]["goal_selection"] for item in outcomes if item[0]==200)
        checks["concurrent_selection_image_matches_winner"] = hashlib.sha256(http("/goal.jpg")[1]).hexdigest()==winner["sha256"]

        # Legacy explicit start/replace paths remain usable and synchronize selection.
        _,legacy,_ = http("/tasks",{**mutation(),"goal":{"rgb_jpeg_base64":encoded(first)}})
        checks["legacy_start_synchronizes_selection"] = legacy["task"]["selection_id"]==legacy["goal_selection"]["selection_id"] and legacy["goal_selection"]["sha256"]==hashlib.sha256(first).hexdigest()
        _,replaced,_ = http("/task/goal",{**mutation(),"goal":{"rgb_jpeg_base64":encoded(second)}})
        checks["legacy_replace_synchronizes_selection"] = (replaced["task"]["goal_version"]==2 and
            replaced["task"]["selection_id"]==replaced["goal_selection"]["selection_id"] and http("/goal.jpg")[1]==second)
        _,new_selection,_ = http("/goal/import",{**mutation(),"goal":{"rgb_jpeg_base64":encoded(first)}})
        checks["new_selection_cancels_active_task"] = new_selection["state"]=="CANCELLED" and new_selection["goal_selection"]["ready"]

        # A fresh runtime cannot inherit authority or accept the old session.
        old_request = {**mutation(),"selection_id":runtime.goal_selection["selection_id"]}
        other = ImageGoalRuntime(runtime.output/"restart_fixture",RuntimeConfig(profile="simulated-replay"))
        checks["restart_defaults_to_live"] = other.status()["goal_selection"]["mode"]=="live" and other.goal_jpeg is None
        try:
            other.create_task(old_request)
        except ValueError as exc:
            checks["old_session_start_rejected_after_restart"] = "expected_session_id" in str(exc)
        else:
            checks["old_session_start_rejected_after_restart"] = False
        try:
            other.create_task({"expected_session_id":other.session_id,"expected_epoch":other.epoch,
                               "selection_id":old_request["selection_id"]})
        except ValueError as exc:
            checks["old_selection_rejected_in_new_session"] = "selection_id" in str(exc)
        else:
            checks["old_selection_rejected_in_new_session"] = False
        other.select_goal({"expected_session_id":other.session_id,"expected_epoch":other.epoch,
                           "goal":{"rgb_jpeg_base64":encoded(first)}},source="import")
        stale_id = other.goal_selection["selection_id"]
        other.ingest_rgb({"rgb_jpeg_base64":encoded(second),"frame_id":"stale-software-fixture",
                          "capture_timestamp_ns":time.time_ns()-2_000_000_000,"clock_domain":"unix_epoch_ns",
                          "timestamp_source":"synthetic_selection_commissioning","frame_name":"fixture_rgb","synthetic":True})
        try:
            other.select_goal({"expected_session_id":other.session_id,"expected_epoch":other.epoch},source="camera")
        except ValueError as exc:
            checks["stale_capture_preserves_frozen_goal"] = "stale" in str(exc) and other.goal_selection["selection_id"]==stale_id
        else:
            checks["stale_capture_preserves_frozen_goal"] = False
        checks["no_model_inference_or_motion"] = (runtime.metrics["inference_started"]==0 and other.metrics["inference_started"]==0 and
            all(runtime.proposal[key]==0 for key in ("vx_m_s","vy_m_s","wz_rad_s")))
        report["ok"] = all(checks.values())
    finally:
        runtime.close()
        if other is not None:
            other.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        atomic_json(args.output/"goal_selection_report.json",report)
    print(json.dumps({"ok":report.get("ok",False),"checks":checks,"output":str(args.output)},indent=2))
    return 0 if report.get("ok") else 1


if __name__=="__main__":
    raise SystemExit(main())
