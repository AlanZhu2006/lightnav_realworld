"""Exercise slow HTTP target bodies against Stop; no sensor/model/SDK calls."""
from __future__ import annotations
import argparse
import http.client
import json
from pathlib import Path
import threading

from deployment.nomad.shadow_client import encode_rgb_jpeg,request_json
from .http_api import RuntimeHTTPServer
from .runtime import ImageGoalRuntime,RuntimeConfig,atomic_json,json_bytes


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(argv)
    runtime=ImageGoalRuntime(args.output,RuntimeConfig())
    server=RuntimeHTTPServer(("127.0.0.1",0),runtime)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f"http://127.0.0.1:{server.server_port}"
    encoded,_=encode_rgb_jpeg(args.goal)
    def post(path,value):return request_json(base+path,timeout_s=2,payload=value)[0]
    def slow_post(path,value):
        body=json_bytes(value)
        connection=http.client.HTTPConnection("127.0.0.1",server.server_port,timeout=3)
        connection.putrequest("POST",path)
        connection.putheader("Content-Type","application/json")
        connection.putheader("Content-Length",str(len(body)))
        connection.endheaders();connection.send(body[:32])
        return connection,body[32:]
    report={"not_for_motion":True,"sdk_initialized":False,"checks":{}}
    try:
        original=runtime.status()
        connection,remaining=slow_post("/tasks",{"goal":{"rgb_jpeg_base64":encoded},"expected_epoch":original["epoch"]})
        stopped=post("/task/cancel",{"reason":"stop_before_body_arrived"})
        connection.send(remaining);response=connection.getresponse();detail=json.loads(response.read());connection.close()
        after=runtime.status()
        report["checks"]["idle_stop_advances_epoch"]=stopped["epoch"]>original["epoch"]
        report["checks"]["late_create_rejected"]=response.status==400 and "expected_epoch" in detail["error"]
        report["checks"]["late_create_never_revives_task"]=after["task"] is None and after["state"]=="IDLE"
        task=post("/tasks",{"goal":{"rgb_jpeg_base64":encoded},"expected_epoch":after["epoch"]})
        connection,remaining=slow_post("/task/goal",{"goal":{"rgb_jpeg_base64":encoded},"expected_epoch":task["epoch"]})
        stopped=post("/task/cancel",{"reason":"stop_before_old_goal_body"})
        fresh=post("/tasks",{"goal":{"rgb_jpeg_base64":encoded},"expected_epoch":stopped["epoch"]})
        connection.send(remaining);response=connection.getresponse();detail=json.loads(response.read());connection.close()
        after=runtime.status()
        report["checks"]["late_goal_rejected"]=response.status==400 and "expected_epoch" in detail["error"]
        report["checks"]["late_goal_cannot_modify_new_task"]=(after["task"]["task_id"]==fresh["task"]["task_id"] and after["task"]["goal_version"]==1)
        post("/task/cancel",{"reason":"cancel_barrier_commissioning_complete"})
        report["checks"]["final_zero"]=all(runtime.status()["proposal"][key]==0 for key in ("vx_m_s","vy_m_s","wz_rad_s"))
        report["ok"]=all(report["checks"].values())
    finally:
        runtime.close();server.shutdown();server.server_close();thread.join(timeout=2)
        atomic_json(args.output/"cancel_barrier_report.json",report)
    print(json.dumps(report,indent=2))
    return 0 if report.get("ok") else 1


if __name__=="__main__":raise SystemExit(main())
