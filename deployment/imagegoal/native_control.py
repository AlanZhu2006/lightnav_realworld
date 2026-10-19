"""Own only the explicit native ImageGoal executor started by this runtime.

No SDK is imported here. Browser/Foxglove Start creates a new task and launches
the independent guarded executor. Stop signals that exact owned child; it
does not kill another experiment or claim a software signal proves standstill.
"""
from __future__ import annotations

import json
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid


class NativeExecutorManager:
    def __init__(self, runtime):
        self.runtime = runtime
        self.lock = threading.RLock()
        self.process = None
        self.record = {"running":False,"state":"not_started","task_id":None,"error":None}
        self.log_handle = None

    def status(self):
        with self.lock:
            result = dict(self.record)
            result["running"] = self.process is not None and self.process.poll() is None
            live_path = self.record.get("live_path")
            if live_path:
                try:
                    live = json.loads(Path(live_path).read_text())
                    if (live.get("schema_version")=="imagegoal.native_executor_live.v1"
                        and live.get("task_id")==self.record.get("task_id")
                        and 0 <= (time.time_ns()-live["timestamp_ns"])/1e9 <= 1):
                        result.update(live=live,state=live.get("state",result["state"]),
                                      error=live.get("error") or result.get("error"),
                                      command_active=live.get("command_active",False))
                except (OSError,ValueError,TypeError,KeyError):
                    pass
            return result

    def start(self, task_id, runtime_url, duration_s):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                raise ValueError("native executor is still running or stopping")
            root = Path(__file__).resolve().parents[2]
            directory = self.runtime.output / "native_executor" / uuid.uuid4().hex
            directory.mkdir(parents=True)
            execution_directory = directory / "execution"
            command = ["bash",str(root/"deployment/go2/run_go2.sh"),"native-executor",
                       "--execute-supervised","--operator-present","--task-id",task_id,
                       "--runtime-url",runtime_url,"--duration-s",str(duration_s),
                       "--output",str(execution_directory)]
            self.log_handle = (directory/"executor.log").open("ab")
            try:
                self.process = subprocess.Popen(command,cwd=root,stdout=self.log_handle,
                                                stderr=subprocess.STDOUT,start_new_session=True)
            except Exception:
                self.log_handle.close()
                self.log_handle = None
                raise
            self.record = {"running":True,"state":"starting","task_id":task_id,
                           "pid":self.process.pid,"error":None,
                           "started_timestamp_ns":time.time_ns(),"report_path":str(execution_directory/"report.json"),
                           "live_path":str(execution_directory/"live.json"),
                           "log_path":str(directory/"executor.log"),"command":command,
                           "physical_standstill_verified":False}
            process,handle = self.process,self.log_handle
            threading.Thread(target=self._watch,args=(process,handle,task_id),
                             name="native-executor-owner",daemon=True).start()
            return self.status()

    def _watch(self, process, handle, task_id):
        code = process.wait()
        handle.close()
        with self.lock:
            if self.process is process:
                self.record.update(running=False,state="stopped" if code==0 else "exited_with_error",
                                   returncode=code,finished_timestamp_ns=time.time_ns())
                if code:
                    self.record["error"] = f"executor exited with code {code}; inspect executor.log/report.json"
        # Never acquire the runtime lock while holding the manager lock.
        with self.runtime.lock:
            if ((self.runtime.task or {}).get("task_id")==task_id
                and self.runtime.state not in {"IDLE","CANCELLED","SUCCEEDED","FAILED","TIMED_OUT"}):
                self.runtime._finish("CANCELLED","native_executor_finished")

    def stop(self, reason="operator_stop"):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.record.update(state="stopping",stop_reason=str(reason),
                                   stop_requested_timestamp_ns=time.time_ns())
                try:
                    self.process.send_signal(signal.SIGTERM)
                except ProcessLookupError:
                    pass
            return self.status()
