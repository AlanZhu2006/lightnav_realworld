"""Launch one explicitly assigned GPU; preparation never selects a GPU."""
from __future__ import annotations

import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time


def rows(fields: str, query="gpu"):
    value = subprocess.check_output(
        ["nvidia-smi", f"--query-{query}={fields}", "--format=csv,noheader,nounits"], text=True)
    return [[field.strip() for field in row] for row in csv.reader(value.splitlines()) if row]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gpu", default=os.environ.get("MAXWELL_GPU_ID"),
                        help="one GPU index or UUID, assigned after coordination")
    parser.add_argument("--allow-shared-gpu", action="store_true",
                        default=os.environ.get("MAXWELL_ALLOW_SHARED_GPU") == "1",
                        help="use only after the user explicitly approves sharing this GPU")
    parser.add_argument("--min-free-mib", type=int, default=16000)
    args, server_args = parser.parse_known_args()
    if not args.gpu or "," in args.gpu:
        parser.error("GPU assignment pending: coordinate one GPU/time window; no automatic selection")
    if args.min_free_mib < 1:
        parser.error("--min-free-mib must be positive; default is a conservative 16000 MiB")
    inventory = rows("index,uuid,name,memory.total,memory.used,utilization.gpu")
    matches = [row for row in inventory if args.gpu in row[:2]]
    if len(matches) != 1:
        parser.error("GPU must name exactly one current index or UUID")
    index, uuid, name, total, used, utilization = matches[0]
    free = int(total) - int(used)
    pids = [pid for gpu, pid in rows("gpu_uuid,pid", "compute-apps") if gpu == uuid]
    busy = bool(pids) or int(used) > 2048 or int(utilization) > 5
    if busy and not args.allow_shared_gpu:
        parser.error(f"GPU {index} already has a workload ({used} MiB, {utilization}%, PIDs {pids}); coordinate before sharing")
    if free < args.min_free_mib:
        parser.error(f"GPU {index}: only {free} MiB free; coordinate another GPU/time window")
    base = Path(os.environ["MAXWELL_DATA_ROOT"]).resolve()
    if not base.is_relative_to(Path("/local_data")):
        parser.error("all runtime files must remain under /local_data")
    lockdir = base / "locks"
    lockdir.mkdir(parents=True, exist_ok=True)
    lock = (lockdir / "lightnav.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("this account already has a LightNav instance")
    record = {"timestamp_ns": time.time_ns(), "pid": os.getpid(), "gpu_index": index,
              "gpu_uuid": uuid, "gpu_name": name, "free_mib_before_load": free,
              "existing_compute_pids": pids, "explicit_sharing_flag": args.allow_shared_gpu,
              "single_device": True}
    lock.seek(0)
    lock.truncate()
    lock.write(json.dumps(record) + "\n")
    lock.flush()
    os.set_inheritable(lock.fileno(), True)
    env = {**os.environ, "CUDA_VISIBLE_DEVICES": uuid, "CUDA_DEVICE_ORDER": "PCI_BUS_ID"}
    project = Path(__file__).resolve().parents[2]
    os.chdir(project)
    print(json.dumps({"launch": "LightNav", **record}), flush=True)
    os.execvpe("bash", ["bash", str(project / "deployment/lightnav/run_server.sh"), *server_args], env)


if __name__ == "__main__":
    main()
