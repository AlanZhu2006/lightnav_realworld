"""Low-rate host diagnostics and reproducibility snapshots, off the ROS thread."""

import hashlib
import json
import os
import platform
import shutil
import subprocess
import threading
import time
from pathlib import Path


def command(args, cwd=None, timeout=3):
    try:
        result = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout)
        return {
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr[:2000],
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def snapshot(directory, repo, model=None, params_ready=None):
    directory = Path(directory)
    repo = Path(repo)
    directory.mkdir(parents=True, exist_ok=True)
    info = {
        "wall_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "host": platform.node(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "git_head": command(["git", "rev-parse", "HEAD"], repo),
        "git_status": command(["git", "status", "--porcelain", "--untracked-files=no"], repo),
        "clock": command(["timedatectl", "show", "-p", "NTPSynchronized", "-p", "Timezone"]),
        "power_mode": command(["nvpmodel", "-q"]),
        "environment": {
            k: os.environ[k]
            for k in [
                "RMW_IMPLEMENTATION",
                "ROS_DOMAIN_ID",
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                "CUDA_VISIBLE_DEVICES",
                "OMP_NUM_THREADS",
            ]
            if k in os.environ
        },
    }
    diff = command(
        [
            "git",
            "diff",
            "HEAD",
            "--",
            "src",
            "robot_deploy",
            "deployment",
            "work",
            "pyproject.toml",
            "robot-env.sh",
        ],
        repo,
    )
    (directory / "working_tree.patch").write_text(diff.get("stdout", ""))
    info["diff_error"] = diff.get("error", diff.get("stderr"))
    files = []
    for base in ["src", "robot_deploy/src", "robot_deploy/scripts", "config", "deployment", "work"]:
        folder = repo / base
        if folder.exists():
            files.extend(
                p
                for p in folder.rglob("*")
                if p.is_file()
                and p.suffix
                in {
                    ".py",
                    ".yaml",
                    ".yml",
                    ".xml",
                    ".json",
                    ".toml",
                    ".js",
                    ".html",
                    ".css",
                    ".sh",
                    ".example",
                    ".txt",
                }
                and "__pycache__" not in p.parts
            )
    files.extend(
        p
        for p in [repo / "work/serve.sh", repo / "robot-env.sh", repo / "pyproject.toml"]
        if p.is_file()
    )
    info["source_files"] = []
    for src in sorted(set(files)):
        relative = src.relative_to(repo)
        target = directory / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, target)
        info["source_files"].append({"path": str(relative), "sha256": digest(target)})
    if model:
        info["model_files"] = []
        for src in sorted(Path(model).rglob("*")):
            if not src.is_file() or ".cache" in src.parts or ".git" in src.parts:
                continue
            info["model_files"].append(
                {
                    "path": str(src.relative_to(model)),
                    "size": src.stat().st_size,
                    "sha256": digest(src),
                }
            )
            if src.suffix in {".json", ".yaml", ".yml", ".toml"}:
                target = directory / "model_config" / src.relative_to(model)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(src, target)
    # ROS parameter snapshot is read-only and performed in this background worker.
    if os.environ.get("ROS_DISTRO"):
        if params_ready is not None:
            params_ready.wait()
        info["ros_parameters_wall_ns"] = time.time_ns()
        info["ros_parameters_phase"] = "after_recording" if params_ready is not None else "snapshot"
        info["ros_parameters"] = {}
        for name in ["/vln_client", "/vln_web", "/vln_mpc", "/go2_adapter", "/camera/camera"]:
            result = command(["ros2", "param", "dump", name], timeout=10)
            if result.get("returncode") != 0:
                result = command(["ros2", "param", "dump", name], timeout=10)
            info["ros_parameters"][name] = result
    tmp = directory / "runtime.json.tmp"
    tmp.write_text(json.dumps(info, ensure_ascii=False, indent=2))
    tmp.replace(directory / "runtime.json")
    return info


class HostTelemetry:
    def __init__(self, directory, repo=None, interval=2.0, model=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.repo = repo
        self.model = model
        self.interval = interval
        self.stop_event = threading.Event()
        self.error = None
        self.sensors = list(Path("/sys/class/thermal").glob("thermal_zone*/temp"))
        self.gpu_load = list(Path("/sys/devices").glob("platform/*gpu*/load"))
        self.iw = shutil.which("iw")
        self.nvidia = shutil.which("nvidia-smi")
        self.thread = threading.Thread(target=self.run, name="host-telemetry", daemon=True)
        self.thread.start()
        self.snapshot_thread = None
        if repo:
            self.snapshot_thread = threading.Thread(
                target=self.capture_snapshot, name="runtime-snapshot", daemon=True
            )
            self.snapshot_thread.start()

    def capture_snapshot(self):
        try:
            snapshot(
                self.directory / "provenance",
                self.repo,
                model=self.model,
                params_ready=self.stop_event,
            )
        except Exception as exc:
            self.error = str(exc)
            (self.directory / "provenance_error.txt").write_text(str(exc))

    def sample(self):
        data = {
            "wall_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "host": platform.node(),
            "load_average": os.getloadavg(),
        }
        for name in [
            "stat",
            "meminfo",
            "diskstats",
            "net/dev",
            "net/snmp",
            "pressure/cpu",
            "pressure/io",
            "pressure/memory",
        ]:
            try:
                data[name] = Path("/proc", name).read_text()
            except OSError:
                pass
        data["temperatures_mC"] = {
            str(p): p.read_text().strip() for p in self.sensors if p.exists()
        }
        data["gpu_load_raw"] = {str(p): p.read_text().strip() for p in self.gpu_load if p.exists()}
        data["disk_free_bytes"] = shutil.disk_usage(self.directory).free
        if self.nvidia:
            data["gpu"] = command(
                [
                    self.nvidia,
                    "--query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,temperature.gpu,power.draw,clocks.sm",
                    "--format=csv,noheader,nounits",
                ],
                timeout=1,
            )
        if self.iw:
            data["wifi"] = {
                p.name: command([self.iw, "dev", p.name, "link"], timeout=1)
                for p in Path("/sys/class/net").iterdir()
                if (p / "wireless").exists() or p.name.startswith("wl")
            }
        return data

    def run(self):
        with (self.directory / "system_metrics.jsonl").open("a") as output:
            while not self.stop_event.is_set():
                started = time.monotonic()
                try:
                    row = self.sample()
                except Exception as exc:
                    row = {"wall_ns": time.time_ns(), "sample_error": str(exc)}
                output.write(json.dumps(row, ensure_ascii=False) + "\n")
                output.flush()
                self.stop_event.wait(max(0.1, self.interval - (time.monotonic() - started)))

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=5)
        if self.snapshot_thread:
            self.snapshot_thread.join(timeout=120)
        return {
            "sampler_finished": not self.thread.is_alive(),
            "snapshot_finished": self.snapshot_thread is None
            or not self.snapshot_thread.is_alive(),
            "error": self.error,
            "interval_s": self.interval,
        }


if __name__ == "__main__":
    import argparse
    import signal

    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True)
    parser.add_argument("--repo")
    parser.add_argument("--model")
    parser.add_argument("--snapshot-only", action="store_true")
    args = parser.parse_args()
    if args.snapshot_only:
        snapshot(args.directory, args.repo, args.model)
    else:
        monitor = HostTelemetry(args.directory, repo=args.repo, model=args.model)
        signal.signal(signal.SIGTERM, lambda *a: monitor.stop_event.set())
        signal.signal(signal.SIGINT, lambda *a: monitor.stop_event.set())
        monitor.stop_event.wait()
        monitor.close()
