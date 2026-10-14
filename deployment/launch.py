"""Compose both deployment roles and supervise only processes started here."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import uuid

from .configuration import REPO_ROOT, RobotConfig, ServerConfig
from .preflight import check_robot, check_server


def _stop_group(process: subprocess.Popen, timeout: float = 30) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def run_server(config: ServerConfig) -> int:
    name = time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
    runtime = config.root / "runtime" / name
    runtime.mkdir(parents=True)
    ready = runtime / "server.ready"
    monitor_dir = config.recordings / "runtime" / name
    env = config.environment()
    env["LIGHTNAV_RUNTIME_SNAPSHOT"] = str(monitor_dir / "provenance")
    processes = []
    previous_handlers = {}

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, interrupt)
    server = None
    try:
        if config.telemetry:
            processes.append(
                subprocess.Popen(
                    [
                        str(config.python),
                        str(config.root / "work/experiment_observability.py"),
                        "--directory",
                        str(monitor_dir),
                        "--repo",
                        str(config.root),
                        "--model",
                        str(config.model),
                    ],
                    cwd=config.root,
                    env=env,
                    start_new_session=True,
                )
            )
        server = subprocess.Popen(
            config.command(ready), cwd=config.root, env=env, start_new_session=True
        )
        processes.append(server)
        descriptor = dict(
            pid=server.pid,
            ready_file=str(ready),
            runtime=str(runtime),
            telemetry=str(monitor_dir),
            command=config.command(ready),
        )
        temporary = config.root / "runtime/server.json.tmp"
        temporary.write_text(json.dumps(descriptor, indent=2) + "\n")
        temporary.replace(config.root / "runtime/server.json")
        print(json.dumps(descriptor), flush=True)
        return server.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        for process in reversed(processes):
            _stop_group(process)
        ready.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=["server", "robot"])
    parser.add_argument("--dry-run", action="store_true", help="Print the command; starts nothing")
    parser.add_argument("--check", action="store_true", help="Check installation; starts no nodes")
    args = parser.parse_args(argv)
    try:
        if args.role == "server":
            config = ServerConfig.from_env()
            command = config.command(REPO_ROOT / "runtime/SESSION/server.ready")
            if args.dry_run:
                print(json.dumps(dict(role=args.role, command=command), indent=2))
                return 0
            # Select CUDA before Torch is imported by the preflight.
            os.environ["CUDA_VISIBLE_DEVICES"] = config.gpu
            report = check_server(config)
            print(json.dumps(report), flush=True)
            return 0 if args.check else run_server(config)
        config = RobotConfig.from_env()
        if args.dry_run:
            print(json.dumps(dict(role=args.role, command=config.command()), indent=2))
            return 0
        print(json.dumps(check_robot(config)), flush=True)
        if args.check:
            return 0
        os.environ["LIGHTNAV_REPO_ROOT"] = str(config.root)
        os.environ["LIGHTNAV_RECORDINGS_DIR"] = str(config.recordings)
        os.chdir(config.root)
        os.execv(sys.executable, config.command())
    except (ValueError, OSError, ImportError, KeyError) as exc:
        print(f"Deployment preflight failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
