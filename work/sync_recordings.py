#!/usr/bin/env python3
"""Read-only pull from Jetson; only finalized, SHA-256-verified sessions publish."""

import argparse
import hashlib
import json
import shutil
import time
import urllib.parse
import urllib.request
from pathlib import Path, PurePosixPath


def fetch_json(url):
    with urllib.request.urlopen(url, timeout=15) as response:
        return json.load(response)


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def safe_path(directory, name):
    part = PurePosixPath(name)
    if part.is_absolute() or ".." in part.parts or not part.parts:
        raise ValueError("Invalid archive path")
    path = directory.joinpath(*part.parts)
    path.resolve().relative_to(directory.resolve())
    return path


def link_inference(directory, root):
    metadata = directory / "metadata.json"
    if not metadata.exists():
        return
    traces = json.loads(metadata.read_text()).get("inference_trace_ids", [])
    links = []
    for trace in traces:
        if len(trace) != 32 or any(c not in "0123456789abcdef" for c in trace):
            continue
        for episode in sorted(
            (root.parent / "recordings-inference" / "episodes").glob(
                "*/vln_client_" + trace + "/episode_*"
            )
        ):
            item = dict(
                trace_id=trace,
                directory=str(episode),
                finalized=(episode / "actions.json").exists(),
            )
            if item["finalized"]:
                actions = json.loads((episode / "actions.json").read_text())
                item["steps"] = len(actions)
            links.append(item)
    temporary = directory / "inference_links.json.tmp"
    temporary.write_text(json.dumps(dict(trace_ids=traces, episodes=links), indent=2))
    temporary.replace(directory / "inference_links.json")


class ExperimentActive(Exception):
    pass


def ensure_idle(base):
    state = fetch_json(base + "/api/health")
    if state.get("recording", {}).get("active") or state.get("vln", {}).get("enabled"):
        raise ExperimentActive("Backup paused while experiment is running")


def sync(base, root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    report = dict(synced=[], failed=[])
    try:
        ensure_idle(base)
    except ExperimentActive:
        print(json.dumps(dict(paused=True, reason="experiment active")), flush=True)
        return
    for item in fetch_json(base + "/api/recordings")["sessions"]:
        name = item["id"]
        if "/" in name or name.startswith("."):
            raise ValueError("Invalid session ID")
        directory = root / name
        directory.mkdir(exist_ok=True)
        try:
            ensure_idle(base)
            data = fetch_json(base + "/recordings/" + urllib.parse.quote(name) + "/manifest.json")
            marker = directory / "sync-complete.json"
            if (
                marker.exists()
                and json.loads(marker.read_text()).get("manifest") == data
                and all(
                    safe_path(directory, e["path"]).is_file()
                    and safe_path(directory, e["path"]).stat().st_size == e["size"]
                    for e in data["files"]
                )
            ):
                link_inference(directory, root)
                continue
            for entry in data["files"]:
                target = safe_path(directory, entry["path"])
                if (
                    target.is_file()
                    and target.stat().st_size == entry["size"]
                    and sha(target) == entry["sha256"]
                ):
                    continue
                if shutil.disk_usage(root).free < entry["size"] + 5 * 1024**3:
                    raise RuntimeError("4090 archive disk would have less than 5 GiB free")
                target.parent.mkdir(parents=True, exist_ok=True)
                temporary = target.with_name(target.name + ".part")
                digest = hashlib.sha256()
                count = 0
                url = (
                    base
                    + "/recordings/"
                    + urllib.parse.quote(name)
                    + "/"
                    + urllib.parse.quote(entry["path"], safe="/")
                )
                last_idle_check = 0.0
                with (
                    urllib.request.urlopen(url, timeout=60) as response,
                    temporary.open("wb") as output,
                ):
                    for block in iter(lambda: response.read(1024 * 1024), b""):
                        if time.monotonic() - last_idle_check >= 2:
                            ensure_idle(base)
                            last_idle_check = time.monotonic()
                        output.write(block)
                        digest.update(block)
                        count += len(block)
                if count != entry["size"] or digest.hexdigest() != entry["sha256"]:
                    raise RuntimeError("Checksum mismatch: " + entry["path"])
                temporary.replace(target)
            (directory / "manifest.json").write_text(json.dumps(data, indent=2))
            temporary = marker.with_suffix(".tmp")
            temporary.write_text(json.dumps(dict(manifest=data, verified=True), indent=2))
            temporary.replace(marker)
            link_inference(directory, root)
            report["synced"].append(name)
        except ExperimentActive:
            report["paused"] = True
            break
        except Exception as exc:
            report["failed"].append(dict(id=name, error=str(exc)))
    print(json.dumps(report, ensure_ascii=False), flush=True)
    if report["failed"]:
        raise RuntimeError("Some recordings could not be synchronized; next timer run retries")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", required=True, help="Robot web URL, e.g. http://ROBOT:8088")
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    sync(args.base.rstrip("/"), args.root)
