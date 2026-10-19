#!/usr/bin/env python3
"""Import only the portable Go2 source snapshot; never copy environments or data."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path

DIRECTORIES = ("lightnav", "imagegoal", "go2", "bringup", "nomad", "operator", "sensors", "maxwell")
OMIT_PARTS = {".venv", ".runtime", "__pycache__", "node_modules", "dist", "downloads"}
OMIT_NAMES = {"dashboard.js", "fixture-status.json", "live-rgb.jpg", "live-goal.jpg"}
SOURCE_SUFFIXES = {".py", ".sh", ".md", ".json", ".yaml", ".yml", ".txt", ".toml", ".ts", ".tsx", ".js", ".mjs", ".css", ".html"}
DOCUMENTS = (
    "DEJAVIEW_JETSON_GPU_RGBD_20261017.md",
    "DEJAVIEW_NOMAD_BRINGUP_20261017.md",
    "DEJAVIEW_GO2_NOMAD_DEPLOYMENT_PLAN_20261017.md",
    "DEJAVIEW_GO2_LIGHTNAV_DEPLOYMENT_PLAN_20261015.md",
    "DEJAVIEW_LIGHTNAV_IMAGEGOAL_20261017.md",
)


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def allowed(path: Path) -> bool:
    return (not any(part in OMIT_PARTS or part.startswith(".venv-") for part in path.parts)
            and path.name not in OMIT_NAMES and not path.name.startswith("preview-")
            and path.suffix not in {".foxe", ".pyc", ".pyo", ".png", ".jpg", ".jpeg"}
            and (path.suffix in SOURCE_SUFFIXES or path.name.startswith("LICENSE") or path.name in {"Dockerfile", "NOTICE"})
            and not any(part.startswith('.') for part in path.parts)
            and path.as_posix() not in {"deployment/maxwell/AGENTS.md", "deployment/maxwell/README.md", "deployment/maxwell/prepare_snapshot.py"})


def portable_text(relative: Path, data: bytes) -> tuple[bytes, list[str]]:
    """Generalize deployment identities; vendored upstream bytes stay untouched."""
    if "vendor" in relative.parts:
        return data, []
    try:
        value = data.decode("utf-8")
    except UnicodeDecodeError:
        return data, []
    changes = []
    substitutions = {
        r"\b[\w-]+-go2\b": "go2",
        r"\bunitree@(?:\d{1,3}\.){3}\d{1,3}\b": "go2",
        r"/home/unitree/work/DejaView": "/home/unitree/work/lightnav_realworld",
        r"~/work/DejaView": "~/work/lightnav_realworld",
        r"/local_data/[A-Za-z0-9_-]+": "/local_data/example-user",
        r"\b[a-z][a-z0-9_-]+@(?:\d{1,3}\.){3}\d{1,3}\b": "${MAXWELL_SSH_DESTINATION:?Set MAXWELL_SSH_DESTINATION to your SSH destination}",
    }
    for pattern, replacement in substitutions.items():
        updated = re.sub(pattern, lambda match: replacement, value)
        if updated != value:
            value = updated
            changes.append("deployment identity/path replaced with portable example")
    if relative.as_posix() == "deployment/lightnav/launcher.py":
        value = value.replace('maxwell_root = Path("/local_data/example-user")', 'maxwell_root = Path("/local_data")')
        changes.append("shared-server cold-start protection covers every /local_data user")
    if relative.suffix == ".md":
        value = re.sub(r"/home/[^/]+/workspace/[^\s`\"')]+", "/path/to/local-reference", value)
        changes.append("local workstation examples generalized")
    return value.encode(), changes


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    target = Path(__file__).resolve().parents[1]
    paths = []
    for directory in DIRECTORIES:
        for parent, children, files in os.walk(source / "deployment" / directory):
            children[:] = [name for name in children if name not in OMIT_PARTS and not name.startswith(".venv-")]
            paths.extend((Path(parent) / name).relative_to(source) for name in files)
    paths.extend(Path("checks") / name for name in ("test_lightnav_launcher.py", "test_open_console_launch.py"))
    paths.extend(Path("docs") / name for name in DOCUMENTS)
    paths = sorted(path for path in paths if allowed(path))
    if any((source / path).is_symlink() or (source / path).stat().st_size > 2 * 1024 * 1024 for path in paths):
        raise SystemExit("Source snapshots reject symlinks and files over 2 MiB")
    snapshot = {path: (source / path).read_bytes() for path in paths}
    # Fail if any source changed while taking the snapshot.
    for path, data in snapshot.items():
        if digest((source / path).read_bytes()) != digest(data):
            raise SystemExit(f"Concurrent source change: {path}")
    records = []
    for relative, data in snapshot.items():
        content, changes = portable_text(relative, data)
        destination = target / relative
        if destination.exists():
            raise SystemExit(f"Refusing to overwrite existing file: {relative}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        destination.chmod((source / relative).stat().st_mode & 0o777)
        records.append({"path": relative.as_posix(), "source_sha256": digest(data),
                        "sha256": digest(content), "bytes": len(content), "transformations": changes})
    commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    manifest = {"schema": "lightnav.go2.source_snapshot.v1", "date": "2026-10-20",
                "source_repository": "https://github.com/ai4ce/DejaView", "source_base_commit": commit,
                "source_worktree_dirty": True, "destination_base_commit": "ddfd37059c49e8090217e11c4ffa6f6b3ff9d554",
                "note": "Exact per-file source hashes identify unpublished deployment work; the base commit alone does not identify it.",
                "excluded": sorted(OMIT_PARTS | OMIT_NAMES), "files": records}
    (target / "deployment/go2-source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"files": len(records), "bytes": sum(item["bytes"] for item in records)}))


if __name__ == "__main__":
    main()
