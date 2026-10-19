#!/usr/bin/env python3
"""Verify the portable source/artifact manifest without loading models/devices."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    manifest = json.loads((root / "deployment/go2-source-manifest.json").read_text())
    count = 0
    for item in [*manifest["files"], *manifest.get("generated_or_release_files", [])]:
        path = root / item["path"]
        if path.is_symlink() or not path.is_file():
            raise SystemExit(f"Missing or symlinked snapshot member: {item['path']}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
            raise SystemExit(f"Snapshot member changed: {item['path']}")
        count += 1
    # policy verifies both the fork source manifest and the independently pinned
    # official hash map. This import does not initialize Torch, CUDA or a device.
    from deployment.lightnav.policy import verify_sources

    verify_sources()
    print(json.dumps({"ok": True, "snapshot_members": count, "gpu_or_robot_started": False}))


if __name__ == "__main__":
    main()
