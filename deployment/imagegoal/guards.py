"""Verified imports of the unchanged pure GEM depth and latency guards."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_gem_guards():
    root = Path(__file__).parent / "vendor/gem"
    pin = json.loads((root / "SOURCE.json").read_text())
    for name, item in pin["files"].items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError(f"GEM source hash mismatch: {name}")
    control = _load(root / "trajectory_control.py", "_imagegoal_gem_trajectory_control")
    # The original latency file uses this absolute pure-module import. Scope
    # the compatibility name to its load, preserving any preexisting module.
    previous = sys.modules.get("trajectory_control")
    sys.modules["trajectory_control"] = control
    try:
        latency = _load(root / "latency_motion_guard.py", "_imagegoal_gem_latency_guard")
    finally:
        if previous is None:
            sys.modules.pop("trajectory_control", None)
        else:
            sys.modules["trajectory_control"] = previous
    return control, latency
