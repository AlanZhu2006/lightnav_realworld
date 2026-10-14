"""Workstation entrypoint for the shared, ROS-independent host diagnostics."""

import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_deploy/src/vln_web"))

if __name__ == "__main__":
    runpy.run_module("vln_web.observability", run_name="__main__")
