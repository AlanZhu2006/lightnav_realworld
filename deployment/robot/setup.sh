#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../common.sh"
lightnav_load_config robot
if [[ ! -f "$LIGHTNAV_ROS_SETUP" ]]; then
    printf 'Install ROS 2 Humble first; setup file missing: %s\n' "$LIGHTNAV_ROS_SETUP" >&2
    exit 1
fi
source "$LIGHTNAV_ROS_SETUP"
python_bin="${LIGHTNAV_ROBOT_SETUP_PYTHON:-/usr/bin/python3.10}"
"$python_bin" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Robot requires ROS Humble Python 3.10"'
"$python_bin" -m venv --system-site-packages "$LIGHTNAV_ROBOT_VENV"
"$LIGHTNAV_ROBOT_VENV/bin/python" -m pip install --upgrade pip
"$LIGHTNAV_ROBOT_VENV/bin/python" -m pip install -r "$LIGHTNAV_REPO_ROOT/deployment/robot/requirements.txt"
if [[ ! -f "$LIGHTNAV_REPO_ROOT/config/robot.env" ]]; then
    cp "$LIGHTNAV_REPO_ROOT/config/robot.env.example" "$LIGHTNAV_REPO_ROOT/config/robot.env"
fi
printf 'Robot Python ready. Install SDK2 and ROS dependencies from docs/REALWORLD_ROBOT.md, then build.\n'
