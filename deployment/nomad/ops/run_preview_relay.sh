#!/usr/bin/env bash
set -euo pipefail
OPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${NOMAD_ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo "Usage: $0 [ROS arguments]"
  echo "Example: $0 --ros-args -p depth_enabled:=false"
  exit 0
fi
[[ -r "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
set +u
source "$ROS_SETUP"
set -u
exec /usr/bin/python3 "$OPS_DIR/preview_relay.py" "$@"
