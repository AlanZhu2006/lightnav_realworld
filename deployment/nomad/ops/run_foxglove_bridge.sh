#!/usr/bin/env bash
# NoMaD observation launcher; does not start other stack components.
set -euo pipefail

OPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${NOMAD_ROS_SETUP:-/opt/ros/humble/setup.bash}"

case "${1:-}" in
  --help|-h)
    echo "Usage: $0 [--check]"
    echo "Starts a read-only Foxglove endpoint at 127.0.0.1:8765."
    echo "NOMAD_ROS_SETUP selects the ROS setup file. No services or motion APIs."
    exit 0
    ;;
  --check|"") ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo "Too many arguments" >&2; exit 2; }
[[ -r "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
set +u
source "$ROS_SETUP"
set -u
ros2 pkg prefix foxglove_bridge >/dev/null
[[ -r "$OPS_DIR/foxglove_bridge.yaml" ]] || exit 1

if [[ "${1:-}" == --check ]]; then
  echo "ROS and foxglove_bridge found; bridge not started."
  exit 0
fi
echo "Observation only: ws://127.0.0.1:8765"
echo "Import layout: $OPS_DIR/nomad_observe.foxglove-layout.json"
exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  --params-file "$OPS_DIR/foxglove_bridge.yaml"
