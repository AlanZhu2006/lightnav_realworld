#!/usr/bin/env bash
set -euo pipefail
OPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The data-validation mode intentionally does not source or require ROS.
for argument in "$@"; do
  if [[ "$argument" == --validate-only || "$argument" == --help || "$argument" == -h ]]; then
    exec /usr/bin/python3 "$OPS_DIR/replay_shadow.py" "$@"
  fi
done
ROS_SETUP="${NOMAD_ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -r "$ROS_SETUP" ]] || { echo "Missing ROS setup: $ROS_SETUP" >&2; exit 1; }
set +u
source "$ROS_SETUP"
set -u
exec /usr/bin/python3 "$OPS_DIR/replay_shadow.py" "$@"
