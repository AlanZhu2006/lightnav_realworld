#!/usr/bin/env bash
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/humble/setup.bash
# SDK2 explicitly owns CycloneDDS domain 0. Keep ROS on its installed FastDDS
# backend so an inherited Cyclone RMW cannot create the same domain first.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export CYCLONEDDS_HOME="$project_root/.runtime/cyclonedds_humble"
export LD_LIBRARY_PATH="/opt/ros/humble/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
cd "$project_root"
go2_python="$project_root/.venv-go2-runtime/bin/python"
action=${1:-disabled}
if [[ $# -gt 0 ]]; then shift; fi
case "$action" in
  disabled) exec "$go2_python" -m deployment.go2.bridge --mode disabled "$@" ;;
  preflight) exec "$go2_python" -m deployment.go2.preflight "$@" ;;
  telemetry) exec "$go2_python" -m deployment.go2.telemetry "$@" ;;
  camera) exec "$go2_python" -m deployment.go2.camera "$@" ;;
  native-smoke) exec "$go2_python" -m deployment.go2.native_smoke "$@" ;;
  native-imagegoal) exec "$go2_python" -m deployment.go2.start_native_imagegoal "$@" ;;
  native-executor) exec "$go2_python" -m deployment.go2.native_imagegoal "$@" ;;
  hardware-locked) exec "$go2_python" -m deployment.go2.bridge --mode hardware-locked "$@" ;;
  execute) exec "$go2_python" -m deployment.go2.bridge --execute "$@" ;;
  *) echo "Unknown action: $action" >&2; exit 2 ;;
esac
