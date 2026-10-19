#!/usr/bin/env bash
# This starts the physical camera only. It imports no robot SDK.
set -eo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
sensor_overlay=${DEJAVIEW_REALSENSE_SETUP:-"$project_root/.runtime/realsense_20261017/setup.bash"}
if [[ ! -f "$sensor_overlay" ]]; then
  echo "Missing RealSense overlay: $sensor_overlay" >&2
  exit 2
fi
source "$sensor_overlay"
exec ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  enable_color:=true enable_depth:=true \
  enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15 \
  align_depth.enable:=true enable_sync:=true \
  pointcloud.enable:=false publish_tf:=true \
  enable_rgbd:=false initial_reset:=false \
  "$@"
