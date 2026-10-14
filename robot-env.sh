#!/usr/bin/env bash
# Source on the Jetson; this configures the environment without launching nodes.
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment/common.sh"
lightnav_load_config robot || return 1
if [[ ! -f "$LIGHTNAV_ROS_SETUP" ]]; then
    printf 'ROS setup not found: %s\n' "$LIGHTNAV_ROS_SETUP" >&2
    return 1
fi
source "$LIGHTNAV_ROS_SETUP"
if [[ ! -f "$LIGHTNAV_ROBOT_VENV/bin/activate" ]]; then
    printf 'Robot environment not found: %s\n' "$LIGHTNAV_ROBOT_VENV" >&2
    return 1
fi
source "$LIGHTNAV_ROBOT_VENV/bin/activate"
if [[ -n "$LIGHTNAV_CYCLONEDDS_HOME" ]]; then
    export CYCLONEDDS_HOME="$LIGHTNAV_CYCLONEDDS_HOME"
    export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:${LD_LIBRARY_PATH:-}"
fi
if [[ -n "$LIGHTNAV_REALSENSE_SETUP" ]]; then
    if [[ ! -f "$LIGHTNAV_REALSENSE_SETUP" ]]; then
        printf 'RealSense overlay not found: %s\n' "$LIGHTNAV_REALSENSE_SETUP" >&2
        return 1
    fi
    source "$LIGHTNAV_REALSENSE_SETUP"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="$LIGHTNAV_FASTDDS_PROFILE"
if [[ -f "$LIGHTNAV_REPO_ROOT/robot_deploy/install/setup.bash" ]]; then
    source "$LIGHTNAV_REPO_ROOT/robot_deploy/install/setup.bash"
fi
