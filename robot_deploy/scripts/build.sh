#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"

source "${workspace_dir}/../robot-env.sh"
if ! "${LIGHTNAV_ROBOT_VENV}/bin/python" -c 'import colcon_core.command'; then
    echo "Run deployment/robot/setup.sh before building the ROS workspace" >&2
    exit 1
fi
cd "${workspace_dir}"
"${LIGHTNAV_ROBOT_VENV}/bin/python" -c 'from colcon_core.command import main; raise SystemExit(main())' \
    build --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=${LIGHTNAV_ROBOT_VENV}/bin/python" "$@"
