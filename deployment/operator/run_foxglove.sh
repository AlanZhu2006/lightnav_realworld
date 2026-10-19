#!/usr/bin/env bash
set -eo pipefail
operator_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd "$operator_dir/../.." && pwd)
source "${DEJAVIEW_FOXGLOVE_SETUP:-$project_root/.runtime/foxglove_20261017/setup.bash}"
exec ros2 run foxglove_bridge foxglove_bridge --ros-args --params-file "$operator_dir/foxglove.yaml"
