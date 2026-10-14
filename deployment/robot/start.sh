#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../common.sh"
lightnav_load_config robot
cd "$LIGHTNAV_REPO_ROOT"
if [[ " ${*} " == *" --dry-run "* ]]; then
    exec python3 -m deployment.launch robot "$@"
fi
source "$LIGHTNAV_REPO_ROOT/robot-env.sh"
exec "$LIGHTNAV_ROBOT_VENV/bin/python" -m deployment.launch robot "$@"
