#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../common.sh"
lightnav_load_config server
cd "$LIGHTNAV_REPO_ROOT"
if [[ " ${*} " == *" --dry-run "* ]]; then
    exec python3 -m deployment.launch server "$@"
fi
if [[ ! -x "$LIGHTNAV_SERVER_PYTHON" ]]; then
    printf 'Server Python not found: %s. Run deployment/server/setup.sh first.\n' "$LIGHTNAV_SERVER_PYTHON" >&2
    exit 1
fi
exec "$LIGHTNAV_SERVER_PYTHON" -m deployment.launch server "$@"
