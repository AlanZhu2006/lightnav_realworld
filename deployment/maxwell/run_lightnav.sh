#!/usr/bin/env bash
set -euo pipefail
maxwell_script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
maxwell_project_root=$(cd "$maxwell_script_dir/../.." && pwd)
source "$maxwell_script_dir/environment.sh"
cd "$maxwell_project_root"
exec "$LIGHTNAV_PYTHON" -m deployment.maxwell.launch "$@"
