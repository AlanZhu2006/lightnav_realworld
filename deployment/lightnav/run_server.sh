#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"
exec "${LIGHTNAV_PYTHON:-$repo_root/deployment/lightnav/.venv/bin/python}" -m deployment.lightnav.server \
  --model-path "${LIGHTNAV_MODEL_PATH:-$repo_root/deployment/lightnav/.runtime/weights}" \
  --host "${LIGHTNAV_HOST:-127.0.0.1}" --port "${LIGHTNAV_PORT:-18884}" \
  --record-dir "${LIGHTNAV_RECORD_DIR:-$repo_root/deployment/lightnav/.runtime/recordings}" "$@"
