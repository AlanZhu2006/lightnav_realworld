#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/../common.sh"
cd "$LIGHTNAV_REPO_ROOT"
python_bin="${LIGHTNAV_SETUP_PYTHON:-python3.11}"
"$python_bin" -c 'import sys; assert sys.version_info[:2] == (3, 11), "Server requires Python 3.11"'
"$python_bin" -m venv .venv-server
.venv-server/bin/python -m pip install --upgrade pip
.venv-server/bin/python -m pip install -e '.[vllm,video,test,realworld-test]' 'ruff==0.17.0'
if [[ ! -f config/server.env ]]; then cp config/server.env.example config/server.env; fi
printf 'Server environment ready. Configure config/server.env and obtain the checkpoint.\n'
