#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
cd "$LIGHTNAV_REPO_ROOT"
export PYTHONPATH="$LIGHTNAV_REPO_ROOT:$LIGHTNAV_REPO_ROOT/src"
for package in vln_client vln_mpc vln_web; do
    export PYTHONPATH="$PYTHONPATH:$LIGHTNAV_REPO_ROOT/robot_deploy/src/$package"
done
for package in go2_adapter tron_adapter; do
    export PYTHONPATH="$PYTHONPATH:$LIGHTNAV_REPO_ROOT/robot_deploy/src/robot_adapters/$package"
done
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
if [[ "${1:-}" == "--robot" ]]; then
    shift
    python_bin="${PYTHON:-$LIGHTNAV_REPO_ROOT/.venv-robot/bin/python}"
    suites=(robot_deploy/src tests/deployment/test_configuration.py tests/deployment/test_provenance.py)
else
    python_bin="${PYTHON:-$LIGHTNAV_REPO_ROOT/.venv-server/bin/python}"
    suites=(tests robot_deploy/src)
fi
exec "$python_bin" -m pytest --import-mode=importlib -q -m 'not gpu' "${suites[@]}" "$@"
