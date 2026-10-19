#!/usr/bin/env bash
# Actual locally validated CUDA path; imports no robot APIs.
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
gpu_python=${DEJAVIEW_NOMAD_GPU_PYTHON:-"$project_root/.venv-nomad-gpu/bin/python"}
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
cd "$project_root"
exec "$gpu_python" -m deployment.nomad.server \
  --upstream third_party/visualnav_transformer \
  --diffusion-policy third_party/diffusion_policy \
  --checkpoint assets/nomad/nomad.pth \
  --checkpoint-sha256 70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333 \
  --device cuda --port 18883 "$@"
