#!/usr/bin/env bash
# Creates a private environment; never installs packages with sudo.
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
gpu_venv=${1:-"$project_root/.venv-nomad-gpu"}
if [[ $(uname -m) != aarch64 ]] || [[ ! -f /etc/nv_tegra_release ]]; then
  echo 'This pinned environment requires Jetson aarch64.' >&2
  exit 2
fi
if [[ -e "$gpu_venv" ]]; then
  echo "Environment already exists; choose a new path: $gpu_venv" >&2
  exit 2
fi
if ! /usr/local/cuda-12.6/bin/nvcc --version >/dev/null; then
  echo 'Host CUDA 12.6 is required; this script never installs or upgrades CUDA.' >&2
  exit 2
fi
python3 -m venv --system-site-packages "$gpu_venv"
"$gpu_venv/bin/python" -m pip install 'pip==25.0.1'
"$gpu_venv/bin/python" -m pip install --no-deps -r "$project_root/deployment/sensors/requirements-jetson-gpu.txt"
"$gpu_venv/bin/python" -m pip install -r "$project_root/deployment/sensors/requirements-nomad-runtime.txt"
"$gpu_venv/bin/python" - <<'PY'
import json
import torch
import torchvision
import diffusers
assert torch.__version__ == '2.8.0', 'CUDA wheel was unexpectedly replaced'
assert torchvision.__version__ == '0.23.0'
assert torch.version.cuda == '12.6'
assert torch.cuda.is_available()
a = torch.ones((256, 256), device='cuda')
assert (a @ a).mean().item() == 256.0
indices = torchvision.ops.nms(torch.tensor([[0., 0., 1., 1.]], device='cuda'),
                              torch.tensor([1.], device='cuda'), 0.5)
assert indices.tolist() == [0]
print(json.dumps({'torch': torch.__version__, 'torchvision': torchvision.__version__,
                  'diffusers': diffusers.__version__, 'cuda': torch.version.cuda,
                  'cuda_available': True, 'device': torch.cuda.get_device_name(0),
                  'matmul_passed': True, 'torchvision_cuda_nms_passed': True}, indent=2))
PY
"$gpu_venv/bin/python" -m pip freeze --local > "$gpu_venv/requirements.actual.txt"
