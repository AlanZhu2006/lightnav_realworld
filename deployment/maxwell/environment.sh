#!/usr/bin/env bash
# Source this file before project commands. It never changes HOME or selects a GPU.
export DEJAVIEW_DATA_ROOT="${DEJAVIEW_DATA_ROOT:-/local_data/$(id -un)}"
case "$DEJAVIEW_DATA_ROOT" in /local_data/"$(id -un)"|/local_data/"$(id -un)"/*) ;; *) echo "Project data must remain in /local_data/$(id -un)" >&2; return 2 2>/dev/null || exit 2 ;; esac
export MAXWELL_DATA_ROOT="$DEJAVIEW_DATA_ROOT"
export LIGHTNAV_PYTHON="$DEJAVIEW_DATA_ROOT/envs/lightnav/bin/python"
export LIGHTNAV_MODEL_PATH="$DEJAVIEW_DATA_ROOT/models/LightNav-0/826dc5fbfa37afa8293d2e336d329b6ffc0bfb64"
export LIGHTNAV_RECORD_DIR="$DEJAVIEW_DATA_ROOT/runs/lightnav/recordings"
export XDG_CACHE_HOME="$DEJAVIEW_DATA_ROOT/cache"
export XDG_DATA_HOME="$DEJAVIEW_DATA_ROOT/share"
export XDG_STATE_HOME="$DEJAVIEW_DATA_ROOT/state"
export XDG_CONFIG_HOME="$DEJAVIEW_DATA_ROOT/config"
export UV_CACHE_DIR="$XDG_CACHE_HOME/uv"
export UV_PYTHON_INSTALL_DIR="$DEJAVIEW_DATA_ROOT/python"
export UV_PYTHON_BIN_DIR="$DEJAVIEW_DATA_ROOT/tools/bin"
export UV_TOOL_DIR="$DEJAVIEW_DATA_ROOT/tools/uv-tools"
export UV_TOOL_BIN_DIR="$DEJAVIEW_DATA_ROOT/tools/bin"
export PIP_CACHE_DIR="$XDG_CACHE_HOME/pip"
export PIP_CONFIG_FILE="$DEJAVIEW_DATA_ROOT/config/pip/pip.conf"
export PYTHONUSERBASE="$DEJAVIEW_DATA_ROOT/python-user"
export HF_HOME="$XDG_CACHE_HOME/huggingface"
export HF_HUB_CACHE="$HF_HOME/hub"
export HUGGINGFACE_HUB_CACHE="$HF_HUB_CACHE"
export HF_DATASETS_CACHE="$HF_HOME/datasets"
export HF_ASSETS_CACHE="$HF_HOME/assets"
export HF_XET_CACHE="$HF_HOME/xet"
export TORCH_HOME="$XDG_CACHE_HOME/torch"
export TORCHINDUCTOR_CACHE_DIR="$XDG_CACHE_HOME/torchinductor"
export TRITON_CACHE_DIR="$XDG_CACHE_HOME/triton"
export CUDA_CACHE_PATH="$XDG_CACHE_HOME/nv/ComputeCache"
export NUMBA_CACHE_DIR="$XDG_CACHE_HOME/numba"
export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
export CONDA_PKGS_DIRS="$XDG_CACHE_HOME/conda/pkgs"
export CONDA_ENVS_PATH="$DEJAVIEW_DATA_ROOT/envs/conda"
export MAMBA_ROOT_PREFIX="$DEJAVIEW_DATA_ROOT/mamba"
export CONDARC="$DEJAVIEW_DATA_ROOT/config/conda/condarc"
export TMPDIR="$DEJAVIEW_DATA_ROOT/tmp"
export TMP="$TMPDIR"
export TEMP="$TMPDIR"
export PIP_DISABLE_PIP_VERSION_CHECK=1
export UV_CONCURRENT_DOWNLOADS=4
export UV_CONCURRENT_BUILDS=2
export UV_CONCURRENT_INSTALLS=2
export MAX_JOBS=2
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export UV_LINK_MODE=hardlink
# Explicit permission and an explicit device are required before a later GPU run.
export CUDA_VISIBLE_DEVICES="${DEJAVIEW_CUDA_VISIBLE_DEVICES:-}"
export PATH="$DEJAVIEW_DATA_ROOT/tools/bin:$PATH"
for _dejaview_dir in "$XDG_CACHE_HOME" "$XDG_DATA_HOME" "$XDG_STATE_HOME" "$XDG_CONFIG_HOME" "$UV_PYTHON_INSTALL_DIR" "$UV_TOOL_DIR" "$UV_TOOL_BIN_DIR" "$TMPDIR"; do
  mkdir -p -- "$_dejaview_dir" || return 2 2>/dev/null || exit 2
done
unset _dejaview_dir
