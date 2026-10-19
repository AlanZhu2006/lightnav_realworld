#!/usr/bin/env bash
# Run without sudo. Downloads and installation remain on the user's /local_data.
set -euo pipefail
_dejaview_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
_dejaview_base=${DEJAVIEW_DATA_ROOT:-/local_data/$(id -un)}
case "$_dejaview_base" in /local_data/"$(id -un)"|/local_data/"$(id -un)"/*) ;; *) echo 'Refusing installation outside the user /local_data directory' >&2; exit 2 ;; esac
mkdir -p "$_dejaview_base/migration/20261017" "$_dejaview_base/tmp"
exec 9>"$_dejaview_base/migration/20261017/environment.lock"
flock -n 9 || { echo 'Another environment preparation is running' >&2; exit 2; }
exec > >(tee -a "$_dejaview_base/migration/20261017/environment_install.log") 2>&1
# Preserve the existing cache as a whole before creating project cache subfolders.
if [[ -L "$HOME/.cache" ]]; then
  [[ $(readlink -f "$HOME/.cache") == "$_dejaview_base/cache" ]] || { echo 'Existing .cache symlink points elsewhere; no automatic replacement' >&2; exit 2; }
elif [[ -e "$HOME/.cache" ]]; then
  [[ -d "$HOME/.cache" && ! -e "$_dejaview_base/cache" ]] || { echo 'Cache migration needs an empty destination; existing data preserved' >&2; exit 2; }
  mv -- "$HOME/.cache" "$_dejaview_base/cache"
  ln -s -- "$_dejaview_base/cache" "$HOME/.cache"
else
  mkdir -p "$_dejaview_base/cache"
  ln -s -- "$_dejaview_base/cache" "$HOME/.cache"
fi
export DEJAVIEW_DATA_ROOT="$_dejaview_base"
# This migration never grants GPU use, even if a caller supplied a device.
unset DEJAVIEW_CUDA_VISIBLE_DEVICES
source "$_dejaview_script_dir/environment.sh"
export CUDA_VISIBLE_DEVICES=''
_dejaview_lock=${1:-"$_dejaview_script_dir/../lightnav/requirements-server.lock.txt"}
[[ -f "$_dejaview_lock" ]] || { echo "Missing requirements: $_dejaview_lock" >&2; exit 2; }
mkdir -p "$DEJAVIEW_DATA_ROOT/tools/bin" "$DEJAVIEW_DATA_ROOT/wheels/bootstrap" "$DEJAVIEW_DATA_ROOT/wheels/lightnav"
# uv's exact official PyPI wheel is pinned and verified before extracting its binary.
python3 - "$DEJAVIEW_DATA_ROOT" <<'PY'
import hashlib,json, pathlib, urllib.request, zipfile, os
base=pathlib.Path(__import__('sys').argv[1])
filename='uv-0.13.1-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl'
url='https://files.pythonhosted.org/packages/14/0b/6602a613533602e7a083e908c4375f9b0e78a94ba2e21c29b21c41a0a611/'+filename
expected='696dcf6d807d85da978f4db8472da81dccc1254b18b52cbd1367c4d923c421e0'
path=base/'wheels/bootstrap'/filename
if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
    partial=path.with_suffix('.part')
    with urllib.request.urlopen(url,timeout=90) as response,partial.open('wb') as stream:
        __import__('shutil').copyfileobj(response,stream,length=1<<20)
    if hashlib.sha256(partial.read_bytes()).hexdigest()!=expected:
        raise RuntimeError('Official uv wheel hash mismatch')
    partial.replace(path)
with zipfile.ZipFile(path) as archive:
    members=[name for name in archive.namelist() if name.endswith('.data/scripts/uv')]
    if len(members)!=1:
        raise RuntimeError('Unexpected uv wheel layout')
    executable=base/'tools/bin/uv'
    executable.write_bytes(archive.read(members[0]));executable.chmod(0o755)
(base/'migration/20261017/uv_bootstrap.json').write_text(json.dumps({'version':'0.13.1','filename':filename,'url':url,'sha256':expected,'bytes':path.stat().st_size,'verified':True},indent=2)+'\n')
PY
uv python install 3.11.17
_dejaview_env="$DEJAVIEW_DATA_ROOT/envs/lightnav"
if [[ ! -f "$_dejaview_env/pyvenv.cfg" ]]; then
  uv venv --python 3.11.17 --no-project "$_dejaview_env"
fi
# packaging is needed only to select compatible wheels using the interpreter's tags.
uv pip install --python "$_dejaview_env/bin/python" --only-binary :all: --no-deps --index-url https://pypi.org/simple 'packaging==26.3'
"$_dejaview_env/bin/python" "$_dejaview_script_dir/check_environment.py" --requirements "$_dejaview_lock" --download-wheels "$DEJAVIEW_DATA_ROOT/wheels/lightnav" --wheel-manifest "$DEJAVIEW_DATA_ROOT/migration/20261017/lightnav_wheels.json"
uv pip sync --python "$_dejaview_env/bin/python" --no-index --require-hashes --only-binary :all: --find-links "$DEJAVIEW_DATA_ROOT/wheels/lightnav" "$DEJAVIEW_DATA_ROOT/migration/20261017/lightnav_wheels.requirements.txt"
"$_dejaview_env/bin/python" "$_dejaview_script_dir/check_environment.py" --requirements "$_dejaview_lock" --wheel-manifest "$DEJAVIEW_DATA_ROOT/migration/20261017/lightnav_wheels.json" --receipt "$DEJAVIEW_DATA_ROOT/migration/20261017/environment_receipt.json"
