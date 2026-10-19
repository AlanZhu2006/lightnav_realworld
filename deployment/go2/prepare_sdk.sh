#!/usr/bin/env bash
# Uses the host Humble CycloneDDS libraries through a private prefix.
set -euo pipefail
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
sdk_venv=${1:-"$project_root/.venv-go2-runtime"}
sdk_source="$project_root/third_party/unitree_sdk2_python"
sdk_commit=814556d15970dd2ecf1c9984e845ca02ab07e206
dds_prefix="$project_root/.runtime/cyclonedds_humble"
if [[ -e "$sdk_venv" ]]; then
  echo "Environment exists; choose a new path: $sdk_venv" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/lib/aarch64-linux-gnu/libddsc.so ]]; then
  echo 'Matching host ROS Humble CycloneDDS is required.' >&2
  exit 2
fi
if [[ ! -d "$sdk_source/.git" ]]; then
  git clone --no-checkout https://github.com/unitreerobotics/unitree_sdk2_python.git "$sdk_source"
  git -C "$sdk_source" checkout --detach "$sdk_commit"
fi
if [[ $(git -C "$sdk_source" rev-parse HEAD) != "$sdk_commit" ]]; then
  echo "SDK checkout is not pinned $sdk_commit; refusing to change existing source." >&2
  exit 2
fi
mkdir -p "$dds_prefix"
for sdk_component in lib include bin; do
  case "$sdk_component" in
    lib) sdk_target=/opt/ros/humble/lib/aarch64-linux-gnu ;;
    *) sdk_target="/opt/ros/humble/$sdk_component" ;;
  esac
  if [[ -e "$dds_prefix/$sdk_component" || -L "$dds_prefix/$sdk_component" ]]; then
    [[ $(readlink "$dds_prefix/$sdk_component") == "$sdk_target" ]] || exit 2
  else
    ln -s "$sdk_target" "$dds_prefix/$sdk_component"
  fi
done
python3 -m venv --system-site-packages "$sdk_venv"
export CYCLONEDDS_HOME="$dds_prefix"
export LD_LIBRARY_PATH="/opt/ros/humble/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
"$sdk_venv/bin/python" -m pip install 'cyclonedds==0.10.2'
"$sdk_venv/bin/python" -m pip install --no-deps "$sdk_source"
"$sdk_venv/bin/python" - <<'PY'
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, LowState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
print('SDK imports passed; no DDS initialization and no SportClient construction.')
PY
"$sdk_venv/bin/python" -m pip freeze --local > "$sdk_venv/requirements.actual.txt"
