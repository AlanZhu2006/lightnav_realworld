#!/usr/bin/env bash
# A Jetson-owned, restricted SSH connection to Maxwell. No model or robot process
# is started, and the current LightNav port 18884 is never changed.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
SSH_DIR="$PROJECT_ROOT/.runtime/ssh"
KEY="$SSH_DIR/maxwell_lightnav"
KNOWN_HOSTS="$SSH_DIR/known_hosts.maxwell"
CONTROL="$SSH_DIR/maxwell.cm"
LOG="$SSH_DIR/maxwell_link.log"
DESTINATION="${MAXWELL_SSH_DESTINATION:?Set MAXWELL_SSH_DESTINATION to your SSH destination}"
ACTION="${1:-status}"

usage() {
  cat <<'EOF'
Usage: bash deployment/maxwell/jetson_link.sh start|status|stop

Run on the Jetson. The independently managed connection provides:
  Jetson  127.0.0.1:18885 -> Maxwell 127.0.0.1:18884 (standby LightNav)
  Maxwell 127.0.0.1:18093 -> Jetson  127.0.0.1:8093  (operator web)
  Maxwell 127.0.0.1:18766 -> Jetson  127.0.0.1:8766  (Foxglove)

This connection does not select a GPU, start a model, switch a runtime URL,
create a navigation task, initialize the robot SDK, or expose public sockets.
The project-specific key and pinned host key must first be provisioned in
.runtime/ssh. The Maxwell authorized_keys entry limits this key to these
three forwarding destinations; interactive sessions are disabled.
EOF
}

case "$ACTION" in
  start|status|stop) ;;
  -h|--help|help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

if [[ ! -r "$KEY" || ! -r "$KNOWN_HOSTS" ]]; then
  echo "Missing project key or pinned Maxwell host key in $SSH_DIR" >&2
  exit 2
fi
if [[ ${#CONTROL} -gt 100 ]]; then
  echo "SSH control socket path is too long: $CONTROL" >&2
  exit 2
fi

SSH_OPTIONS=(
  -F /dev/null
  -p 22
  -i "$KEY"
  -o BatchMode=yes
  -o IdentitiesOnly=yes
  -o PreferredAuthentications=publickey
  -o StrictHostKeyChecking=yes
  -o "UserKnownHostsFile=$KNOWN_HOSTS"
  -o GlobalKnownHostsFile=/dev/null
  -o HostKeyAlgorithms=ssh-ed25519
  -o ConnectTimeout=15
  -o ServerAliveInterval=15
  -o ServerAliveCountMax=3
  -o ExitOnForwardFailure=yes
  -o ControlPersist=no
  -S "$CONTROL"
)

master_running() {
  ssh "${SSH_OPTIONS[@]}" -O check "$DESTINATION" >/dev/null 2>&1
}

case "$ACTION" in
  start)
    if master_running; then
      echo "Maxwell project connection already running."
      exec bash "$0" status
    fi
    # Do not unlink an existing socket: an unresponsive master needs review.
    if [[ -e "$CONTROL" ]]; then
      echo "Existing unresponsive control socket; inspect $CONTROL before retrying." >&2
      exit 1
    fi
    umask 077
    ssh "${SSH_OPTIONS[@]}" -M -fNT -E "$LOG" \
      -L 127.0.0.1:18885:127.0.0.1:18884 \
      -R 127.0.0.1:18093:127.0.0.1:8093 \
      -R 127.0.0.1:18766:127.0.0.1:8766 \
      "$DESTINATION"
    exec bash "$0" status
    ;;
  status)
    if ! master_running; then
      echo "Maxwell project connection is stopped." >&2
      exit 1
    fi
    ssh "${SSH_OPTIONS[@]}" -O check "$DESTINATION"
    echo "Jetson standby model: http://127.0.0.1:18885 (model may remain offline)"
    echo "Maxwell operator web: http://127.0.0.1:18093 (loopback only)"
    echo "Maxwell Foxglove: ws://127.0.0.1:18766 (loopback only)"
    echo "SSH log: $LOG"
    ;;
  stop)
    if master_running; then
      ssh "${SSH_OPTIONS[@]}" -O exit "$DESTINATION"
      echo "Stopped only the dedicated Maxwell project connection."
    else
      echo "Maxwell project connection is already stopped."
    fi
    ;;
esac
