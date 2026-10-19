#!/usr/bin/env bash
# Adapted in purpose from TinyNav's independent VNC desktop; see SOURCES.md.
# Modified: no desktop termination, lock removal, default password or LAN bind.
set -euo pipefail

OPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${NOMAD_PROJECT_ROOT:-$(cd "$OPS_DIR/../../.." && pwd)}"
DISPLAY_NUM="${NOMAD_VNC_DISPLAY_NUM:-10}"
GEOMETRY="${NOMAD_VNC_GEOMETRY:-1280x720}"
PASS_FILE="${NOMAD_VNC_PASSWORD_FILE:-}"

case "${1:-}" in
  --help|-h)
    echo "Usage: NOMAD_VNC_PASSWORD_FILE=/private/path $0 [--check]"
    echo "Current user only; separate localhost-only TigerVNC desktop, display :10."
    echo "Optional: NOMAD_PROJECT_ROOT, NOMAD_VNC_DISPLAY_NUM, NOMAD_VNC_GEOMETRY."
    echo "Create the private password file interactively with tigervncpasswd first."
    exit 0
    ;;
  --check|"") ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo "Too many arguments" >&2; exit 2; }
[[ "$DISPLAY_NUM" =~ ^[1-9][0-9]?$ ]] || { echo "Display must be 1..99" >&2; exit 2; }
[[ "$GEOMETRY" =~ ^[1-9][0-9]{2,3}x[1-9][0-9]{2,3}$ ]] || { echo "Invalid geometry" >&2; exit 2; }
PORT=$((5900 + DISPLAY_NUM))
for program in tigervncserver startxfce4 dbus-run-session ss stat; do
  command -v "$program" >/dev/null || { echo "Missing dependency: $program" >&2; exit 1; }
done
[[ -d "$PROJECT_ROOT" ]] || { echo "Missing project: $PROJECT_ROOT" >&2; exit 1; }
[[ -n "$PASS_FILE" && -f "$PASS_FILE" && ! -L "$PASS_FILE" && -r "$PASS_FILE" ]] || {
  echo "Set NOMAD_VNC_PASSWORD_FILE to an existing private VNC password file." >&2
  exit 1
}
[[ "$(stat -c %u "$PASS_FILE")" == "$(id -u)" ]] || {
  echo "Password file must belong to the current user." >&2; exit 1;
}
[[ "$(stat -c %a "$PASS_FILE")" =~ ^[46]00$ ]] || {
  echo "Password file mode must be 600 or 400." >&2; exit 1;
}
[[ ! -e "/tmp/.X${DISPLAY_NUM}-lock" && ! -e "/tmp/.X11-unix/X${DISPLAY_NUM}" ]] || {
  echo "Display :$DISPLAY_NUM is occupied or has a lock; no cleanup attempted." >&2; exit 1;
}
[[ -z "$(ss -H -ltn "sport = :$PORT")" ]] || {
  echo "TCP port $PORT is occupied; no process was stopped." >&2; exit 1;
}
if [[ "${1:-}" == --check ]]; then
  echo "VNC dependencies, private password file and display checked; not started."
  exit 0
fi
export NOMAD_PROJECT_ROOT="$PROJECT_ROOT"
echo "Separate desktop :$DISPLAY_NUM for $(id -un), project $PROJECT_ROOT"
echo "Use SSH forwarding to localhost:$PORT; password contents are never logged."
exec tigervncserver ":$DISPLAY_NUM" -fg -localhost yes \
  -SecurityTypes VncAuth -PasswordFile "$PASS_FILE" \
  -geometry "$GEOMETRY" -depth 24 -desktop "NoMaD observe" \
  -xstartup "$OPS_DIR/vnc_xstartup.sh"
