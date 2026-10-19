#!/bin/sh
# Independent virtual desktop; no access to the physical display session.
set -eu
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
cd "${NOMAD_PROJECT_ROOT:?NOMAD_PROJECT_ROOT must be set by run_vnc_desktop.sh}"
exec dbus-run-session -- startxfce4
