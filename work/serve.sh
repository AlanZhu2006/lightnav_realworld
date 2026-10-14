#!/usr/bin/env bash
# Compatibility entrypoint for the original workstation workflow.
exec "$(dirname -- "${BASH_SOURCE[0]}")/../deployment/server/start.sh" "$@"
