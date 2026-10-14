#!/usr/bin/env bash
# Sourcing this file starts no device or service.
LIGHTNAV_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LIGHTNAV_REPO_ROOT
lightnav_load_config() {
    local role="$1"
    local config_path="${LIGHTNAV_CONFIG_FILE:-${LIGHTNAV_REPO_ROOT}/config/${role}.env}"
    if [[ ! -f "$config_path" ]]; then
        if [[ -n "${LIGHTNAV_CONFIG_FILE:-}" ]]; then
            printf 'Configuration file does not exist: %s\n' "$config_path" >&2
            return 1
        fi
        config_path="${LIGHTNAV_REPO_ROOT}/config/${role}.env.example"
    fi
    set -a
    source "$config_path"
    set +a
}
