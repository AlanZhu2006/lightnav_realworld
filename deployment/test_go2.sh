#!/usr/bin/env bash
# Portable CPU regressions only. No ROS launch, GPU model or SDK initialization.
set -euo pipefail
go2_repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$go2_repo_root"
export CUDA_VISIBLE_DEVICES=''
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$go2_repo_root${PYTHONPATH:+:$PYTHONPATH}"
exec "${PYTHON:-python3}" -m unittest -v \
  deployment.lightnav.tests.test_tracking \
  deployment.lightnav.tests.test_official_lightnav_policy \
  deployment.lightnav.tests.test_reference_image_policy \
  deployment.lightnav.tests.test_session_cleanup \
  deployment.lightnav.tests.test_http_lifecycle \
  deployment.lightnav.audit_imagegoal_adaptation \
  deployment.imagegoal.audit_official_lightnav \
  deployment.imagegoal.audit_lightnav_imagegoal_runtime \
  deployment.imagegoal.audit_lightnav_cleanup \
  deployment.imagegoal.audit_lightnav_dispatch \
  deployment.imagegoal.audit_lightnav_transport \
  deployment.imagegoal.audit_lightnav_http \
  deployment.imagegoal.audit_lightnav_backlog \
  deployment.operator.tests.test_lightnav_history \
  deployment.operator.tests.test_ensure_observation \
  checks.test_lightnav_launcher \
  checks.test_open_console_launch \
  checks.test_lightnav_profiles \
  checks.test_lightnav_source_isolation "$@"
