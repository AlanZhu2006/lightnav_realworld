# Working in lightnav_realworld

This repository integrates the complete GPU server and Jetson / Go2 ROS 2 stack.
The upstream model and RVQ decoder are preserved. Main human documentation is
README.md and docs/REALWORLD_*.md. UPSTREAM_README.md preserves the original guide.

## Layout

- src/lightnav: original inference, protocol, scheduler, decoder, and CLI.
- deployment: portable configuration, preflight, role launchers, and offline smoke.
- robot_deploy: ROS 2 Humble workspace, camera client, MPC, Go2 / TRON adapters, web UI.
- work: compatibility server entrypoint, diagnostics, and verified recording sync.
- config: public templates; actual *.env files are private and ignored.
- tests and robot_deploy/src/*/test: offline regression coverage.

## Verification

Install the GPU role with deployment/server/setup.sh, or install
`pip install -e '.[test,video,realworld-test]' ruff` for CPU verification.
Run `make check` and `make test PYTHON=/path/to/python3.11` after changes.
make test includes both inference and robot tests, uses importlib to avoid duplicate
test module basenames, and never launches physical devices.

Tests requiring rclpy / rosbag2_py may skip on a non-ROS host; report those skips.
Robot protocol and MPC validation functions remain independently testable.
For server changes, when a checkpoint and free GPU are available, start a separate
loopback server on a free port and use deployment.smoke_protocol. Use only its own
process group for cleanup. Do not replace a user's running service.

## Runtime constraints

- GPU role: Python 3.11; Transformers 5.8.0, vLLM 0.19.1 and
  nvidia-cutlass-dsl 4.5.2. Never bypass the vLLM version guard or silently fall back.
- Robot role: ROS 2 Humble / Python 3.10. Do not install root pyproject dependencies
  into Jetson; use deployment/robot/requirements.txt.
- Official checkpoint: LightOriginsHQ/LightNav-0, including all model shards and
  action_tokenizer bundle. Never substitute another model to make a test pass.
- Blackwell sm_103 with the HF backend requires Torch 2.10.0+cu129 /
  torchvision 0.25.0+cu129; stock cu128 NVRTC does not support that architecture.
- Do not commit weights, recordings, private .env files, ROS build/install outputs,
  credentials, or machine-specific runtime snapshots.
- Add environment options to docs/REALWORLD_CONFIGURATION.md; upstream server flags
  belong in docs/CONFIGURATION.md.
- vln_mpc.protocol holds ROS-independent task modes, response validation and parameters.
  Keep waypoint coordinates / capture-time alignment and stop-mode semantics intact.

## Physical hardware

Never start a robot-side ROS launch, initialize Unitree hardware communication,
publish motion commands, or start camera devices as part of automated verification.
Hardware bring-up is performed by the operator. --dry-run prints commands;
--check imports dependencies and reads package/interface information without nodes.

Keep source provenance and upstream attribution. No new learning algorithm or
navigation performance claim is implied by deployment integration.
