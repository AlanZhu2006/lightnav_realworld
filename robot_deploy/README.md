# On-robot ROS 2 workspace

This is the Jetson half of the complete [lightnav_realworld deployment](../README.md).
For the tested Go2 / RealSense configuration, use the
[robot installation and operation guide](../docs/REALWORLD_ROBOT.md).
The GPU server is installed separately via [the server guide](../docs/REALWORLD_SERVER.md).

## Packages

| Package | Responsibility |
| --- | --- |
| vln_client | Camera -> GPU WebSocket -> vln/response, capture stamps and trace IDs |
| vln_mpc | Capture-time odometry alignment and CasADi MPC at 10 Hz |
| vln_web | Browser :8088, manual / autonomous control, recording and diagnostics |
| vln_bringup | Go2 / RealSense launch plus upstream Orbbec and TRON references |
| go2_adapter | SDK2 velocity, mode / state, watchdog and control arbitration |
| tron_adapter | Upstream TRON1 WebSocket adapter |

No model weights or Torch are needed on the robot. Source ../robot-env.sh and build
with scripts/build.sh; both use the dedicated Python 3.10 venv. The main entrypoint
is ../deployment/robot/start.sh. --dry-run starts nothing; --check starts no nodes.

The RealSense launch subscribes to an existing image stream. The retained upstream
go2.launch.py / tron.launch.py start Orbbec and require its separately installed drivers.

## Shared adapter interface

- Subscribe: web/cmd_vel and mpc/cmd_vel (TwistStamped).
- Publish: odom, cmd_vel, control/source, diagnostics, robot/events.
- Control services: control/set_manual, control/set_auto, control/stop.
- Robot services: robot/stand, robot/walk, robot/sit, robot/emergency_stop.

The adapter owns manual / autonomous arbitration, stale-command stopping and
hand-back to the remote. Go2 uses ObstaclesAvoidClient for velocity and SportClient
for posture, and does not acquire API velocity control at startup.

See [data flow](../docs/REALWORLD_DATAFLOW.md) and [recording](../docs/REALWORLD_RECORDING.md)
for coordinates, timestamps, stop modes and archived signals.
