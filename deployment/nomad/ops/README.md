# NoMaD observation tools

These tools observe the NoMaD deployment. They do not import Unitree SDK2,
start a policy, publish motion, expose services or change system configuration.
They are intended for the Jetson workspace `/home/unitree/work/lightnav_realworld`; paths
are resolved from the scripts, so no username is embedded in the launchers.

The reference stacks and their frozen revisions are listed in [SOURCES.md](SOURCES.md).
This is a reduced NoMaD profile, not the GEM CEC operator workflow.

## Dependencies and initial checks

The host needs ROS 2 Humble, `foxglove_bridge`, `sensor_msgs`, `nav_msgs`,
`geometry_msgs`, `std_msgs`, `cv_bridge`, Python NumPy, OpenCV and Pillow. The relay uses
the host `/usr/bin/python3`, not the policy model environment. Optional VNC needs
`tigervncserver`, Xfce, `dbus-run-session` and `ss`. These files do not install
any dependencies. On `go2`, ROS 2 Humble and `cv_bridge` are available.
Foxglove Bridge 3.6.0 is unpacked as a user overlay at
`~/work/lightnav_realworld/.runtime/foxglove_20261017`; use its setup file through
`NOMAD_ROS_SETUP` rather than assuming a system-installed bridge.

The bridge dependency check does not start it:

```bash
cd ~/work/lightnav_realworld
NOMAD_ROS_SETUP="$HOME/work/DejaView/.runtime/foxglove_20261017/setup.bash" \
  bash deployment/nomad/ops/run_foxglove_bridge.sh --check
```

When the camera and policy publisher are running in observation/shadow mode,
start these two commands in separate terminals:

```bash
bash deployment/nomad/ops/run_preview_relay.sh
NOMAD_ROS_SETUP="$HOME/work/DejaView/.runtime/foxglove_20261017/setup.bash" \
  bash deployment/nomad/ops/run_foxglove_bridge.sh
```

From the workstation, forward the bridge through the existing SSH host alias:

```bash
ssh -N -L 8765:127.0.0.1:8765 go2
```

Connect Foxglove to `ws://127.0.0.1:8765` and import
`nomad_observe.foxglove-layout.json`. The profile advertises only connection
graph inspection. Client topic publishing, services and parameter access are
disabled with both capabilities and explicit deny patterns. It does not expose
GEM's CEC controls or arbitrary file assets. No static TF publisher is started.

The deployed SDK-based bridge uses the `foxglove.sdk.v1` WebSocket subprotocol.
A client offering only the older `foxglove.websocket.v1` receives HTTP 400;
use a Foxglove client compatible with the SDK protocol. The upstream
[handshake implementation](https://docs.rs/foxglove/latest/src/foxglove/websocket/handshake.rs.html)
defines this requirement; see also the
[official ROS 2 SDK bridge README](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md).
Actual GUI connection, layout import and panel rendering remain unverified.

## Topic contract

| Topic | ROS type | Producer / meaning |
| --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RealSense RGB; relay default input |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | RealSense depth; display only |
| `/nomad/goal_image` | `sensor_msgs/msg/Image` | Runtime's complete active goal image |
| `/nomad/foxglove/rgb/compressed` | `sensor_msgs/msg/CompressedImage` | Relay RGB preview |
| `/nomad/foxglove/goal/compressed` | `sensor_msgs/msg/CompressedImage` | Relay retained goal preview |
| `/nomad/foxglove/depth_color/compressed` | `sensor_msgs/msg/CompressedImage` | Relay depth color preview |
| `/nomad/path` | `nav_msgs/msg/Path` | NoMaD metric path with a valid frame and source timestamp |
| `/nomad/mpc/reference` | `nav_msgs/msg/Path` | Reference used by the actual MPC solve |
| `/nomad/mpc/prediction` | `nav_msgs/msg/Path` | Predicted MPC trajectory |
| `/nomad/cmd_vel_shadow` | `geometry_msgs/msg/Twist` | Computed command with no actuator subscriber |
| `/nomad/go2/odometry` | `nav_msgs/msg/Odometry` | Actual robot feedback |
| `/nomad/go2/battery` | `sensor_msgs/msg/BatteryState` | Actual battery feedback; discoverable in the bridge |
| `/nomad/status` | `std_msgs/msg/String` | Runtime status JSON; source for validity and mode |

The layout's fixed frame is `go2_odom` and its follow frame is `base_link`.
Publish the measured odometry TF, and anchor paths with the pose matching the
source RGB timestamp. A body-relative path must not be relabeled `go2_odom`.
Missing odometry/TF may leave the 3D panel empty; do not insert an identity
transform to conceal that missing evidence. The relay does not supply path,
MPC, goal, status, odometry or TF producers.

The relay copies source headers, preserves image aspect ratio, bounds previews
to 640 pixels wide and publishes at most 5 Hz. It publishes only new input
messages, so a stopped camera is not made to look fresh. The RGB and depth
preview topics use sensor QoS. The goal output is reliable/transient-local so
Foxglove can join after the goal was set; it remains a picture of the last goal,
not proof that the runtime still considers that goal valid.

Default goal input QoS accepts a volatile publisher. Start the relay before a
one-shot goal publication, republish the goal, or use a transient-local goal
publisher with `-p goal_transient_local:=true`. The source topics and display
settings can be changed with ROS parameters, for example:

```bash
bash deployment/nomad/ops/run_preview_relay.sh --ros-args \
  -p goal_transient_local:=true -p rate_hz:=3.0 -p width:=480
```

For display, `16UC1`/`mono16` depth defaults to 0.001 metres per unit, while
`32FC1` is interpreted as metres. Verify this contract against the actual
camera publisher. The color range defaults to 0.2–4 m; black means invalid.
This lossy preview is neither a metric sensor output nor an obstacle stop gate.
Use `-p depth_enabled:=false` to omit it.

## Recorded shadow replay

`replay_shadow.py` is a separate observation source for the real recorded RGB
and `shadow_client.py` outputs. It never contacts the model server or robot,
and never publishes velocity, odometry, TF or services. Its namespace is
`/nomad/shadow/`, separate from live camera and runtime topics.

First validate the actual files without ROS:

```bash
bash deployment/nomad/ops/run_shadow_replay.sh \
  --manifest runs/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --goal runs/nomad_commissioning_20261017/rgb_capture_001/rgb/000012.png \
  --results runs/nomad_commissioning_20261017/local_shadow_001/results.jsonl \
  --validate-only
```

The Jetson retains these runs under `~/work/lightnav_realworld/runs/nomad_commissioning_20261017/`.
The workstation's archived copy uses
`data/realworld/nomad_commissioning_20261017/`; substitute that prefix when
validating the workstation copy. The three CLI paths select existing data.
Copied `input_manifest.json` is also supported when its
image paths, the original manifest directory or recorded `resolved_path`
entries still resolve. Validation checks image/goal hashes and the result's
identity and chronological source frames rather than silently substituting
another picture.

After validation, run the same command without `--validate-only`, optionally
adding `--hz 0.5 --loop`. Start the read-only bridge normally and import
`nomad_shadow.foxglove-layout.json`. No preview relay is needed for this replay.
Stopping at the last result leaves the node displaying the last frame with
`playback: ENDED`; Ctrl-C exits it.

For the deployed Jetson overlay, a replay launch uses:

```bash
NOMAD_ROS_SETUP="$HOME/work/DejaView/.runtime/foxglove_20261017/setup.bash" \
  bash deployment/nomad/ops/run_shadow_replay.sh \
  --manifest runs/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --goal runs/nomad_commissioning_20261017/rgb_capture_001/rgb/000012.png \
  --results runs/nomad_commissioning_20261017/local_shadow_001/results.jsonl \
  --hz 0.5 --loop
```

All replayed images carry an IO diagnostic/recorded watermark. Status always
identifies `RECORDED_SHADOW_NOT_LIVE`, preserved source timestamp provenance,
and no robot feedback. Message headers use replay time for display; they are
not live sensor timestamps. The only path frame is `nomad_shadow`, a synthetic
capture-body origin reset per record. The green line shows NoMaD's eight
cumulative waypoints, purple the spatially sampled path, yellow the MPC
reference and blue the solver prediction. No world motion is inferred between
records. The profile requires explicitly synthetic zero MPC poses.

The 2026-10-17 capture contains ceiling/beam views. Its nine successful windows
validate camera/model/adapter/solver IO only. They are not evidence of
navigating to a target or avoiding an obstacle. This caveat remains present in
the replay status and panel titles.

## Optional independent VNC desktop

Use this only when a desktop tool such as RViz is useful. It starts a separate
virtual desktop as the current SSH user. It neither shares nor restarts the
physical desktop, and refuses an occupied display or a stale X lock.

Create a private password file interactively on the Jetson. Password values
are never shell arguments, environment variables or script output:

```bash
install -d -m 700 ~/.vnc
tigervncpasswd ~/.vnc/nomad.passwd
chmod 600 ~/.vnc/nomad.passwd
NOMAD_VNC_PASSWORD_FILE="$HOME/.vnc/nomad.passwd" \
  bash deployment/nomad/ops/run_vnc_desktop.sh --check
```

Run the same command without `--check` to start display `:10` on loopback port
5910 in the foreground. The script requires the password file to belong to
the current user and have mode 600 or 400. `NOMAD_PROJECT_ROOT`,
`NOMAD_VNC_DISPLAY_NUM` and `NOMAD_VNC_GEOMETRY` are optional overrides. No
default password is provided.

Forward VNC from the workstation and connect a VNC client to localhost:5910:

```bash
ssh -N -L 5910:127.0.0.1:5910 go2
```

Before connecting, inspect the actual listening sockets on the Jetson. Local
TigerVNC mandatory configuration may override command-line settings; if it
does not listen only on loopback, stop this new session and correct its local
configuration. The script does not edit TigerVNC or desktop configuration.

## Validation status

As of 2026-10-17, shell/Python syntax, layout JSON and the YAML observation
policy have passed static checks. On `go2`, Humble/`cv_bridge` imports
and the user-overlay Foxglove Bridge 3.6.0 startup have passed. The bridge
listens at `127.0.0.1:8765`, and the nine-window recorded replay has actually
published its ROS messages.

A WebSocket SDK protocol handshake succeeded. The server advertised only
`connectionGraph` and these seven shadow channels, with `sysinfo: false`:

- `/nomad/shadow/rgb/compressed`
- `/nomad/shadow/goal/compressed`
- `/nomad/shadow/path`
- `/nomad/shadow/path_sampled`
- `/nomad/shadow/mpc/reference`
- `/nomad/shadow/mpc/prediction`
- `/nomad/shadow/status`

WebSocket subscriptions also received actual binary message data on all seven
channels. The protocol receipt is retained on the Jetson at
`runs/nomad_commissioning_20261017/foxglove_001/handshake.json`.

Foxglove GUI connection, layout import and rendered panels have not yet been
verified. The live preview relay, live ROS camera QoS and VNC have not been
run. The recorded replay is an IO observation check, not a navigation result.
