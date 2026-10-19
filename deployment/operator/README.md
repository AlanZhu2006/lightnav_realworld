# Navigation Console

Foxglove extension **0.1.11** provides an English console with dark/light themes,
responsive task cards, live camera and goal images, trajectories, source freshness,
and separate mission/runtime information. **Stop robot** stays in the sticky header.
The registered `ImageGoal` panel ID, ROS topics, task payloads and existing backend
capability checks are unchanged.

## Install the updated interface

Drag the [0.1.11 extension](dist/dejaview.dejaview-imagegoal-0.1.11.foxe) into a Foxglove
visualization page. Existing ImageGoal panels continue to work. Import the
[English console layout](native_imagegoal.foxglove-layout.json) to update a saved
panel title to **Navigation Console**; the [standard console layout](imagegoal-console.foxglove-layout.json)
uses the same panel. Updating this extension does not require a robot runtime restart.

The [extension guide](foxglove-extension/README.md) documents builds, browser checks,
all controls, image decoding and the full command/acknowledgement protocol.

## Prepare services and connect

On the current **GPU workstation**, from the repository root:

```bash
bash deployment/operator/open_lightnav.sh --no-open
```

Use the deployment's assigned `CUDA_VISIBLE_DEVICES` when a cold model start is
needed. The helper reuses a healthy model already reachable by Jetson; otherwise
it prepares the existing local model environment, waits for weights to load,
and establishes the reverse inference connection. It prepares the existing Go2
observation profile and preserves a frozen goal when switching from shadow
configuration. It does not replace a running task, start an executor, or move Go2.
This command must not be used on a Mac to cold-start the GPU model. Maxwell still
requires its existing GPU coordination procedure; the helper only reuses a healthy
model there and will not automatically start one.

On the operator Mac, connect directly to the current Jetson address:

```bash
ssh -NT -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=2 -o ServerAliveCountMax=10 \
  -L 18766:127.0.0.1:8766 go2
```

Keep this terminal open, then connect Foxglove to `ws://127.0.0.1:18766`.
Do not create another forward while an existing connection is healthy. If a
helper prints another local port because the preferred one is occupied, use its
printed URL. Connection recovery never restarts an old navigation task.

For workstation viewing, `open_lightnav.sh` without `--no-open` also opens Foxglove
locally. `--layout-id YOUR_LAYOUT_ID` selects a saved layout. The equivalent helper
is `python3 deployment/operator/open_console.py --backend lightnav --prepare-native`.
To connect NoMaD only, use `python3 deployment/operator/open_console.py --backend nomad`.
See [bringup instructions](../bringup/README.md) for service status/stop commands
and optional forwarding of the runtime web interface at port 18093.

## Choose a task

| Mode | Input | Model profile |
|---|---|---|
| NoMaD ImageGoal | Frozen full goal image | Existing NoMaD goal-conditioned policy |
| LightNav ImageGoal | Frozen full goal image; optional extra instruction | Explicit Go2 reference-image adaptation, experimental |
| LightNav Navigation | Required instruction; ObjNav or VLN | Official RGB-history and language interface |
| LightNav Tracking | Required target description | Official tracking profile / `trackvla` |

Navigation and tracking do not send a goal image. Example navigation instruction:
`Go to the coat rack beside the windows.` Default tracking target:
`Follow the person in the white T-shirt.` Each task keeps its own instruction text.
Selecting a mode, restoring settings, installing the extension or reconnecting
never starts a task.

**Start preview** runs the selected policy in shadow mode. **Run ... on robot**
requests one supervised native run, limited to 60 seconds and the runtime's
existing capabilities. Tracking uses **Run LightNav tracking**. Model availability,
source freshness and executor authorization are displayed separately: a running
process or accepted start does not prove robot motion.

**Stop robot** uses the independent priority native-stop path, including while
another operation is pending. Pause, resume, stop task and manual arrival
confirmation retain their existing behavior. Policy STOP is not independent
arrival verification. Tracking can hold on model STOP/zero trajectories and
continue observing; a fresh eligible plan can resume within the same active run.
An operator stop ends that run. Target visibility is the official model's output,
not an independent ReID guarantee. See [tracking details](../imagegoal/TRACKING.md).

## Image goals

Before a goal is chosen, the goal view follows the live preview and Start is
disabled. **Capture frame** freezes the latest full RGB from the inference path;
it can differ from the displayed preview by one sample. **Import JPEG** retains
the original JPEG, up to 4 MiB and 4096 × 2160 pixels, without cropping or resizing.
Capture/import does not start a task. Start requires a runtime-confirmed selection
and its successfully decoded image. Goal selection is identified by its canonical
selection ID and SHA256; goal pixels are not resent in the Start command.

**Reset goal** stops the current task, clears the selection, and returns to live
preview. Ordinary Stop preserves the goal. Reconnecting the same runtime restores
its selection; restarting the runtime does not resume the old task. Stale images,
old epochs and late acknowledgements cannot restore an obsolete selection.

## Images, trajectories and diagnostics

The console subscribes to the bounded preview `/imagegoal/shadow/rgb/compressed`,
not the full-resolution native stream. Frozen-goal subscription is demand-based
and stops after the confirmed goal is decoded. Image decoding retains one active
and one newest pending frame; RGB updates do not rebuild the dashboard.

Body-local model previews and odom-anchored model/MPC paths remain separate.
Missing real odometry leaves world paths empty; it is never replaced with a zero
pose. Plan/task/session/goal identities and source frames must match before a
path is displayed. Source ages and inference RTT remain visible.

Unitree image timestamps are host RPC receipt times, not measured sensor exposure
times. Browser/robot clock offsets affect the displayed time difference. The UI
labels this limitation and does not claim to measure complete camera latency.
Runtime gate messages have English labels; raw diagnostic data remains available.

No button publishes raw velocity, TF, odometry or robot SDK commands directly.
The independent runtime/executor still validates every physical command. The
separate RGB-D operator-run path retains its calibration requirements; the
supervised native path retains its current commissioning and operator limits.

Offline browser fixtures are explicitly marked simulated or recorded snapshots.
They validate UI behavior, not current connectivity, real motion or navigation
success. The UI update does not enable a NavGPT-3 backend before its actual API
and runtime contract are connected.
