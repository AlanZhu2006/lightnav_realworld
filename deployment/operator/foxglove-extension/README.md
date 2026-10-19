# DejaView Navigation Console

An English native Foxglove dashboard for the DejaView navigation runtime. Version 0.1.11 follows the Foxglove dark/light theme and responds to the width of the panel. Camera and goal images, telemetry and trajectories occupy the main workspace; Mission, Runtime and Readiness occupy the control column. The registered panel remains **ImageGoal**, so existing layouts keep working. The panel combines camera/goal images, local model preview, odom-anchored model/MPC paths, source freshness, task gates, and acknowledged task controls. **Start preview** starts an observation task. **NoMaD ImageGoal** and **LightNav ImageGoal** use frozen full goal images. The latter is a Go2 ImageGoal adaptation, not an officially validated reference-image capability; its extra instruction is optional. **LightNav Navigation** uses RGB history and a required navigation instruction, with official `objnav` or `vln` task modes and no reference image. Both native buttons start one supervised run, limited to 60 seconds. The button follows the selected model; the execution banner follows the actual running task.

The **LightNav Tracking** task option uses RGB history and a required target description instead of a goal image. Its default description is `Follow the person in the white T-shirt.` Select this option and click **Run LightNav tracking** for one supervised native run, limited to 60 seconds; **Start tracking preview** observes the same task without native execution. Selecting a task never starts or stops a run. ImageGoal, navigation and tracking instructions are remembered separately. The tracking status shows the running target description and distinguishes visible, unknown and safe hold; visibility is a model output, not an independent identity-tracking guarantee.

## Install

Drag `dejaview.dejaview-imagegoal-0.1.11.foxe` into a Foxglove visualization page. Select **ImageGoal** in Add panel (default title: **Navigation Console**). Local extension installation and direct WebSocket connections require a Foxglove developer seat under current Foxglove licensing.

- Package name: `dejaview-imagegoal`
- Publisher: `dejaview`
- Registered panel name: `ImageGoal`
- Layout panel type: `dejaview-imagegoal.ImageGoal`
- Example layout instance key: `dejaview-imagegoal.ImageGoal!operator`

The layout naming above follows the current [LayoutActions API](https://docs.foxglove.dev/docs/extensions/extension-api/interfaces/LayoutActions). The parent operator directory contains the ready-to-import layout and connection launcher.

## Build and verify

Use Node 22.18+ (Node 24 was used for verification) and npm:

```sh
npm ci
npm run typecheck
npm test
npm run package
```

The official `create-foxglove-extension` build and packaging tools generate the `.foxe`. `npm run local-install` installs into the desktop extension directory. Patched compatible build dependencies are pinned in `overrides`; `npm audit` reports zero vulnerabilities at build time.

```sh
npm run test:browser
```

Browser tests use an installed Chrome executable. Override the default macOS path with `CHROME_PATH`. They verify native DOM interactions, acknowledged epochs, stop while start is pending, stale status, HTML text safety, upload limits, and desktop/narrow rendering.

On a Jetson without Node/npm/dependencies, `python3 build_offline.py` builds this dependency-free panel using a deliberately limited, explicit type-erasure pass over the same TypeScript sources. It requires the installed GJS JavaScript runtime, checks generated JavaScript syntax, runs the existing model tests, and exercises native button interactions with a mock DOM before producing the `.foxe`. `dist/offline-build.json` records source/output hashes and validation. This fallback does not run TypeScript type checking or the browser/Foxglove client; use the npm workflow for those checks on the development workstation.

## Runtime contract

The panel subscribes to:

- `/imagegoal/shadow/status` — JSON in `std_msgs/String`
- `/imagegoal/shadow/bridge_status` — JSON in `std_msgs/String`
- `/imagegoal/operator/result` — JSON acknowledgement in `std_msgs/String`
- `/imagegoal/shadow/rgb/compressed` and `/imagegoal/shadow/goal/compressed` — `sensor_msgs/CompressedImage`

It advertises `/imagegoal/operator/command` as `std_msgs/msg/String` with the one-field ROS 2 String message definition. The String's `data` contains `{request_id, session_id, action, payload}`. Panel actions: `start`, `native_start`, `native_stop`, `pause`, `resume`, `cancel`, `complete`, `capture_goal`, `import_goal`, `clear_goal`. The legacy replace action remains part of the adapter contract, but this panel uses the frozen-selection workflow below.

Start/pause/resume/complete/capture/import/clear include `payload.expected_epoch`. The adapter supplies `expected_session_id` to runtime mutations. Start forces `execution_mode: 'shadow'`. NoMaD sends the current canonical `selection_id`; LightNav sends an explicit task mode: `objnav`, `vln` and `tracking` require a trimmed instruction, while `imagegoal` adds a canonical `selection_id` and optional extra instruction. Start never resends JPEG pixels.

`native_start` maps through the ROS operator adapter to `POST /native/start`. All native requests contain `{expected_session_id, expected_epoch, backend, instruction, timeout_s:60, operator_present:true}`. NoMaD adds `selection_id` and requires the runtime-confirmed frozen full goal plus a decoded image. LightNav official navigation adds `task_mode:'objnav'` or `task_mode:'vln'`, requires a nonempty instruction, and sends no `selection_id` or goal pixels. Runtime enforces native-language conditioning. The separate Go2 ImageGoal adapter adds `task_mode:'imagegoal'` plus the frozen `selection_id`, with an optional instruction and no resent JPEG. Runtime selects experimental reference-image conditioning only for this explicit mode. Missing LightNav task modes are rejected rather than silently converted.

A native button requires fresh writable status, `supervised_native_configured`, `operator_supervised`, a supported `supervised_native_backends` entry, and explicit `supervised_native_task_modes` support for LightNav. Shadow LightNav also requires the corresponding `task_modes` capability. LightNav ImageGoal additionally requires `lightnav_imagegoal_adapter:true`, and native adaptation requires `supervised_native_task_modes` to include `imagegoal`. An unavailable capability or empty required instruction shows its reason beside the disabled button. Older NoMaD-only runtimes remain compatible. Clicking the native button expresses supervision for that one run.

Tracking uses `backend:'lightnav', task_mode:'tracking'` and a nonempty trimmed instruction, with **no** `selection_id`, goal pixels or frozen-image prerequisite. Native requests retain current session/epoch, `timeout_s:60` and `operator_present:true`; shadow requests retain the configured timeout and force `execution_mode:'shadow'`. The runtime must advertise `execution_capability.task_modes` and, for native tracking, `supervised_native_task_modes` containing `tracking`. Older runtimes cannot silently interpret tracking as ImageGoal. NoMaD ImageGoal payloads retain their previous shape and default task mode. A running native executor or pending operation blocks a duplicate start.

The actual task reports `task.task_mode`. Top-level `tracking` reports `{enabled, instruction, visible, target_state, hold_reason, source_frame_id, source_stamp_ns}`; the panel reads its visibility and hold state without inventing a target bounding box. `TRACK_HOLD` means motion is held while the tracking episode continues to observe. Model STOP or a zero path holds motion while continuing observations. Target visibility is diagnostic only, following the official pointing output; unknown/negative visibility does not independently gate motion. These outputs do not establish identity recovery. The native stop/cancel barriers remain authoritative.

**Stop robot** sends `native_stop` to `POST /native/stop` with the last known `expected_session_id`. It stays independent of a pending start and remains available with stale status when a writable publisher and session identity are still known. A stop supersedes pending start acknowledgements; an old start cannot re-enable execution in the panel. **Stop task** also uses native stop when the native executor/task or a pending native start exists; ordinary shadow stop remains `cancel`. Native stop is responsible for cancelling the task and ending its executor on the backend.

The native banner and execution label use the actual `task.backend`, even if the model selector changes during a run. `status.native_execution.backend` also records the backend selected by the runtime.

`status.native_executor` reports `running`, `state`, `pid`, `task_id`, `error`, and `report_path`; `status.native_execution` reports authorization, command and sensor/plan provenance. The panel displays process state and authorization separately: an accepted start or a running process does not establish robot motion. Full details remain available in diagnostics. No control panel button publishes velocity or SDK messages directly.

For NoMaD and LightNav ImageGoal, the goal frame initially follows the selected live RGB stream and is explicitly labelled **Live preview · no goal captured**. Start stays disabled. **Capture frame** captures the latest raw inference-path RGB and freezes it in the runtime without starting a task; it can differ from the display by one inference sampling interval. **Import JPEG** commits original JPEG bytes to the same runtime selection API. Imports accept JPEG only, maximum 4 MiB and 4096 × 2160 pixels; no crop, rotation, or transcode is applied. A successful capture/import returns `{status, goal:{rgb_jpeg_base64}, selection}`. The panel waits for confirmed canonical selection and successful decoding of its frozen image before enabling Start.

`status.goal_selection` declares `{schema_version:'imagegoal.goal_selection.v1', mode:'live'|'frozen', ready, selection_id, sha256, source, source_frame_id, source_metadata, selected_timestamp_ns}`. The frozen target remains fixed while RGB updates. **Reset goal** calls `clear_goal`, which atomically stops the old task and returns to live preview. An ordinary Stop preserves the selected target. Reconnection restores the canonical selection from status; the panel accepts a goal image only when `header.frame_id` equals `imagegoal_frozen_goal_` plus that selection ID. Missing or older frozen images cannot enable Start.

Results are `{request_id, action, ok, result?, error?}`. A successful mutation returns the current runtime status directly in `result`; the panel applies it immediately and ignores lower epoch/timestamp status messages from the same session. Capture/import return the structured selection result above; clear returns full status. Unknown acknowledgements are ignored.

A start or goal mutation requires a writable data source and status received within 2.5 seconds. Acknowledgements time out after 10 seconds and block additional mutations until a confirmed stop. Stop bypasses pending non-stop requests and supersedes their acknowledgements. A runtime session or camera-source change clears pending requests, invalidates unfinished upload decoding, and prevents old capture/import acknowledgements from restoring an obsolete target. Controls never publish velocity, TF, odometry or robot SDK messages.

Body-local previews require current epoch/session/task/goal identities. Terminal tasks and identity mismatches clear old paths. Odom/MPC arrays share the plan's frame; an odom pose from a different frame is omitted. The two coordinate systems are never overlaid. Inference-input ages use runtime values plus local elapsed time. Only the bandwidth-limited `/imagegoal/shadow/rgb/compressed` preview is subscribed. Runtime RGB `frame_name` / `timestamp_source` and image headers validate camera provenance; native Go2 and RealSense previews are never mixed. The full-resolution native stream is not subscribed. Camera-preview source time comes from `CompressedImage.header.stamp` (`nanosec` or Foxglove-normalized `nsec`, with `nanosec` taking precedence); browser/camera clock differences affect this displayed time difference. Legacy `imagegoal_display_time_preview` headers are never treated as source capture time. Native Go2 headers carry host RPC receive time, not sensor exposure time; the panel labels their clock difference as **RPC receipt time difference** and does not claim exposure latency. Receipt age remains separate. RGB rendering keeps only the latest message per Foxglove render frame, with one in-flight decode and one newest pending frame. New images decode offscreen and replace the displayed image atomically; old good frames and their object URLs remain valid until replacement. The live goal shares committed frames without resetting its source on every status update. RGB-only updates do not rebuild the dashboard.

Form settings (model, task mode, separate ImageGoal/navigation/tracking instructions, timeout) persist in the Foxglove layout. Goal bytes and selection authority live in the runtime and are not persisted in layout JSON. Restoring these settings does not start a task.

## Browser preview harness

```sh
npm run demo:build
npm run demo:serve
```

Open `http://127.0.0.1:8098/`. The harness invokes the actual native panel with a mock extension context. A persistent **DEMO / Simulated data / No robot connection** strip labels every interactive fixture, including `native`; the `live` fixture is labelled **RECORDED SNAPSHOT**. “Connected” inside the panel means its mock context is supplying status, not that a robot is online. This harness is not a robot control endpoint.

- `?fixture=body` (default): local model preview without odom.
- `?fixture=odom`: model path plus MPC reference/prediction in one odom frame.
- `?fixture=native`: simulated NoMaD/LightNav native capabilities, start and priority stop for browser tests; no robot connection.
- `?fixture=stale`: status stops updating so controls disable.
- `?fixture=live`: reads `demo/fixture-status.json`, `demo/live-rgb.jpg`, and optional `demo/live-goal.jpg` as a **recorded snapshot**. Publishing and all controls are disabled. Original source ages are frozen, and the dashboard explicitly says it does not represent current connectivity.

Existing RGB topic schemas are unchanged. The bridge must preserve the source header and its timestamp provenance. For Unitree VideoClient this is explicitly host RPC receive time and an uncalibrated logical optical frame, rather than a sensor capture timestamp. `bridge_status.preview` metadata is optional and is not required by this panel.

Official references: [custom panel guide](https://docs.foxglove.dev/docs/extensions/guides/create-custom-panel), [panel API](https://docs.foxglove.dev/docs/extensions/extension-api/type-aliases/PanelExtensionContext), [installation](https://docs.foxglove.dev/docs/extensions/local-development).
