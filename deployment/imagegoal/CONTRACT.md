# ImageGoal task runtime, version 1

The runtime listens on loopback port 8093. It never imports a Unitree SDK or
publishes a robot command. `proposal.vx_m_s / vy_m_s / wz_rad_s` are diagnostic
data. Default tasks retain `not_for_motion: true`; the explicitly configured
operator-run profile can issue a separate, short-lived execution envelope.
Simulated feedback is explicitly
marked and uses a moving unicycle, not a fixed zero pose. A physical robot
transport is deliberately a separate deployment boundary.

`GET /status` returns `schema_version: imagegoal.runtime.v1`, session identity,
task, state, `gates` (reason strings), sensors/ages, proposal, current plan,
metrics, canonical `goal_selection` and the latest event sequence. `GET /events?after=0` returns events with
strictly increasing `sequence`. `GET /frame/latest.jpg` and `/goal.jpg` return
the most recent RGB and sealed full goal. `GET /run.zip` exports the run evidence.

Goal selection starts as `{mode:"live",ready:false}`. `POST /goal/capture`
freezes a fresh original RGB, `POST /goal/import` freezes an externally supplied
`goal.rgb_jpeg_base64`, and `POST /goal/clear` stops the active task and returns
to live preview. All three require `expected_session_id` and `expected_epoch`.
Capture/import return status with a unique `selection_id`, SHA256, source
metadata and stored JPEG path; neither creates a task or starts inference.
Invalid imports leave the previous target intact. Ordinary Stop preserves it.

`GET /goal.jpg?selection_id=ID` returns only that current frozen selection,
with `X-ImageGoal-Selection-Id`; a mismatched ID is rejected. Runtime restart
starts in live mode. A same-process reconnect recovers the current frozen goal.

The Foxglove console starts the already selected image explicitly:

```json
{"selection_id":"ID_FROM_STATUS","expected_session_id":"CURRENT_SESSION","expected_epoch":1,"backend":"nomad","timeout_s":120,"instruction":"","execution_mode":"shadow"}
```

`POST /tasks` creates and starts a task, cancelling any previous task. The
legacy inline-goal form remains an atomic external-import/start operation:

```json
{"goal":{"rgb_jpeg_base64":"..."},"backend":"nomad","timeout_s":120,"instruction":"","execution_mode":"shadow","expected_epoch":0}
```

Backends are `nomad` and `lightnav`. Both can use `task_mode:imagegoal` with a
frozen `selection_id` or full inline `goal`. LightNav ImageGoal explicitly uses
`control_profile:go2_imagegoal_adaptation` and
`conditioning_mode:experimental_reference_image`; its instruction is optional.
It is a deployment adaptation, rather than an official ImageGoal capability claim.
`execution_capability.lightnav_imagegoal_adapter:true` declares that profile.

The official LightNav modes `objnav`, `vln` and `tracking` remain separate, with
a required `instruction` and without `goal`, `goal_image` or `selection_id`. For example:

```json
{"backend":"lightnav","task_mode":"objnav","instruction":"Go to the coat rack by the windows and stop.","execution_mode":"shadow","expected_epoch":0}
```

`POST /goal/capture` selects an image target for either ImageGoal backend; it never starts navigation.
Capture is not proof of arrival.
`POST /task/goal` replaces the active full
goal using `{"goal":{"rgb_jpeg_base64":"..."}}`; goal version and execution
epoch increase immediately, old paths and in-flight responses become invalid.

`POST /task/cancel {"reason":"operator_stop"}` synchronously stops the shadow
proposal. `/task/pause` and `/task/resume` explicitly pause/resume an active
task. `/task/complete {"evidence":"operator_confirmed"}` records an operator
outcome. A fresh official LightNav navigation STOP ends the task with
`SUCCEEDED`, reason `official_lightnav_model_stop`, and
`arrival_independently_validated:false`, matching upstream model-driven
completion. Adapted LightNav ImageGoal similarly stops with reason
`go2_imagegoal_model_stop` and does not assert independently verified arrival.
Tracking STOP holds movement and continues its episode. NoMaD's
optional distance stop remains a hold, without automatic success.

All input POSTs require JSON. RGB:

```json
{"frame_id":"camera-42","capture_timestamp_ns":1790000000000000000,
 "timestamp_source":"realsense_ros_header_system_time",
 "clock_domain":"unix_epoch_ns","frame_name":"camera_color_optical_frame",
 "synthetic":false,"rgb_jpeg_base64":"..."}
```

Odom:

```json
{"capture_timestamp_ns":1790000000000000000,"pose_m_rad":[0,0,0],
 "velocity_v_w":[0,0],"frame_name":"go2_odom",
 "timestamp_source":"go2_dds_receive_host_system_time_not_sensor_source_time","clock_domain":"unix_epoch_ns",
 "synthetic":false,"commissioning":false,"body_frame_name":"base_link"}
```

Depth:

```json
{"capture_timestamp_ns":1790000000000000000,"depth_png_base64":"...",
 "scale_m":0.001,"timestamp_source":"realsense_ros_header_system_time",
 "clock_domain":"unix_epoch_ns","frame_name":"camera_color_optical_frame",
 "synthetic":false,"aligned_to_rgb":true}
```

Depth PNG must be single-channel uint16. The runtime calculates the original
GEM front-ROI clearance from metric depth; caller-supplied clearance values
are not accepted. Depth is for the independent execution gate, not NoMaD's
model input. The initial gate checks freshness and conservative center-ROI
clearance; it does not certify the full swept footprint or all obstacles.

Ingress endpoints are `/inputs/rgb`, `/inputs/odom`, `/inputs/depth`. Unix
epoch nanoseconds and explicit clock provenance are mandatory. Wall-clock
future timestamps, out-of-order streams, frame changes and mixing synthetic
with real sources are rejected. Capture-pose interpolation uses the original
LightNav geometry implementation; current feedback is independently checked.

Optional ROS bridge subscribes to real RGB/depth/odom and emits only
`/imagegoal/shadow/status` (std_msgs/String), `/imagegoal/shadow/path`,
`/imagegoal/shadow/reference`, `/imagegoal/shadow/prediction` (nav_msgs/Path).
Synthetic frames use `imagegoal_sim_odom`; physical feedback keeps its measured
odom frame. There is no TF, Odometry or cmd_vel publisher in this package.

The CLI accepts `--static-dir` for a separate operator UI. `/api/...` is an
alias for the corresponding endpoint. Runtime artifacts include sealed goals,
submitted RGB copies, requests/responses, events, 10 Hz status/commands and an
atomic latest status. The simulated replay profile retimestamps playback into
an explicit `synthetic_playback_clock_not_sensor_exposure` domain and preserves
original frame timestamps in provenance. Replay never establishes navigation
success.

## Explicit operator-run authority

The default runtime execution profile and task mode are both `shadow`.
Starting with `--execution-profile operator-run --robot-id ID --mount-receipt
FILE --timing-receipt FILE` only permits a later task to request
`execution_mode: "operator_run"`; it does not authorize existing/default tasks.
`execution_capability` reports `operator_run_configured`,
`calibration_verified`, `commissioning`, and concrete `blocking_reasons`.

The calibration helper [calibration.py](calibration.py) validates both raw
receipt hashes, their robot ID, measured-evidence file hashes, verified-by
identity/time, mount SE(3), exact RGB/depth/odom frame and source provenance,
same-host boot ID and clock domain, and <=50 ms recorded alignment uncertainty.
Unverified templates never pass. A measured stamp correction is applied once
to the original header: `estimated_capture = raw_header + stamp_correction`;
both times and the receipt hash remain in each sensor's metadata. This is not
a re-stamp at inference return. Missing source measurements remain blocked.

Only a matching active operator-run task, real non-commissioning inputs,
verified calibration, valid source ages, accepted capture-anchored plan and
current MPC feedback can produce `transport_authority: "operator_run"`,
`authority_schema: "imagegoal.robot_execution_authorized.v1"`,
`eligible_for_robot_tracking: true`, and `not_for_motion: false`. The command
inside that envelope has `not_for_motion: false`; the raw `proposal` remains
diagnostic. The envelope includes task/goal/epoch, monotonic sequence,
last-controller evaluation time, source provenance, calibration hashes and
gate snapshot. Its lease is at most 250 ms and ends sooner when any source
expires, including measured alignment uncertainty. HTTP status polling cannot
renew an old controller evaluation. Cancel, pause, goal change and failed
gates immediately withdraw authority. A separate sole Go2 adapter still needs
its own explicit operator binding and hardware preflight before SDK startup.

`--commissioning-real-inputs` is an offline real-schema fixture mode, not a
physical source. All fixture inputs require `commissioning: true`; envelopes
stay `transport_authority: "commissioning_only"`, `not_for_motion: true`, and
`eligible_for_robot_tracking: false`. The corresponding positive software
decision is exposed only as `candidate_transport_authority: "operator_run"`
with `eligible_for_commissioning_tracking: true`. Only FakeSportTransport may
consume this distinct branch; physical validators always reject it.

Task create and goal replacement accept `expected_epoch`, atomically compared
inside the task lock. It is mandatory for operator-run tasks. UI clients send
the epoch from their latest status for both modes. Stop advances the epoch
even while IDLE, so an older target POST that finishes arriving after Stop
is rejected instead of creating a new task. Legacy shadow clients can omit
the field; they do not receive this concurrent-target protection.

NoMaD's optional `--nomad-stop-distance-steps` is disabled by default.
`--nomad-stop-consecutive` must be at least two. Only fresh, contract-valid,
same-goal/epoch responses with distinct source frame IDs count; failed gates
reset the streak. Configured consecutive low temporal scores latch
`MODEL_STOP` and zero, never success. `stop_policy` reports mode, threshold,
streak and latch. These learned steps are not meters; a field-calibrated
threshold is necessary before using this automatic hold physically.
