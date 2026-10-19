# ImageGoal task runtime

This is the shared robot-side task scheduler for NoMaD and the separate
LightNav reference-image deployment. It runs real model inference, tracks the
result with the unchanged, hash-checked LightNav CasADi MPC, reuses GEM's
unchanged depth and visual-plan age guards, and records a complete task trace.
There is no Unitree SDK import, cmd_vel publisher or robot transport here.

The API, browser UI and ROS observer are usable without Go2 feedback. Missing
measured odometry keeps proposals at zero; real images and model responses
remain inspectable. An independent simulated profile supplies moving feedback
and synthetic depth to exercise the full task/controller loop. Its recorded
images do not change with the simulated pose, so this is an IO/controller
commissioning tool, not a navigation benchmark.

## Run with measured inputs

Use the validated NoMaD environment with NumPy, Pillow and CasADi installed.
Start the model server separately; on the current Jetson its loopback endpoint
is port 18883. LightNav uses a separate endpoint on port 18884.

```bash
python -m deployment.imagegoal \
  --profile external --output runs/imagegoal/UNIQUE_RUN \
  --nomad-url http://127.0.0.1:18883 \
  --lightnav-url http://127.0.0.1:18884 \
  --static-dir deployment/operator/web --port 8093
```

After sourcing ROS Humble and its camera overlay, start the observer in an
environment with `rclpy`, `cv_bridge`, Pillow and NumPy:

```bash
python -m deployment.imagegoal.ros_bridge \
  --runtime-url http://127.0.0.1:8093 \
  --rgb-topic /camera/camera/color/image_raw \
  --depth-topic /camera/camera/aligned_depth_to_color/image_raw \
  --odom-topic /go2/odometry --depth-scale-m 0.001
```

The observer pairs original RGB-D headers within 40 ms, samples distinct
frames at 4 Hz, and uses a latest-only handoff. It does not open a second
RealSense device. NoMaD uses four chronological images; LightNav appends real
observations to its server-side episode history. There is exactly one
model request in flight. New requests consume the newest complete history;
slow inference never creates a request queue. The observed Jetson FP32 model
rate is below 4 Hz, while its input history continues to sample at 4 Hz.

Browse the forwarded loopback service, upload a complete RGB goal or capture
the current frame, then start a task. API details are in [CONTRACT.md](CONTRACT.md).
Task cancellation synchronously clears the plan and zeros the proposal; a
late response cannot revive it. Replacing a goal increments goal version and
execution epoch, invalidates old MPC state, and requires a post-change image.
Pause/resume follows the same freshness rule. Overall time budgets use a
monotonic clock and continue during pauses.

NoMaD distance is a learned temporal score, not a metric arrival detector.
LightNav ImageGoal uses a full frozen goal image and the explicit Go2 adaptation
profile; its supplementary language instruction is optional. Reference-image
conditioning and bounded controller compensation are recorded as deployment
adaptations. See [the ImageGoal adaptation](GO2_IMAGEGOAL_ADAPTATION.md).
Official LightNav navigation (`objnav`/`vln`) remains available separately with
RGB history and a required language instruction, without a goal image. STOP completes the
task and releases the executor, as upstream does; the outcome separately records
that arrival was not independently verified. Tracking STOP temporarily holds
motion and continues observing. NoMaD retains explicit operator outcome handling.
See [the official LightNav deployment contract](OFFICIAL_LIGHTNAV.md).

### LightNav upload limits and retirement

The HTTP lifecycle v2 protocol checks task capacity with a small metadata request
before the first image upload and binds requests to the model server's boot ID.
By default a request is limited to eight pending frames, 384 KiB of serialized
JSON, and two seconds of pending source age. Configure these resource budgets
with `--lightnav-max-pending-frames`, `--lightnav-max-request-bytes`, and
`--lightnav-max-pending-age-s`. Exceeding a budget ends the episode explicitly;
the runtime never trims its history to make the request fit.

A LightNav HTTP/transport failure ends the current episode, clears execution,
and cancels its owned connection. It does not retry an expanding JPEG history.
The operator must explicitly start a new task after the connection recovers.
A verified history acknowledgement may advance even when its trajectory has
expired; acknowledging observations cannot authorize motion.

Terminal cleanup immediately retires the exact task identity through
`POST /reset` with `retire: true`. Retirement blocks late requests independently
of GPU availability; physical resource release runs on the serial model worker.
Cleanup failures remain pending with bounded exponential backoff until shutdown.
The model service additionally retires idle, non-inflight sessions after its
120-second default TTL when handling service traffic. `POST /session/status`
reports retirement, pending release, and the last committed observation without
waiting for inference. It does not renew idle sessions. Neither metadata queries
nor reconnection resume a robot task.

NoMaD can optionally latch a stop proposal using a physically calibrated
temporal-score threshold: `--nomad-stop-distance-steps VALUE
--nomad-stop-consecutive 3`. The threshold is disabled by default and is
reported as `stop_policy.mode=operator_only_unconfigured`. When configured,
only contract-valid, fresh, current-goal/current-epoch responses from distinct
source frames count toward the streak; invalid responses, failed gates and
goal changes reset it. Reaching the configured streak latches `MODEL_STOP`
and zero rather than `SUCCEEDED`. The learned score is not meters. Upstream's
3.0 close-threshold is a reference for commissioning, not an automatically
valid physical threshold for this single-goal deployment.

## Feedback and gates

Capture odometry comes from the original LightNav interpolation function.
Each accepted path is anchored once at the newest image's capture pose, then
the original MPC uses fresh measured feedback at 10 Hz. Solving never changes
the source timestamp. NoMaD is scaled exactly once by its pinned 0.05 m/model
unit, then spatially resampled; yaw is the path tangent. LightNav preserves
its original ten metric SE(2) poses and in-place rotations in the official
navigation/tracking profiles, without resampling or adding an origin. Their reference selection, local-frame optimization, weights,
previous-command state and output scaling follow the official Go2 launch.
The existing hardware velocity envelope is applied after that output scaling.
The separate Go2 ImageGoal profile resamples references and applies configurable
speed-floor and heading compensation, bounded for short predicted local actions.

The execution loop checks source RGB and plan age, current odom age, capture
odom gap, depth age, aligned RGB-D frame/skew, valid depth and conservative
front clearance. A failed gate clears the active path, outputs zero, and
requires a new post-stop image and accepted plan before continuing. Explicit
operator stop, timeout, error and model STOP do not automatically resume.

Every status includes an execution envelope: session/task/goal/epoch, source
lineage, gate snapshot, measured input provenance and ages, expiration and
bounded command. Default authority is `shadow_only_no_robot_transport`.
An explicit operator-run software profile and per-task mode can issue a
250 ms physical envelope only after installation/time calibration and all
real-input gates pass. The sole GEM transport consumes this envelope plus
its own operator binding, actuator watchdog and actual Go2 feedback; the
browser is never a velocity controller.

## Full replay and fault commissioning

The replay manifest is a captured RGB manifest with four or more real,
chronological source images. Playback has an explicit synthetic wall clock;
original captures and identities stay in the request provenance. The ideal
unicycle integrates the actual MPC proposal and reports measured simulated
velocity, rather than repeatedly resetting to a zero pose.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m deployment.imagegoal \
  --profile simulated-replay --manifest CAPTURE/manifest.json \
  --goal CAPTURE/rgb/000012.png --nomad-url http://127.0.0.1:18882 \
  --output runs/imagegoal/UNIQUE_REPLAY --duration-s 30 \
  --static-dir deployment/operator/web --port 8093
```

`--scenario scenario.json` schedules simulation-only operations. Example:

```json
[
  {"after_s":3,"action":"fault","name":"response_delay_s","value":2},
  {"after_s":4,"action":"goal","path":"another_goal.jpg"},
  {"after_s":7,"action":"fault","name":"response_delay_s","value":0},
  {"after_s":9,"action":"fault","name":"obstacle","value":true},
  {"after_s":11,"action":"fault","name":"obstacle","value":false},
  {"after_s":13,"action":"fault","name":"depth_paused","value":true},
  {"after_s":15,"action":"fault","name":"depth_paused","value":false},
  {"after_s":17,"action":"pause"},
  {"after_s":18,"action":"resume"},
  {"after_s":22,"action":"cancel","payload":{"reason":"commissioning_stop"}}
]
```

Additional source faults are `rgb_paused` and `odom_paused`. Fault injection
is rejected in the measured-input profile. Inference delay is applied after
a real model response, allowing old-goal and over-deadline response handling
to be exercised without replacing the model with a stub.

Artifacts include `run.json`, `events.jsonl`, 10 Hz `status.jsonl`, atomic
`status.json`, sealed goal images, submitted RGB, request lineage and full
model responses. Download the current run through the operator UI, or audit
its feedback/control invariants without any hardware connection:

```bash
python -m deployment.imagegoal.audit_run RUN --output RUN/audit.json
```

## Verified commissioning on 2026-10-17

The workstation real-NoMaD run `.runtime/imagegoal_commissioning_001` completed
69 requests, accepted 51, rejected 18, and solved the original MPC 139 times
without an MPC error. Synthetic feedback moved 1.48 m from its initial pose.
Goal-switch stale responses, response deadlines, obstacle stop, stale depth,
stale odom, pause/resume and cancel were exercised. Artifact auditing passed
all recorded invariants. A separate real-model run reached `TIMED_OUT`, then
retained zero proposals. These checks establish task/controller behavior;
they do not establish physical navigation success or collision safety.

ROS output is limited to `/imagegoal/shadow/status`, bridge counters,
`path`, `reference`, `prediction`, and watermarked RGB/goal compressed previews.
Preview headers are display-time stamps; original sensor stamps remain in
status. No TF, Odometry, Twist or motion publisher is created.

## Operator-run software profile and calibration

The execution implementation is complete even when the Go2 link is blocked.
Missing physical feedback/calibration stays a hard preflight failure; no
software fixture is substituted into a physical task. See
[CONTRACT.md](CONTRACT.md#explicit-operator-run-authority) for exact rules and
the independent Go2 adapter in `deployment/go2/` for hardware preflight.

```bash
python -m deployment.imagegoal --profile external --port 8093 \
  --output runs/imagegoal/UNIQUE_OPERATOR_SERVICE \
  --nomad-url http://127.0.0.1:18883 --lightnav-url http://127.0.0.1:18884 \
  --execution-profile operator-run --robot-id YOUR_GO2_ID \
  --mount-receipt config/calibration/mount.json \
  --timing-receipt config/calibration/timing.json \
  --static-dir deployment/operator/web
```

This still creates shadow tasks by default. Only a task POST explicitly
containing `"execution_mode":"operator_run"` requests authority. Do not mark
the provided templates verified until real installation, source timing and
measurement evidence exist. Their default `verified:false`, missing evidence
and placeholder boot ID intentionally fail validation. Timing receipts must
be measured again after a host reboot; a camera reinstall needs a new mount
receipt. Calibration evidence and raw receipt hashes are preserved in run
artifacts and checked independently by the physical bridge.

Offline authority commissioning uses the real NoMaD model and moving fake
feedback, while every input/receipt remains explicitly commissioning:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m deployment.imagegoal.commission_execution \
  --manifest CAPTURE/manifest.json --goal CAPTURE/rgb/000012.png \
  --server-url http://127.0.0.1:18882 --output runs/imagegoal/UNIQUE_AUTHORITY_AUDIT
```

It exercises default-task isolation, explicit operator candidates, stale
feedback withdrawal, fresh-plan recovery, non-renewable controller leases,
cancel and physical rejection of fixtures. A recorded candidate can also
drive `python -m deployment.go2.audit_adapter --fixture CANDIDATE_STATUS_JSON`
using the unchanged GEM bridge methods and FakeSportTransport. No DDS, SDK
client or physical camera is initialized by these audits.

For a bounded real-model stop-policy check using recorded same-view images:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m deployment.imagegoal.commission_nomad_stop \
  --manifest CAPTURE/manifest.json --goal CAPTURE/rgb/000012.png \
  --alternate-goal CAPTURE/rgb/000011.png --threshold-steps 3 --consecutive 3 \
  --server-url http://127.0.0.1:18882 --output runs/imagegoal/UNIQUE_STOP_AUDIT \
  --duration-s 8
```

The explicit threshold in this audit is an engineering stimulus. It does not
claim successful real navigation or physical threshold calibration.
