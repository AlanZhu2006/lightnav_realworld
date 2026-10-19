# LightNav native profiles and Go2 ImageGoal adapter

LightNav is an independent baseline beside DejaView → NoMaD. It exposes two
explicit choices. The released language-navigation/tracking profiles consume
real egocentric RGB history and a nonempty instruction. The Go2 ImageGoal
profile additionally consumes a destination photo as a deployment extension.
`native_language` remains the default; supplying a photo never implicitly
selects the extension. DejaView/NoMaD retains its own image-goal pipeline.

| Conditioning mode | Task modes | Input beyond real observations | Response implementation |
| --- | --- | --- | --- |
| `native_language` | `objnav`, `vln` | Required instruction; no goal image | `official_release` |
| `native_tracking` | `tracking` | Required instruction; no goal image | `official_release` |
| `experimental_reference_image` | `imagegoal` | Required full goal image; optional instruction | `go2_imagegoal_adaptation` |

The ImageGoal path is explicitly an inference adapter, not a claim that the
released checkpoint has a separate official ImageGoal API or evaluation. It
preserves the original weights, preprocessing, real observation history and RVQ
decoder while adding an independently labelled static reference segment and
prompt prefix. Navigation quality still requires actual task evaluation.

## Source and model contract

The official reference is [LightNav-0 commit
3015508](https://github.com/lightorigins/LightNav-0/tree/3015508b70fb6e30bbd66701e94c9b916dcb8df8).
The existing source mirror in `vendor/upstream` remains pinned to the successful
[AlanZhu2006/lightnav_realworld](https://github.com/AlanZhu2006/lightnav_realworld)
commit `ddfd37059c49e8090217e11c4ffa6f6b3ff9d554`, preserving its notices and
`SOURCE_MANIFEST.json`. Startup verifies every mirrored file, then separately
verifies all 42 copied Python module hashes, apart from the unused mirrored
WebSocket server, against the official reference. These 42 files are
byte-identical to the official release; the fork's WebSocket telemetry changes are not used by this HTTP adapter.
`official_release_commit` and `source_mirror_commit` report both identities.
`official_core_sha256` is a deterministic SHA256 of the sorted map of all 42
module hashes, returned by health and every inference profile.

The official checkpoint is `LightOriginsHQ/LightNav-0`, revision
`826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`. Its safetensors shard SHA256 is
`ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
All checkpoint assets are checked against their generated manifest before load.
No weights enter Git.

Both native model paths use upstream `NavigationPolicy.observe` and its complete
SlowFast history, upstream `engine.generate_from_frames` with the same HF
arguments as `BatchedTrackingService`, and upstream
`TrackingAgent.decode_waypoints`/`decode_prediction_signals`. Native samples, prompts,
preprocessing, RVQ decoding, and STOP interpretation are unchanged. Native calls
do not disable sample caches or reset engine state before every prediction;
upstream session reset manages episode state. The HF backend processes the full
sample and does not use the optional vLLM ViT cache.

`native_language` uses `tasks.vlnce` and the upstream `vlnce_traj` generation
path. Both `objnav` and `vln` task modes use that path. `native_tracking` uses the
checkpoint's explicit `tasks.trackvla` settings and upstream `tracking` path,
without falling back to VLN. Navigation and tracking share loaded HF weights and
tokenizer while retaining their own task bundle, data processor, engine and
session history. A missing tracking task or incompatible ten-step RVQ action
bundle is an initialization error, not a second model load or fallback.

For `experimental_reference_image`, the original `build_vln_traj_sample` is
built first. The full static destination photo is resized by the same upstream
frame preprocessing and duplicated into a two-frame Qwen3-VL tubelet. This
separate visual segment is prepended to the sample, accompanied by a prefix
that identifies it as a stationary reference and anchors pointing coordinates
to the newest real robot observation. It is never inserted into observation
history, frame IDs, source timestamps, or SlowFast memory. This branch calls
`engine.generate(sample)` and sets `_allow_vit_cache=false` to prevent reference
IDs from aliasing observation/other-goal IDs. Native calls continue through
unmodified `generate_from_frames`, with their original sample cache flags.
The profile's default instruction requests navigation to the reference scene
and stopping there; an explicit custom instruction replaces that default.

## Install and serve

Use an isolated Python 3.11 environment with the locked HF backend dependencies:
Torch 2.10.0, torchvision 0.25.0, and Transformers 5.8.0. There is no version-guard
bypass or automatic vLLM fallback. The weights require approximately 9.7 GB.
On shared servers, place the environment, model files, recordings and package
caches on the approved data volume, not a quota-limited home directory.

```bash
python3.11 -m venv deployment/lightnav/.venv
deployment/lightnav/.venv/bin/python -m pip install --no-cache-dir \
  -r deployment/lightnav/requirements-server.lock.txt
python3 -m deployment.lightnav.download_model \
  --output deployment/lightnav/.runtime/weights --workers 8
deployment/lightnav/.venv/bin/python -m deployment.lightnav.check
bash deployment/lightnav/run_server.sh
curl --fail http://127.0.0.1:18884/healthz
```

The downloader pins metadata, verifies original LFS SHA256 values, and resumes
verified 8 MiB download ranges. Optional `packed_transfer.py`/`serve_asset.py`
transport preserves original bytes and requires the same final model hash.
Stop temporary transfer relays after delivery.

The listener defaults to loopback **18884**. Overrides are `LIGHTNAV_PYTHON`,
`LIGHTNAV_MODEL_PATH`, `LIGHTNAV_HOST`, `LIGHTNAV_PORT`, and
`LIGHTNAV_RECORD_DIR`. Inference and reset run serially on one persistent model
worker thread. HTTP/1.1 keepalive is supported; extra simultaneous model requests
receive 429. Recordings preserve exact requests, JPEG bytes, responses and errors.
The server contains no ROS/DDS/Unitree command transport and cannot arm the robot.
`/healthz` also reports `active_session_count`, retirement counts, and up to
eight `active_sessions` identities/history counts, including while inference is
busy. `retirement_protocol.supported=true` identifies durable terminal cleanup. This read-only
diagnostic does not reset any history or infer which operator owns a session.

To forward the already-running upper-computer service to the lower computer:

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -R 127.0.0.1:18884:127.0.0.1:18884 go2
```

`bash deployment/operator/open_lightnav.sh` is the operator connection helper;
see [operator instructions](../operator/README.md). A healthy old server must not
be reused merely because a model is loaded: `/healthz` must report
`implementation: "official_core_with_deployment_adapters"`, the pinned
`official_release_commit`/`official_core_sha256`,
`imagegoal_adaptation_support: true`, and the default native contract
`goal_image_consumed: false`. An older native-only or old experimental service
is rejected and preserved rather than silently adopted or restarted.

## HTTP request

`POST /infer` accepts bounded JSON:

```json
{
  "identity": {
    "session_id": "experiment-session", "task_id": "lightnav-task",
    "goal_id": "instruction-generation-id", "goal_version": 0,
    "frame_id": "newest-camera-frame-id"
  },
  "observations": [
    {"frame_id": "newest-camera-frame-id", "capture_timestamp_ns": 1,
     "rgb_jpeg_base64": "..."}
  ],
  "task_mode": "objnav",
  "conditioning_mode": "native_language",
  "instruction": "Go to the coat rack beside the windows and stop."
}
```

`conditioning_mode` defaults to `native_language`; its other values are
`native_tracking` and `experimental_reference_image`. The corresponding default
`task_mode` is `objnav`, `tracking`, or `imagegoal`. Incompatible combinations
fail. Native modes require an instruction and reject `goal`, including null.
The canonical target-photo field is `goal`; `goal_image` is rejected to avoid
ambiguous inputs. The ImageGoal extension must be explicitly selected:

```json
{
  "identity": {
    "session_id": "experiment-session", "task_id": "imagegoal-task",
    "goal_id": "goal-photo-generation-id", "goal_version": 0,
    "frame_id": "newest-camera-frame-id"
  },
  "observations": [
    {"frame_id": "newest-camera-frame-id", "capture_timestamp_ns": 1,
     "rgb_jpeg_base64": "..."}
  ],
  "task_mode": "imagegoal",
  "conditioning_mode": "experimental_reference_image",
  "goal": {"rgb_jpeg_base64": "..."}
}
```

For this profile only, missing/null/blank `instruction` selects the documented
reference-image navigation instruction. An optional supplied instruction is
passed verbatim after input validation.

Provide 1–64 distinct chronological real source frames, normally sampled at the
checkpoint's 4 Hz. Overlapping requests are deduplicated by source ID, capture
timestamp and decoded-pixel fingerprint, including image geometry. The upstream
history buffer retains its original full-episode SlowFast behavior. Model time
encoding uses the original frame indices and checkpoint FPS; capture timestamps
remain provenance. A model failure retains appended observations for safe retry.
Reusing an ID with different pixels/stamp, inserting an unseen older frame, or
sending an older newest frame returns 409. Large capture gaps should start a
new task/history, not be filled with repeated frames.

`goal_id`/`goal_version` identify the instruction/reference-image generation.
Changing goal pixels, an instruction, task mode, or conditioning mode requires a new
strictly increasing generation and clears that task history. `POST /reset` with
`{"identity":{"session_id":"...","task_id":"..."}}` clears matching sessions;
omit `task_id` to clear the whole session. Normal reset permits explicit history
reconstruction with the same identity. Terminal cleanup additionally sends
`"retire": true`: the server clears matching history and permanently rejects
late inference for that task/session with 409 for its remaining process lifetime.
The response must acknowledge `retired: true`; an older server ignoring the field
is not a successful retirement. No other operator's identity is affected, and
ordinary reset cannot undo retirement. The small ID-only retirement sets do not
automatically expire or evict recently closed tasks. Reset while inference is running
returns 429. It does not interrupt a GPU kernel; cancellation invalidates pending
results in the shared runtime. Defaults are two active sessions and 4,096 real
frames per episode. Overflow fails explicitly, never silently truncates SlowFast.

## HTTP response and execution

The transport keeps `schema_version: "lightnav.imagegoal.inference.v1"` for
existing trajectory-client compatibility. Each response reports its actual
implementation and profile, the official commit and core digest, actual
`task_mode`, `conditioning_mode`, and `eval_task` (`vlnce`/`trackvla`). Native
responses use `implementation: "official_release"`,
`goal_image_consumed: false`, and `goal_encoding: null`. ImageGoal responses use
`implementation: "go2_imagegoal_adaptation"`, `goal_image_consumed: true`, and
`goal_encoding: "separate_static_two_frame_video_tubelet"`. Both retain
`native_imagegoal_support: false` to distinguish this added interface from the
released native API. Health describes the combined server rather than a
particular inference request.

`trajectory_body_m_rad` contains ten cumulative `[forward_m, left_m, yaw_rad]`
poses relative to the robot at the newest image capture time, with positive yaw
CCW. `trajectory_body_m` is the XY display projection. Preserve yaw and in-place
turns in execution. The waypoints have no native time base; the repository's
0.1 s MPC interval is a controller convention, not a model-trained timestamp.

The original decoder determines `model_stop` from its RVQ STOP/zero-action
semantics. `model_visible` remains tri-state (`true`, `false`, or `null` when
unknown). `raw_text`, exact executed `prompt`, `pointing`, source lineage,
source frame intervals and history counts are retained for inspection.
`arrival_confirmed: false` means this inference transport does not independently
verify a destination pose; task completion follows the model STOP semantics in
the execution runtime. In continuous tracking, STOP can mean a temporary hold
rather than task completion. Any device-specific execution or terminal-approach
behavior belongs to the shared runtime, not this inference transport.

## CPU checks and recorded replay

The CPU checks load no model weights, allocate no GPU and start no robot:

```bash
deployment/lightnav/.venv/bin/python -m unittest \
  deployment.lightnav.tests.test_tracking \
  deployment.lightnav.tests.test_official_lightnav_policy \
  deployment.lightnav.tests.test_reference_image_policy \
  deployment.lightnav.tests.test_session_cleanup \
  checks.test_lightnav_launcher -v
```

To replay real camera history through an already-running model:

```bash
deployment/lightnav/.venv/bin/python -m deployment.lightnav.client \
  --manifest data/realworld/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --instruction 'Go to the coat rack beside the windows and stop.' \
  --output deployment/lightnav/.runtime/official-replay-001
```

Tracking additionally uses `--conditioning-mode native_tracking --task-mode
tracking` and a tracking instruction. Run directories retain source/JPEG hashes,
exact model tokens, full trajectories, lineage and request latency. Both replay
and commissioning tools retire their own unique session in `finally`, including
inference timeout and response-validation failures. Reset retries busy/transport
errors for at least 120 seconds because client timeout does not cancel GPU work.
`final_cleanup.json` records success or an explicit cleanup failure; another
operator's sessions are never reset.
To replay the explicit ImageGoal adapter:

```bash
deployment/lightnav/.venv/bin/python -m deployment.lightnav.client \
  --manifest data/realworld/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --conditioning-mode experimental_reference_image --task-mode imagegoal \
  --goal /path/to/full-reference-photo.jpg \
  --output deployment/lightnav/.runtime/imagegoal-replay-001
```

The real-model contract driver exercises language navigation, tracking, STOP
instruction reporting, rejection of target photos in native modes and missing
goal photos in the ImageGoal mode, history
retries/overlap, stale frames, instruction generations, conflicts and resets:

```bash
deployment/lightnav/.venv/bin/python -m deployment.lightnav.commission \
  --manifest data/realworld/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --instruction 'Go to the coat rack beside the windows and stop.' \
  --output deployment/lightnav/.runtime/official-commission-001
```

These tools never publish robot commands. Contract checks and valid trajectories
alone do not establish real navigation success. Previous reference-image
recordings describe their own recorded adapter/execution settings and do not
validate a later configuration. This change itself has not run a new GPU
inference or initiated physical motion.
