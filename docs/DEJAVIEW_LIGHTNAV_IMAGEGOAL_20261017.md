> Sanitized historical source snapshot. Dates, old hardware observations and results refer to the original deployment; paths below are portable examples, not a claim that this checkout is deployed. Current entry: [REALWORLD_GO2_CONSOLE.md](REALWORLD_GO2_CONSOLE.md).

# LightNav ImageGoal deployment — 2026-10-17

LightNav is deployed as an independent baseline. DejaView's primary deployment
continues to use real memory keyframes → NoMaD → the common LightNav MPC. Both
backends share operator/task semantics and controller checks, while model
outputs, timings and experimental claims remain separate.

## Verified source contract

The successful repository is pinned to
`AlanZhu2006/lightnav_realworld@ddfd37059c49e8090217e11c4ffa6f6b3ff9d554`.
Its original inference prompt and WebSocket protocol accept egocentric RGB
history and a language instruction. Neither the repository nor the released
official checkpoint defines a separate ImageGoal input. The
[official model card](https://huggingface.co/LightOriginsHQ/LightNav-0) likewise
describes RGB history plus instruction; the
[official repository](https://github.com/lightorigins/LightNav-0) says task
semantics come from the instruction. A captioning bridge must not be presented
as a faithful native ImageNav baseline.

Our `experimental_reference_image` adaptation sends the entire destination RGB
as a separately labelled static visual segment. It does not insert that image
into the chronological camera history. The original model, visual
preprocessing, SlowFast sampling, navigation prompt body, RVQ codes and action
decoder are copied unchanged with per-file hashes. The extra reference framing
and task instruction are explicit changes to the inference input distribution;
no navigation-performance claim follows merely from a successful request.

The original `native_language` path is retained as a separate control mode with
`goal_image_consumed=false`. Goal-switch/cancel invalidation and stop proposals
are handled by the common task runtime, never by interpreting visibility as
arrival.

## Software and protocol

- Sources and executable entrypoints: [deployment/lightnav/README.md](../deployment/lightnav/README.md).
- Official weights: revision `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`,
  9,695,744,034-byte shard SHA256
  `ffc4a925378a881afa761865048eb8d07c55cacf5eaf66548b6641c39f67af18`.
- Isolated Python 3.11, Torch 2.10.0, torchvision 0.25.0,
  Transformers 5.8.0, explicit official HF backend.
- HTTP loopback `127.0.0.1:18884`, `GET /healthz`, `POST /infer`,
  `POST /reset`. One in-flight inference, bounded JPEG payloads, no SDK/ROS
  dependency or motion endpoint.
- Requests carry task/session/goal/version/newest-frame identity, source RGB
  history, full goal JPEG and optional task instruction. Responses echo all
  lineage and contain 10×3 cumulative body-frame SE(2) poses in metres/radians,
  original model stop/visibility/pointing/raw tokens, exact input prompt and
  explicit adaptation markers.
- Overlapping 1–64 frame request windows are deduplicated by source identity,
  timestamp, RGB mode, dimensions and pixel fingerprint. Full original SlowFast episode history is
  retained server-side up to an explicit 4,096-frame capacity; no silent
  four-frame/64-frame truncation occurs. A goal-generation change clears its
  history, reset frees finished tasks, and history/capacity conflicts fail
  explicitly. Model-generation failures retain already appended true frames for
  deduplicated retries.
- Capture-time timestamps remain source provenance. The official model encodes
  4 Hz frame-index timestamps; callers sample the observation history at that
  rate and clear it after large gaps.
- Model waypoints have no native timestep. The controller's 0.1 s execution
  sampling is an explicit deployment decision. Preserve the model yaw and
  in-place rotation segments rather than replacing them by XY tangents.

## Commissioning evidence

The full official checkpoint is installed: all 18 files, totalling
9,709,007,978 bytes, and the original 48 vendored source files have verified
hashes. Model transport was lossless, with each raw/compressed block and the
final original shard checked; no quantization or substitute weights were used.
Temporary transfer relays and redundant downloads have been stopped.

The isolated Python 3.11.17 environment passes `pip check` and an actual CUDA
kernel on the RTX 5090, compute capability 12.0. It uses Torch 2.10.0 with CUDA
12.8, torchvision 0.25.0 and Transformers 5.8.0. All 57 resolved packages are
locked. The complete official model is running on loopback port 18884. Restart
with `bash deployment/lightnav/run_server.sh` or the shared bringup manager's
`deployment/bringup/workstation_lightnav.json`; the manager supports adopting
the existing healthy model of the same revision.

The final direct-model run, `deployment/lightnav/.runtime/commission_002`,
passes **14/14 contract cases**, with **7 actual GPU model inferences**:

- Native RGB-history/language control and a separately recorded stop instruction.
- Full reference-image conditioning, exact-history retry, overlap deduplication
  and history append, preserving four frames then adding the fifth.
- Rejection of an old newest frame, changed goal pixels without a new goal
  generation, an old goal generation and malformed request identity.
- New goal generation, explicit session reset and fresh history after reset.

Five warm reference-image round trips were **116.99–123.79 ms**, median
**121.53 ms**; the first native-language request was **436.05 ms**. These are
short-history workstation measurements, not a promised end-to-end control
rate. Each request/response saves the full reference RGB, actual camera
history, source identity, prompt, original model tokens and ten metric poses.
A separate identity regression confirms that equal raw RGB bytes with
different dimensions cannot bypass the source/goal consistency checks.

The native stop instruction **did not produce a stop token** on this scene;
the model never emitted a learned stop in these seven requests. The report
records both facts as false rather than substituting a synthetic stop result.
Independently, original RVQ codes `[6, 122, 174]` decode to ten exact zero poses
and `model_stop=true`; that verifies decoder semantics only. Arrival detection
and stop obedience remain unvalidated. Task pause and cancellation are handled
deterministically by the shared runtime.

The shared browser/operator task loop also completed a LightNav run against
the real GPU model, using a simulated odometry/actuator sink: **10 model
requests, 9 accepted trajectory responses, 22 nonzero MPC proposals, zero model
errors and zero MPC errors**. Pause, resume, goal generation 2 and cancellation
passed. This verifies image/goal transport → actual model → full SE(2)
trajectory → original MPC → simulated command sink and task lifecycle; it did
not issue robot motion commands.

The recorded D435i inputs used for direct model acceptance show stationary
ceiling beams. Later live RGB-D frames show a sideways office/table view; a
forward-facing installation is still not accepted. These sources establish
I/O, history/goal separation, token decode, lineage and controller integration,
not ImageGoal navigation quality. The service remains available through the
workstation-to-Jetson loopback tunnel for live operator commissioning.

Detailed local evidence is retained under `deployment/lightnav/.runtime/`:
`preflight.json`, `identity_regression.json`, `decoder_receipt.json`,
`commission_002/`, `recordings/`, checkpoint `SOURCE_MANIFEST.json` and the
model's `.transport-receipt.json`. Runtime assets, source images and weights
are excluded from Git. Available root-volume space after commissioning is
approximately 7.3 GiB; the setup does not modify the pre-existing memnav
environment.

## Remaining physical acceptance

After the Go2 connection is available: use a forward mounted camera, valid
capture-time odometry, measured obstacle-guard extrinsics and independent goal
arrival labels. Compare NoMaD and LightNav under the same full goal images,
start/goal pairs and control limits. Report the LightNav condition as the
reference-image inference adaptation, together with its native-language control
and its actual latency/failure rate; do not label it a released ImageNav model.
