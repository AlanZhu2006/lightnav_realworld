# Go2 LightNav console deployment

This is the entry point for the Go2 integration imported on 2026-10-20. The
production system from which it was imported still runs from the original
DejaView workspace. This checkout has not replaced that system. Creating this
branch ran no GPU inference, opened no camera and sent no robot motion.

## Profiles and implementation boundaries

| Selected profile | Model inputs | Controller behavior |
| --- | --- | --- |
| LightNav ObjNav / VLN | Real RGB history + required instruction | Official ten-pose MPC path, then hardware limits/leases |
| LightNav tracking | Real RGB history + required tracking instruction | Official tracking; model STOP holds while observations continue |
| LightNav ImageGoal | Real RGB history + full frozen target photo + optional instruction | Experimental static goal tubelet plus four explicit Go2 control adapters |
| NoMaD ImageGoal | Real RGB history + full target photo | Optional separate model baseline using the shared runtime |

ImageGoal is a deployment extension, not a reproduced official ImageNav API or
proof of successful image-goal navigation. The four configurable adapters are
goal-photo injection, reference-speed resampling, minimum-speed boosting, and
small-heading/command suppression. Short local actions soften boosting and
suppression. Native language/tracking keep the official input/controller path.
See [official boundaries](../deployment/imagegoal/OFFICIAL_LIGHTNAV.md) and
[Go2 adaptation details](../deployment/imagegoal/GO2_IMAGEGOAL_ADAPTATION.md).

The current hardware path uses Go2 EDU built-in RGB, native odometry and the
Unitree avoidance interface, with a Jetson and one explicitly assigned upper
computer GPU. RealSense is an optional historical path. RGB receive timestamps
are a host proxy, not verified sensor exposure times. Camera calibration and
mounting must be checked on each installation. Missing fresh odometry permits
local prediction preview but never enables execution.

DejaView memory retrieval and a multi-keyframe route supervisor are future work.
Neither is implemented by this import. NoMaD files remain because the runtime
directly imports their geometry, original MPC wrapper and HTTP primitives; simply
removing that directory breaks LightNav too. The model itself is separate.

## Two inference services coexist

The existing WebSocket service uses this repo's `src/lightnav` through
`deployment/server/start.sh`, and the original ROS stack stays under
`robot_deploy`. The imported HTTP service uses its hash-locked
`deployment/lightnav/vendor/upstream` core through
`deployment/lightnav/run_server.sh`, default loopback port 18884. They are
different launch paths and must not share one Python process. A source-origin
guard rejects a previously imported foreign `lightnav` module.

Official core commit: `3015508b70fb6e30bbd66701e94c9b916dcb8df8`.
Checkpoint: `LightOriginsHQ/LightNav-0` revision
`826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`.
The vendor manifest identifies the original fork mirror independently; its
unchanged module bytes are also checked against the official release hashes.
No model weights, environments or recordings are in Git.

## Upper-computer preparation

Use a dedicated Python 3.11 environment for the HTTP service. Its exact reference
package list is `deployment/lightnav/requirements-server.lock.txt`; it is
separate from the root project's WebSocket/vLLM environment. The RTX 5090
reference used Torch 2.10.0 / torchvision 0.25.0 and Transformers 5.8.0. Choose
the matching CUDA wheel build for the actual GPU; this import does not establish
compatibility for a different server. Do not install this environment on Jetson.

Set `LIGHTNAV_PYTHON`, `LIGHTNAV_MODEL_PATH`, and `LIGHTNAV_RECORD_DIR` to the
dedicated interpreter, the verified model bundle and a writable recording root.
If the verified bundle is already present, reuse it rather than downloading a
second copy. To fetch it explicitly:

```bash
python3 -m deployment.lightnav.download_model --output /path/to/LightNav-0
```

Only after the operator has allocated one GPU, launch the HTTP service with the
assigned device; never infer a GPU allocation from low utilization:

```bash
CUDA_VISIBLE_DEVICES=ASSIGNED_GPU bash deployment/lightnav/run_server.sh
```

`/healthz` must report the model revision, official commit/core digest,
`implementation=official_core_with_deployment_adapters`, ImageGoal adapter
support and `lightnav.owned_retirement.v1`. A mismatched or occupied endpoint is
preserved, not killed or silently adopted. Task cleanup retires only its own
session, preventing late requests from recreating finished histories.

## Jetson and operator configuration

Use an independent checkout under `~/work/lightnav_realworld` when preparing a
new installation. Source-only migration does not copy the working Jetson's
environments. ROS 2 Humble / Python 3.10, the runtime environment, and the private
Unitree SDK environment must be prepared independently. The historical tested
setup scripts remain under `deployment/go2` and `deployment/sensors`.

The portable CPU runtime environment can be prepared after ROS 2 Humble and
its Python messages/cv_bridge are installed (this does not install model weights):

```bash
python3 -m venv --system-site-packages .venv-nomad-cpu
.venv-nomad-cpu/bin/python -m pip install -r deployment/imagegoal/requirements-runtime.txt
```

The inherited `.venv-nomad-cpu` name is shared-runtime compatibility, not a
requirement to install the NoMaD model. `deployment/go2/prepare_sdk.sh` creates
the independent SDK environment; review the host ROS/CycloneDDS prerequisites.

`deployment/bringup/jetson_lightnav.json` is the LightNav-only shadow profile;
`jetson_native_lightnav.json` prepares the supervised runtime. Neither starts a
NoMaD model or requires its weights. The combined `jetson_unitree_camera.json`
and `jetson_native_imagegoal.json` remain optional examples for installations
that separately prepare NoMaD. On a newly configured Jetson the explicit
LightNav-only preparation command is:

```bash
bash deployment/go2/run_go2.sh native-imagegoal --prepare-only --operator-present --stack-profile lightnav
```

This prepares cameras/telemetry/runtime but creates no task or executor. These
templates are not proof that a fresh clone already has either environment or
the correct network card.
Set the robot network interface in `deployment/go2/config.json` for the actual
Jetson; the included `enP8p1s0` is an example from the reference installation.

The operator helper accepts an SSH alias and explicit robot checkout:

```bash
python3 deployment/operator/open_console.py --backend lightnav \
  --host go2 --remote-root /home/unitree/work/lightnav_realworld --prepare-native --no-open
```

`--backend lightnav` selects the LightNav-only observation/native profiles by
default; `--stack-profile combined` explicitly opts into the combined stack.
It validates the endpoint and forwards Foxglove to `ws://127.0.0.1:18766`.
Preparing a new observation service or `--prepare-native` is an operator bringup
action; these commands are not part of CPU verification. A live existing task is
preserved. Motion starts only from the explicit console button after sensors,
hardware parameters and remote takeover have been checked. The example native
profile caps forward speed at 0.3 m/s, yaw at 0.55 rad/s and lateral speed at zero.

Install [Foxglove 0.1.11](../deployment/operator/dist/dejaview.dejaview-imagegoal-0.1.11.foxe)
and import the [native layout](../deployment/operator/native_imagegoal.foxglove-layout.json).
For ImageGoal, freeze the current full RGB or import a photo, then explicitly
start shadow or supervised navigation. Text is optional in the experimental
LightNav ImageGoal mode. Language/tracking require text and reject a goal photo.
Model STOP completes navigation but reports arrival as independently unverified.
Remote takeover, cancel and lease expiry remain higher priority.

## Shared-server storage

`deployment/maxwell` is optional. All caches, environments, installers, weights,
temporary files and logs must reside under `/local_data/$(id -un)` on a host
with the 10 GB home quota. Its environment file never reassigns `HOME` and keeps
CUDA hidden until explicit allocation. The console's auto-start path refuses
cold starts on a shared `/local_data` installation. Use the shared-server
launcher with exactly one assigned GPU after coordination. Tunnel destinations,
keys and host keys are private local configuration, absent from this branch.

## Verification

The original `make check` and `make test PYTHON=/path/to/python3.11` remain the
baseline checks. Imported tests run in a separate process so the root and vendor
`lightnav` modules cannot contaminate one another:

```bash
make test-go2 PYTHON=/path/to/python3.11
cd deployment/operator/foxglove-extension
npm ci
npm run typecheck
npm run lint:ci
npm test
npm run test:browser
```

The CPU suite exercises real JPEG encoding, controller transforms/limits,
history/session retirement, stale responses and STOP/cancel ordering using
synthetic or mocked inputs. It never opens SDK devices. Tests needing private
recordings are retained as explicit manual audit tools and are not claimed as
portable CI results. UI demo images/status are synthetic, not camera recordings.

The dated `docs/DEJAVIEW_*.md` files are sanitized historical evidence, not fresh
deployment status; some describe an older RealSense setup. New CPU results are
recorded in `docs/REALWORLD_GO2_VALIDATION.json`. Physical navigation performance
and end-to-end success need a separate operator-initiated run.
