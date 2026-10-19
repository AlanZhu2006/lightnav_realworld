# LightNav tracking in the shared Foxglove runtime

Foxglove extension 0.1.8 offers **LightNav 跟随** beside NoMaD ImageGoal and
official LightNav navigation. Describe the target and click **启动 LightNav 跟随**.
The default description is `Follow the person in the white T-shirt.`. No goal
capture or image import is required. The normal shadow button can inspect the
same task without starting an executor. Selection and reconnection do not start
tasks. Native Stop remains independent of a pending Start.

The task uses the existing LightNav model service, source-stamped RGB history,
capture-pose alignment, MPC, and supervised-native Go2 executor. The backend
explicitly uses the upstream `build_tracking_sample` and checkpoint
`tasks.trackvla` parameters. It shares the already-loaded HF weights with
language navigation but keeps task processing and session history separate. This does
not change the existing inference backend to vLLM or assert the original
repository's throughput for this deployment.

## Request and lifecycle

Use the existing operator `native_start` action, or `POST /native/start`:

```json
{
  "expected_session_id": "CURRENT_SESSION",
  "expected_epoch": 0,
  "backend": "lightnav",
  "task_mode": "tracking",
  "instruction": "Follow the person in the white T-shirt.",
  "operator_present": true,
  "timeout_s": 60
}
```

Use the actual status epoch. `goal` and `selection_id` are rejected in tracking
requests. The target is bound to an instruction SHA and task ID; no fake target
image is created or sent. Existing frozen ImageGoal selections are preserved.
Changing the target requires a new task, which isolates old results and model
history. A running native executor must be stopped before another native Start.
LightNav defaults to language ObjNav; NoMaD defaults to ImageGoal.

The runtime sends `conditioning_mode: "native_tracking"` and no goal JPEG to
LightNav. It validates the returned mode, lineage, visibility type, metric
trajectory, source ages, and capture odometry. Wrong-mode responses are rejected.
Missing tracking checkpoint configuration is an explicit error; no navigation
prompt or cached model output is substituted.

## Continuous observation and temporary holds

For a valid fresh model response:

| Model output | Runtime behavior |
| --- | --- |
| Visible target and nonzero valid trajectory | Follow through the existing MPC |
| STOP | Revoke movement, continue observations |
| `model_visible: false` | Record visibility; execute the model trajectory as upstream does |
| Missing/null visibility | Record unknown visibility; execute the model trajectory |
| Zero trajectory | Revoke movement without projecting zeros toward an old pose |

Holds use `TRACK_HOLD`, preserve the task/model episode, and never claim arrival.
They do not set navigation's terminal STOP latch. A subsequent fresh
motion prediction can resume, subject to the existing post-stop image barrier
and executor validation. There is no automatic search or identity resolver.
Manual Stop, pause, sensor faults, expired plans, and goal changes retain the
existing cancellation and freshness gates.

`status.tracking` reports the instruction, tri-state visibility, freshness,
target state, hold reason, and source frame/timestamp. `inference_held` counts
valid tracking hold responses separately from accepted motion plans and rejected
responses. Native execution additionally binds `task_mode`. Pointing visibility
is diagnostic and does not introduce an extra controller gate.

## Verification

```bash
.venv-nomad-server/bin/python -m deployment.imagegoal.audit_tracking
.venv-nomad-server/bin/python -m deployment.imagegoal.audit_lightnav_native
.venv-nomad-server/bin/python -m deployment.go2.audit_native_imagegoal
deployment/lightnav/.venv/bin/python -m unittest deployment.lightnav.tests.test_tracking -v
```

These checks exercise task dispatch, the actual MPC, native lease validation,
hold/resume, stale results, mode isolation, and the unchanged ImageGoal modes.
Model policy tests use explicit fake engines. They do not start robot hardware
or establish physical tracking success. Frontend model/browser tests live in
`deployment/operator/foxglove-extension/tests`.
