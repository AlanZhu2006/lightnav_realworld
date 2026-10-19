# Official LightNav deployment

This document describes the official navigation/tracking comparison profile.
The separate [Go2 ImageGoal adaptation](GO2_IMAGEGOAL_ADAPTATION.md) also exists;
it restores reference-image conditioning and configurable controller compensation.

The behavior reference is `lightorigins/LightNav-0` commit
`3015508b70fb6e30bbd66701e94c9b916dcb8df8`. Existing vendored inference, MPC
and geometry modules are verified against that release without editing them.

## Inputs and model

NoMaD remains the ImageGoal backend. LightNav accepts `objnav`, `vln` or
`tracking`, using the robot's chronological RGB history and a nonempty language
instruction. There is no custom reference-image tubelet, prompt prefix,
goal-pose estimator, image-matching arrival check or terminal servo.
Requests containing a separate goal image are rejected in these official modes.

The official camera encoder resizes to 480×270 using OpenCV `INTER_AREA`, then
encodes JPEG quality 80. The model's original preprocessing converts those
observations to its checkpoint grid. Official history/sample builders,
generation entry point, prompt, weights and RVQ decoder are used unchanged.
The first current observation can initiate prediction; the four-frame NoMaD
minimum is not imposed on LightNav.

## Controller

The original ten cumulative `[x,y,yaw]` points are transformed into capture-time
odometry without resampling or prepending an origin. The original weighted
nearest-pose reference builder selects five points and clamps at the endpoint.
The original optimizer solves in the current local frame at 10 Hz. Parameters
follow upstream's Go2 launch: Q=(10,10,1), R=(0.1,0.1), acceleration=(10,10),
w_max=3, navigation/track v_max=0.8/1.5. Previous control is the previous
unscaled solver output. Output scales are (2,1).

There is no minimum-speed boost, fixed reference-speed interpolation, or
small-heading suppression. The independent hardware envelope still clips the
final scaled command to the configured limits, currently vx≤0.3 m/s and
|wz|≤0.55 rad/s in the supervised profile. This explicit hardware configuration
differs from unrestricted upstream operation. Remote priority and stale-input
watchdogs also remain active. HTTP orchestration and the robot's native camera
and odometry are deployment adapters, rather than the entire upstream ROS launch.
`status.lightnav_control` and each `run.json` record these distinctions.

## STOP

For navigation, a valid fresh model STOP clears the plan, releases the native
executor, ends inference and completes the task with reason
`official_lightnav_model_stop`. Model-driven `SUCCEEDED` is separate from the
recorded `arrival_independently_validated:false`.

Tracking STOP/zero action holds motion but keeps receiving observations.
Pointing visibility is displayed without becoming an extra movement gate.
Explicit user Stop cancels the episode and fences previous results.

## Verification

```bash
.venv-nomad-server/bin/python -m unittest deployment.imagegoal.audit_official_lightnav deployment.imagegoal.audit_lightnav_transport deployment.imagegoal.audit_lightnav_dispatch deployment.imagegoal.audit_lightnav_http -v
.venv-nomad-server/bin/python -m deployment.imagegoal.audit_lightnav_native
.venv-nomad-server/bin/python -m deployment.imagegoal.audit_tracking
.venv-nomad-server/bin/python -m deployment.go2.audit_native_imagegoal
deployment/lightnav/.venv/bin/python -m unittest discover -s deployment/lightnav/tests -v
```

These checks use recorded or explicit fixture observations and model outputs.
They test software conformance and do not establish physical navigation success.
