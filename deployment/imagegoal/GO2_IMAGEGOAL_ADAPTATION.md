# LightNav ImageGoal on Go2

Select **LightNav ImageGoal**, capture or import a full goal image, then start
a shadow or onsite supervised task. Supplementary language is optional. The
official ObjNav/VLN and Tracking profiles remain available for comparison.
The reference image is consumed by an explicit deployment extension; this does
not establish that the public checkpoint has validated ImageGoal performance.

## Four adaptations

1. The complete target image is encoded as a separate static two-frame video
   segment. A prefix identifies it as the destination rather than a past robot
   observation. It never enters the chronological observation history. Original
   RGB preprocessing, checkpoint weights and RVQ decoding are retained.
2. The original ten metric SE(2) poses are resampled at the nominal configured
   reference rates, initially 0.2 m/s with angular rate bounded by the configured
   0.55 rad/s. This creates references for the 10 Hz MPC. Original decoded poses
   and the actual resampling settings are retained in the evidence.
3. Nonzero MPC requests can receive a minimum-speed boost, nominally 0.1 m/s
   and 0.2 rad/s in the supervised Go2 profile. These are engineering settings,
   rather than measured firmware dead zones. Zero requests never become motion.
4. The heading filter suppresses small corrections on nominal straight paths
   using an 8-degree angle band and 0.04 rad/s command band. Both reference
   bearing and reference yaw are checked, so a position-bearing of zero does
   not by itself erase a requested in-place turn.

Short-action softening uses the model's **predicted local action**, never an
estimated distance to the image goal. Its budget uses raw path length, angular
travel and the 0.5 s MPC horizon; remaining local trajectory extent also limits
the added minimum-speed boost. Short turning actions get a smaller heading
band. Valid MPC commands are not truncated merely because this boost budget
is smaller. This improves the numerical behavior of small actions, without
claiming that sub-threshold commands have been proven effective on the robot.

The shared MPC uses the same configuration whether adapted resampling is on
or off, allowing that switch to be examined independently. The official
comparison profile keeps its own original Go2 controller configuration.

## Configuration and evidence

The four boolean runtime configuration fields are:

- `lightnav_adapt_resample`
- `lightnav_adapt_speed_floor`
- `lightnav_adapt_heading_filter`
- `lightnav_adapt_short_actions`

They default to true for the explicit LightNav ImageGoal profile. Corresponding
CLI options accept `--lightnav-adapt-resample` /
`--no-lightnav-adapt-resample`, and the same pattern for `speed-floor`,
`heading-filter` and `short-actions`. Official task modes and NoMaD retain their
separate behavior. Changing configuration requires restarting an idle runtime.

Status and artifacts record the selected control profile, goal fingerprint,
official core digest, raw and reference paths, active switches, effective
reference rates, speed floors and command filtering. STOP, stale input, remote
takeover and user cancellation retain priority over compensation. Model STOP
ends LightNav navigation while recording `arrival_independently_validated:false`.

## Verification

```bash
.venv-nomad-server/bin/python -m unittest deployment.imagegoal.audit_lightnav_imagegoal_runtime deployment.lightnav.audit_imagegoal_adaptation deployment.imagegoal.audit_official_lightnav -v
deployment/lightnav/.venv/bin/python -m unittest discover -s deployment/lightnav/tests -v
```

These tests use explicit image/model fixtures and the real CPU MPC. They do
not establish physical navigation accuracy or calibrated minimum velocities.
