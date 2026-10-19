# Portable Go2 console deployment

The current LightNav / Go2 integration is documented in
[REALWORLD_GO2_CONSOLE.md](../docs/REALWORLD_GO2_CONSOLE.md). It adds an HTTP
inference adapter and the shared Jetson task runtime alongside this repository's
existing WebSocket / ROS 2 deployment. Existing `deployment/server`,
`deployment/robot`, `src/lightnav`, and `robot_deploy` remain intact.

| Directory | Responsibility |
| --- | --- |
| `lightnav` | Pinned official core, HTTP service, language/tracking and explicit experimental ImageGoal adapter |
| `imagegoal` | Task identity, frame history, source-age checks, MPC, cancel/STOP and execution leases |
| `go2` | Native RGB, read-only odometry, SDK preparation, remote takeover and supervised execution |
| `operator` | Foxglove 0.1.11 source/tests, web console and connection helper |
| `bringup` | Owned-process manager and workstation/Jetson example profiles |
| `nomad` | Shared geometry/MPC/HTTP dependencies and optional NoMaD baseline; no bundled model |
| `sensors` | Optional RealSense and NoMaD setup history; not required by LightNav inference |
| `maxwell` | Optional shared-server storage/GPU guard and private tunnel template |

Only source, tests, lockfiles, attribution, synthetic fixtures and the current
small Foxglove release are included. Weights, Python/Node environments, private
SSH material, recordings and deployment state are excluded. The import is pinned
by [go2-source-manifest.json](go2-source-manifest.json). This source migration
does not switch, restart or test a live robot deployment.
