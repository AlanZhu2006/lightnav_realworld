> Imported setup notes include historical hardware results. Current LightNav uses native Go2 RGB/odom; this checkout has not replaced the live workspace. See [current deployment guide](../../docs/REALWORLD_GO2_CONSOLE.md). Paths are examples for a new installation.

# Go2 SDK、只读反馈与 GEM 执行边界

这一层复用已经在 Go2 上成功部署的 GEM `go2_cmd_bridge.py` 和电池解析。源码保持不变，固定 commit 与逐文件 SHA256 记录在 `vendor/gem/SOURCE.json`。新入口负责在 SDK 初始化前完成预检，并保持默认 disabled。

所有路径在 `/home/unitree/work/lightnav_realworld`。SDK 环境为 `.venv-go2-runtime`；official Unitree SDK2 Python commit `814556d15970dd2ecf1c9984e845ca02ab07e206`，CycloneDDS Python 0.10.2，native libraries 复用主机 Humble 0.10.5，通过私有 `.runtime/cyclonedds_humble` 前缀接入。没有改动系统包和网络。

## 已完成检查

- 本机真实 SDK IDL / ChannelSubscriber imports 成功，未初始化 DDS。
- `bridge` 默认模式校验 GEM SHA256 后明确输出 `disabled`，不构造 SportClient，不发送退出 zero。
- `telemetry --offline-audit` 检查真正的 source progression / duplicate / stale / reset 路径及派生速度，没有初始化 SDK。
- 2026-10-17 真机 link 与真实反馈预检通过：`enP8p1s0=192.168.123.18/24`，Go2 `192.168.123.161` 可达；Sport stamp / LowState tick 持续推进，遥控轴有效。只读 DDS 已使用，未构造 SportClient 或发送运动命令。证据位于 `data/realworld/imagegoal_field_20261017/go2_feedback_001/`。

## 环境与预检

已有环境不用重装。新环境准备脚本要求主机已装 Humble，并拒绝覆盖已有 venv / 不匹配的 SDK checkout。

```bash
cd ~/work/lightnav_realworld
bash deployment/go2/prepare_sdk.sh
bash deployment/go2/run_go2.sh
bash deployment/go2/run_go2.sh telemetry --import-check
bash deployment/go2/run_go2.sh telemetry --offline-audit
bash deployment/go2/run_go2.sh preflight \
  --output runs/go2_preflight_new.json
```

直接运行 `run_go2.sh` 的含义始终是 disabled。上述 import/offline 入口在 link 未通时也能使用；它们不会创建 DDS participant。

## Go2 接通后的只读反馈

```bash
bash deployment/go2/run_go2.sh telemetry --watch
```

该命令先过 link preflight，之后只初始化 `rt/sportmodestate` 和 `rt/lowstate` 订阅者，完全不 import SportClient。限时观察用 `telemetry --duration-s 30`，不加 `--watch`；`--watch` 表示持续运行。正常退出时最终快照为 `telemetry_stopped`，有效测量应在运行期间保存。

| Topic / 文件 | 作用 |
|---|---|
| `/go2/odometry` | `go2_odom` → `base_link` 的真实 SportModeState pose；只在 source stamp 严格推进后发布 |
| `/go2/battery` | 复用 GEM 电池解析，过期时 explicit offline |
| `/go2/telemetry/status` | 来源、source stamp/tick、年龄、遥控器 axes、锁存故障 |
| `.runtime/go2/telemetry.json` | 同一状态的原子快照；preflight 从 reception monotonic stamp 重新算年龄 |

Go2 与 Jetson 时钟尚未同步。header 使用本机 DDS reception 时间，保留真实 Go2 source stamp；timestamp provenance 为 `go2_dds_receive_host_system_time_not_sensor_source_time`。不得把这个近似称为已验证的 capture-pose 对齐。速度由 Sport pose 与其 source stamp 在短窗口内派生到 body frame，不猜测 SDK 原 `velocity` 数组的坐标定义。原 GEM 的未指定 covariance 约定保留，未声明位姿不确定度已标定。

重复 sport stamp / LowState tick 不产生“新鲜”反馈；source clock reset、低状态计数倒退、host clock step >200 ms 会锁存 fault，要求重启反馈与任务。网络接通之后仍须验证 source progression、时延、坐标正负号、真实安装外参、遥控器接管与 STOP 行为，这些是硬件验收。

## GEM bridge 与 ImageGoal envelope

```bash
# 只有明确进行真实硬件验收时才使用，且默认仍 enabled=false。
bash deployment/go2/run_go2.sh hardware-locked \
  --feedback-status .runtime/go2/telemetry.json
```

这个入口要求 link 与实时、非 synthetic、同机 boot、source progression、遥控器有效且居中、无锁存故障全部通过，之后才构造原 GEM bridge 的受控子类。参数来自 `gem_bridge.yaml`：20 Hz、0.25 s command timeout、max vx=.2 m/s、vy=0、wz=.4 rad/s、零速度下限、遥控器优先、默认 disabled。hardware-locked 模式实际会初始化 SDK；退出时保留 GEM 原有 zero / StopMove 逻辑。本轮只验证该入口被预检阻止，没有初始化 SDK。

唯一执行输入是本机 HTTP runtime 的 `execution_envelope`。受控子类把原 GEM 的 raw Twist callback 设为 inert；即使向 `/imagegoal/authorized_cmd_vel` 发布裸命令也不会送给 SDK。通过校验的 envelope 由内部调用原 GEM 的 `on_cmd_vel` / `publish_to_go2` 路径，保留原遥控器优先、速度限制、STOP 与释放逻辑。

`execution_envelope.py` 提供独立严格边界校验器，校验任务绑定、epoch/goal version、source timestamps、各 sensor freshness、有限速度、上限与最长 350 ms lease。当前 runtime 的 `transport_authority=shadow_only_no_robot_transport` / `not_for_motion=true` 必然被拒绝。查看拒绝原因：

```bash
python3 -m deployment.go2.execution_envelope --status runs/status.json
```

物理执行 adapter 已实现；真实反馈与安装/时钟测量通过后，操作入口为：

```bash
bash deployment/go2/run_go2.sh execute \
  --runtime-url http://127.0.0.1:8093 --task-id ACTUAL_OPERATOR_TASK_ID \
  --robot-id ACTUAL_VERIFIED_ROBOT_ID \
  --mount-receipt runs/physical_mount_receipt.json \
  --timing-receipt runs/physical_time_alignment_receipt.json
```

该入口在任何 DDS/SportClient 初始化前重新检查网络、真实反馈，以及 `deployment.imagegoal.calibration.load_calibration` 校验的同 robot_id、证据 SHA256、安装矩阵、host boot、时钟误差与来源。它绑定当前显式 operator task 的 session/task 授权生命周期；同一任务的 goal_version/epoch 可单调前进，先停下并清除旧命令，待 post-stop 新 source 与新 accepted plan 后恢复。旧 generation 一律丢弃。runtime 还必须配置启用 operator-run，并且每个 task 显式选择 `execution_mode=operator_run`。UI 与常规启动仍为 shadow。`commissioning=true`、缺少 flag、synthetic 输入、shadow authority、未通过标定均无法物理执行。

执行过程中每 50 ms 重新读取 runtime envelope，并重新检查真实反馈与 calibration source/receipt hashes；独立 20 ms watchdog 使用 wall-clock lease 与 monotonic reception age，超时/断源立即 zero + clear，旧 plan 不能靠新 lease 复活。恢复要求源图像时间晚于 zero 成功提交时间；这不冒充已经测得实体静止时间。最终每次 SDK Move 还在 RPC 锁内重新核对 lease、速度上限与遥控器优先。真实 SDK RPC timeout=.20 s；停止调度与阻塞上限是软件设计，实际实体停止时延仍需硬件验收。RPC 错误、operator STOP、任务终态或 session/task 改变会锁存，需要新的显式 run；临时传感器暂停、同任务 epoch 与关键帧目标更新可按上述新 source 条件恢复。

runtime 控制求解默认 10 Hz，adapter 的 20 Hz 轮询会正常读到重复 evaluation。相同 evaluation/sequence、任务、目标、源帧和命令的重复读取只保留既有 lease，不刷新 reception 时间、不延长 expiry，也不产生停车 fence；实时 status 的 sensor age 不参与命令身份。更旧 evaluation/sequence、同序号篡改命令均被丢弃，不回滚或打断已接受的新命令。独立 sensor 校验和原 lease 到期仍照常撤销执行。

## 已执行的完整离线验收

```bash
source /opt/ros/humble/setup.bash
.venv-go2-runtime/bin/python -m deployment.go2.audit_adapter \
  --fixture runs/nomad_commissioning_20261017/imagegoal_operator_authority_003/candidate_status.json \
  --output runs/adapter_audit_new.json
```

已消费最终冻结 runtime 的 `imagegoal_operator_authority_003` 真实 NoMaD → MPC 非零 commissioning candidate，并调用原 GEM bridge 方法到 FakeSportTransport，**25/25** 检查通过：physical 拒绝 commissioning、raw Twist 无效、TTL watchdog、旧 plan 不能靠新 lease 复活、遥控器接管、epoch fence、同 task 合法新 epoch/关键帧恢复且无需重启、late 旧 epoch 不干扰新 plan、断源、RPC failure 锁存、STOP 抢占与不可被新包清除；另覆盖 20 Hz poll / 10 Hz controller 连续执行、重复 poll 不续租或抬高 stop fence、live age 变化不改命令身份、同序号篡改与旧 evaluation 不破坏合法 lease，以及重复轮询仍无法越过真实 expiry。fixture 始终 `commissioning_only` / `not_for_motion=true`，fake validator 与 physical validator 分开。虚拟回放时钟从所有已记录 sensor stamp 之后开始；本次比 evaluated 时间晚 4.430227 ms，原 source/lease 时间戳保持原值，物理 future gate 没有放宽。最终结果在 `runs/nomad_commissioning_20261017/go2_actual_runtime_gem_adapter_008.json`。本轮没有初始化 DDS、构造真实 SportClient 或发送物理命令。

来源：[GEM-RealWorld](https://github.com/AlanZhu2006/GEM-RealWorld/tree/fd23b5d4006226e7a76b2c3f0bf03a8ec16e3436/deployment/go2)、[official Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python/tree/814556d15970dd2ecf1c9984e845ca02ab07e206)。
