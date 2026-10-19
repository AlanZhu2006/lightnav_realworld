> Imported setup notes include historical hardware results. Current LightNav uses native Go2 RGB/odom; this checkout has not replaced the live workspace. See [current deployment guide](../../docs/REALWORLD_GO2_CONSOLE.md). Paths are examples for a new installation.

# Go2 NoMaD 部署入口

下位机 workspace 固定为 `/home/unitree/work/lightnav_realworld`（SSH alias：`go2`）。目标是在 Orin NX 本机完成 **RealSense RGB → NoMaD → LightNav MPC → GEM bridge → Go2**。完整 ImageGoal 任务、真实 RGB-D、本机 CUDA、操作界面与进程管理见 [统一部署入口](../bringup/README.md)。本目录保持模型与路径纯接口；机器人执行单独位于 `deployment/go2`。

## 已完成与待完成

2026-10-17 的实际记录：

| 部分 | 已核验状态 |
| --- | --- |
| RealSense | D435i USB2.1，正式使用唯一官方 ROS RGB+aligned depth 640×480@15；100 对真实配对、内参和时间映射通过。下列 V4L2 是早期诊断入口，不与 ROS reader 同时运行 |
| 相机安装 | 本次图像朝向天花板，能检查输入链路；尚不适合验证前向导航 |
| 下位机 ROS | ROS 2 Humble、`cv_bridge`、MCAP 已安装 |
| 模型 | 工作站 RTX 5090 已加载固定官方权重，严格检查 state dict；与官方单目标推理分支完成数值对照 |
| RGB → NoMaD → MPC | 真实 12 帧构成的 9 个四帧窗口全部通过；MPC 使用原 LightNav 求解器，但位姿为明确标记的 synthetic zero pose |
| 实时影子链路 | Jetson 真实相机经 SSH 到工作站 GPU，12/12 次通过；RTT 中位数约 750 ms、最新源图返回年龄约 996 ms，均包含临时链路开销 |
| 性能记录 | 工作站本地录制 RGB 的 HTTP RTT 中位数约 55 ms，首次约 284 ms；两组均不代表 Jetson CUDA 或机器人闭环性能 |
| Jetson 本机模型 | `.venv-nomad-gpu` CUDA / vision NMS / 17 个真实 RGB-D 窗口通过；持久 worker HTTP median354ms；另保留 CPU 单次649ms参考，Docker 不再是本机路径前置 |
| 任务/机器人/GUI | 完整 ImageGoal API 与网页真实渲染/任务控制通过；ROS observer、7频道 Foxglove、本机 SDK2 私有依赖和 Go2 preflight 就绪。Go2 网络/反馈和实装标定仍待现场验收；VNC 未启用 |

真实 RGB 诊断验证了接口、单位、来源元数据和求解器调用，没有使用真实 odom、深度停车或机器人反馈，不代表导航成功。当前工作站服务是参考与临时推理方式，本机闭环目标不变。

已有工作站记录位于：

- `data/realworld/nomad_commissioning_20261017/rgb_capture_001/manifest.json`
- `data/realworld/nomad_commissioning_20261017/local_shadow_001/summary.json`
- `data/realworld/nomad_commissioning_20261017/local_shadow_001/results.jsonl`
- `data/realworld/nomad_commissioning_20261017/live_shadow_001/summary.json`
- `data/realworld/nomad_commissioning_20261017/jetson_cpu_001/summary.json`
- `data/realworld/nomad_commissioning_20261017/foxglove_001/handshake.json`

下位机对应目录为 `runs/nomad_commissioning_20261017/`。完整状态和下一步依赖见 [本轮验收记录](../../docs/DEJAVIEW_NOMAD_BRINGUP_20261017.md)。

## 目录和复用边界

| 文件 | 作用 |
| --- | --- |
| `policy.py`、`server.py` | 官方 NoMaD 架构和数值处理；纯 RGB ImageGoal 推理，独立于 ROS、DDS 和 SDK2 |
| `capture_rgb.py` | 有界的静止 RGB 采集和原始时间来源记录 |
| `trajectory_adapter.py` | 完整二维累计路径统一缩放、保留端点、按空间弧长采样、用切线派生 yaw |
| `mpc_reference.py` | 核验 LightNav 源码哈希后调用原参考轨迹构造和原 CasADi/IPOPT MPC |
| `shadow_client.py` | 从已有 RGB manifest 调用模型，记录接口检查；可在 synthetic pose 下运行真实 MPC 求解器 |
| `live_shadow.py` | 独立相机线程与单个在途 HTTP 请求，记录实时 RGB、模型输出和源图年龄，不连接运动接口 |
| `ops/` | Foxglove、记录回放、图像 relay 和可选 VNC；参见 [操作工具说明](ops/README.md) |
| `prepare_bundle.py` | 将本地锁定源码和权重打包，逐文件记录来源及 SHA256，不联网下载 |
| `Dockerfile` | NVIDIA iGPU 候选镜像定义；需要在目标 BSP 上实际构建和验收 |

NoMaD 固定使用官方 96×96 PIL resize、无裁剪、RGB ImageNet normalization、四帧历史、10 diffusion 步、8 个样本取第 0 个。原 PD 只作为诊断输出。真实跟踪计划复用 LightNav MPC，执行与遥控接管复用 GEM；此处不重写优化器或腿部控制器。

## 1. 采集静止 RGB

在下位机运行。先查询格式，确认当次 `/dev/videoN`；设备编号可能在重插后改变，不用深度/红外节点替代 RGB。停止其他相机采集者后再打开本程序。

```bash
cd ~/work/lightnav_realworld
/usr/bin/python3 -m deployment.nomad.capture_rgb --list-devices
```

`/dev/video4` 是本次核验的 RGB 节点。再次采集使用新的输出目录：

```bash
/usr/bin/python3 -m deployment.nomad.capture_rgb \
  --device /dev/video4 \
  --output runs/rgb_capture_002 \
  --frames 12 --width 640 --height 480 \
  --camera-fps 15 --sample-hz 4
```

输出为 `manifest.json` 和 `rgb/*.png`。相机帧持续读取，按请求频率取样；真实间隔保存在 manifest，不能只看请求参数。这里的 `capture_timestamp_ns` 是 **V4L2 dequeue 后的 host wall clock**，不是已验证的曝光时间。它适合静止 IO 检查；接真实 MPC 前还需要可与 odom 对齐的传感器时间戳。

## 2. 启动模型服务

正式 Jetson CUDA 服务位于 `127.0.0.1:18883`，使用 `bash deployment/sensors/run_nomad_jetson.sh`，由统一 bringup 管理/复用。以下保留工作站 `.venv-nomad-server` 的数值参考服务命令。在本仓库工作站根目录运行下列命令；端口 `18882` 如已有服务，无需重复启动。

```bash
.venv-nomad-server/bin/python -m deployment.nomad.server \
  --upstream data/baselines/upstream/visualnav_transformer_native_v1 \
  --diffusion-policy /path/to/local-reference \
  --checkpoint /path/to/local-reference \
  --checkpoint-sha256 70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333 \
  --device cuda --host 127.0.0.1 --port 18882
```

```bash
curl --fail http://127.0.0.1:18882/healthz
```

`model_loaded: true` 表示模型权重已加载，`contract` 包含来源哈希和处理参数；这不表示相机、Go2 或导航已就绪。`--device cpu` 可供无 CUDA 的模型诊断。服务不下载模型，也不访问机器人。

`Dockerfile` 保留为 Jetson 本机推理候选。它使用 `nvcr.io/nvidia/pytorch:25.06-py3-igpu`，将官方源码与权重作为只读 mount；目标 BSP 上的镜像构建、CUDA、完整 diffusion 推理和延迟仍需验证。当前已用原生私有 GPU wheel 路径跑通本机服务；Docker 候选未构建，不与该已验证路径混为一谈。

## 3. 重放真实 RGB，检查 NoMaD 与 MPC

工作站已存在上述诊断记录。重新运行时指定新输出目录；`--goal` 使用最后一帧仅检查同视图 IO，不是目标导航评测。

```bash
.venv-nomad-server/bin/python -m deployment.nomad.shadow_client \
  --manifest data/realworld/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --goal data/realworld/nomad_commissioning_20261017/rgb_capture_001/rgb/000012.png \
  --server-url http://127.0.0.1:18882 \
  --output data/realworld/nomad_commissioning_20261017/local_shadow_002 \
  --mode windows \
  --mpc-source deployment/nomad/vendor/lightnav/mpc.py \
  --simulate-mpc
```

默认 `--mode latest4` 只发送最后四帧；`windows` 发送所有连续四帧窗口。去掉 `--simulate-mpc` 可只构建参考，不需要 CasADi；连 `--mpc-source` 一起去掉则只检查模型和空间路径。

结果包括 `run.json`、`input_manifest.json`、`results.jsonl` 和 `summary.json`。检查 `failed == 0` 以及记录中的 identity、源帧、时间顺序、有限输出和完整路径缩放检查。`synthetic_pose_no_robot_feedback` 必须保留，不能把这个结果重标成真实里程计跟踪。

若临时在 Jetson 运行无模型客户端，可从工作站建立反向隧道，将 Jetson 的 localhost:18882 转到工作站现有服务。已有同端口隧道时不重复启动：

```bash
ssh -N -R 18882:127.0.0.1:18882 go2
```

客户端随后使用 `--server-url http://127.0.0.1:18882`。隧道只运输推理，不改变时间戳意义；本机闭环验收仍以 Jetson 上的模型为准。

## 4. 运行有界实时影子

在下位机运行，先结束占用同一 RGB 设备的采集程序。下面命令请求 15 Hz 相机流，以 4 Hz 选择真实帧，采集与推理阶段的时长上限为 8 秒；健康检查和关闭另行计时。输出目录必须是新目录。

```bash
cd ~/work/lightnav_realworld
/usr/bin/python3 -m deployment.nomad.live_shadow \
  --device /dev/video4 \
  --server-url http://127.0.0.1:18882 \
  --output runs/live_shadow_new \
  --duration-s 8 \
  --camera-fps 15 --sample-hz 4
```

也可以将 `--duration-s 8` 换为 `--requests 12`，最多尝试 12 次请求；两项互斥。请求次数模式同样设置有限总时长，HTTP 失败后退出，不无限重试或重连。退出时释放相机，并在 `summary.json` 的 `camera_shutdown` 中记录线程是否退出、release 是否调用及采集错误。

采集线程持续读取，相邻样本必须来自不同的真实读取；主线程始终只发送一个在途请求。上次响应结束后直接选择最新四帧，不排队处理过时窗口。15 Hz 原始帧中按 4 Hz 计划选样，实际时间间隔保存在 `recorded_frame_intervals_ms`，不能将请求频率当作实测频率。

没有 `--goal` 时，首张真实采样 RGB 保存为 `goal.png`，并标记 `same_view_io_diagnostic`。这用于检查相机到模型的实时 IO。指定目标时加 `--goal /absolute/path/goal.jpg`；程序复制该图并记录哈希，实际目标文件路径写入 `summary.json` 的 `goal`。目标图与当前图使用同一个 RGB/PIL/JPEG 路径。

`capture_timestamp_ns` 仍是 host dequeue 后的 wall clock 代理；`source_age_at_response_ms` 用同一下位机的 monotonic dequeue 代理与收回响应的时间相减，包含提交前等待、编码、通信和推理，无法计入曝光到 dequeue 之间的未知延迟。`timestamp_source` 与 `source_age_timestamp_source` 保留这一限制。实时影子只做模型、尺度、来源与空间路径检查，不提供 odom，不调用 MPC 或速度接口，所有输出均为 `not_for_motion`。

输出包含 `manifest.json`、`input_manifest.json`、`run.json`、`results.jsonl`、`summary.json`、目标图和完整 `rgb/`。每个提交窗口的四张原 RGB 及其元数据都保留。**先等待实时程序结束，再回放**，因为最终 manifest 和相机退出状态在收尾时写入。移动记录时复制整个运行目录，保留完整 `rgb/` 及相对路径，不能只复制预览或最新帧。

同视图模式结束后，可先验证其记录：

```bash
/usr/bin/python3 deployment/nomad/ops/replay_shadow.py \
  --manifest runs/live_shadow_new/manifest.json \
  --goal runs/live_shadow_new/goal.png \
  --results runs/live_shadow_new/results.jsonl \
  --validate-only
```

以上是实时入口的运行方式；是否通过以本次 `summary.json`、逐请求检查和原始 RGB 为准，不沿用前面工作站离线记录的性能数值。

## 5. 查看已记录的影子结果

先在有 NumPy/Pillow 的 Python 环境下验证记录。将下面路径换成同一台机器上实际存在的记录路径；若移动整个记录，保留 RGB 和 manifest 的相对关系。

```bash
python3 deployment/nomad/ops/replay_shadow.py \
  --manifest data/realworld/nomad_commissioning_20261017/rgb_capture_001/manifest.json \
  --goal data/realworld/nomad_commissioning_20261017/rgb_capture_001/rgb/000012.png \
  --results data/realworld/nomad_commissioning_20261017/local_shadow_001/results.jsonl \
  --validate-only
```

在已配置 ROS 的主机上，去掉 `--validate-only` 并加 `--hz 2 --loop` 可发布记录。回放发布 `/nomad/shadow/` 图像、路径和状态，使用专用显示 frame `nomad_shadow`；它不发布 cmd_vel、odom 或 TF。Foxglove 安装与启动入口、布局和 VNC 见 [ops/README.md](ops/README.md)，系统依赖尚未补齐时 `run_foxglove_bridge.sh --check` 应明确报缺包。

## HTTP / Python 接口契约

`POST /infer` 使用 `Content-Type: application/json`，五张 JPEG 都走相同的 RGB/PIL 编码路径：

```json
{
  "identity": {
    "session_id": "session-1",
    "task_id": "task-1",
    "goal_id": "goal-image-hash",
    "goal_version": 0,
    "frame_id": "frame-4"
  },
  "observations": [
    {"frame_id": "frame-1", "capture_timestamp_ns": 1000000000, "rgb_jpeg_base64": "..."},
    {"frame_id": "frame-2", "capture_timestamp_ns": 1250000000, "rgb_jpeg_base64": "..."},
    {"frame_id": "frame-3", "capture_timestamp_ns": 1500000000, "rgb_jpeg_base64": "..."},
    {"frame_id": "frame-4", "capture_timestamp_ns": 1750000000, "rgb_jpeg_base64": "..."}
  ],
  "goal": {"rgb_jpeg_base64": "..."}
}
```

四帧必须从旧到新、时间严格递增、ID 唯一，最后帧与 identity 相同。服务不维护 FIFO 或路线游标；采样方保留完整历史。响应原样携带 identity 和源帧，调用方据此拒绝切目标后返回的旧结果，并用正确的时钟域检查年龄。

| 响应字段 | 含义 |
| --- | --- |
| `first_sample_normalized` | `get_action` 后的 8×2 **累计位置**，尚未乘模型到米制的尺度；不是八个位移增量，也不是原 diffusion 噪声 |
| `action_scale_m` | 默认 `0.2 / 4 = 0.05 m` 每模型单位，与 Go2 实际限速分开配置 |
| `trajectory_body_m` | 上述完整路径统一缩放一次；按采图时刻 body frame，x 前、y 左 |
| `distance_steps` | 学习的时间/步数距离，不是逐像素深度或米制距离 |
| `diagnostic_pd` | 官方第 2 号 waypoint 的 PD 数值诊断，不是运动出口 |

空间适配器接收 `first_sample_normalized` 并使用 `action_scale_m` 缩放一次；不要把已经是米的 `trajectory_body_m` 再乘 0.05。八个预测点没有经验证的实际持续时间，MPC 的参考采样间距由 `v_ref_mps * mpc_dt_s` 另行定义；yaw 来自路径切线。

纯 Python 等价入口为 `NoMaDPolicy.predict(observations, goal_rgb, identity)`，使用 `RGBFrame` 和 `InferenceIdentity`。两者都没有 `goal_reached` 字段。单图模式不能照搬官方 `closest_node == goal_node` 的到达规则。

## 下一阶段验收

按顺序补齐：前向固定相机与合适 USB 连接；Jetson 本机模型实际推理；带可信时间戳的 RGB/深度；SDK2 只读 odom 与遥控；RGB 时刻 odom 对齐的实时影子；停车与接管；单张目标图低速导航。随后保持模型、MPC 和 bridge 不变，只把目标图来源换成 DejaView 的有序关键帧。

完整方案见 [NoMaD 真机部署计划](../../docs/DEJAVIEW_GO2_NOMAD_DEPLOYMENT_PLAN_20261017.md)。部署包的 `third_party/gem_runtime/` 是待接入执行组件，不是已运行节点。
