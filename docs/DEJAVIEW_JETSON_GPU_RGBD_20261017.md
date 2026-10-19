> Sanitized historical source snapshot. Dates, old hardware observations and results refer to the original deployment; paths below are portable examples, not a claim that this checkout is deployed. Current entry: [REALWORLD_GO2_CONSOLE.md](REALWORLD_GO2_CONSOLE.md).

# go2 CUDA / RGB-D 实际部署记录（2026-10-17）

本轮已把相机与模型从此前的 RGB/V4L2 + CPU 验证推进为 **官方 RealSense ROS RGB-D + Jetson 本机 CUDA NoMaD**。所有安装均位于 `/home/unitree/work/lightnav_realworld`，未修改系统包、内核、相机固件、网络或功耗设置。未启动机器人 SDK/DDS，未发送运动。

## 机器与环境

只读复核确认主机已有 CUDA 工具链，之前只是 `nvcc` 未进入默认 PATH：

- Orin NX 16 GB，L4T 36.5.0，aarch64，Python 3.10。
- `/usr/local/cuda-12.6/bin/nvcc` 可执行；CUDA 12.6.68、cuDNN 9.3.0.75、TensorRT 10.3.0.30 已存在。
- 私有 `.venv-nomad-gpu` 安装固定 Jetson AI Lab cu126 torch 2.8.0 / torchvision 0.23.0；原 `.venv-nomad-cpu` 没有改动。
- `torch.cuda.is_available() == True`，device `Orin`；真实 CUDA matmul 与 torchvision CUDA NMS 均通过。
- 模型全部参数位于 `cuda:0`，NoMaD checkpoint 严格加载，SHA256 为 `70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333`。

GPU wheel 均按 URL 中固定 SHA256 校验；可复现实装见 [prepare_jetson_gpu.sh](../deployment/sensors/prepare_jetson_gpu.sh) 与两个 requirements 文件。

## 官方 RealSense 驱动

相机 D435i，serial `EXAMPLE_SERIAL`，FW `5.14.0`，SDK 识别总线为 USB 2.1。没有为 USB 2 降级成单 RGB：实际成功启用 RGB8 与 Z16 各 640×480@15，并打开 `align_depth.enable` 与 `enable_sync`。

采用官方 Humble APT 包，用户态解包到 `.runtime/realsense_20261017`：

| 包 | 固定版本 |
|---|---|
| ros-humble-realsense2-camera | 4.58.4-1jammy.20260916.045339 |
| ros-humble-realsense2-camera-msgs | 4.58.4-1jammy.20260907.223031 |
| ros-humble-librealsense2 | 2.58.4-1jammy.20260831.021400 |
| libglfw3 / libglfw3-dev | 3.3.6-1 |

APT 模拟确认只有这五个新增包，无升级、卸载。实际只执行 `apt-get download`、SHA256 校验与 `dpkg-deb -x`，没有系统安装。overlay 的 `lib/aarch64-linux-gnu` 和 ROS message Python 路径已纳入 setup。

早期独立 pip SDK 2.55.1 在本机枚举为空；这不是当前相机未连接的证据。换用上述官方 SDK 2.58.4 后即正常识别并工作；运行方案统一使用官方 ROS 驱动。

## 实测 RGB-D 与时钟

唯一相机节点为 `/camera/camera`。禁止另开 V4L2 capture 与它争抢设备。

| 数据 | 实际 Topic |
|---|---|
| RGB | `/camera/camera/color/image_raw` |
| 对齐到 RGB 的深度 | `/camera/camera/aligned_depth_to_color/image_raw` |
| RGB 内参 | `/camera/camera/color/camera_info` |
| 对齐深度内参 | `/camera/camera/aligned_depth_to_color/camera_info` |
| 原 SDK metadata | `/camera/camera/color/metadata`、`/camera/camera/depth/metadata` |

RGB 与对齐深度的尺寸均 640×480，frame 均为 `camera_color_optical_frame`，K 相同。实测 fx=608.576538、fy=608.600037、cx=316.750549、cy=235.639847。深度为 16UC1，官方 ROS 驱动按毫米发布，乘 0.001 得米；没有把原始深度误认为已对齐。

第一轮 `rgbd_audit_001`：20/20 对真帧保存；全部 RGB/depth header stamp 差为 0 ms，深度有效像素 75.50%–76.06%，有效深度中位数 2.532–2.564 m。RGB 接收年龄中位数 79.00 ms、最大 89.21 ms。

第二轮 `rgbd_audit_002`：393 条 RGB、393 条 depth，100/100 对样本保存；27.09 秒，alignment/calibration 全部通过；RGB 接收年龄中位数 76.91 ms、最大 85.50 ms。采样保存约 4 Hz，驱动仍以实际 15 FPS 出图。

SDK metadata 的 `clock_domain=global_time`，记录硬件 timestamp、映射的 frame timestamp、time_of_arrival 与实际约 14.989 FPS。ROS header 来源为 SDK 的系统时钟映射，已经区别于此前 host dequeue 的时间代理。最终运行仍须用同一机器 ROS clock 检查输入年龄，并等待真实 odom 后建立同一时基的 capture-pose 插值。

启动时 SDK 曾报一次 `Left MIPI error` hardware notification，随后上述两轮真实 RGB-D 连续通过；未据此伪造设备健康结论。长期运行应持续监控 metadata、配对率与 frame age。SDK 对 FW 5.14 的 power_line_frequency 默认值范围也有警告，未阻止 RGB-D 出图。

## 本机 NoMaD 与性能定位

`127.0.0.1:18883` 是 Jetson 本机服务。18882 仍是原工作站隧道，不能混用作为本机性能证据。

真实 `rgbd_audit_001` 四帧滑窗共 17 个请求，全部通过 schema、identity/source lineage、有限数值、完整 8×2 轨迹和单次尺度换算检查。官方 8 samples / 10 diffusion steps / FP32 / .05 m model-unit 配置保持不变。

| 方式 | 实际测量 |
|---|---|
| 原 HTTP 每请求新线程 | 17/17，RTT median 636.37 ms，min 632.44 ms |
| 独立同线程热态 FP32 | 三次中位数约 323 ms |
| 独立每次新建线程 | 691.5 / 595.6 / 632.6 ms |
| 独立持久 worker 热态 | 339.8 / 364.7 / 377.0 ms |
| 修复后 HTTP 持久 worker | 17/17，RTT median **353.88 ms**，min 351.47 ms；首次冷请求 1340.56 ms |

瓶颈有真实线程复用证据：同模型、相同精度下，持久推理线程避免每个请求重复付出 CUDA/cuDNN 线程局部初始化开销。服务已加载 `single_persistent_worker_no_request_queue`；忙时拒绝并发，HTTP 不积压模型请求。

独立精度研究中 FP16 / BF16 autocast 约 446 / 445 ms，反而慢于 FP32；同 seed 的最大路径差分别约 0.000223 m / 0.001884 m。本轮不自动替换数值配置，不用这一组输入宣称部署精度等价。

CUDA event 显示热态 FP32 视觉编码约 86 ms，10 次 diffusion denoise 合计约 200 ms，距离头约 0.8 ms。参数始终在 CUDA；只读 tegrastats 也记录到 GR3D 负载。只读 GPU devfreq 为 min 306 MHz / max 612 MHz，未修改功耗或锁频。

354 ms 仍不是 4 Hz 策略输出。观测 history 可保持 4 Hz，推理必须单 inflight 并消费最新四帧。首个冷请求应在可控启动阶段预热或被 age gate 拒绝。真实控制应使用 source capture stamp 计算 plan age，不能把回复时间当采集时间。

## 运行与证据路径

启动入口：[相机](../deployment/sensors/run_realsense.sh)、[本机模型](../deployment/sensors/run_nomad_jetson.sh)；有 publisher 时直接订阅，禁止重复启动。

下位机证据根目录：`~/work/lightnav_realworld/runs/nomad_commissioning_20261017/`：

- `rgbd_audit_001/{report,manifest}.json` 与真实 RGB/depth PNG。
- `rgbd_audit_002/{report,manifest}.json` 与 100 对真实 PNG。
- `jetson_cuda_rgbd_shadow_001/`：原服务 17 请求。
- `jetson_cuda_persistent_shadow_001/`：持久 worker 17 请求。
- `nomad_precision_profile_001/`、`nomad_precision_profile_002/`：精度、线程对比、参数设备与只读 tegrastats。

当前相机仍朝向天花板；这些输入只验证感知与模型链路。相机内部 RGB-depth 标定已验证，相机到机器人 `base_link` 的安装外参仍需实装测量。没有把相机 frame 假绑定到 base_link，也没有以合成 MPC pose 冒充真实反馈。Go2 未连通前，系统能完整验证感知、目标图、推理、轨迹、可视化与拒绝执行；真机反馈闭环和导航成功仍待实体联通验收。

官方来源：[NVIDIA Jetson PyTorch 安装](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)、[NVIDIA 讨论中的 JetPack 6.2 wheels](https://forums.developer.nvidia.com/t/pytorch-and-torchvision-for-jetpack-6-2/325257)、[官方 RealSense Jetson 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)、[官方 ROS wrapper](https://github.com/realsenseai/realsense-ros)。
