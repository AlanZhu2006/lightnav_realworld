> Imported setup notes include historical hardware results. Current LightNav uses native Go2 RGB/odom; this checkout has not replaced the live workspace. See [current deployment guide](../../docs/REALWORLD_GO2_CONSOLE.md). Paths are examples for a new installation.

# Jetson CUDA 与 RealSense RGB-D

此目录提供 go2 已真实验证的用户态环境与相机入口。所有项目文件位于 `/home/unitree/work/lightnav_realworld`，不要求 sudo、Docker、内核补丁、固件升级或功耗修改。

## 已验证配置

- Orin NX 16 GB，L4T 36.5，现有 CUDA 12.6 / cuDNN 9.3 / TensorRT 10.3。
- 私有 `.venv-nomad-gpu`：Jetson AI Lab torch 2.8.0 + torchvision 0.23.0 CUDA wheels；CPU 环境不变。
- D435i，serial `EXAMPLE_SERIAL`，FW 5.14.0，当前 USB 2.1。
- 官方 Humble RealSense ROS 4.58.4 / SDK 2.58.4 用户态 overlay。
- 真正的 640×480@15 RGB8 + aligned 16UC1 depth；关闭 infra、IMU、点云。
- 100/100 对真实帧通过校准、frame 与同步审计。NoMaD 本机 CUDA 对真实 RGB 17/17 窗口通过，持久推理 worker 的 HTTP RTT 中位数 354 ms。

完整验收证据：[部署记录](../../docs/DEJAVIEW_JETSON_GPU_RGBD_20261017.md)。

## 环境准备

已有环境无需重装。新机器/新目录按以下步骤；脚本拒绝覆盖已有环境，且要求主机已有匹配 CUDA 与 Humble。

```bash
cd ~/work/lightnav_realworld
bash deployment/sensors/prepare_jetson_gpu.sh
python3 deployment/sensors/prepare_realsense_overlay.py \
  --root ~/work/lightnav_realworld/.runtime/realsense_20261017
```

GPU wheel URL 中的 SHA256 与 APT metadata 的 SHA256 均实际校验。APT overlay 仅下载与解包，不运行 maintainer scripts，不更新包数据库。不同日期的软件源若不再有固定版本，应建立新的经审核版本目录，避免静默升级。

## 运行

每台相机只启动一个 ROS publisher；模型服务无需占用 `/dev/video*`。

```bash
bash deployment/sensors/run_realsense.sh
bash deployment/sensors/run_nomad_jetson.sh
```

模型只监听 `127.0.0.1:18883`。其 `/healthz` 应显示 `device=cuda` 与 `single_persistent_worker_no_request_queue`。18882 是此前工作站隧道，不应误当作本机性能。

| 数据 | Topic | 格式 / frame |
|---|---|---|
| RGB | `/camera/camera/color/image_raw` | RGB8 / `camera_color_optical_frame` |
| 对齐深度 | `/camera/camera/aligned_depth_to_color/image_raw` | 16UC1，数值×0.001=m / 同一 optical frame |
| RGB 内参 | `/camera/camera/color/camera_info` | 实测 K、畸变、尺寸 |
| 对齐深度内参 | `/camera/camera/aligned_depth_to_color/camera_info` | 与 RGB K、尺寸、frame 一致 |
| SDK 元数据 | `/camera/camera/{color,depth}/metadata` | 实测 `clock_domain=global_time` |

上述 frame 是相机自身标定；没有自动生成相机到 `base_link` 的安装外参。

## 可复现检查

```bash
source .runtime/realsense_20261017/setup.bash
python3 deployment/sensors/inspect_rgbd.py \
  --output runs/rgbd_audit_new --duration-s 35 --pairs 100 --sample-hz 4

.venv-nomad-gpu/bin/python -m deployment.nomad.shadow_client \
  --manifest runs/rgbd_audit_new/manifest.json \
  --goal runs/rgbd_audit_new/rgb/000100.png \
  --server-url http://127.0.0.1:18883 \
  --output runs/jetson_cuda_shadow_new --mode windows
```

这个同视角目标只检查真实输入到路径的完整计算链；当前朝向天花板的安装不能用来评估导航成功。

`profile_nomad_precision.py` 可对已存真实 RGB 独立研究 FP32/FP16/BF16 与线程开销，记录 CUDA events、真实参数设备和只读 tegrastats。它不会改变正在运行的服务或自动选择精度；当前验收继续采用 FP32。
