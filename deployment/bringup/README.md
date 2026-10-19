> Imported setup notes include historical hardware results. Current LightNav uses native Go2 RGB/odom; this checkout has not replaced the live workspace. See [current deployment guide](../../docs/REALWORLD_GO2_CONSOLE.md). Paths are examples for a new installation.

# ImageGoal 完整观测栈的运行管理

在 `go2` 的 `/home/unitree/work/lightnav_realworld` 运行。默认 profile 包含真实
RealSense RGB-D、本机 CUDA NoMaD、ImageGoal 任务服务、ROS observer 与只读
Foxglove。默认没有机器人运动进程；Go2 不在线也能使用完整目标图任务、模型路径预览、
停止、记录和下载。

```bash
cd ~/work/lightnav_realworld
python3 -m deployment.bringup --config deployment/bringup/jetson.json preflight
python3 -m deployment.bringup --config deployment/bringup/jetson.json start
python3 -m deployment.bringup --config deployment/bringup/jetson.json status
python3 -m deployment.bringup --config deployment/bringup/jetson.json stop
```

## Go2 内置相机的只读观测栈

没有连接 RealSense 时，使用独立 profile；先停止自己原来管理的观测栈，不要同时
争用同一 runtime 或 Foxglove 端口：

```bash
python3 -m deployment.bringup --config deployment/bringup/jetson_unitree_camera.json start
python3 -m deployment.bringup --config deployment/bringup/jetson_unitree_camera.json status
python3 -m deployment.bringup --config deployment/bringup/jetson_unitree_camera.json stop
```

该 profile 持续读取真实 Go2 位姿/遥控状态和内置 RGB，复用已有健康 NoMaD，
提供任务网页和 Foxglove。相机直接向 runtime 送入 RGB；ROS observer 转入 odom，
并发布目标和状态。持续视频频道为 `/go2/front_camera/image_raw/compressed`，
使用 [内置相机布局](../operator/unitree_imagegoal.foxglove-layout.json)，或在 Image
面板直接选择该频道。profile 的 `--preview-compressed-topic` 将同一相机转为
`/imagegoal/shadow/rgb/compressed` 的低带宽预览，不重复推理接入。独立的目标 Image
频道在尚未采集时为空；0.1.3 自定义控制台会在这时用当前 RGB 同步显示目标预览，
采集或导入后切换为服务端保存的冻结目标。

内置取图 API 没有曝光时间戳，记录的是 SDK RPC 接收时间，不能用 RPC 耗时证明
曝光延迟或 capture-pose 对齐。profile 固定为 shadow，不运行运动执行器；缺少独立
障碍保护时仍由 runtime 的 `depth_missing` gate 阻止 MPC 追踪与物理授权。
原生避障接口的物理接线与现场接管验收另行完成。

`preflight` 只检查现有真实消息、模型 health contract 和端口，不启动相机、模型、
任务或 SDK。Go2 odom 缺失作为运行时独立 gate，不把相机或模型检查假标为失败。

`start` 先订阅检查真实 RGB 与 aligned depth 的尺寸、frame、时间差和源年龄。
已有健康相机和模型时复用它们，不争抢设备或重复加载权重。固定 HTTP 端口已占用但
合同不符时直接退出，保留原服务。已有任务 runtime 不会被悄悄接管或取消。

每次启动创建 `runs/imagegoal_<UTC>/`，保存 resolved profile、各进程日志和完整
runtime 证据；服务身份位于 `.runtime/stack/jetson_imagegoal/state.json`。
受管进程使用独立进程组，并用 `/proc` start token 防止 PID 重用。
`stop` 只取消 session 匹配的自有 runtime，再结束自有进程组；先前已存在的相机、
模型及其他实验进程保持运行。异常退出留下受管进程时，先运行 `status` / `stop`，
再启动新 run。

## 环境准备

以下安装均位于 `work/DejaView`，按实际缺项执行；已有环境不重复覆盖。

- 本机模型：`deployment/sensors/prepare_jetson_gpu.sh`。
- 官方相机 ROS user overlay：`deployment/sensors/prepare_realsense_overlay.py`。
- Foxglove user overlay：`deployment/nomad/prepare_foxglove_overlay.py`。
- 控制/任务 Python：当前 `.venv-nomad-cpu` 已有 NumPy、Pillow、CasADi，并能访问系统 ROS。
- SDK 独立环境和物理 preflight：见 [Go2 README](../go2/README.md)。默认观测栈不初始化 SDK。
- LightNav 工作站环境、官方权重和服务：见 [LightNav README](../lightnav/README.md)。

不把 LightNav 的根项目依赖安装进 Jetson 的 ROS 环境；不把工作站 CUDA wheel 复制到
Jetson。固定源、权重、实际 wheel 与环境版本见各部署记录。

## 连接操作界面

在工作站建立隧道。已有同端口隧道时直接使用，不重复启动：

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 18093:127.0.0.1:8093 \
  -L 18766:127.0.0.1:8766 \
  -R 127.0.0.1:18884:127.0.0.1:18884 go2
```

- 浏览器：`http://127.0.0.1:18093`。
- Foxglove：`ws://127.0.0.1:18766`，导入
  [ImageGoal layout](../operator/imagegoal.foxglove-layout.json)。
- Jetson NoMaD：`http://127.0.0.1:18883`，无需跨机往返。
- LightNav：工作站 `18884` 经反向隧道供 Jetson 使用，和 NoMaD 各自保留状态。

工作站的 LightNav 可独立管理。先按其 README 准备实际环境和完整权重，再执行：

```bash
python3 -m deployment.bringup --config deployment/bringup/workstation_lightnav.json start
python3 -m deployment.bringup --config deployment/bringup/workstation_lightnav.json status
python3 -m deployment.bringup --config deployment/bringup/workstation_lightnav.json stop
```

该 profile 只管理模型服务，验证官方模型名称与 revision；已有正确服务时只观察，
停止 profile 不结束外部服务。SSH 隧道独立于模型生命周期，不通过更换模型重建机器人任务。

上传完整同相机目标 RGB 或使用当前视图，选择后端并开始影子任务。可以替换目标、
暂停/继续、停止，查看模型路径、MPC、源年龄、阻断原因与完整事件，并下载运行包。
缺真实 odom 时只显示明确未锚定的局部路径，速度为零；不会补造 TF 或零位姿反馈。

单张当前视图作为目标属于 IO 诊断。相机尚未验收为机器人前向安装，软件链路通过并不
代表完成前向导航。物理执行入口还需要真实 Go2、朝前安装和实际时基/外参记录，默认不会自动启用。

## 验证和结果边界

已验证真实浏览器的上传、任务启动、暂停/继续、停止、轨迹绘制与状态显示；请求仍在
返回时点击停止也不会丢失。管理器已验证不取消外部 runtime、不给失效 PID 发信号、
不因端口冲突重复加载模型。

真实 NoMaD + 动态模拟反馈的故障运行包含切目标晚到响应、HTTP 超时、源 RGB/深度/
odom 断流、障碍、恢复、停止不复活与任务预算。模拟图像不会随模拟位姿变化，这些
运行验证调度和控制，不能用于导航成功率。证据入口：
`python -m deployment.imagegoal.audit_run RUN --output RUN/audit.json`。

网页实际渲染已验证；Foxglove SDK 协议、七个 ImageGoal 频道及消息接收已验证，
Foxglove 客户端的布局导入/渲染另行记录。只读桥不暴露发布速度、服务或参数设置。
