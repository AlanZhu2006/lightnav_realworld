# Jetson / Go2 下位机部署

目标环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。下位机仅安装 ROS workspace 与专用 requirements；根目录模型包要求 Python 3.11 并依赖 Torch，不在这里安装。

## ROS 与 Python

先按 [Humble 官方指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装 ROS。已有工作的 ROS / RealSense / SDK2 环境可复用。录制依赖：

```bash
sudo apt install python3.10-venv python3-rosdep build-essential cmake ffmpeg \
  ros-humble-ros-base ros-humble-ament-cmake ros-humble-ament-index-python \
  ros-humble-launch-ros ros-humble-diagnostic-msgs ros-humble-geometry-msgs \
  ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-std-srvs ros-humble-tf2-ros ros-humble-rosidl-runtime-py \
  ros-humble-rosbag2-py ros-humble-rosbag2-storage-mcap
bash deployment/robot/setup.sh
```

.venv-robot 使用 system-site-packages 获得 rclpy 与 ROS 消息，单独安装 NumPy 1.26.4、OpenCV、aiohttp、CasADi 3.7.2 和 colcon。EmPy 3.3.4 和 setuptools 68.2.2 保留 Humble 构建接口；不采用 EmPy 4。构建通过该 venv 的解释器调用 colcon，让节点使用同一 Python。首次 setup 会创建 config/robot.env；默认使用 /usr/bin/python3.10，可用 LIGHTNAV_ROBOT_SETUP_PYTHON 覆盖。

## Unitree SDK2 / CycloneDDS

优先复用与固件配合工作的 SDK。新机器的安装依据 [Unitree 官方仓库](https://github.com/unitreerobotics/unitree_sdk2_python)，其 Python 依赖要求 cyclonedds==0.10.2。源码放在独立目录：

```bash
mkdir -p ~/lightnav-dependencies
cd ~/lightnav-dependencies
git clone --branch 0.10.2 --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds.git
cmake -S cyclonedds -B cyclonedds/build \
  -DCMAKE_INSTALL_PREFIX="$PWD/cyclonedds/install"
cmake --build cyclonedds/build --target install -j4
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
git -C unitree_sdk2_python checkout 814556d15970dd2ecf1c9984e845ca02ab07e206
```

回到部署仓库，设置 config/robot.env 的 LIGHTNAV_CYCLONEDDS_HOME 为该安装目录的绝对路径：

```bash
source robot-env.sh
export CYCLONEDDS_HOME="$LIGHTNAV_CYCLONEDDS_HOME"
python -m pip install 'cyclonedds==0.10.2'
python -m pip install --no-deps -e "$HOME/lightnav-dependencies/unitree_sdk2_python"
```

--no-deps 避免 SDK 的 opencv-python 与本工程 headless OpenCV 重复安装；其余依赖已显式安装。SDK revision 是整合时官方参考版本，原 Jetson 快照未记录精确 SDK commit，仍需真机验证。Go2 网络/固件依照 [Unitree 官方网络指南](https://support.unitree.com/home/en/developer/Quick_start)。

## 相机

当前实机复用 /camera/camera/color/image_raw。已有相机进程时直接沿用。源码驱动 overlay 可用 LIGHTNAV_REALSENSE_SETUP 指定 install/setup.bash。

新机器按 [RealSense 官方安装说明](https://github.com/realsenseai/realsense-ros#installation-on-ubuntu) 安装；Jetson 的 librealsense / JetPack 适配按其 Jetson 指南选择。已有可用驱动无需替换为最新分支。

仅在没有相机进程时，独立终端运行：

```bash
source robot-env.sh
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false
```

启动部署前读取现有图像流：

```bash
ros2 topic info /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_raw
```

其他输入设置 LIGHTNAV_IMAGE_TOPIC；压缩流另设 LIGHTNAV_IMAGE_TRANSPORT=compressed，并提供 CompressedImage topic。网页和推理客户端共享配置，归档也纳入所选相机 topic。

## 构建与检查

编辑 config/robot.env：

```bash
LIGHTNAV_SERVER_URL="ws://GPU_HOST:8050"
LIGHTNAV_NETWORK_INTERFACE="enP8p1s0"
LIGHTNAV_CYCLONEDDS_HOME="/absolute/path/to/cyclonedds/install"
```

网卡是连接 Go2 的有线接口，不是连接 GPU 服务器的 Wi-Fi。其他变量见 [配置表](REALWORLD_CONFIGURATION.md)。

```bash
source robot-env.sh
bash robot_deploy/scripts/build.sh
source robot-env.sh
bash deployment/robot/start.sh --dry-run
bash deployment/robot/start.sh --check
```

robot-env.sh 只加载环境。--check 仅导入依赖、查询 ROS package index、检查网卡和网页端口，不初始化 Unitree 通道或调用机器人 API；不能证明相机在发布或上位机可访问。

只运行下位机的离线测试（无 ROS 时对应测试会跳过）：

```bash
PYTHON=.venv-robot/bin/python bash deployment/test.sh --robot
```

在 Jetson 环境中也可检查真实推理，不启动机器人：

```bash
python -m deployment.smoke_protocol --server ws://GPU_HOST:8050 \
  --image /path/to/frame.jpg --instruction 'approach the plant'
```

## 人工启动

确认相机流与上位机就绪后，操作员执行：

```bash
bash deployment/robot/start.sh
```

打开 http://JETSON_HOST:8088，检查画面、机器人模式、里程计与 server URL，再输入指令。ObjNav 响应 stop=true 结束；Track 持续追踪并忽略模型 stop。默认速度上限 0.3 m/s、0.5 rad/s 同时传给 MPC 和手动控制。网页仍可热更新参数，实验应保留实际 ROS 参数记录。

Go2 adapter 初始 control/source=disabled；收到有效且所选来源的速度命令时才申请 API 控制，状态过期或输入超时触发原有停控路径。

可选 [用户 systemd 模板](../deployment/robot/lightnav-robot.service.example) 需要替换绝对路径并人工安装/启动；Restart=no，相机另行管理。

保留上游 TRON1 / Orbbec 启动文件作为其他硬件参考。它们没有纳入本次 Go2 / RealSense 验证，需另外安装各自驱动，Orbbec 需要 orbbec_camera / orbbec_description。
