# 双机配置

deployment/common.sh 从脚本位置推导仓库根目录，默认 source config/server.env 或 config/robot.env，缺失时使用对应 .env.example。LIGHTNAV_CONFIG_FILE 可指定文件；显式文件不存在会报错。模板的 ${VAR:-默认值} 保留命令前显式设置的环境变量，路径允许空格。

## 上位机

| 变量 | 默认 | 用途 |
| --- | --- | --- |
| LIGHTNAV_MODEL_PATH | <repo>/checkpoints/LightNav-0 | 完整 checkpoint |
| LIGHTNAV_SERVER_PYTHON | <repo>/.venv-server/bin/python | Python 3.11 |
| LIGHTNAV_BACKEND | vllm_local | vllm_local / hf |
| LIGHTNAV_TASK | vln | 模型 prompt：vln / tracking |
| LIGHTNAV_HOST / LIGHTNAV_PORT | 0.0.0.0 / 8050 | WebSocket |
| LIGHTNAV_GPU | 0 | CUDA_VISIBLE_DEVICES |
| LIGHTNAV_GPU_MEMORY_UTILIZATION | 0.55 | vLLM 显存比例 (0,1] |
| LIGHTNAV_MAX_BATCH_SIZE | 1 | batch 宽度 |
| LIGHTNAV_VIT_CACHE_ENTRIES | 256 | 缓存容量，0 禁用 |
| LIGHTNAV_SERVER_RECORDINGS | <repo>/recordings-inference | episodes / runtime 父目录 |
| LIGHTNAV_HOST_TELEMETRY | 1 | 0 禁用诊断/源快照进程 |
| OMP_NUM_THREADS | 4 | CPU 线程 |

历史帧、pooling 和 tokenizer 保持 checkpoint 默认；直接使用服务高级参数见 [CONFIGURATION.md](CONFIGURATION.md)。

## 下位机

| 变量 | 默认 | 用途 |
| --- | --- | --- |
| LIGHTNAV_SERVER_URL | 空，必须设置 | ws://GPU_HOST:8050 |
| LIGHTNAV_ROBOT_VENV | <repo>/.venv-robot | Python 3.10 环境 |
| LIGHTNAV_ROS_SETUP | /opt/ros/humble/setup.bash | 基础 ROS |
| LIGHTNAV_CYCLONEDDS_HOME | 空 | SDK 的 CycloneDDS 前缀 |
| LIGHTNAV_REALSENSE_SETUP | 空 | 相机 install/setup.bash overlay |
| LIGHTNAV_NETWORK_INTERFACE | enP8p1s0 | Go2 有线接口 |
| LIGHTNAV_IMAGE_TOPIC | /camera/camera/color/image_raw | 绝对相机 topic |
| LIGHTNAV_IMAGE_TRANSPORT | raw | raw / compressed |
| LIGHTNAV_LINEAR_LIMIT | 0.3 | MPC / 手动 m/s |
| LIGHTNAV_ANGULAR_LIMIT | 0.5 | MPC / 手动 rad/s |
| LIGHTNAV_WEB_HOST / LIGHTNAV_WEB_PORT | 0.0.0.0 / 8088 | 控制网页 |
| LIGHTNAV_RECORDINGS_DIR | <repo>/recordings | Jetson episode |
| ROS_DOMAIN_ID | 0 | ROS DDS domain，与 SDK domain 独立 |
| LIGHTNAV_FASTDDS_PROFILE | <repo>/config/fastdds-camera.xml | 相机 SHM / UDP profile |

ROS 的 Fast DDS 配置和 SDK 的 CycloneDDS 通道不同。robot-env.sh 设置 FASTRTPS_DEFAULT_PROFILES_FILE，只有 ROS 使用 Fast DDS 时生效。

速度必须有限且为正。加速度、权重和 watchdog 保持节点默认；网页热更新后的实际 ROS 参数以录制为准。

## ROS launch

go2_realsense.launch.py 声明 server_url、network_interface、image_topic、image_transport、linear_limit、angular_limit、web_host、web_port、recording_directory。统一启动器从环境变量构造；直接 ros2 launch 需要自行传参。

vln_web 新增 recording_directory。单独节点默认 ~/.local/share/lightnav_realworld/recordings；统一启动器默认写仓库目录，不依赖旧名字 LightNav-0。
