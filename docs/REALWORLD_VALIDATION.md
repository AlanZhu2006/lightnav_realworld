# 验证结果与范围

整合验证日期：2026-10-14。机器为 Linux x86_64 / RTX 4090，模型使用官方完整 checkpoint revision `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`。结果摘要见 [validation.json](../deployment/validation.json)。

## 已完成

| 检查 | 结果 | 验证范围 |
| --- | --- | --- |
| Python 3.11 全套 CPU 测试 | 708 passed，5 skipped | 上游推理、协议、decoder，加机器人客户端、MPC、录制、双机配置与真实 socket 对接 |
| 独立 Python 3.10 环境 | 89 passed，4 skipped | 根据下位机 requirements 安装，无 Torch / ROS，验证客户端、MPC、视频/归档工具与配置 |
| 下位机依赖 | uv pip check 通过 | NumPy 1.26.4 / OpenCV 4.11 / CasADi 3.7.2，EmPy 3.3.4 / setuptools 68.2.2 |
| 静态检查 | Ruff 与 Bash syntax 通过 | 新部署脚本与整仓 Python lint |
| Python 3.10 语法 | 49 个文件通过 | deployment 与 robot_deploy 的 Python 文件 |
| 双机 dry-run | 通过 | 参数、路径与命令构造，不启动节点或设备 |
| 真实 vLLM 服务 | 启动、预热、协议请求通过 | 新仓库源码、官方权重与实际 VlnClient，loopback 独立端口 |
| 服务器录制 | 通过 | 保存请求 JPEG、原始响应、客户端 telemetry 和服务器阶段时间 |
| 运行 provenance | 通过 | 源码/网页快照与 20 个 checkpoint 文件哈希；记录生成阶段的源码 |
| 服务收尾 | 通过 | 对自建 launcher 发 SIGTERM，ready 文件移除、端口关闭、GPU 内存回到启动前 |

真实权重 smoke 使用 848×480 的合成图片，经下位机客户端编码成 480×270 JPEG。返回 10 个有限 waypoint、stop、visible、APOS/OPOS 与阶段时间；客户端 seq/capture stamp 校验通过。此测试验证加载、传输、推理和解码，不测导航成功率，也不能作为真实场景延迟 benchmark。

GPU Python 环境复用了本机已经安装的模型依赖，未在新系统上从零安装 CUDA/vLLM。Python 3.10 的 Python dependencies 在独立环境中安装；这不是 Jetson ARM64 / ROS 驱动从零安装验证。

## 跳过项和未验证部分

Python 3.11 的 5 个 skip 分别是：未缓存的公开 Qwen3-VL processor 配置、两个需要 rclpy 的 ROS glue 测试模块、两个需要 rosbag2_py 的 MCAP 测试。Python 3.10 的 4 个 skip 是同样的 ROS glue / MCAP 项。没有用替身运行这些模块后宣称真 ROS 通过。

本次没有启动真机 ROS stack、相机驱动或机器人运动，没有在 Jetson 上重新安装/构建 ROS 环境，也没有验证参考 SDK revision 与具体固件的控制交互。仓库恢复的是已有实机快照；新部署仍由操作员按下位机文档完成硬件 bring-up。

GitHub Actions 已添加 Python 3.11 / 3.10 的离线验证 workflow；本表记录本机结果，不声称云端 CI 已通过。MuJoCo、Habitat、EVT benchmark 和真实导航成功率不在此次整合评测范围。

## 复测

```bash
make check
make test PYTHON=.venv-server/bin/python
PYTHON=.venv-robot/bin/python bash deployment/test.sh --robot -rs
bash deployment/server/start.sh --check
```

GPU 可用且服务已启动后：

```bash
.venv-server/bin/python -m deployment.smoke_protocol \
  --server ws://127.0.0.1:8050 --image /path/to/frame.jpg \
  --instruction 'walk through the corridor' --output runtime/protocol-smoke.json
```

真机复测顺序是现有相机 topic、上位机跨机推理、ROS package / 网卡检查、人工启动、页面里程计/模式/轨迹状态，最后由操作员发起导航。传输正确、可执行控制与任务成功需分别判断。
