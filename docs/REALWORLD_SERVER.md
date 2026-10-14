# GPU 上位机部署

本机验证采用 RTX 4090、Python 3.11、Torch 2.10.0、Transformers 5.8.0、vLLM 0.19.1 和 nvidia-cutlass-dsl 4.5.2。上游依赖 vLLM 内部 API，不能随意升级或关闭版本检查。

## 安装与配置

安装 Python 3.11、venv 支持与 NVIDIA 驱动后：

```bash
bash deployment/server/setup.sh
.venv-server/bin/hf download LightOriginsHQ/LightNav-0 \
  --revision 826dc5fbfa37afa8293d2e336d329b6ffc0bfb64 \
  --local-dir checkpoints/LightNav-0
cp -n config/server.env.example config/server.env
```

setup 创建独立环境并安装模型、视频与离线测试依赖，不启动服务、不安装系统驱动。可用 `LIGHTNAV_SETUP_PYTHON=/path/to/python3.11` 选择解释器。下载如需授权，在该环境中执行 `hf auth login`。已有 checkpoint 可在配置中指定绝对路径，不必复制。必须是完整权重和 RVQ bundle，不能只给 LoRA adapter。

默认 `vllm_local`、`task=vln`、GPU 0、监听 `0.0.0.0:8050`、显存预算 0.55、batch size 1、ViT cache 256。其他 GPU 的 CUDA/Torch 构建限制见 [官方安装说明](../UPSTREAM_README.md)，尤其 Blackwell 的 HF 后端。

```bash
bash deployment/server/start.sh --dry-run
bash deployment/server/start.sh --check
bash deployment/server/start.sh
```

`--check` 校验 Python、版本、CUDA、权重分片、10 步 RVQ 与端口；实际启动后的预热才验证推理。配置可显式选择 HF 后端；vLLM 失败时不会自动切换。

## 就绪、诊断和退出

每次启动创建唯一的 `runtime/<run-id>/`，把模型服务 PID、命令和 ready_file 写入 `runtime/server.json`。预热完成且端口绑定后才创建 ready_file，停止后移除。历史 server.json 可能保留，检查就绪时也应确认服务进程存在。

源码、checkpoint 配置/哈希和系统采样写入 `recordings-inference/runtime/<run-id>/`，由独立进程完成，不阻塞推理。checkpoint 哈希可能稍晚完成。响应中的 runtime_snapshot 指向该次 provenance。

Ctrl-C / SIGTERM 会收尾此启动器创建的服务与诊断进程组，包括 vLLM 子进程；不会按进程名清理其他服务。兼容入口 `work/serve.sh` 转发到相同启动器。

## 验证跨机推理

Jetson 配置 `LIGHTNAV_SERVER_URL=ws://GPU_HOST:8050`。图片与轨迹走该端口，ROS 和控制网页在 Jetson。

```bash
.venv-server/bin/python -m deployment.smoke_protocol \
  --server ws://127.0.0.1:8050 --image /path/to/frame.jpg \
  --instruction 'approach the plant' --output runtime/protocol-smoke.json
```

使用实际 VlnClient，执行 login/reset/next，检查 10 个有限 waypoint、seq、capture stamp 和 pointing 范围。不连接机器人。输出结构兼容只证明传输和解码，不证明导航成功。

## 可选 systemd

替换 [用户服务模板](../deployment/server/lightnav-serve.service.example) 中的绝对路径，复制到 `~/.config/systemd/user/lightnav-serve.service`，手动执行：

```bash
systemctl --user daemon-reload
systemctl --user start lightnav-serve
journalctl --user -u lightnav-serve -f
```

按需另行 enable。配置通过启动脚本 source，不能将 Bash 模板直接当成 systemd EnvironmentFile。
