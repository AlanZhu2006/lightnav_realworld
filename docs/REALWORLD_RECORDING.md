# 录制与归档

| 目录 | 机器 | 内容 |
| --- | --- | --- |
| recordings-inference/episodes | GPU | JPEG、actions.json、原始响应、telemetry 和阶段时间 |
| recordings-inference/runtime | GPU | 源码、checkpoint 哈希/配置与系统采样 |
| recordings | Jetson | 事件、原始 ROS MCAP、视频、manifest、诊断/源码 |
| recordings-unitree | GPU，可选 | 从 Jetson 下载并验证的归档 |

## Jetson

网页启动导航开启录制，停止/任务结束收尾。ROS CDR 原始消息经队列写入独立进程 MCAP，保留原 header 时间戳，bag timestamp 是本机接收时间。默认 topic 集在 recording_archive.py，所选相机 topic 额外纳入 discovery；未发布的 topic 不会生成数据。

停止后生成 raw.mp4 / annotated.mp4，避免导航期间在 ROS 回调编码。overlay 使用网页同款投影与 APOS/OPOS，未做标定重投影验证。MP4 是有损查看材料，MCAP 保存原始消息。

metadata.json 包括任务、模式、时间、trace IDs、写入状态与计数；manifest.json 记录收尾文件大小和 SHA-256。应用队列丢包、DDS reported loss 与不可观察的传感器丢包需分别解释。有 manifest 不代表所有传感器帧完整。

HostTelemetry 后台采样系统与网络。源快照包含 Python、launch、网页 JS/HTML/CSS、shell、配置模板和部署脚本，排除私有 config/*.env。ROS 参数查询延迟到停止后，避免导航时密集查询。

统一启动器设置 LIGHTNAV_REPO_ROOT。单独节点不设置该变量时仍采样系统，但跳过代码快照。

## GPU

收到的 JPEG 原样保存，记录发给客户端的 JSON、raw_text、server_timing 与客户端 telemetry。诊断性记录失败不改变模型响应，完整性应根据日志和元数据判断。

双方通过 vln_client_<trace-id> 和 episode trace IDs 关联；相同指令字符串不能标识一次实验。

## 同步

在 GPU 上执行：

```bash
.venv-server/bin/python work/sync_recordings.py \
  --base http://JETSON_HOST:8088 --root recordings-unitree
```

工具轮询同步，只在 Jetson 没有导航/录制时拉取收尾数据。先写 .part，验证长度和 SHA-256，再发布文件与 sync-complete.json；可 Ctrl-C 退出。

默认 inference link 假定 recordings-unitree 和 recordings-inference 为相邻目录。修改目录后需检查 inference_links.json，未匹配不等于没有推理。

网页 /api/recordings 与 /recordings/... 提供目录和文件。运行中 episode 不作为完整下载包。文件哈希验证与录制阶段丢包统计是不同指标。
