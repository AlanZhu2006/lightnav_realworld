# 双机数据流与控制语义

```text
RealSense --ROS2 Image--> vln_client
  --JPEG base64 + instruction + seq + telemetry / WebSocket-->
GPU lightnav-serve --RVQ decoder / JSON response--> vln_client
  --ROS2 vln/response--> vln_mpc --TwistStamped--> go2_adapter
  --Unitree SDK2 / wired DDS--> Go2
Go2 --sport/low state--> go2_adapter --ROS2 odom--> vln_mpc
```

## 传输与坐标

相机、MPC、Go2 adapter 和录制在 Jetson。两机间传送图像推理请求与轨迹响应，不传整个 ROS graph 或关节电机指令。

客户端把 RGB / 压缩图像变成 480×270、quality 80 的 JPEG，base64 放进 next 请求。login 携带会话 trace ID，reset 清空本连接历史；连接之间不共享视觉历史或 ViT cache。

同时只保留一个推理请求；等待响应时的相机帧不排队。相机 FPS 不等于推理 FPS。capture stamp、收图、编码/发送/等待阶段与服务器时间用于追踪链路。

响应的 10 个 waypoint 为 [forward_m, left_m, yaw_rad]，相对于拍摄时机器人的 body frame。它们是轨迹点，不是需要逐点累加的 motor command。RVQ decoder 从 token 解码轨迹。pointing 在传输的 480×270 图像中；协议见 [PROTOCOL.md](PROTOCOL.md)。

## 时间与 MPC

vln_client 保留原 capture stamp。MPC 从 odom 历史匹配拍摄时姿态，把轨迹投影到 odom frame，再从当前姿态计算参考与控制。因此不能把延迟轨迹直接当作当前位置的 body-frame 参考。

图像与 odom 时间需要可比较的本机时基。跨主机 wall_ns 只做诊断，未验证时钟同步时不能用它宣称单程网络延迟；客户端 monotonic 阶段适合往返统计。

控制运行 10 Hz，CasADi/IPOPT 优化 horizon 5、dt 0.1 s；模型 chunk 和控制 horizon 是两个概念。默认 MPC / 手动限制 0.3 m/s、0.5 rad/s。

## Go2 和模式

adapter 仲裁 web/cmd_vel 与 mpc/cmd_vel。速度经 ObstaclesAvoidClient.Move(v, 0, w)，姿态经 SportClient。rt/lf/sportmodestate 和 rt/lowstate 形成 odom 与诊断。

control/source 初始 disabled，MPC enabled=false。输入 watchdog 默认 0.35 s，Go2 状态新鲜度默认 0.5 s；保持实机快照的模式/身高判断修正及停控逻辑。

上位机 LIGHTNAV_TASK=vln|tracking 决定模型 prompt family；网页 ObjNav|Track 决定本机任务完成条件，切换网页模式不自动重启模型。ObjNav 遇 stop=true 结束，Track 忽略 stop。
