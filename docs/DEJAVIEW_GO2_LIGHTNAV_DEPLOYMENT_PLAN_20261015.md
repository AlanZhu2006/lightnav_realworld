> Sanitized historical source snapshot. Dates, old hardware observations and results refer to the original deployment; paths below are portable examples, not a claim that this checkout is deployed. Current entry: [REALWORLD_GO2_CONSOLE.md](REALWORLD_GO2_CONSOLE.md).

# LightNav 在 Go2 上的独立 baseline 部署方案

> 这是 2026-10-15 的原生语言 baseline 历史计划。当前用户要求优先 ImageGoal，并行部署以 [2026-10-17 LightNav ImageGoal 记录](DEJAVIEW_LIGHTNAV_IMAGEGOAL_20261017.md) 和 [统一运行入口](../deployment/bringup/README.md) 为准；当前 DejaView 局部策略主线为 NoMaD。LightNav 原生模型不支持 ImageGoal，此轮图像条件路径显式标为 inference adaptation。

2026 年 10 月 15 日。当前主部署为 DejaView 加 NavDP 关键帧导航 (original workspace document; not part of this LightNav import)。LightNav 独立运行，接受语言任务并使用自己的视觉策略和控制栈。此前的 DejaView 路线到语言指令转换设计已取消，不列入主部署。

设备为 Go2 EDU、外部 RTX 4090、RealSense。使用已经核查的 [lightnav_realworld](https://github.com/AlanZhu2006/lightnav_realworld/tree/ddfd37059c49e8090217e11c4ffa6f6b3ff9d554)，与 DejaView 实验使用不同运行记录、服务与任务状态。

## 输入 输出和分工

```text
语言任务 + 当前 RGB 历史
  → 4090 LightNav / RVQ decoder
  → 10 个相对拍摄时刻的 [forward_m, left_m, yaw_rad] 轨迹点
  → 机器人侧 odom 对齐与 CasADi MPC
  → 本地控制仲裁与 SDK2
  → Go2 自带步态
```

LightNav 不接收 DejaView 的图路线、历史目标图片、物体检索结果或记忆读出。每个独立任务使用它自己的 episode/history；语言目标与真实物体对应关系在任务集冻结时确定。

## 环境与已记录验证

GPU 使用 Python 3.11、Transformers 5.8.0、vLLM 0.19.1、CUTLASS DSL 4.5.2；checkpoint revision 为 `826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`，包含完整 RVQ bundle。Jetson 使用 Humble/Python 3.10、NumPy 1.26.4、CasADi 3.7.2，不安装根项目模型依赖。

[仓库验证记录](https://github.com/AlanZhu2006/lightnav_realworld/blob/ddfd37059c49e8090217e11c4ffa6f6b3ff9d554/docs/REALWORLD_VALIDATION.md) 报告离线检查与 RTX 4090 真权重 loopback smoke；该记录未验证此次现场相机、ROS、ARM64 安装或导航成功。安装、dry-run/check 与跨机 smoke 按仓库部署文档执行，并记录实际两机版本。

## 独立真机执行需处理的缺口

源码核查 (original workspace document; not part of this LightNav import) 已确认：当前 MPC 主要检查 odom 年龄，没有独立的源图像/轨迹年龄 gate；机器人链路未接入 RealSense 深度 guard；网页手动仲裁不能替代物理遥控活动接管；重新 enable 和换任务需明确失效旧路径。

在该 baseline 的机器人控制侧补新鲜度、深度停车、遥控接管和取消语义，保留 LightNav 模型与原 MPC。可参考 GEM 控制实现，但不同 SDK API 和消息格式单独适配。LightNav 当前速度走 ObstaclesAvoidClient，姿态/停止动作走 SportClient；验证现场避障开关、控制权申请/释放和实际停车响应。

与 DejaView/NavDP 实验交替运行时，最后仅一个 SDK 写入者。启动新实验先停止上一运行的运动链路，保留其服务和数据；新的源码同步、重连或基线选择不恢复已取消的运动任务。

## 任务与比较

选与 DejaView/NavDP 相同的物理目标和起点，固定语言任务、速度限制、执行预算和独立成功判断。唯一目标与同类歧义目标分开列出；已经在目标视图的起点单独标记。

LightNav ObjNav 的 `stop=true` 触发停车；模型停止、操作员完成判断与评分结果分别保存。保持原始 RGB、请求、响应、trace ID、采集时间、轨迹、odom、guard 事件和视频。

LightNav 与 NavDP 的目标输入、感知和 tracker 不同，该比较回答系统级表现。DejaView 记忆的直接效果由主方案中的“DejaView 加 NavDP”与“直接 ImageGoal NavDP”控制对照检验。

## 实施顺序

先完成主方案的 NavDP 单目标和 DejaView 多关键帧闭环，再独立恢复 LightNav 推理与真机执行。两者的模型、记忆、episode 和结果目录不互相混用。首版无需把 LightNav 改成 ImageGoal 模型，也无需加入语言路线编译器。
