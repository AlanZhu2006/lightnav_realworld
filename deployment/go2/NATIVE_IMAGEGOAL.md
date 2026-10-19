> Historical integration notes; for the current native RGB/odom profiles and source-migration status see [the deployment guide](../../docs/REALWORLD_GO2_CONSOLE.md).

# NoMaD / LightNav ImageGoal 真机控制

使用 Go2 内置前相机、NoMaD 或上位机 LightNav、真实 odom、同一 MPC 和宇树
`ObstaclesAvoidClient.Move`。两种模型共用遥控接管和停止通道，不自动确认到达。

Foxglove 安装 [0.1.6 扩展](../operator/dist/dejaview.dejaview-imagegoal-0.1.11.foxe)，
并导入 [真机控制台布局](../operator/native_imagegoal.foxglove-layout.json)。
已有单面板 ImageGoal 控制台可直接使用新扩展；旧四面板观察布局需添加 ImageGoal 面板或导入新布局。
扩展和布局也可从 ImageGoal 网页顶部下载。

在 Jetson 本机终端切换到监督观测栈：

```bash
cd /home/unitree/work/lightnav_realworld
bash deployment/go2/run_go2.sh native-imagegoal --prepare-only --operator-present
```

此时不创建任务、不启动执行器。已有冻结目标会原样保存完整 JPEG 和 SHA256，并导入新观测栈。入口只通过 bringup 正常停止已知自有观测 profile，不停止外部服务、不恢复旧任务。随后在网页选择完整目标图和“监督真机测试”，点击“开始监督真机任务”；Foxglove 控制面板也可冻结目标并点击“开始真机”。按钮会同时创建新任务并启动执行器。

若要在终端直接使用当前已冻结的目标执行一次，运行：

```bash
bash deployment/go2/run_go2.sh native-imagegoal --execute-supervised --operator-present
```

直接执行默认最多 60 秒。可追加 `--goal /absolute/path/goal.jpg`；`--duration-s` 指定为 5–120 秒。
默认后端仍是 NoMaD；LightNav 追加 `--backend lightnav`，需要额外指令时追加 `--instruction`。
Foxglove 中直接选择模型，按钮会显示“启动 NoMaD 真机”或“启动 LightNav 真机”。LightNav 的
原始 RGB 历史、参考目标图和完整 SE(2) 轨迹走同一执行器；原生参考图能力仍属于实验适配。

默认不加两个执行参数时只做只读准备，不启动服务、不发运动命令。网络 socket 被环境拒绝时，会在任何服务或任务改变前退出；请在 Jetson 本机终端执行。

速度上限为 0.30 m/s、0.55 rad/s；非零候选使用 TinyNav 的 0.10 m/s、0.20 rad/s 门限参考。零速始终保持零。最终是否下发仍由实时任务、相机、odom、命令时效、遥控接管和宇树原生避障执行端检查。

浏览器保持原来的 8093 地址。点击“停止任务”、Foxglove“停止真机”、终端执行模式下 Ctrl-C 或遥控接管可终止；执行器退出会归零并释放控制，只取消本次相同 session/task，保留相机和可视化服务。普通 shadow 任务仍只做观测。

记录位于 `runs/native_imagegoal_session_*/`：`goal.jpg`、`goal.sha256`、`goal_provenance.json`、`launch_report.json`，以及 `executor/report.json`。运行时记录路径见 bringup 输出。

这次使用官方前相机默认前向安装和主机接收时间近似；没有宣称获得官方外参矩阵、完成曝光时间与 odom 标定，或已验证自动到达。NoMaD 的距离停车阈值仍未标定，靠现场手动停止判断本轮接近目标的效果。
