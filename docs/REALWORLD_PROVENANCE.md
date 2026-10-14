# 源码来源与恢复

- 官方基线：lightorigins/LightNav-0，commit 3015508b70fb6e30bbd66701e94c9b916dcb8df8，保留历史、Apache-2.0 和 UPSTREAM_README.md。
- 实机源快照：已保存的 Go2 / RealSense session 20261009_151148_dbd02d。恢复前验证 94 个源码文件 SHA-256。
- 先应用完整 working-tree patch，恢复 Go2、客户端与网页改动；再复制快照的录制、归档、诊断、launch 和测试。JS/HTML 没有进入原 source 列表，通过 patch 恢复。
- 上位机 ws_server.py、对应测试和协议文档本地改动单独恢复：客户端 telemetry、完整响应、阶段时间和错误记录。
- 本次新增双机配置、启动、预检查、进程收尾、部署文档和协议测试；去除旧目录名和机器地址。MPC 的纯协议/参数函数移到 vln_mpc.protocol，保持逻辑并允许离线测试。

[source-manifest.json](../deployment/source-manifest.json) 的哈希记录的是**恢复输入**，不是移植/格式化后的源码；最终工程由 Git commit 标识。以后运行快照会重新哈希运行版本。

模型参考 revision 826dc5fbfa37afa8293d2e336d329b6ffc0bfb64。权重、MCAP、视频、日志、私有环境和主研究仓库的实验分支均不进入部署 Git。

MuJoCo / Habitat / EVT 保留作上游参考，其评测与本次双机部署验证不同。本次没有新增 world model、训练模型或宣称导航成功率提升。
