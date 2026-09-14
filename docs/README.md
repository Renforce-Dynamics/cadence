# Cadence 文档

Cadence 提供通用状态机、运动组合与执行后端。按使用阶段选择入口：

| 文档 | 内容 |
| --- | --- |
| [A3 上机操作](a3-onboard.md) | 板端环境与 AimRT 构建、PM/HAL 交接、只读与命令运行、停机恢复和排错 |
| [独立部署](deployment.md) | `deploy.sh`、后端选择、配置快照、运行参数与 SDK mock |
| [配置管理](configuration.md) | 配置继承、覆盖、资源路径和来源追踪 |
| [运动组合](motion-composition.md) | 下肢 locomotion、固定上肢与持续关节目标、状态进入和断流保持 |
| [外部定位](localization.md) | 共享定位协议、坐标系、输入时效及失效处理 |
| [系统架构](architecture.md) | 仓库边界、协议归属、状态扩展与调用关系 |

软件安装与最短运行命令见 [仓库 README](../README.md)。首次真机操作从 A3 上机教程开始；
`deploy.sh --check` 只验证软件配置，不表示 HAL 或机器人在线。
