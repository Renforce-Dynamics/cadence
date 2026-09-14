# 独立部署

Cadence 独立运行通用状态、下肢策略和输入接收。cadence-rally 在相同基础设施上增加任务状态；
启动任务应用时无需再启动一个 Cadence 进程。首次真机部署先完成 [A3 上机教程](a3-onboard.md)。

## 运行

```bash
./scripts/run.sh --config configs/entry/entry_sim.yaml
./scripts/run.sh --config configs/entry/entry_sdk_mock.yaml --output runs/sdk-session
```

配置路径相对于调用时的工作目录。入口放在根 `configs/entry/`；后端、时长、初始状态、窗口和网络均由其引用的配置决定。
默认保存到 `runs/deploy-时间-随机编号`，`--output` 可指定目录。`runtime.duration_s: 0` 持续运行；
正值指定秒数。Ctrl+C 或 SIGTERM 请求正常退出并清理资源。

`--check` 加载继承、注册表、状态工厂、模型、关节布局、网络配置和后端参数；
不创建接收端、不启动 RobotIO，也不要求真机发布确认。通过检查不表示 HAL 或机器人在线。

| `configs/entry/` 下的入口 | 后端和状态 |
| --- | --- |
| `entry_mock.yaml` / `entry_sim.yaml` | 双关节 mock / MuJoCo |
| `entry_a3_operator.yaml` | A3 通用状态和固定上肢，内存后端 |
| `entry_a3_operator_stream.yaml` | A3 通用状态和实时上肢，内存后端 |
| `entry_sdk_mock.yaml` | native SDK 静止状态与内存命令接收 |
| `entry_onboard_a3_real_readonly.yaml` | 真实 AimRT 状态，只读计算 |
| `entry_onboard_a3_real.yaml` | 真实 AimRT 状态与命令发布 |
| `entry_onboard_upper_stream_readonly.yaml` / `entry_onboard_upper_stream.yaml` | 只读 / 命令模式，实时上肢 |

A3 operator 入口从 damping 启动，注册 passive=0、damping=1、fixedpos=2、loco=3。
先请求 fixedpos，再请求 loco；实际切换仍受状态进入条件和安全监督控制。
入口通过 `runtime.state_registry_config` 引用注册表，注册表再引用 `configs/states/` 的状态配置。

## A3 SDK mock

```bash
./scripts/bootstrap.sh --extra a3 --extra inference
./scripts/run.sh --config configs/entry/entry_sdk_mock.yaml
```

`a3` 安装并构建 SDK，`inference` 用于注册表中的 loco 模型；即使从 damping 启动，加载时也会检查该模型。
这两个依赖都不需要 Rally。仅有无模型状态的自定义配置可以只安装 `a3`。

SDK mock 显式设置 `transport: sdk_mock`、`read_only: false`、`command_publish_enabled: false`。
它向 native SDK 注入静止的 31 维零位置状态，转换为规范 29 维输出；命令写入内存接收器，
仍检查序号、时效、数值和写入结果。它不启动 AimRT、不模拟接触动力学。
需要运动效果验证时使用机器人对应的 MuJoCo 场景；后端不会在硬件依赖缺失时自动退回 mock。

## A3 只读与命令

真实运行需要厂商消息库、AimRT/Iceoryx 以及启用 AimRT 的 native SDK。构建及 HAL 步骤见 [上机教程](a3-onboard.md)。
通信参数位于 [A3 后端配置](../configs/backends/a3_readonly.yaml) 和 [AimRT 配置](../configs/aimrt/aimrt_iceoryx.yaml)。

```bash
./scripts/run.sh --config configs/entry/entry_onboard_a3_real_readonly.yaml
# 结束只读进程后，再启动唯一的命令进程
A3_CONFIRM_ONBOARD=YES ./scripts/run.sh --config configs/entry/entry_onboard_a3_real.yaml
```

只读模式读取真实状态并进行 shadow 计算，不调用 SDK 命令写入。其状态变化不代表机器人执行，
摘要记录 `command_writes: 0`、`command_publish_enabled: false` 和 `shadow_steps`。
命令模式遵循 prepare → guard → SDK 写确认 → commit；过期命令被拒绝，其他写入错误停止运行。
SDK 负责新状态等待、序号与时效验证、命令 watchdog 和正常退出 damping。

`startup_timeout_s` 控制首帧等待；`state_timeout_ms` 控制后续状态等待及已读取状态的新鲜度。
A3 关节顺序必须与规范 29 维名称一致；SDK 单独保持 neck 两关节。
命令发布保留 `A3_CONFIRM_ONBOARD=YES`；通用 Cadence 没有发球专属确认。

## 输入与运行结果

operator、实时上肢、定位分别由 `configs/inputs/` 中的配置启用。跨机器接入时修改接收地址和发送目标；
实时上肢接收器目前只绑定 loopback。默认状态请求见 [A3 教程](a3-onboard.md)，上肢生产者见 [运动组合](motion-composition.md)。

operator 过期后清除状态请求并向速度变化率限制器输入零值。当前通用 loco 未要求 operator 链路持续在线，
因此不能把手柄断流当作自动急停。上肢目标在激活期间保持最新值；定位输入则按 TTL 过期，
由状态契约决定保持与回退。外部感知使用共享 `planet.localization.v1`，详见 [外部定位](localization.md)。

本次快照保存解析配置、来源、版本与运行摘要。读取 `result.json` 时同时检查 `failure`、`halted`、
`command_writes` 和 `shadow_steps`；收到输入、通过状态请求或完成 shadow 计算均不等于硬件动作执行成功。
配置文件组织与继承见 [配置约定](configuration.md)，全部命令见 [CMD.md](../CMD.md)。
