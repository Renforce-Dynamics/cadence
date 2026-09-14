# 独立部署

首次 A3 上机请先阅读 [A3 现场操作教程](a3-onboard.md)：板端环境与 AimRT 构建、
原厂 PM/HAL 交接、只读验证，以及正常停机和恢复。本文说明本机部署入口与配置行为。

Cadence 可以独立部署通用状态机，通过共享 Planet 协议接收 PlanetJoystick、上肢关节目标和外部定位，
不需要安装或启动 cadence-rally。cadence-rally 在同一执行基础设施上注册发球、击球等
任务状态，并提供任务协议适配和配置；它作为应用启动时，也不需要另开一个 Cadence 进程。

配置和线协议由独立 planetConfig 仓库的 `planet-config`、`planet-protocol` 提供。
Cadence 和 Planet 组件都依赖这层共享库；Planet 组件不依赖 Cadence 或 SDK，
可以连接实现同一协议的其他执行端。

## 入口与配置

每个仓库提供自己的 `scripts/deploy.sh`。Cadence 的脚本调用 `cadence deploy`：

```bash
./scripts/deploy.sh --config pkg://cadence/data/a3_operator_demo.yaml --duration-s 1 --check
./scripts/deploy.sh --config pkg://cadence/data/a3_operator_demo.yaml --duration-s 1
./scripts/deploy.sh --venv /path/to/venv --config ./site.yaml --output ./runs/site
```

相对配置路径和输出目录相对于调用时的工作目录。省略 `--output` 时，deploy 为本次运行
创建 `runs/deploy-...` 配置快照；`cadence run` 仍可用于不保存快照的短时开发运行。
`--duration-s 0` 持续运行，正值指定秒数；Ctrl+C 或 SIGTERM 请求正常退出与资源清理。

`--check` 解析继承、状态工厂、模型、关节布局、网络接收配置和后端参数，不启动后端，
不开网络接收端，也不发布命令。它不要求真机发布确认；通过检查表示软件配置可以加载，
不代表 AimRT 中间件或机器人在线。

| 配置资源（均为 `pkg://cadence/data/` 下的路径） | 后端与用途 |
| --- | --- |
| `a3_operator_demo.yaml` | Python mock，通用状态请求和固定上肢姿态 |
| `a3_operator_stream_demo.yaml` | Python mock，通用状态请求和实时上肢目标 |
| `deployment/a3_sdk_mock.yaml` | Native SDK 静止状态与内存命令接收，用于验证 SDK 接口 |
| `deployment/a3_readonly.yaml` | 真实 AimRT 状态，readonly 计算，不创建硬件命令 publisher |
| `deployment/a3_command.yaml` | 真实 AimRT 状态与命令发布 |

这些配置共用 `passive=0`、`damping=1`、`fixedpos=2`、`loco=3`，从 damping 启动。
设备端采用 `pkg://planetj/data/operator.yaml`。状态名和 ID 可通过 PlanetJoystick 的
`--check-remote` 与运行中的部署进行匹配检查，检查本身不请求状态切换。

```bash
planetj --config pkg://planetj/data/operator.yaml --check-remote
planetj --config pkg://planetj/data/operator.yaml
```

先请求 fixedpos（RB+A），再请求 loco（RB+X）；能否切换仍由状态机的进入条件决定。
默认 operator 绑定 loopback；跨机器使用时，在 site 配置中显式设置接收地址及发送端目标。

## A3 SDK mock

在 Cadence 仓库内安装可选 SDK 与推理依赖：

```bash
./scripts/bootstrap.sh --extra a3 --extra inference
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_sdk_mock.yaml --check
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_sdk_mock.yaml --duration-s 1
```

`a3` extra 提供 SDK I/O；上述部署注册了 loco 状态，因此还需 `inference` extra，
即使从 damping 启动或只执行 `--check` 也会加载注册状态的模型。只使用无模型状态的
自定义部署可以仅安装 `a3`。这两个 extra 都不依赖 rally 仓库。

该模式显式配置 `backend.transport: sdk_mock`、`read_only: false` 和
`command_publish_enabled: false`。每次读取向 native SDK 注入新的静止 31 维零位置快照，
SDK 转换成规范 29 维状态；命令写入内存接收器，仍经过 SDK 的序号、状态时效、命令值和
写入结果检查。它不加载 AimRT，不发布硬件命令，也不模拟运动动力学。

需要动力学反馈时使用 mock 或 MuJoCo；不要把静止 SDK fixture 的运行结果作为 loco 的
运动效果验证。后端不会因 SDK 或 AimRT 不可用而自动切换到 mock。

## A3 readonly 与命令运行

真实 AimRT 模式除了 `a3` extra，还需要目标机器的厂商消息依赖、AimRT/Iceoryx 环境及
启用 AimRT 的 native SDK。示例构建命令如下，具体依赖见
[agi3sdk 原生接口文档](https://github.com/Renforce-Dynamics/agi3sdk/blob/main/native/README.md)：

```bash
./external/agi3sdk/scripts/build-native.sh \
  -DAGI3SDK_WITH_AIMRT=ON -DAGI3_AIMRT_SOURCE_DIR=/path/to/AimRT
```

Python wheel 包含绑定，不自动包含所有硬件运行依赖。分开安装 native SDK 时，可通过
`backend.library_path` 指定共享库，也可使用 SDK 的 `AGI3SDK_LIBRARY` 查找设置。
通用 AimRT 配置位于 `pkg://cadence/data/aimrt/aimrt_iceoryx.yaml`，部署 overlay 中指定
六路状态 topic、四路命令 topic、同步偏差和超时参数。站点差异通过新配置覆盖：

```yaml
extends: pkg://cadence/data/deployment/a3_readonly.yaml
backend:
  aimrt:
    config_path: ./aimrt-site.yaml
runtime:
  operator:
    host: 0.0.0.0
```

`aimrt.config_path` 按声明它的 YAML 所在目录解析；AimRT 自身的 plugin 路径由该文件
配置。A3 部署的关节列表必须与 SDK 的 29 维规范顺序完全一致，不能只检查维数相同。
SDK 保持 neck 两关节；29 维策略命令按 legs、waist、arms 排列。

```bash
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_readonly.yaml --check
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_readonly.yaml --duration-s 5

./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_command.yaml --check
A3_CONFIRM_ONBOARD=YES ./scripts/deploy.sh \
  --config pkg://cadence/data/deployment/a3_command.yaml
```

readonly 运行读取真实状态并进行 shadow 计算，允许观察状态转移与策略输出，但不调用 SDK
命令写入。其计算进度不表示机器人执行；运行摘要明确记录未发布硬件命令。普通命令模式
遵循 prepare → guard → SDK 写确认 → commit；过期命令被拒绝，不能推进已提交的策略历史。
SDK 负责等待新状态、序号与时效验证、命令 watchdog 和正常退出 damping。

命令部署保留已有的 `A3_CONFIRM_ONBOARD=YES` 确认要求。Cadence 不引入发球专属确认；
cadence-rally 的任务级发布条件仍由任务入口维护。

## 配置组合与实时上肢

任务可以组合通用状态配置和后端 overlay，而不复制运行循环：

```yaml
extends:
  - pkg://cadence/data/a3_operator_stream_demo.yaml
  - pkg://cadence/data/backends/a3_readonly.yaml
runtime:
  duration_s: 5
```

换用 `backends/a3_command.yaml` 后使用相同的状态请求与上肢目标接口。实时上肢的默认姿态
仅用于本次状态进入，之后持续执行最新的有效关节目标；输入断流不恢复默认姿态。离开状态
或安全监督介入时仍按运行时规则执行。默认上肢 UDP 端口只接受 loopback，生产者可以是
[PlanetJoystick 连续发送示例](https://github.com/Renforce-Dynamics/planetJoystick/tree/main/examples/upper_stream)。

operator 和上肢目标是两个独立输入。PLNJ 过期后不再提供状态请求，并向速度变化率限制器
提供零输入；这本身不等于所有状态自动急停。是否要求持续 operator 连接是状态声明的
`requires_operator_link` 契约，安全信号与其他监督继续按既有规则处理。

## 外部定位

配置 `runtime.localization` 才会创建通用定位接收端；省略或设置 `null` 时不监听端口：

```yaml
extends: pkg://cadence/data/deployment/a3_readonly.yaml
runtime:
  localization:
    host: 127.0.0.1
    port: 15110
    source: mocap
    frame_id: world
    child_frame_id: policy_root
    max_age_s: 0.25
    max_datagrams_per_poll: 64
```

发送端只需安装 planetConfig 的轻量 `planet-protocol`。下面展示单帧接口；连续接入时由感知进程根据
新样本重复调用 `send`，并填写采样时刻及发送时的样本年龄：

```python
from planet_protocol.localization import LocalizationClient

with LocalizationClient("127.0.0.1", 15110, source="mocap") as client:
    client.send(
        position_w_m=(0.0, 0.0, 0.9),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_w_mps=(0.0, 0.0, 0.0),
        source_age_s=0.0,
    )
```

协议名为 `planet.localization.v1`。位置和线速度位于 `frame_id`，单位为 m 和 m/s；
单位四元数按 wxyz 排列，表示从 `child_frame_id` 到 `frame_id` 的旋转。
接收端要求 source 和两个 frame 名称完全匹配，不进行隐式坐标转换。

Cadence 同时接收旧 `cadence.localization.v1`。operator 和上肢目标默认分别使用
`planet.operator.v1`、`planet.joint-target.v1`，也兼容对应旧名称；回复沿用请求的 schema。
旧 `cadence_protocol` 导入作为执行侧兼容层保留，新生产者直接使用 `planet_protocol`。

`session_id` 是递增的生产者 epoch，同一 session 中 `sequence` 递增；乱序与旧 session
不能覆盖最新样本。有效时间取发送端 `ttl_s` 和接收端 `max_age_s` 的较小值，年龄由发送端
`source_age_s` 加接收后的单调时钟时间计算。`source_timestamp_us` 只用于追踪对齐，不作为
跨机器时效判据；未测量的网络或系统排队延迟不会自动包含在年龄内。

显式 `valid: false` 或更新但已过期的样本撤回定位。接收端不会无限保持过期位姿；是否
暂存最后有效定位、拒绝进入依赖定位的状态或退回其他状态，由 Cadence 的状态契约决定。
发球时间、击球目标、球拍动作等语义属于 task 协议，仍在 rally 层适配。

完整的仓库分工见 [架构说明](architecture.md)，上肢帧与控制语义见
[运动组合契约](motion-composition.md)。
