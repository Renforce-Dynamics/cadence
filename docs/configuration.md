# 配置约定

`cadence-config` 是无机器人依赖的小包，planetJoystick、planetRecord、planetRelay、planet-rally 与应用入口共同使用。配置加载不扫描相邻仓库。

```yaml
extends: pkg://cadence/data/demo.yaml
compose:
  robot: robot.yaml
  backend: backend.yaml
  task: task.yaml
  site: site.yaml
  experiment: experiment.yaml
runtime:
  duration_s: 5
```

合并顺序：`extends` 的顺序 → robot → backend → task → site → experiment → 当前文件 → CLI overrides。字典递归合并，列表整体替换。重复 YAML key、循环继承、不存在的资源、未知 override 字段都会报错。领域字段由组件自身的 schema 检查；`cadence config validate` 检查组合结构，`cadence deploy --check` 检查部署的状态、模型和后端配置而不开启 I/O。

```bash
cadence config resolve configs/demo.yaml --set runtime.duration_s=2 --output runs/config
cadence config diff configs/demo.yaml other.yaml
```

输出 `resolved.yaml`、`provenance.json`、`config.sha256`。普通相对资源路径通过 `ResolvedConfig.path("backend.model")` 相对于**声明该字段的 YAML**解析，继承不会改变来源。CLI 输入文件可以相对当前目录。

资源支持：

- `pkg://cadence/data/two_joint.xml`：安装包内的资源。
- `artifact://policy`：通过显式 `artifacts={"policy": {"path": ..., "sha256": ...}}` 清单读取并验证内容。
- 绝对路径或声明文件旁的相对路径。

`cadence-rally` 兼容旧配置中明确的 `configs/`、`assets/`、`models/`、`contracts/` 前缀：它们相对于应用 bundle。新写的其他相对路径按声明来源解析。planet-rally 的旧配置也保留显式 bundle 引用，默认资源已随 wheel 分发。修改新应用的 bundle 通过 `CADENCE_RALLY_BUNDLE` 显式选择，不依赖运行目录。

应用的 `artifacts.lock` 校验模型和机器人资产；`cadence-rally doctor` 和 profile 启动会校验清单。修改资产后显式执行 `cadence-rally lock` 更新。`--output` 保存配置、运行参数、版本和资产清单，运行参数覆盖记录在 `deployment.json`。

Cadence 自己的 `cadence deploy` 与 `scripts/deploy.sh` 也提供独立部署及快照入口，默认输出到
`runs/deploy-...`；可用 `--output` 指定目录、`--set dotted=value` 覆盖已有字段。
模拟、A3 readonly 与命令部署都不要求引用 cadence-rally，见 [部署指南](deployment.md)。

## 后端 overlay

`pkg://cadence/data/backends/` 提供 `a3_readonly.yaml`、`a3_command.yaml` 和
`a3_sdk_mock.yaml`。将后端层与通用状态配置组合，得到完整部署：

```yaml
extends:
  - pkg://cadence/data/a3_operator_demo.yaml
  - pkg://cadence/data/backends/a3_readonly.yaml
runtime:
  duration_s: 5
```

完整入口也已提供在 `pkg://cadence/data/deployment/` 下。`backend.kind: a3` 的
`transport` 必须显式为 `aimrt` 或 `sdk_mock`。AimRT readonly 配置
`read_only: true`、`command_publish_enabled: false`；真实发布配置相反。
`sdk_mock` 可用 `read_only: false` 验证内存写确认，但 `command_publish_enabled` 必须为
false，且不能携带 AimRT 配置。

`backend.aimrt.config_path` 和 `backend.library_path` 由组合配置按声明来源解析。
前者默认引用 Cadence 包内的通用 Iceoryx 配置；SDK 控制 topic 名、同步偏差、状态等待超时、
命令 watchdog 与 neck 保持增益在 `backend` 下明确配置。
`backend.startup_timeout_s` 是首帧等待时长；`state_timeout_ms` 是后续状态等待与 SDK
已交付快照的新鲜度限制。A3 关节顺序必须与规范关节名一致，不能通过改动列表重排 SDK 输出。


## 配置与源码依赖

`cadence-config` 和 `cadence-protocol` 是 Cadence 仓库中的独立 distribution。PlanetJoystick 的 submodule 固定 Cadence 源码，bootstrap 只安装这两个轻量包；PlanetRecord 和 PlanetRelay 只需配置包。它们都不会因此安装控制运行时、NumPy 或 SDK。Cadence 本身的 `external/agi3sdk` 提供可选 A3 backend，默认不会安装或编译 SDK。完整关系见 [架构与仓库职责](architecture.md)。

`freeze()` 输出还包含 `overrides.json`。只有使用 `ResolvedConfig.path()` 的资源字段才按声明来源定位；录制输出目录保持相对进程工作目录的语义，设备路径和抽象 socket 地址不作路径重写。各组件的配置文档明确这些字段。

现行任务继承：PlanetJoystick 通用设备默认 → Cadence 通用状态键位 → planet-rally 的 rally 请求 → 显式 site overlay → cadence-rally stack 的本次连接与设备覆盖。`inputs.requests` 使用状态名作为键时可逐项覆盖，`null` 禁用继承项；旧列表仍整体替换。legacy recorder 与 debugger 也使用统一 loader；业务 schema 分别校验。状态机父子关系由运行时和应用定义，与 YAML 的 `extends` 无关。

## 通用 operator 接入

Cadence 的 `runtime.operator` 显式启用 PLNJ 接收与只读状态查询。未配置时不创建 operator socket：

```yaml
extends: pkg://cadence/data/a3_operator_demo.yaml
runtime:
  operator:
    host: 127.0.0.1
    port: 50560
    signal_mode: level
    mapping:
      velocity_axes: [left_y, left_x, right_x]
      velocity_scales: [0.4, 0.2, 0.5]
      emergency_signal_id: 0
      reset_signal_id: 1
    linear_slew_rate_mps2: 0.5
    yaw_slew_rate_radps2: 1.0
    velocity_deadzone: 0.1
```

通用默认值来自 `pkg://cadence/data/operator.yaml`。轴是协议中的归一化输入，由执行部署映射成 `vx`、`vy`（m/s）和 yaw（rad/s），再进行变化率限制。`level` 持续提供按住的安全信号；`rising` 只提供上升沿。PLNJ 断流或 TTL 过期时不再提供状态请求，并向变化率限制器输入零速度。它不会改写上肢目标邮箱。

`a3_operator_demo.yaml` 注册 `passive=0`、`damping=1`、`fixedpos=2`、`loco=3`，从 damping 启动。`a3_operator_stream_demo.yaml` 继承它，把 loco 工厂替换为实时上肢状态并启用独立目标端口。配套发送端是 `pkg://planetj/data/cadence.yaml`；先运行 `planetj --config pkg://planetj/data/cadence.yaml --check-remote` 验证 ID 和状态名。

## 通用外部定位

`runtime.localization` 独立于 operator 与上肢目标输入；省略或设为 `null` 时不创建接收端。
其配置字段如下，source 与坐标系名称必须与生产者完全匹配：

```yaml
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

协议为 `cadence.localization.v1`：位置 m、线速度 m/s、单位四元数 wxyz。接收端检查
source、frame、session/sequence 顺序及 TTL，不做隐式坐标变换。可用年龄为
`source_age_s + 接收后的单调时钟时间`，上限取发送端 TTL 和 `max_age_s` 的较小值；未测量
的网络排队时间不包含在内。过期或显式无效的新样本撤回定位，状态级保持与回退由运行时
契约决定。字段语义与轻量发送端示例见 [部署指南](deployment.md#外部定位)。

## 可复用运动状态配置

`pkg://cadence/data/motion/a3_lower.yaml` 包含完整的 A3 下肢契约：机器人参考姿态、关节分区与限位、PD 增益、下肢模型和上肢默认姿态。`a3_lower_stream.yaml` 继承它，供实时上肢状态使用。应用的状态配置继承这些包资源，仅覆盖部署差异：

```yaml
extends: pkg://cadence/data/motion/a3_lower.yaml
upper:
  default_position: [0.25, 0.10, 0, 0.90, 0, 0, 0, 0.25, -0.10, 0, 0.90, 0, 0, 0]
```

`upper.default_position` 是双臂关节角度，单位 rad。它与模型参考姿态 `robot.default_position` 分开：调整上肢控制姿态不改变策略的观测偏移或 action 解码。`lower.factory` 选择模型适配器，状态 registry 的 `factory` 选择 `cadence.motion:LowerLocoState` 或 `cadence.motion:LowerLocoStreamState`。

在 Cadence CLI 的运行配置中，`catalog.states.<id>.config` 可引用状态 YAML 文件；被引用的状态配置也支持 `extends`。`lower.model` 可使用包资源，默认值为 `pkg://cadence/data/models/a3_loco_lower.onnx`。应用无需复制 Cadence 的模型文件。

实时目标接收由运行配置显式启用：

```yaml
runtime:
  upper_target_udp: {state: loco, host: 127.0.0.1, port: 15100}
```

`state` 必须选择具有上肢输入邮箱的实时状态；接收端只支持 loopback。普通运行配置省略该字段便不创建接收端。此配置没有超时或默认姿态回退字段：实时状态在本次激活内持续保持最新目标；默认姿态在进入状态时重置。

完整字段与可运行示例见 [下肢移动与上肢关节控制](motion-composition.md)。
