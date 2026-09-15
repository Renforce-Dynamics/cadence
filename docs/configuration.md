# 配置约定

`configs/` 是运行配置的唯一来源，入口统一放在 `configs/entry/**/entry_*.yaml`。
Python 包只分发代码；配置、模型和仿真资源分别保存在仓库根 `configs/`、`models/`、`assets/`。启动时显式选择入口：

```bash
./scripts/run.sh --config configs/entry/examples/entry_sim.yaml
```

后端、初始状态、运行时长、窗口、网络、注册表和上肢模型选择都由入口的配置链决定。
运行命令只接收配置路径、快照目录和检查选项。

## 文件分工

| 路径 | 内容 |
| --- | --- |
| `configs/entry/examples/` | 双关节 mock 与 MuJoCo 示例 |
| `configs/entry/a3/mock/` | A3 模型、operator 与 SDK mock 验证 |
| `configs/entry/a3/onboard/` | 真机只读、命令与流式上肢 |
| `configs/entry/joystick/` | 独立手柄进程 |
| `configs/robots/` | 关节名称、顺序、限位 |
| `configs/backends/` | mock、MuJoCo、A3 transport 与命令发布设置 |
| `configs/state_registries/` | 状态 ID、key、factory、state config 与安全目的状态 |
| `configs/states/` | 每个状态的姿态、增益、下肢模型和上肢默认值 |
| `configs/inputs/` | operator、上肢目标和定位接收配置 |
| `configs/operators/` | 与本仓库状态注册表匹配的手柄设备、键位、发送地址 |
| `configs/runtime/` | 频率、控制期限、默认时长和窗口设置 |
| `configs/aimrt/` | AimRT/Iceoryx 插件配置 |
| `models/` | 下肢模型和校验清单 |
| `assets/` | 仿真 XML 等资源 |

例如 [A3 operator 入口](../configs/entry/a3/mock/entry_a3_operator.yaml) 组合通用基础配置，并选择独立注册表：

```yaml
extends:
  - ../../../runtime/control.yaml
  - ../../../robots/a3.yaml
  - ../../../backends/mock.yaml
  - ../../../inputs/operator.yaml
runtime:
  state_registry_config: ../../../state_registries/a3_operator.yaml
  start_state: damping
  duration_s: 0
```

注册表本身是 catalog mapping，不再嵌入入口。它也支持继承：

```yaml
# configs/state_registries/a3_lower.yaml
extends: safety.yaml
states:
  3:
    key: loco
    factory: cadence.motion:LowerLocoState
    config: ../states/a3_lower.yaml
```

`runtime.state_registry_config` 相对于声明它的文件解析，`states.<id>.config` 相对于声明该引用的注册表解析。
被引用的状态文件继续按自己的来源解析模型路径。继承不会改变路径来源，运行目录不会改变这些引用。

手柄进程单独使用 `planetj --config configs/entry/joystick/entry_joystick.yaml`，继承本仓库完整的
`configs/operators/joystick.yaml`。它与 `a3_operator.yaml`、`a3_operator_stream.yaml` 均匹配
`0 passive / 1 damping / 2 fixedpos / 3 loco`；归一化轴由手柄发送，速度比例由 runtime 的
`configs/inputs/operator.yaml` 决定。两个进程各自选择 entry；保存配套配置不会增加 PlanetJoystick
安装依赖或 submodule。键位与启动命令见 [CMD.md](../CMD.md#独立手柄进程)。

## 修改与继承

直接修改现有入口及其引用文件即可。需要保存现场版本时，在 `configs/entry/a3/onboard/entry_site.yaml`
引用既有入口，只写差异：

```yaml
extends: entry_onboard_a3_real_readonly.yaml
runtime:
  duration_s: 5
  operator:
    host: 0.0.0.0
```

```bash
./scripts/run.sh --config configs/entry/a3/onboard/entry_site.yaml
```

`extends` 按列表顺序合并，最后合并当前文件。字典递归合并，列表整体替换。
共享 loader 也支持显式 `compose` 层；仓库入口以 `extends` 展示依赖。
重复 YAML key、循环引用和缺失文件会报错；运行加载进一步校验字段、状态、模型和关节布局。
YAML 继承只合并配置，状态工厂选择实现，运行时决定状态切换与生命周期。

```bash
.venv/bin/cadence config resolve configs/entry/examples/entry_sim.yaml --output runs/config
./scripts/run.sh --config configs/entry/examples/entry_sim.yaml --check
```

`config resolve` 展示入口的合并结果；运行加载还会展开注册表及其状态文件。
运行快照包含解析后的配置、来源与摘要，详见 [独立部署](deployment.md)。

## A3 后端和运动状态

[后端配置](../configs/backends/a3_readonly.yaml) 显式选择 transport：

| 用途 | `transport` | `read_only` | `command_publish_enabled` |
| --- | --- | --- | --- |
| 真机只读 | `aimrt` | `true` | `false` |
| 真机命令 | `aimrt` | `false` | `true` |
| SDK 接口验证 | `sdk_mock` | `false` | `false` |

`backend.aimrt.config_path` 指向根 `configs/aimrt/` 中的配置；现场共享库可由 `backend.library_path` 指定。
两者按声明文件解析。topic、同步偏差、超时和 neck 保持增益都在后端配置中。
A3 的 29 维关节名称和顺序必须与 SDK 一致。

[固定上肢状态配置](../configs/states/a3_lower.yaml) 包含完整 A3 下肢契约；
[实时上肢配置](../configs/states/a3_lower_stream.yaml) 继承它。
调整上肢姿态修改 `upper.default_position`，完整提供 14 个弧度值。
`robot.default_position` 属于模型观测和动作解码契约，不应作为上肢动作配置修改。
`lower.factory`、`lower.model` 选择下肢适配器和模型文件。默认模型在根 `models/a3_loco_lower.onnx`，
状态配置通过 `../../models/a3_loco_lower.onnx` 引用它；仿真 XML 在根 `assets/two_joint.xml`。
文件路径相对于声明它的 YAML 解析，不查询安装包或其他仓库；缺失文件直接报错。

任务仓库通过固定版本的 Cadence submodule 引用这些文件，例如任务的 `configs/states/loco.yaml`：

```yaml
extends: ../../external/cadence/configs/states/a3_lower.yaml
upper:
  default_position: [0.25, 0.10, 0, 0.90, 0, 0, 0, 0.25, -0.10, 0, 0.90, 0, 0, 0]
```

任务状态可以在自己的 state config 中声明具名文件资源，例如：

```yaml
# configs/states/reference.yaml；文件由任务仓库提供
resources:
  trajectory: ../../assets/reference.npz
```

`resources` 是名称到显式文件路径的映射。每个路径相对于声明该字段的 YAML 解析；继承仅覆盖
指定的名称，其余资源保留原来的路径来源。空值、URI、非字符串和缺失文件都会报错，不查找包内资源。
Cadence 将绝对路径保留在 state config 中交给任务工厂，文件格式、关节映射和播放语义由工厂校验。
所有部署状态同时获得 `services.dimension` 和 `services.joint_names`（与 `robot.joints` 同序的 tuple）；
任务应据此映射具名关节。`--check` 同样解析资源并加载工厂，但不启动输入或 RobotIO。
部署快照的 `deployment.json.resource_sha256` 记录每个声明资源的内容哈希。

operator 的状态响应在已有字段之外可包含 `substate`，对应最近已接受控制结果的 `skill_state`。
任务可以用它报告 `READY`、`HOLD` 等阶段；未发布子状态的启动快照和旧调用方仍省略此字段。
状态主动交接后先报告 `ENTERING`，新状态的首个控制结果被接受后才报告它自身的子状态。
可选布尔字段 `entry_gate_ready` 独立报告最近已提交的进入条件：启动和状态切换后为 `false`，
例如固定姿态达到配置的插值进度与位置容差且命令被接受后才为 `true`，不能从 `substate` 推断。
嵌入式调用通过只读的 `RuntimeKernel.entry_gate_ready` 获取这一值，`RuntimeOutput` 的字段保持兼容。
同时核对 `mode`、`safety_halted` 和 `execution`，其中 `shadow` 只表示只读计算结果。

## 三类输入

- [operator](../configs/inputs/operator.yaml)：归一化轴映射成速度并限速；默认 UDP 50560。
- [upper targets](../configs/inputs/upper_targets.yaml)：将关节目标交给注册的流式状态；默认 `127.0.0.1:15100`，显式 host 可使用 IPv4/IPv6 单播或监听通配地址。
- [localization](../configs/inputs/localization.yaml)：接收显式 source 和坐标系的定位；默认 UDP 15110。

入口没有引用对应输入时就不创建接收端。operator 断流后不再提供状态请求，并向变化率限制器输入零速度；
这不等于自动急停，是否强制回退取决于状态的 `requires_operator_link` 契约。
流式上肢在当前激活期间持续保持最新目标，断流不会恢复默认姿态。
定位按 TTL 过期，状态自行决定保持或回退。详细字段见 [运动组合](motion-composition.md) 与 [外部定位](localization.md)。

配置与协议实现来自独立 [planetConfig](https://github.com/Renforce-Dynamics/planetConfig)。
Cadence 的 `cadence_config`、`cadence_protocol` 仅保留兼容导入；Planet 系列直接依赖共享库，不安装 Cadence 或 SDK。
