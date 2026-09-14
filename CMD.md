# 常用命令

以下命令在 Cadence 仓库根目录执行。运行入口必须显式传入 `configs/entry/entry_*.yaml`。

```bash
./scripts/run.sh --config configs/entry/entry_sim.yaml
./scripts/run.sh --config configs/entry/entry_a3_operator.yaml
```

前者运行双关节 MuJoCo 示例，后者运行 A3 通用状态机和内存后端，等待 operator 请求。

## 选择配置

| 入口，均位于 `configs/entry/` | 用途 | 依赖 extra |
| --- | --- | --- |
| `entry_mock.yaml` | 双关节内存后端 | 无 |
| `entry_sim.yaml` | 双关节 MuJoCo 仿真 | `sim` |
| `entry_a3_lower.yaml` | A3 下肢策略与固定上肢，内存后端 | `inference` |
| `entry_a3_lower_stream.yaml` | A3 下肢策略与实时上肢，内存后端 | `inference` |
| `entry_a3_operator.yaml` | 手柄请求通用 A3 状态，固定上肢 | `inference` |
| `entry_a3_operator_stream.yaml` | 手柄请求通用 A3 状态，实时上肢 | `inference` |
| `entry_sdk_mock.yaml` | native SDK 静止状态与内存写确认 | `a3`、`inference` |
| `entry_onboard_a3_real_readonly.yaml` | A3 真实状态，只读计算 | `a3`、`inference`、AimRT |
| `entry_onboard_a3_real.yaml` | A3 真实状态与命令 | 同上 |
| `entry_onboard_upper_stream_readonly.yaml` | 真实状态与实时上肢，只读计算 | 同上 |
| `entry_onboard_upper_stream.yaml` | A3 命令模式，实时上肢 | 同上 |

按实际用途安装依赖，例如：

```bash
./scripts/bootstrap.sh --extra sim
./scripts/bootstrap.sh --extra a3 --extra inference
```

A3 的硬件环境、HAL 启动和命令发布确认见 [上机教程](docs/a3-onboard.md)。

## 独立手柄进程

在两个终端分别运行；第二个终端使用已安装 PlanetJoystick 的环境：

```bash
# 终端 1：Cadence runtime
./scripts/run.sh --config configs/entry/entry_a3_operator.yaml
# 终端 2：PlanetJoystick producer
planetj --config configs/entry/entry_joystick.yaml
```

实时上肢 runtime 可换用 `entry_a3_operator_stream.yaml`，手柄入口相同。
`entry_joystick.yaml` 只引用本仓库 `configs/operators/joystick.yaml`，由 PlanetJoystick 读取；
不传给 `scripts/run.sh`。键位、设备和目标地址在该配置修改，机器人速度比例在 `configs/inputs/operator.yaml` 修改。
异机使用时，发送目标与 runtime 的接收地址、端口应对应。

| Linux Xbox 按键 | 请求/信号 |
| --- | --- |
| RB + 左摇杆按下 | `passive=0` |
| RB + B | `damping=1` |
| RB + A | `fixedpos=2` |
| RB + X | `loco=3` |
| Xbox/Home | 急停信号 `0` |
| RB + Menu | 复位信号 `1` |

左摇杆控制前后/横移，右摇杆横轴控制转向；默认速度比例为 `0.4 m/s`、`0.2 m/s`、`0.5 rad/s`。
先进入 fixedpos 完成姿态过渡，再请求 loco。runtime 已启动时可检查名称与 ID：

```bash
planetj --config configs/entry/entry_joystick.yaml --check-remote
```

## 运行选项

```bash
./scripts/run.sh --config configs/entry/entry_sdk_mock.yaml --output runs/sdk-session
./scripts/run.sh --config configs/entry/entry_onboard_a3_real.yaml --check
```

`--config` 必需；`--output` 指定快照目录，默认是 `runs/deploy-时间-随机编号`。
`--check` 加载配置、状态和模型，不创建网络接收端、不连接机器人，也不要求发布确认。
`deploy.sh` 保留为相同运行逻辑的兼容别名。

后端、初始状态、模型、接收地址及 `runtime.duration_s`、`runtime.headless` 都写在入口配置链中。
`duration_s: 0` 持续运行；正数表示运行秒数。按 Ctrl+C 正常停止并写出结果。
需要现场差异时，修改所选配置；需要保留独立版本时，在 `configs/entry/entry_site.yaml`
显式 `extends` 现有入口，再覆盖差异，见 [配置约定](docs/configuration.md)。

## 配置与开发工具

```bash
.venv/bin/cadence config resolve configs/entry/entry_sim.yaml --output runs/config
.venv/bin/cadence config diff configs/entry/entry_mock.yaml configs/entry/entry_sim.yaml
./scripts/doctor.sh
./scripts/test.sh
./scripts/build.sh
./scripts/submodules.sh check
```

配置解析展示入口的合并结果；状态注册表及状态文件在运行加载阶段继续解析，完整检查使用 `run.sh --check`。
详细行为见 [独立部署](docs/deployment.md)、[运动组合](docs/motion-composition.md) 和 [外部定位](docs/localization.md)。
