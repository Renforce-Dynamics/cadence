# cadence

机器人执行与仿真运行时：通用状态机、下肢 locomotion、上肢姿态与关节目标流。任务状态由应用扩展。

## 安装

Linux、Python 3.10+、`uv`；A3 原生库还需 CMake 和 C++ 编译器。

```bash
git clone --recurse-submodules git@github.com:Renforce-Dynamics/cadence.git
cd cadence
./scripts/bootstrap.sh --extra a3 --extra inference
```

## A3 上机

先按 [A3 操作教程](docs/a3-onboard.md) 完成 AimRT 构建、PM 服务交接和 HAL 启动，再在机器人上运行：

```bash
# 接收真实状态，不发布命令
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_readonly.yaml

# 正式控制；结束上一进程后执行
A3_CONFIRM_ONBOARD=YES ./scripts/deploy.sh \
  --config pkg://cadence/data/deployment/a3_command.yaml
```

使用现场配置：`./scripts/deploy.sh --config ./site.yaml --output ./runs/session`。
`deploy.sh` 在当前机器启动 runtime，并保存配置快照。同步源码、安装依赖在启动前完成。

## 操作与上肢

在已安装 [PlanetJoystick](https://github.com/Renforce-Dynamics/planetJoystick) 的控制机运行：

```bash
planetj --config pkg://planetj/data/operator.yaml --check-remote
planetj --config pkg://planetj/data/operator.yaml
```

默认状态：`passive=0`、`damping=1`、`fixedpos=2`、`loco=3`。先 RB+A 到 fixedpos，再 RB+X 到 loco。跨机器时按教程配置接收地址和发送目标。

| 通用状态 | 上肢行为 |
| --- | --- |
| `LowerLocoState` | 执行配置中的固定关节姿态 |
| `LowerLocoStreamState` | 进入时使用默认姿态，随后执行最新关节角度；断流保持最新命令 |

上肢角度单位为弧度。配置及发送示例见 [运动组合](docs/motion-composition.md)；定位输入见 [外部定位](docs/localization.md)。

## 常用命令

```bash
.venv/bin/cadence config resolve ./site.yaml --output ./runs/config
./scripts/doctor.sh
./scripts/test.sh
./scripts/build.sh
./scripts/submodules.sh check
```

[文档索引](docs/README.md) · [配置](docs/configuration.md) · [部署参数](docs/deployment.md) · [架构](docs/architecture.md)

共享配置与协议来自 [planetConfig](https://github.com/Renforce-Dynamics/planetConfig)，A3 I/O 来自 [agi3sdk](https://github.com/Renforce-Dynamics/agi3sdk)；乒乓球状态在 [cadence-rally](https://github.com/Renforce-Dynamics/cadence-rally)。

Renforce Dynamics · [作者](AUTHORS.md) · [MIT](LICENSE)
