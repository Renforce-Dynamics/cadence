# cadence

机器人执行与仿真运行时：通用状态机、下肢 locomotion、固定上肢姿态和实时关节目标。

Linux、Python 3.10+、`uv`；A3 原生库构建需要 CMake 和 C++ 编译器。

```bash
git clone --recurse-submodules git@github.com:Renforce-Dynamics/cadence.git
cd cadence
./scripts/bootstrap.sh --extra sim --extra inference --extra a3
```

运行仿真：

```bash
./scripts/run.sh --config configs/entry/examples/entry_sim.yaml
```

A3 上机先完成 [环境、AimRT 构建和 HAL/PM 交接](docs/a3-onboard.md)，再依次运行只读验证与正式控制：

```bash
./scripts/run.sh --config configs/entry/a3/onboard/entry_onboard_a3_real_readonly.yaml
# 结束只读进程后启动命令模式
A3_CONFIRM_ONBOARD=YES ./scripts/run.sh --config configs/entry/a3/onboard/entry_onboard_a3_real.yaml
```

运行方式由配置决定；修改后端、时长、初始状态、网络和上肢姿态，都修改所选入口的配置链。

```text
cadence/
├── configs/
│   ├── entry/
│   │   ├── examples/   # 双关节 mock / MuJoCo
│   │   ├── a3/mock/    # A3 策略、operator、SDK mock
│   │   ├── a3/onboard/ # 真机只读 / 命令、实时上肢
│   │   └── joystick/   # 独立手柄进程
│   └── ...             # robots、backends、state_registries、states、inputs 等
├── models/             # ONNX 模型、校验清单
├── assets/             # MuJoCo XML 等运行资源
└── src/cadence/         # Python 代码
```

A3 operator 默认状态为 `passive=0`、`damping=1`、`fixedpos=2`、`loco=3`。固定上肢持续使用配置姿态；流式上肢进入时采用默认姿态，此后保持最新关节目标，断流也保持。角度单位为弧度。

手柄是独立进程，使用本仓库配套入口：`planetj --config configs/entry/joystick/entry_joystick.yaml`。
PlanetJoystick 在发送端单独安装；Cadence 不安装它。运行时和手柄各用自己的 entry，配套命令和键位见 [CMD.md](CMD.md)。

[命令与入口选择](CMD.md) · [A3 上机教程](docs/a3-onboard.md) · [配置](docs/configuration.md) · [运动组合](docs/motion-composition.md) · [全部文档](docs/README.md)

共享配置与协议来自 [planetConfig](https://github.com/Renforce-Dynamics/planetConfig)，A3 I/O 来自 [agi3sdk](https://github.com/Renforce-Dynamics/agi3sdk)；乒乓球状态由 [cadence-rally](https://github.com/Renforce-Dynamics/cadence-rally) 扩展。

Renforce Dynamics · [作者](AUTHORS.md) · [MIT](LICENSE)
