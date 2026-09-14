# 下肢移动与上肢关节控制

Cadence 提供两个可跨任务复用的运动状态：下肢策略持续控制移动与支撑，上肢选择固定姿态或实时关节目标。A3 的下肢模型、观测适配、默认配置与状态实现均由 Cadence 提供；应用负责选择动作来源和业务切换。

| 状态工厂 | 上肢行为 | 默认配置 |
| --- | --- | --- |
| `cadence.motion:LowerLocoState` | 整个激活期间保持配置姿态 | `pkg://cadence/data/motion/a3_lower.yaml` |
| `cadence.motion:LowerLocoStreamState` | 进入时采用配置姿态，随后执行并保持最新关节目标 | `pkg://cadence/data/motion/a3_lower_stream.yaml` |

实时输入是**关节角度，单位 rad**。上游可以是动作生成器、遥操作或运动学模块；若上游产生末端位姿，应先由上游求解成关节角度。状态将关节目标写入 `q_des`，使用配置中的 `kp`、`kd`，`dq_des` 与 `tau_ff` 为零。

## 独立运行

在 Cadence 仓库安装推理依赖后即可运行，不需要安装任务应用：

```bash
./scripts/bootstrap.sh --extra inference
.venv/bin/cadence run --config pkg://cadence/data/a3_lower_demo.yaml --duration-s 1
```

该示例运行真实 A3 ONNX 策略，使用内存 mock backend；它验证推理和命令执行链，不模拟接触动力学。运行摘要中的 `dimension` 为 29，`mode` 为 `loco`。Cadence 自带独立 A3 readonly、命令和 native SDK mock 部署配置，见 [部署指南](deployment.md)；MuJoCo 的机器人场景由部署配置选择。

实时上肢示例在本地端口 `15100` 接收目标：

```bash
.venv/bin/cadence run --config pkg://cadence/data/a3_lower_stream_demo.yaml --duration-s 0
```

在另一终端发送 14 个双臂关节角度：

```bash
python scripts/send-upper-target.py --port 15100 --status
python scripts/send-upper-target.py --port 15100 --sequence 0 --q-des \
  0.25 0.10 0 0.90 0 0 0  0.25 -0.10 0 0.90 0 0 0
python scripts/send-upper-target.py --port 15100 --sequence 1 --q-des \
  0.30 0.12 0 0.80 0 0 0  0.30 -0.12 0 0.80 0 0 0
```

脚本在未指定 `--activation` 时先查询当前激活编号。连续生产者应在状态进入后获取一次编号，使用该编号持续发送递增的 `sequence`。端口默认关闭，仅当运行配置显式设置 `runtime.upper_target_udp` 时启动；适配器只绑定 loopback 地址。

## PlanetJoystick 配套入口

连续发送示例由 [planetJoystick](https://github.com/Renforce-Dynamics/planetJoystick/tree/main/examples/upper_stream) 维护，使用 planetConfig 定义的共享协议。本配套部署由 Cadence 接收：operator 进程发送状态请求、轴和安全信号；独立的 `planetj-upper` 进程发送关节目标。PlanetJoystick 不依赖 Cadence 或 SDK，Cadence 持有本部署的状态机、下肢模型、PD 增益和提交边界。

```bash
# Cadence 环境：默认从 damping 启动
cadence run --config pkg://cadence/data/a3_operator_stream_demo.yaml
# PlanetJoystick 环境：查询绑定，再运行手柄输入
planetj --config pkg://planetj/data/operator.yaml --check-remote
planetj --config pkg://planetj/data/operator.yaml
# 第三个终端：默认采集实体手柄；也可显式选择 --source sine 或 scripted
planetj-upper --config pkg://planetj/data/upper_stream.yaml
```

使用 RB+A 请求 fixedpos，随后 RB+X 请求 loco。上肢发送端只发现 activation 和发布帧，不触发状态切换。实体手柄断开后停止发布，不发送默认姿态；重新连接同一 activation 时继续递增序号。新任务直接继承这些接口和配置，无需依赖 rally。

## 默认姿态与配置继承

任务只需覆盖上肢姿态，不需要复制机器人契约和模型：

```yaml
# my_lower.yaml：可作为 registry 中一个状态的 config
extends: pkg://cadence/data/motion/a3_lower.yaml
upper:
  default_position: [0.25, 0.10, 0, 0.90, 0, 0, 0, 0.25, -0.10, 0, 0.90, 0, 0, 0]
```

固定姿态状态在进入后持续使用该配置值。实时状态可继承 `a3_lower_stream.yaml` 设置同一字段：每次进入都重新采用默认姿态，之后只按收到的新帧更新目标。默认配置不会按控制周期重新覆盖上肢目标。

单独运行时，以下配置将自定义状态文件接到示例 registry；两个文件放在同一目录：

```yaml
# run.yaml
extends: pkg://cadence/data/a3_lower_demo.yaml
runtime:
  velocity_command: [0.0, 0.0, 0.0]  # vx m/s、vy m/s、yaw rad/s
catalog:
  states:
    3:
      config: my_lower.yaml
```

```bash
.venv/bin/cadence run --config run.yaml --duration-s 1
.venv/bin/cadence config resolve my_lower.yaml --output runs/motion-config
```

要在自定义运行配置启用实时输入，选择实时状态工厂并显式配置接收端：

```yaml
extends: pkg://cadence/data/a3_lower_stream_demo.yaml
runtime:
  upper_target_udp:
    state: loco
    host: 127.0.0.1
    port: 15100
catalog:
  states:
    3:
      config: my_lower.yaml
```

两个状态使用相同的配置 schema；实时或固定行为由 `factory` 选择。YAML 的 `extends` 负责配置值继承，registry 的 `factory` 负责实现选择，运行时激活负责控制生命周期。

| 字段 | 责任 |
| --- | --- |
| `robot.default_position` | 模型观测和动作解码的参考姿态 |
| `robot.action_scale` | 将下肢归一化 action 转为关节目标的比例 |
| `robot.position_min` / `position_max` | 全关节限位 |
| `robot.lower_joints` / `upper_joints` | 不重叠且覆盖全身的控制分区 |
| `control.kp` / `kd` | 所有受控关节的 PD 增益，可为向量或统一标量 |
| `lower.factory` | 模型适配器；默认 `cadence.motion.a3:A3LowerPolicy` |
| `lower.model` | 默认 `pkg://cadence/data/models/a3_loco_lower.onnx` |
| `lower.history_frames` | A3 H4 模型固定为 4 |
| `lower.mask_upper_observation` | 是否将观测中的上肢归零到训练参考姿态；A3 默认配置为 `false` |
| `upper.default_position` | 上肢控制目标，顺序遵循 `robot.upper_joints` |
| `entry_smoothing_s` | 状态进入时的命令过渡时长 |

自定义上肢姿态应修改 `upper.default_position`。`robot.default_position` 是模型契约，改变它会同时改变观测与 action 解码。配置数组整体替换，因此上肢姿态必须提供完整 14 个角度。

## 实时状态的保持语义

1. **进入**：重置下肢观测历史，生成新的 activation，丢弃上次激活的帧，先提交配置中的上肢默认姿态。
2. **接收**：当前 activation 内，只保留 `sequence` 更大的有效帧；控制周期读取当时最新的目标。接收快于控制时，中间帧可以被覆盖。
3. **执行**：将最新上肢位置与本周期下肢输出合并成一条完整 PD 命令。命令被 backend 接受后，才更新已提交的上肢序号和动作反馈。
4. **保持**：未收到新帧、网络延迟或发送端断流时，持续发送最后的目标；没有 TTL，没有断流回退默认姿态。尚未收到第一帧时保持进入姿态。
5. **退出**：停用输入并清除上一轮帧。重新进入生成新 activation，重新采用默认姿态。

第一条默认命令尚未提交时到达的实时帧会保留，待默认命令成功提交后的控制周期再处理。重复、乱序和上一轮 activation 的帧不替换当前目标；维度错误、非有限值或超限目标被拒绝，已保存目标保持不变。

```mermaid
stateDiagram-v2
  [*] --> INACTIVE
  INACTIVE --> DEFAULT: 状态进入，创建 activation
  DEFAULT --> DEFAULT: 首条默认命令提交成功
  DEFAULT --> HOLDING: 后续周期无新帧，保持进入姿态
  DEFAULT --> TRACKING: 默认已提交，新目标提交成功
  HOLDING --> TRACKING: 新目标提交成功
  TRACKING --> TRACKING: 更新目标提交成功
  TRACKING --> HOLDING: 没有新目标，保持最后目标
  HOLDING --> HOLDING: 延迟或断流，继续保持
  DEFAULT --> INACTIVE: 状态退出
  HOLDING --> INACTIVE: 状态退出
  TRACKING --> INACTIVE: 状态退出
```

急停和安全故障优先输出安全命令，暂停运动命令执行。安全锁存本身不立即退出当前状态，因此上肢邮箱的 activation 仍可存在，接收确认也不能视为执行许可。执行 reset 或显式状态切换时，运行时调用退出回调并清除旧 activation。持续保持目标的规则服从这一安全提交边界。

## 进程内接入与提交确认

同进程的生产者可以直接发布 `JointTargetFrame`，不需要 UDP：

```python
from cadence.motion import JointTargetFrame


# 生产者在本轮状态进入后获取一次；不要把旧轨迹自动绑定到新激活。
activation = state.upper_targets.activation


def publish_upper_position(state, activation, sequence, positions_rad):
    if activation is None:
        return False
    return state.upper_targets.publish(JointTargetFrame(
        activation=activation,
        sequence=sequence,
        q_des=positions_rad,
    ))
```

`publish()` 返回 `True` 或 UDP 回应 `accepted: true`，仅表示最新帧邮箱已接收。它不保证该帧一定执行：更晚帧可能覆盖它，状态也可能退出。`state.committed_upper_sequence` 在 backend 接受整条命令后才推进；`last_action` 根据实际接受的完整关节命令计算。backend 接受命令也不等于关节已经到达该角度，物理效果应读取机器人反馈。

命令模式的执行链是 `prepare` → `guard_pending` → backend write → `commit`。安全拒绝或写入失败时不推进上肢提交序号，A3 adapter 的观测历史也回滚。readonly 部署进行 shadow 计算而不写命令，其状态进度不表示机器人执行。输入接收在独立线程运行，控制周期读取不可变快照，不等待网络到包。

## A3 模型和通用状态的边界

| A3 分区 | 全身索引 | 控制来源 |
| --- | --- | --- |
| 双腿 12 关节 | `0:12` | 下肢策略 |
| 腰部 3 关节 | `12:15` | 下肢策略 |
| 左臂 7 关节 | `15:22` | 固定姿态或最新上肢目标的前 7 维 |
| 右臂 7 关节 | `22:29` | 固定姿态或最新上肢目标的后 7 维 |

每侧双臂的顺序是 shoulder pitch、shoulder roll、shoulder yaw、elbow、wrist roll、wrist pitch、wrist yaw。A3 adapter 严格要求上述 29 维布局，观测为 397 维，输出腿腰 15 维；25 维训练上下文中的固定占位值保留为模型 ABI，不引入任务 planner 输入。

H4 观测按项排列四帧历史，最旧帧在前，重置后的第一帧填满历史。`mask_upper_observation: true` 将观测中的双臂位置替换为 `robot.default_position`，速度和已执行 action 置零；它不修改机器人原始反馈，也不修改实际上肢命令。是否屏蔽由所选模型和部署配置决定。

`LowerLocoState` 与 `LowerLocoStreamState` 本身不固定机器人维度。另一种机器人可以配置自己的关节分区、增益和限位，并通过 `lower.factory` 提供适配器：

```python
class LowerPolicyAdapter:
    def __init__(self, config, services): ...
    def reset(self): ...
    def infer(self, frame, last_action):
        # 返回 lower_joints 顺序的归一化 action 和一维观测。
        ...
    def snapshot(self): ...
    def restore(self, snapshot): ...
```

适配器有历史时应实现 `snapshot()` / `restore()`；状态在拒绝命令后恢复快照。下肢适配器、上肢帧生产者和任务状态机都不直接写 backend。需要更复杂的多控制器组合时，可继续使用 `cadence.control.composition` 的 `CommandPart` / `compose_commands` 接口。

实现与回归入口：[运动状态](../src/cadence/motion/states.py)、[目标接入](../src/cadence/motion/targets.py)、[状态测试](../tests/test_motion_states.py)、[输入测试](../tests/test_motion_targets.py)、[A3 数值回归](../tests/test_a3_lower_motion.py)。
