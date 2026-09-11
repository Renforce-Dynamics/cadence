# Cadence

机器人执行节奏：按确定的控制周期组合输入、执行技能、校验命令，并在 backend 接受命令后提交技能进度。机器人关节数量来自 backend 与配置。

```bash
./scripts/setup.sh --wheelhouse /path/to/wheels --extra sim
./scripts/run.sh -- --backend mock --duration-s 1
./scripts/run.sh -- --backend mujoco --duration-s 1
./scripts/test.sh
./scripts/build.sh
```

- `cadence-api`：状态快照、关节命令、RobotIO 结构接口；不包含模型、任务或消息接收器。
- `cadence-config`：独立配置分层、严格 YAML、资源解析、来源追踪和快照。
- `cadence`：执行内核、状态注册、层次状态机、关节所有权合成、ONNX runner 和 backend。

应用通过 `cadence.applications` entry point 注册，例：`cadence run --profile cadence-rally/continuous`。通用包不 import A3、Planet 或应用技能。

一次真实提交使用 `prepare(input)` → `guard_pending()` → `backend.write_command(...)` → `commit(ticket)`；backend 拒绝则 `reject(ticket)`。`tick()` 是无外部 I/O 的已接受步，供测试和 shadow 计算使用。deadline 连续超限会锁存安全状态，急停优先于同周期 reset。

`HierarchicalMachine` 提供父子取消、激活编号和转换表。并行技能使用 `compose_commands` 明确声明关节范围，重叠声明直接拒绝；未声明关节保留传入的 fallback 命令。安全监督独立于业务状态机，最终命令只经一个 backend 出口。

参考 [配置文档](docs/configuration.md)、`configs/demo.yaml` 与 `tests/test_execution.py`。
