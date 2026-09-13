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

合并顺序：`extends` 的顺序 → robot → backend → task → site → experiment → 当前文件 → CLI overrides。字典递归合并，列表整体替换。重复 YAML key、循环继承、不存在的资源、未知 override 字段都会报错。领域字段由组件自身的 schema 检查；`cadence config validate` 检查组合结构，组件 `doctor` 检查实际可运行配置。

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


## 配置与源码依赖

配置包是 Cadence 仓库中的独立 distribution；Planet 组件的 submodule 固定 Cadence 源码，但 bootstrap 只安装 `packages/cadence-config`。Cadence 本身的 `external/agi3sdk` 提供可选 A3 backend，默认不会安装或编译 SDK。

`freeze()` 输出还包含 `overrides.json`。只有使用 `ResolvedConfig.path()` 的资源字段才按声明来源定位；录制输出目录保持相对进程工作目录的语义，设备路径和抽象 socket 地址不作路径重写。各组件的配置文档明确这些字段。

现行任务继承：planetJoystick 通用设备默认 → planet-rally 的 rally 请求 → 显式 site overlay → cadence-rally stack 的本次连接与设备覆盖。legacy recorder 与 debugger 也使用统一 loader；业务 schema 分别校验。状态机父子关系由运行时和应用定义，与 YAML 的 `extends` 无关。
