# A3 上机操作：环境、HAL 与 PM

本教程用于在 A3 MDU 上运行 Cadence 的通用状态机。Cadence 负责策略与状态切换，
agi3sdk 负责 AimRT 状态和命令接口；厂商 HAL 与 `agibot_pm` 由现场单独管理。
乒乓球状态与发球确认属于 [cadence-rally](https://github.com/Renforce-Dynamics/cadence-rally)。

以下示例使用板端目录 `/agibot/cadence`，`ssh agi` 指向现场 MDU；请先核对 SSH 别名。
厂商环境路径沿用现有 A3 部署：`/agibot/software/v0`、ROS Jazzy 和
`/opt/agibot/share/ros2_package/aima_protocol_ros2_package`。设备版本不同应先核对实际路径。

## 1. 板端源码与 Python 环境

当前 `scripts/deploy.sh` 是**本机配置快照与运行入口**，不会 SSH 同步、安装远端软件，
也不管理 HAL 或 PM。旧 `agi3dep/scripts/onboard/deploy_to_a3.sh` 的远端部署工具没有
迁入本仓库；下面直接在目标机安装。不要把控制机的 x86 虚拟环境或共享库复制到 A3 使用。

板端需要 aarch64 Linux、Python 3.10+、`uv`、Git、CMake 3.20+ 和支持 C++20 的编译器。
由有部署目录写权限的账号执行：

```bash
ssh agi
uname -m
cd /agibot
git clone --recurse-submodules git@github.com:Renforce-Dynamics/cadence.git
cd /agibot/cadence
./scripts/submodules.sh check
./scripts/bootstrap.sh --extra a3 --extra inference
```

`uname -m` 应为 `aarch64`。bootstrap 安装本仓库及固定版本的源码依赖，并构建 SDK；
首次构建默认没有启用 AimRT，下一节才构建硬件 transport。注册表包含 loco 模型，
因此只读入口也需要 `inference`。已有源码更新时，应核对选定的 commit、恢复其 submodule
版本，再重新安装和构建；不要在控制进程运行时替换环境或共享库。

目标机无法访问 GitHub/包源时，需事先准备完整的递归源码与匹配 aarch64/Python 版本的
Python 依赖。`bootstrap.sh --offline` 只使用已有包缓存，不会自动下载或制作离线 bundle。

## 2. 加载厂商环境并构建 AimRT SDK

准备与现场插件匹配的 AimRT 1.6.0 源码；本仓库不附带旧项目的 AimRT 离线 bundle。
下面的 `/agibot/deps/AimRT-1.6.0` 是显式示例路径，须替换为已准备、核验过的源码目录。
在板端构建终端执行：

```bash
cd /agibot/cadence
source /agibot/software/v0/entry/env/env.sh
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="/opt/agibot/share/ros2_package/aima_protocol_ros2_package:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="/opt/agibot/share/ros2_package/aima_protocol_ros2_package/lib:${LD_LIBRARY_PATH:-}"
export PYTHONNOUSERSITE=1

A3_AIMRT_SOURCE=/agibot/deps/AimRT-1.6.0
test -f "$A3_AIMRT_SOURCE/CMakeLists.txt"
./external/agi3sdk/scripts/build-native.sh \
  -DCMAKE_BUILD_TYPE=Release \
  -DAGI3SDK_WITH_AIMRT=ON \
  -DAGI3_AIMRT_SOURCE_DIR="$A3_AIMRT_SOURCE"
```

该脚本执行 CMake 配置、构建与 SDK 单元测试，不启动 HAL 或控制机器人。SDK CMake 会
启用 AimRT Core 与 ROS2 消息 type support；实际通信使用 Iceoryx，不启动 ROS2 node。
`joint_msgs` 来自厂商消息包，`sensor_msgs` 来自 ROS 环境。

离线构建时，还需准备 AimRT 的依赖源码。若统一放在 `/agibot/deps/aimrt-deps`，用下面
这组完整参数替代上一条构建命令；各子目录必须包含对应依赖的源码。源码应使用可写的
构建副本，避免上游 CMake 补丁改动已校验的归档：

```bash
A3_AIMRT_DEPS=/agibot/deps/aimrt-deps
./external/agi3sdk/scripts/build-native.sh \
  -DCMAKE_BUILD_TYPE=Release \
  -DAGI3SDK_WITH_AIMRT=ON \
  -DAGI3_AIMRT_SOURCE_DIR="$A3_AIMRT_SOURCE" \
  -Dlibunifex_LOCAL_SOURCE="$A3_AIMRT_DEPS/libunifex" \
  -Dfmt_LOCAL_SOURCE="$A3_AIMRT_DEPS/fmt" \
  -Djsoncpp_LOCAL_SOURCE="$A3_AIMRT_DEPS/jsoncpp" \
  -Dyaml-cpp_LOCAL_SOURCE="$A3_AIMRT_DEPS/yaml-cpp" \
  -Devents_executor_LOCAL_SOURCE="$A3_AIMRT_DEPS/events_executor" \
  -Dasio_LOCAL_SOURCE="$A3_AIMRT_DEPS/asio" \
  -Dgflags_LOCAL_SOURCE="$A3_AIMRT_DEPS/gflags" \
  -Dtbb_LOCAL_SOURCE="$A3_AIMRT_DEPS/tbb" \
  -Dbackward_LOCAL_SOURCE="$A3_AIMRT_DEPS/backward" \
  -DFETCHCONTENT_FULLY_DISCONNECTED=ON
```

明确选择本次构建的库，并检查动态链接与现场插件：

```bash
export AGI3SDK_LIBRARY=/agibot/cadence/external/agi3sdk/build/native/libagi3sdk.so
test -f "$AGI3SDK_LIBRARY"
ldd "$AGI3SDK_LIBRARY"
test -f /opt/agibot/bin/libaimrt_iceoryx_plugin.so
ldd /opt/agibot/bin/libaimrt_iceoryx_plugin.so
```

`ldd` 不应出现 `not found`。默认 AimRT 配置中的插件路径就是上面的 `/opt/agibot/bin`；
若现场不同，复制 [AimRT 配置](../src/cadence/data/aimrt/aimrt_iceoryx.yaml) 到部署目录修改，
再通过 `backend.aimrt.config_path` 指向该文件。不要只改 SDK 库路径而遗漏插件依赖。
构建选项和 ABI 细节见 [SDK 文档](https://github.com/Renforce-Dynamics/agi3sdk/blob/main/native/README.md)。

## 3. 准备本次运行配置

在 `/agibot/cadence` 下创建独立配置。示例允许控制机经 UDP 50560 接入；也可以将
`host` 改为 MDU 实际网卡地址。本机输入使用 `127.0.0.1`：

```bash
cd /agibot/cadence
mkdir -p deployment
cat > deployment/a3-readonly.yaml <<'YAML'
extends: pkg://cadence/data/deployment/a3_readonly.yaml
runtime:
  operator:
    host: 0.0.0.0
    port: 50560
YAML
cat > deployment/a3-command.yaml <<'YAML'
extends: a3-readonly.yaml
backend:
  read_only: false
  command_publish_enabled: true
YAML
./scripts/deploy.sh --config deployment/a3-readonly.yaml --check
./scripts/deploy.sh --config deployment/a3-command.yaml --check
```

`--check` 加载配置与模型，但不连接 AimRT、不检查 HAL 是否在线。部署快照默认保存在
`runs/deploy-*`，包括解析后的配置、来源和覆盖值。配置机制详见 [独立部署](deployment.md)。

每个新的 runtime 终端都需要重新加载第 2 节的厂商环境、ROS、消息库路径，并设置
`AGI3SDK_LIBRARY`。`deploy.sh` 不会代替这些 shell 环境准备。

## 4. 检查 PM，交接 HAL

停止原厂服务或进入命令模式前，现场需确认机器人可靠支撑、物理急停可用、关节活动范围
无人，并明确急停负责人。软件阻尼和 watchdog 不能替代物理急停。先检查，不直接停止：

```bash
ssh agi
systemctl is-active agibot_pm
systemctl status agibot_pm --no-pager
pgrep -af '[a]imrt_main_hal'
pgrep -af '[c]adence|[r]un_python_onboard.py'
```

已有 runtime 或 HAL 时，先用 `ps -fp PID` 确认所属任务和操作者。原厂 PM 为 active 时，
不能同时启动自定义命令 runtime。完成现场交接后：

```bash
sudo systemctl stop agibot_pm
systemctl is-active agibot_pm
pgrep -af '[a]imrt_main_hal'
```

PM 应变为 `inactive`。此命令在 inactive 时返回非零退出码是正常现象。旧 HAL 若仍存在，
先确认归属，不盲目 `pkill -9`，也不再启动第二份。

在一个独立前台终端中，根据现场硬件**只启动一种 HAL**。

eLink：

```bash
ssh -t agi
cd /agibot/software/v0
source entry/env/env.sh
bash scripts/hal_elink/start_hal_elink.sh
```

EtherCAT：

```bash
ssh -t agi
cd /agibot/software/v0
source entry/env/env.sh
bash scripts/hal_ethercat/start_hal_ethercat.sh
```

保留 HAL 前台终端，检查关节状态输出和设备错误。另一个终端通过
`pgrep -af '[a]imrt_main_hal'` 确认进程；仅存在进程不等于六路状态同步正常，下一步用只读
runtime 验证。HAL 退出、状态缺失或设备报错时，先解决底层问题。

## 5. 只读验证，再进入命令模式

在已经加载第 2 节环境的 runtime 终端中：

```bash
cd /agibot/cadence
./scripts/deploy.sh --config deployment/a3-readonly.yaml --duration-s 5
```

只读模式接收真实状态并执行 shadow 计算，不创建硬件命令 publisher。检查退出时的摘要
及本次 `runs/deploy-*/result.json`：`ticks > 0`、`failure: null`、`halted: false`、
`command_writes: 0`，并有 `shadow_steps`。只读状态变化不代表机器人已经执行动作。

控制机上的 PlanetJoystick 独立安装后，配置发送目标为 MDU 实际地址。例如：

```yaml
extends: pkg://planetj/data/operator.yaml
target: {host: 192.168.120.121, port: 50560}
```

将它保存为控制机上的 `operator-a3.yaml`。若需要验证键位与 runtime catalog 的对应关系，
让只读 runtime 用 `--duration-s 0` 持续运行，在控制机执行：

```bash
planetj --config operator-a3.yaml --check-remote
planetj --config operator-a3.yaml
```

关闭只读 runtime 后，再次确认支撑、急停、HAL 与进程状态，启动唯一的命令 runtime：

```bash
cd /agibot/cadence
A3_CONFIRM_ONBOARD=YES ./scripts/deploy.sh \
  --config deployment/a3-command.yaml --duration-s 0
```

默认从 `damping` 启动。通用 profile 的 IDs 为 passive=0、damping=1、fixedpos=2、loco=3；
先请求 fixedpos（RB+A）确认姿态收敛，再请求 loco（RB+X）。DAMPING 为 RB+B。
修改键位后以实际配置和 catalog 检查为准。通用 Cadence 不需要 `A3_CONFIRM_SERVE`；
cadence-rally 的任务入口另有发球确认。确认变量只用于当前命令，不写入 shell profile。

## 6. 正常停机与恢复原厂程序

1. 通过 operator 请求 DAMPING，确认进入阻尼状态。
2. 在 runtime 终端按 `Ctrl-C`，等待进程退出和结果写入；再停止手柄等输入进程。
3. 在 HAL 前台终端按 `Ctrl-C`，确认 `aimrt_main_hal` 已退出。
4. 现场需要恢复原厂控制时，才启动 PM。

正常退出会关闭 SDK，命令模式尝试发送 safe damping。watchdog 与 Python 在同一进程，
无法覆盖进程崩溃或 `SIGKILL`；不要将 `kill -9` 作为常规停机方式。

终端不可用时，在 MDU 上检查进程并对核实过的 PID 发送 `SIGTERM`：

```bash
pgrep -af '[c]adence|[r]un_python_onboard.py'
A3_RUNTIME_PID=12345  # 替换为刚刚核实的 runtime PID
ps -fp "$A3_RUNTIME_PID"
kill -TERM "$A3_RUNTIME_PID"
ps -p "$A3_RUNTIME_PID" -o pid,args
```

恢复 PM 前，确认自定义 runtime 与人工 HAL 均已停止：

```bash
pgrep -af '[c]adence|[r]un_python_onboard.py'
pgrep -af '[a]imrt_main_hal'
sudo systemctl start agibot_pm
systemctl is-active agibot_pm
systemctl status agibot_pm --no-pager
```

预期 PM 为 `active`。启动失败时查看 `journalctl -u agibot_pm -n 100 --no-pager`。
发生抖动、异响、姿态突变或人员进入活动范围时，应立即使用物理急停。

## 7. 常见故障

| 现象 | 检查与处理 |
| --- | --- |
| CMake 找不到 `joint_msgs` / `sensor_msgs` | 重新加载厂商环境、ROS Jazzy 和 `CMAKE_PREFIX_PATH`；确认对应消息包已安装。 |
| AimRT 功能不可用 | 确认构建使用 `AGI3SDK_WITH_AIMRT=ON`，检查 `external/agi3sdk/build/CMakeCache.txt`；`AGI3SDK_LIBRARY` 应指向本次硬件构建。 |
| 动态库或 Iceoryx 插件加载失败 | 对 SDK `.so` 和插件执行 `ldd`；检查 `LD_LIBRARY_PATH` 与配置中的 plugin path。 |
| 超时等待机器人状态 | 检查 HAL、Iceoryx 环境及六路 state topic；默认最大时间戳偏差 20 ms、状态超时 50 ms。不要用放宽超时掩盖缺帧。 |
| `command publication requires A3_CONFIRM_ONBOARD=YES` | 完成现场确认后，仅在本次命令启动前设置该变量。 |
| `Cannot assign requested address` | 执行 `ip -br addr`，确保 receiver 的 bind 地址属于本机。 |
| 手柄查询超时或输入断流 | 核对 MDU IP、UDP 50560、接收端 bind 和 sender 配置；用 `ss -lunp` 检查端口占用。 |
| PM 无法恢复 | 确认自定义 runtime/HAL 均已退出，再查看 `journalctl -u agibot_pm -n 100 --no-pager`。 |

六路输入是 waist、neck、arms、legs joint state，以及 pelvis、torso IMU；四路命令对应
waist、neck、arms、legs。具体 topic 名和时效参数见
[A3 readonly 后端配置](../src/cadence/data/backends/a3_readonly.yaml)。
