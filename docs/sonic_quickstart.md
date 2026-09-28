# SONIC 全身模仿状态 · 快速上手

SONIC 035 **a3_fast** 全身模仿策略（`models/model_step_200000_a3_fast.onnx`，
obs 1570 维 → 全身 29 关节位置目标，50 Hz）在 cadence 中提供两个通用状态，
共用 `cadence.sonic.SonicTrackState`：

| 状态 key | 语义 | 子状态 |
|---|---|---|
| `sonic_clip` | **固定轨迹**：播放 `data/motions/` 下的 NPZ 动作片段 | RAMP（入场 blend 到站姿）→ READY（待机，十字键选剪辑）→ CUE（上参考第 0 帧）→ PLAYING → loco；左键播放则播完 RETURN（回站姿）→ READY |
| `sonic_stream` | **等传输流**：跟踪 UDP 运动参考流 | WAITING（PD 站桩等流）→ TRACKING → LOST（blend 回站姿）→ WAITING |

两态都是 `active_policy`：必须先 fixedpos 等 `entry_gate_ready`。状态 ID 由部署
的 registry 决定（参考注册表 `configs/state_registries/a3_operator_sonic.yaml`
为 6/7）。

**clip 交互**（READY 内读取 PLNJ 十字键边沿，按住不重复触发）：

| 十字键 | 动作 |
|---|---|
| 上 / 下 | 在 `clip.clips` 里循环选择剪辑（事件 `sonic_clip_selected:<名>`） |
| 右 | 播放选中剪辑一次，最后一拍接受后直接交还 loco（事件 `sonic_clip_started/finished`） |
| 左 | 播放选中剪辑一次，播完 RETURN 回站姿 READY，供循环试播 |

READY 永远停在**默认站姿**而不是剪辑第 0 帧：剪辑起点可能是动态不平衡姿势，
静态停不住；触发后由 CUE 相位在 `clip.ramp_s` 内平滑上到第 0 帧并立即交给策略。

## 配置与轨迹

- 状态配置：`configs/states/a3_sonic_clip.yaml` / `a3_sonic_stream.yaml`
  （024 增益表、模型路径、限位；剪辑列表 `clip.clips` + 默认 `clip.select`，
  每个名字对应 `resources.clip_<名字>`；流参数：220 ms 延迟线、10 帧 ×20 ms
  前瞻、失流 >250 ms 判定、0.5 s blend）。
- 剪辑：`data/motions/*.npz`（字段与来源见 `data/motions/README.md`）。
  新剪辑：`scripts/convert_sonic_clip.py <输入.csv> <输出.npz>`
  （SONIC 扁平 CSV → 50 Hz NPZ，关节序转 IsaacLab），然后加进
  `clip.clips` 与 `resources.clip_<名字>`。
- 流协议：`cadence.motion-ref.v1`（默认 `127.0.0.1:15120`），测试生产者：
  `scripts/send-motion-ref.py data/motions/BMD_0319_stand.npz --loop`。
- 非策略相位（RAMP / READY / CUE / RETURN / WAITING / LOST）的增益由
  `entry_gains` 选择。随仓发布的真机与 sim 配置统一显式使用 `pd_stand`
  （量产 PD_STAND 站立增益）；PLAYING / 有效参考的 TRACKING 策略指令始终使用
  `control.kp/kd` 的 024 增益。自定义配置省略此项时仍保留旧的 `policy` 默认，
  因此自定义上机配置也应明确继承或设置 `entry_gains: pd_stand`。

## 跑法

**mock（无物理，纯管线冒烟）**

```bash
./scripts/run.sh --config configs/entry/a3/mock/entry_a3_operator_sonic.yaml
# 另一终端请求状态并触发播放（6=sonic_clip，7=sonic_stream）：
./scripts/request_state.py 6 --expect-mode SONIC_CLIP \
    --await-substate READY --pulse-dpad-x 1 --then-substate PLAYING
# （clip 在 READY 等十字键：上下选剪辑、右=播放回 loco、左=播放回 READY；
#   request_state.py 的 --pulse-dpad-y ±1 可选剪辑）
# 一键验收（fixedpos 门控→clip 触发/播放/回站姿→stream 等流/跟踪/失流回退）：
./scripts/verify_sonic_mock.sh
```

**MuJoCo sim2sim（物理闭环，headless）**

```bash
./scripts/run.sh --config configs/entry/a3/mock/entry_a3_sonic_sim.yaml
./scripts/verify_sonic_sim.sh   # 站立 12 s + 行走 15 s，输出 z 高度与跟踪 RMSE
```

2026-09-24 实测（macOS）：stand RMSE 0.033 rad、walk RMSE 0.065 rad、全程无
safety halt。原理与近似（串联直驱踝/腰、PD_STAND 门控）见
[sonic_sim2sim.md](sonic_sim2sim.md)。

## 排障

- 从 loco 切入后尚未播放就软下来：检查最终 SONIC 配置是否有
  `entry_gains: pd_stand`。RAMP/READY 是静态 PD 保持，loco 已经退出，SONIC
  actor 尚未运行；不能拿较软的 024 策略增益代替站立增益。旧配置只在 sim
  覆盖此项，导致仿真正常而真机配置缺失。若日志出现 HALT，还需按对应
  `safety_reason` 检查，增益修复不等于排除了现场通信或超时问题。
- 进不了 sonic 状态：先看 fixedpos 的 `entry_gate_ready`。
- 卡在 READY：clip 模式在 READY 等十字键触发，不会自动播放（右=播放回
  loco，左=播放回 READY，上下选剪辑）。
- 状态查询：`cadence_protocol.client.OperatorClient(host, 50560).status()`，
  关注 `mode` / `substate` / `safety_halted`（halt 时带 `safety_reason`）。
- 回归基线：`./scripts/test.sh`。
