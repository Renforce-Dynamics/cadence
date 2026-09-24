# SONIC 全身模仿状态 · 快速上手

SONIC 035 **a3_fast** 全身模仿策略（`models/model_step_200000_a3_fast.onnx`，
obs 1570 维 → 全身 29 关节位置目标，50 Hz）在 cadence 中提供两个通用状态，
共用 `cadence.sonic.SonicTrackState`：

| 状态 key | 语义 | 子状态 |
|---|---|---|
| `sonic_clip` | **固定轨迹**：播放 `data/motions/` 下的 NPZ 动作片段 | RAMP（平滑上参考第 0 帧）→ PLAYING → DONE（默认 hold 末帧；`on_finish: loco` 可交还） |
| `sonic_stream` | **等传输流**：跟踪 UDP 运动参考流 | WAITING（PD 站桩等流）→ TRACKING → LOST（blend 回站姿）→ WAITING |

两态都是 `active_policy`：必须先 fixedpos 等 `entry_gate_ready`。状态 ID 由部署
的 registry 决定（参考注册表 `configs/state_registries/a3_operator_sonic.yaml`
为 6/7）。

## 配置与轨迹

- 状态配置：`configs/states/a3_sonic_clip.yaml` / `a3_sonic_stream.yaml`
  （024 增益表、模型路径、限位；流参数：220 ms 延迟线、10 帧 ×20 ms 前瞻、
  失流 >250 ms 判定、0.5 s blend）。
- 剪辑：`data/motions/*.npz`（字段与来源见 `data/motions/README.md`）。
  新剪辑：`scripts/convert_sonic_clip.py <输入.csv> <输出.npz>`
  （SONIC 扁平 CSV → 50 Hz NPZ，关节序转 IsaacLab）。
- 流协议：`cadence.motion-ref.v1`（默认 `127.0.0.1:15120`），测试生产者：
  `scripts/send-motion-ref.py data/motions/BMD_0319_stand.npz --loop`。
- 非策略相位（RAMP / WAITING / LOST）的增益由 `entry_gains` 选择：
  `policy`（默认，024 策略增益）或 `pd_stand`（量产 PD_STAND 硬增益，
  sim 变体使用）。

## 跑法

**mock（无物理，纯管线冒烟）**

```bash
./scripts/run.sh --config configs/entry/a3/mock/entry_a3_operator_sonic.yaml
# 另一终端请求状态（6=sonic_clip，7=sonic_stream）：
./scripts/request_state.py 6 --expect-mode SONIC_CLIP
# 一键验收（fixedpos 门控→clip 全程→stream 等流/跟踪/失流回退）：
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

- 进不了 sonic 状态：先看 fixedpos 的 `entry_gate_ready`。
- 状态查询：`cadence_protocol.client.OperatorClient(host, 50560).status()`，
  关注 `mode` / `substate` / `safety_halted`。
- 回归基线：`./scripts/test.sh`。
