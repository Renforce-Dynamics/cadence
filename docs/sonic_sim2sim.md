# SONIC MuJoCo sim2sim

Closed-loop verification of the SONIC whole-body states against MuJoCo
physics, no hardware. Works on macOS (Apple Silicon) and x86 Linux.

## Setup

MuJoCo is cadence's optional `sim` dependency (task repositories do not
inherit it):

```bash
uv pip install --python .venv/bin/python "mujoco>=3,<4"
```

## Assets and approximations

`assets/a3_mujoco/a3.xml` adapts the SONIC serial-chain MJCF
(`a3_t2d5_passive_foot_twostage_fit_optimized.xml`, Mulan PSL v2 — see
`assets/a3_mujoco/NOTICE.md`):

- One `<joint>_motor` actuator per cadence policy joint, `ctrlrange` set to the
  024 effort limits.
- **The real robot's ankles and waist are closed-chain; this model drives the
  serial joints directly.** The closed chain's effective stiffness is much
  higher than the serial joint's, which is why the sim needs the PD_STAND
  bring-up gains (below).
- Pelvis IMU site (`imu_in_pelvis`) feeds the `pelvis-orientation` framequat
  and `pelvis-angular-velocity` gyro sensors (noise removed); the site doubles
  as the root site. Passive compliant-foot joints keep their fitted springs;
  head joints stay locked.
- Joint armature values from `gear_sonic/.../a3.py` are applied to every
  policy joint; start height 1.069 m (sole contact at the default pose).

## Configuration

- `configs/backends/a3_mujoco.yaml` — `kind: mujoco`, 1 ms physics (20
  substeps per 50 Hz control tick), `initial_joint_position` = SONIC default
  stand pose (cadence order).
- `configs/state_registries/a3_operator_sonic_sim.yaml` — overrides only state 2
  (fixedpos) and states 6/7 (sonic) for sim:
  - `configs/states/a3_fixedpos_sim.yaml` uses the released A3 PD_STAND gains
    (ankle kp 500). Open-loop standing with the soft 024 policy gains
    (ankle kp 60) is statically unstable for the serial-ankle model; the same
    is true on the real robot, where manual bring-up uses PD_STAND.
  - `configs/states/a3_sonic_clip_sim.yaml` / `a3_sonic_stream_sim.yaml` set
    `entry_gains: pd_stand`: the non-policy PD phases (clip RAMP; stream
    WAITING/LOST) run PD_STAND gains, matching SONIC production bring-up.
    Policy phases always use the 024 gains from `control.kp/kd`.
- `configs/entry/a3/mock/entry_a3_sonic_sim.yaml` — the sim entry.
  `start_state: fixedpos`: in damping the simulated robot falls before the
  first request arrives, and ramping from the fallen pose passes through
  out-of-limit commands (kernel safety halt). Starting in the entry posture
  avoids this.

## Run

```bash
./scripts/run.sh --config configs/entry/a3/mock/entry_a3_sonic_sim.yaml
```

then drive states with `./scripts/operator.sh --config
configs/entry/operator/entry_sonic_clip.yaml` etc. as usual.

## Verification

```bash
./scripts/verify_sonic_sim.sh
```

Stage 1 starts the real runtime process on the sim entry and drives
fixedpos → sonic_clip (walk) → damping over the operator protocol, asserting
no safety halt. Stage 2 (`scripts/verify_sonic_sim.py`) runs the same chain
in-process on sim time and reports physics metrics; current numbers:

| Stage | Pelvis z min–max | Joint tracking RMSE |
| --- | --- | --- |
| fixedpos gate | 0.857 m (standing) | — |
| sonic_clip stand clip, 12 s | 0.857–0.861 m | 0.033 rad |
| sonic_clip walk clip, 15 s | 0.822–0.863 m | 0.065 rad |

Nominal standing pelvis site height is 0.857 m; the acceptance band is
0.60–1.00 m with RMSE < 0.15 rad and no safety halt. The full 33 s walk clip
also completes: DONE hold at z=0.859 m, RMSE 0.064 rad over the clip.

Known limitations: the serial-ankle approximation makes the sim softer than
hardware (closed-chain stiffness is not modeled); the walk clip's root
translation is tracked only implicitly through the policy (no world-frame
localization feedback is used by the state).
