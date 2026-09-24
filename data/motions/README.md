# SONIC reference motions

50 Hz NPZ clips for the `sonic_clip` state (`cadence.sonic.SonicTrackState`),
converted from SONIC-for-A3 flat CSVs (Apache License 2.0, AgiBot Inc. 2026)
with `scripts/convert_sonic_clip.py`:

| File | Frames | Duration | Source CSV |
| --- | --- | --- | --- |
| `001_walk_front_slow.npz` | 1652 | 33.0 s | `a3_data/agibot_a3/001_walk_front_slow.csv` (120 Hz, stride 4) |
| `BMD_0319_stand.npz` | 727 | 14.5 s | `gear_sonic_deploy/assets/a3_runtime/teleop_motions/BMD_0319_a3_filtered_20260319_164305__stand_Skeleton0.csv` |

NPZ fields: `q_ref`/`dq_ref` [T,29] in IsaacLab policy order,
`root_quat_wxyz` [T,4], `dt` = 0.02, `joint_names`.
