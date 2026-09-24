# Asset notice

`a3.xml` and the `meshes*` directories are adapted from the public
[A3/A3U robot model](https://github.com/AgibotTech/A3-A3U-robot-model)
(Mulan PSL v2, see `LICENSE-MulanPSL-2.0.txt`), redistributed through the
SONIC-for-A3 release (AgiBot Inc., Apache License 2.0 for the SONIC code).

Adaptations in `a3.xml` relative to
`gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_passive_foot_twostage_fit_optimized.xml`:

- `meshdir` points at this directory; the referenced mesh subdirectories are
  copied alongside.
- One `<joint>_motor` actuator per the 29 cadence policy joints, with
  `ctrlrange` set to the 024 actuator effort limits
  (`gear_sonic/envs/manager_env/robots/a3.py`). The source model's
  generic 31-motor actuator block (no `ctrlrange`, head motors included) was
  removed. The real robot's ankle and waist are closed-chain; this sim model
  drives the serial joints directly (sim v1 approximation).
- Joint armature values from `gear_sonic/envs/manager_env/robots/a3.py`
  applied to every policy joint (rotor inertia used in training).
- Pelvis IMU gyro sensor noise removed for deterministic simulation.
- Pelvis start height set to 1.069 m (sole contact at the default pose;
  matches the source model's own stand keyframe).
- Passive compliant-foot joints are kept unactuated with their fitted
  springs; head joints remain locked as in the source model.
