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
- `<visual><global>` gains `offwidth/offheight="1920x1080"` so offscreen
  rendering at up to 1080p works (`scripts/render_sonic_sim.py`).
- Passive compliant-foot joints are kept unactuated with their fitted springs.
- Head yaw/pitch use agi3dep's physical stops (yaw ±60°, pitch −25°/+15°),
  damping 1.0, friction loss 0.1, and armature 0.0008100893338. They remain
  passive, outside the 29-joint policy contract, matching agi3dep's Python
  sim2sim backend (its head motors receive no commands). The source's
  `range="0 0"` disabled MuJoCo autolimits rather than locking the head;
  under gravity the unbounded pitch joint could flip almost 180°. These
  explicit nonzero limits fix that dynamics error; this is not a visual-only
  change or an implementation of the hardware SDK's neck hold.
- Upper-body visuals (torso_Link, torso_shell_Link, head_yaw_Link,
  head_pitch_Link) retain their source body frames, matching agi3dep and
  the collision hulls. Their front faces +X. The previous extra 180-degree
  visual rotation reversed the head and chest and has been removed. A
  world +X arrow and front/side renders establish the orientation; camera
  azimuth alone does not identify the robot's front. These visual geoms
  have zero density and no collision, so this correction leaves dynamics
  unchanged.
