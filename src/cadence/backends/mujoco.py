"""Name-bound MuJoCo backend and PD stepping, independent of any task."""

from pathlib import Path
import numpy as np
from cadence_api import JointCommand, RobotState


class MujocoBackend:
    def __init__(
        self,
        model_xml,
        joint_names,
        control_dt_s=0.02,
        physics_timestep_s=0.005,
        initial_joint_position=None,
        gyro_sensor=None,
        quaternion_sensor=None,
        root_site=None,
    ):
        if "://" in str(model_xml):
            raise ValueError("MuJoCo model requires an explicit filesystem path")
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_xml).resolve()))
        self.data = mujoco.MjData(self.model)
        if not joint_names or len(set(joint_names)) != len(joint_names):
            raise ValueError("joint names must be nonempty and unique")
        if (
            not np.isfinite(control_dt_s)
            or not np.isfinite(physics_timestep_s)
            or min(control_dt_s, physics_timestep_s) <= 0
        ):
            raise ValueError("timesteps must be finite and positive")
        self.model.opt.timestep = physics_timestep_s
        ratio = control_dt_s / physics_timestep_s
        self.physics_steps = round(ratio)
        if self.physics_steps < 1 or abs(ratio - self.physics_steps) > 1e-6:
            raise ValueError(
                "control period must be an integer multiple of physics timestep"
            )
        self.qpos = np.array([self._joint_address(n, False) for n in joint_names])
        self.qvel = np.array([self._joint_address(n, True) for n in joint_names])
        self.actuators = np.array([self._actuator_id(n) for n in joint_names])
        self.gyro = None if gyro_sensor is None else self._sensor_slice(gyro_sensor, 3)
        self.quaternion = (
            None
            if quaternion_sensor is None
            else self._sensor_slice(quaternion_sensor, 4)
        )
        self.root_site = None if root_site is None else self._site_id(root_site)
        self.torso_gyro = self.torso_quaternion = self.torso_acceleration = None
        self._sequence = 1
        if initial_joint_position is not None:
            q = np.asarray(initial_joint_position, dtype=float)
            if q.shape != (len(joint_names),) or not np.all(np.isfinite(q)):
                raise ValueError("invalid initial joints")
            self.data.qpos[self.qpos] = q
        self.mujoco.mj_forward(self.model, self.data)

    def before_substep(self):
        pass

    def after_control_step(self):
        pass

    def step(self, command):
        for _ in range(self.physics_steps):
            self.before_substep()
            torque = (
                command.kp * (command.q_des - self.data.qpos[self.qpos])
                + command.kd * (command.dq_des - self.data.qvel[self.qvel])
                + command.tau_ff
            )
            ranges = self.model.actuator_ctrlrange[self.actuators]
            limited = self.model.actuator_ctrllimited[self.actuators].astype(bool)
            torque[limited] = np.clip(
                torque[limited], ranges[limited, 0], ranges[limited, 1]
            )
            self.data.ctrl[self.actuators] = torque
            self.mujoco.mj_step(self.model, self.data)
        self.after_control_step()

    def _id(self, kind, name: str) -> int:
        value = self.mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise RuntimeError(f"MuJoCo model is missing required name: {name}")
        return int(value)

    def _joint_address(self, name: str, velocity: bool) -> int:
        joint = self._id(self.mujoco.mjtObj.mjOBJ_JOINT, name)
        return int(
            (self.model.jnt_dofadr if velocity else self.model.jnt_qposadr)[joint]
        )

    def _actuator_id(self, joint_name: str) -> int:
        return self._id(self.mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint_name}_motor")

    def _site_id(self, name: str) -> int:
        return self._id(self.mujoco.mjtObj.mjOBJ_SITE, name)

    def _sensor_slice(self, name: str, dimension: int) -> slice:
        sensor = self._id(self.mujoco.mjtObj.mjOBJ_SENSOR, name)
        if int(self.model.sensor_dim[sensor]) != dimension:
            raise RuntimeError(f"MuJoCo sensor {name} must be {dimension}-D")
        start = int(self.model.sensor_adr[sensor])
        return slice(start, start + dimension)

    def _optional_sensor_slice(self, name: str, dimension: int) -> slice | None:
        sensor = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_SENSOR, name
        )
        if sensor < 0:
            return None
        if int(self.model.sensor_dim[sensor]) != dimension:
            raise RuntimeError(f"MuJoCo sensor {name} must be {dimension}-D")
        start = int(self.model.sensor_adr[sensor])
        return slice(start, start + dimension)

    def _optional_joint(self, name: str) -> tuple[int, int] | None:
        joint = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint < 0:
            return None
        return int(self.model.jnt_qposadr[joint]), int(self.model.jnt_dofadr[joint])

    def start(self) -> None:
        """MuJoCo is initialized eagerly; kept for the common RobotIO API."""

    def close(self) -> None:
        """MuJoCo Python objects require no explicit shutdown."""

    def read_state(self, timeout_s: float | None = None) -> RobotState:
        del timeout_s
        gyro, quaternion, q, dq = self.policy_state()
        root_position, _, root_velocity = self.policy_root_values()
        has_torso_imu = (
            self.torso_gyro is not None and self.torso_quaternion is not None
        )
        torso_gyro = (
            self.data.sensordata[self.torso_gyro].copy()
            if self.torso_gyro is not None
            else gyro
        )
        torso_quaternion = (
            self.data.sensordata[self.torso_quaternion].copy()
            if self.torso_quaternion is not None
            else quaternion
        )
        torso_acceleration = (
            self.data.sensordata[self.torso_acceleration].copy()
            if self.torso_acceleration is not None
            else np.asarray((0.0, 0.0, 9.81), dtype=np.float64)
        )
        return RobotState(
            sequence=self._sequence,
            timestamp_ns=round(float(self.data.time) * 1e9),
            gyro_b=gyro,
            quaternion_wxyz=quaternion,
            joint_pos=q,
            joint_vel=dq,
            joint_tau_est=np.zeros(len(self.qpos)),
            root_position_w=root_position,
            root_linear_velocity_w=root_velocity,
            has_torso_imu=has_torso_imu,
            torso_gyro_b=torso_gyro,
            torso_quaternion_wxyz=torso_quaternion,
            torso_acceleration_b=torso_acceleration,
        )

    def write_command(self, command: JointCommand, state_sequence: int) -> None:
        if state_sequence != self._sequence:
            raise RuntimeError(
                f"state sequence mismatch: expected {self._sequence}, got {state_sequence}"
            )
        values = (command.q_des, command.dq_des, command.kp, command.kd, command.tau_ff)
        if any(
            np.asarray(v).shape != (len(self.qpos),) or not np.all(np.isfinite(v))
            for v in values
        ):
            raise ValueError("invalid command layout or values")
        if np.any(command.kp < 0) or np.any(command.kd < 0):
            raise ValueError("negative control gain")
        self.step(command)
        self._sequence += 1

    def policy_state(self):
        return (
            (
                np.zeros(3)
                if self.gyro is None
                else self.data.sensordata[self.gyro].copy()
            ),
            (
                np.array([1.0, 0.0, 0.0, 0.0])
                if self.quaternion is None
                else self.data.sensordata[self.quaternion].copy()
            ),
            self.data.qpos[self.qpos].copy(),
            self.data.qvel[self.qvel].copy(),
        )

    def policy_root_values(self):
        if self.root_site is None:
            return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3)
        velocity = np.zeros(6)
        self.mujoco.mj_objectVelocity(
            self.model,
            self.data,
            self.mujoco.mjtObj.mjOBJ_SITE,
            self.root_site,
            velocity,
            0,
        )
        quaternion = self.data.sensordata[self.quaternion].copy()
        norm = np.linalg.norm(quaternion)
        if norm < 1e-6:
            raise RuntimeError("policy root quaternion is invalid")
        return (
            self.data.site_xpos[self.root_site].copy(),
            quaternion / norm,
            velocity[3:].copy(),
        )
