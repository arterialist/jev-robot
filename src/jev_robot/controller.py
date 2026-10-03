from __future__ import annotations

import mujoco
import numpy as np

from jev_robot.models import Motion

NEUTRAL_POSE = {
    "left_hip_pitch_joint": -0.18,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.38,
    "left_ankle_pitch_joint": -0.20,
    "left_ankle_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.18,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.38,
    "right_ankle_pitch_joint": -0.20,
    "right_ankle_roll_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "waist_roll_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "left_shoulder_pitch_joint": 0.15,
    "left_shoulder_roll_joint": 0.18,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.35,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "right_shoulder_pitch_joint": 0.15,
    "right_shoulder_roll_joint": -0.18,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.35,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}


class MotionController:
    def __init__(
        self, model: mujoco.MjModel, gains: dict[str, tuple[float, float]]
    ) -> None:
        self.model = model
        self._gains = gains
        self.action: Motion = "stand"
        self.action_started = 0.0
        self._actuators: list[tuple[int, int, int, str]] = []
        self._joint_limits: dict[str, tuple[float, float]] = {}
        self._commanded_targets = dict(NEUTRAL_POSE)
        for actuator_id in range(model.nu):
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_name is None:
                continue
            qpos_adr = int(model.jnt_qposadr[joint_id])
            qvel_adr = int(model.jnt_dofadr[joint_id])
            self._actuators.append((actuator_id, qpos_adr, qvel_adr, joint_name))
            self._joint_limits[joint_name] = tuple(
                float(bound) for bound in model.jnt_range[joint_id]
            )

    def reset_targets(self, pose: dict[str, float]) -> None:
        self._commanded_targets = dict(pose)

    @property
    def joint_limits(self) -> dict[str, tuple[float, float]]:
        return dict(self._joint_limits)

    def set_action(self, action: Motion, sim_time: float) -> None:
        if action != self.action:
            self.action = action
            self.action_started = sim_time

    def apply(
        self,
        data: mujoco.MjData,
        joint_targets: dict[str, float],
    ) -> None:
        for actuator_id, qpos_adr, qvel_adr, joint_name in self._actuators:
            low_joint, high_joint = self._joint_limits[joint_name]
            target = float(
                np.clip(
                    joint_targets[joint_name],
                    low_joint + 0.02,
                    high_joint - 0.02,
                )
            )
            previous = self._commanded_targets[joint_name]
            max_step = 0.04
            target = previous + float(np.clip(target - previous, -max_step, max_step))
            self._commanded_targets[joint_name] = target
            kp, kd = self._gains[joint_name]
            torque = kp * (target - data.qpos[qpos_adr]) - kd * data.qvel[qvel_adr]
            low, high = self.model.actuator_ctrlrange[actuator_id]
            data.ctrl[actuator_id] = float(np.clip(torque, low, high))
