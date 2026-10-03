from __future__ import annotations

from collections import deque
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

from jev_robot.models import Motion


def _rotate_into_body(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    axis = quaternion[1:]
    cross = np.cross(axis, vector)
    return vector - 2.0 * quaternion[0] * cross + 2.0 * np.cross(axis, cross)


def velocity_for(action: Motion) -> np.ndarray:
    commands = {
        "walk_forward": (0.30, 0.0, 0.0),
        "walk_backward": (-0.20, 0.0, 0.0),
        "walk_fast": (0.55, 0.0, 0.0),
        "diagonal_left": (0.25, 0.17, 0.0),
        "diagonal_right": (0.25, -0.17, 0.0),
        "step_left": (0.0, 0.30, 0.0),
        "step_right": (0.0, -0.30, 0.0),
        "turn_left": (0.0, 0.0, 0.55),
        "turn_right": (0.0, 0.0, -0.55),
    }
    return np.asarray(commands.get(action, (0.0, 0.0, 0.0)), dtype=np.float32)


class G1GaitPolicy:
    """Unitree's 50 Hz G1 gait policy, with Jev supplying its velocity command."""

    def __init__(self, model: mujoco.MjModel, policy: Path, config: Path) -> None:
        with config.open(encoding="utf-8") as handle:
            settings = yaml.safe_load(handle)
        self._model = model
        self._session = ort.InferenceSession(
            str(policy),
            sess_options=self._session_options(),
            providers=["CPUExecutionProvider"],
        )
        self._input_name = self._session.get_inputs()[0].name
        self._policy_to_actuator = [int(index) for index in settings["joint_ids_map"]]
        if sorted(self._policy_to_actuator) != list(range(model.nu)):
            raise ValueError("G1 gait policy joint map does not match the MuJoCo model")
        self._defaults = np.asarray(settings["default_joint_pos"], dtype=np.float32)
        self._scale = np.asarray(
            settings["actions"]["JointPositionAction"]["scale"], dtype=np.float32
        )
        self._stiffness = np.asarray(settings["stiffness"], dtype=np.float64)
        self._damping = np.asarray(settings["damping"], dtype=np.float64)
        self.step_dt = float(settings["step_dt"])
        self._joint_specs: list[tuple[str, int, int]] = []
        for actuator_id in self._policy_to_actuator:
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if name is None:
                raise ValueError(f"Unnamed G1 joint at actuator {actuator_id}")
            self._joint_specs.append(
                (
                    name,
                    int(model.jnt_qposadr[joint_id]),
                    int(model.jnt_dofadr[joint_id]),
                )
            )
        self.default_targets = {
            name: float(value)
            for (name, _, _), value in zip(self._joint_specs, self._defaults)
        }
        self.gains = {
            name: (
                float(self._stiffness[actuator_id]),
                float(self._damping[actuator_id]),
            )
            for (name, _, _), actuator_id in zip(
                self._joint_specs, self._policy_to_actuator
            )
        }
        self.reset()

    @staticmethod
    def _session_options() -> ort.SessionOptions:
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        return options

    def reset(self) -> None:
        self._history: deque[np.ndarray] = deque(maxlen=5)
        self._last_action = np.zeros(len(self._joint_specs), dtype=np.float32)
        self._targets = dict(self.default_targets)
        self._next_update = 0.0

    def targets(self, data: mujoco.MjData, command: np.ndarray) -> dict[str, float]:
        if data.time + 1e-9 < self._next_update:
            return self._targets
        self._next_update = float(data.time) + self.step_dt

        quaternion = np.asarray(data.sensor("imu_quat").data, dtype=np.float64)
        angular_velocity = np.asarray(data.sensor("imu_gyro").data) * 0.2
        gravity = _rotate_into_body(quaternion, np.array([0.0, 0.0, -1.0]))
        positions = np.asarray(
            [data.qpos[qpos] for _, qpos, _ in self._joint_specs],
            dtype=np.float32,
        )
        velocities = np.asarray(
            [data.qvel[qvel] for _, _, qvel in self._joint_specs],
            dtype=np.float32,
        )
        observation = np.concatenate(
            (
                angular_velocity.astype(np.float32),
                gravity.astype(np.float32),
                command,
                positions - self._defaults,
                velocities * 0.05,
                self._last_action,
            )
        )
        if not self._history:
            self._history.extend(observation.copy() for _ in range(5))
        else:
            self._history.append(observation)
        history = np.stack(tuple(self._history))
        # Unitree's deploy manager concatenates five samples of each observation
        # term, not five complete observations in time order.
        policy_input = np.concatenate(
            [
                history[:, start:end].reshape(-1)
                for start, end in ((0, 3), (3, 6), (6, 9), (9, 38), (38, 67), (67, 96))
            ]
        )[None, :]
        raw = self._session.run(None, {self._input_name: policy_input})[0][0]
        self._last_action = np.clip(raw, -5.0, 5.0).astype(np.float32)
        commanded = self._defaults + self._scale * self._last_action
        self._targets = {
            name: float(value)
            for (name, _, _), value in zip(self._joint_specs, commanded)
        }
        return self._targets
