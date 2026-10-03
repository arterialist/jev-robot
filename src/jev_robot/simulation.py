from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
from rich.console import Console

from jev_robot.arena import BOUNDS, DEFAULT_GOAL, OBSTACLES, START, goal_is_safe
from jev_robot.assets import ensure_g1_policy
from jev_robot.controller import MotionController
from jev_robot.locomotion import G1GaitPolicy, velocity_for
from jev_robot.models import Motion, ObstacleState, WorldState
from jev_robot.primitives import pose_offsets


def quaternion_to_euler(quaternion: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def select_motor_action(
    state: WorldState, requested: Motion, self_thought: str = ""
) -> Motion:
    """Hold at the destination unless the robot has an explicit new intent."""
    if state.route_phase == "arrived" and not self_thought:
        return "stand"
    yaw = state.orientation_rpy[2]
    velocity = velocity_for(requested)
    forward, lateral = float(velocity[0]), float(velocity[1])
    speed = math.hypot(forward, lateral)
    if speed > 0:
        next_x = state.position[0] + 0.35 * (
            forward * math.cos(yaw) - lateral * math.sin(yaw)
        ) / speed
        next_y = state.position[1] + 0.35 * (
            forward * math.sin(yaw) + lateral * math.cos(yaw)
        ) / speed
        low_x, high_x, low_y, high_y = BOUNDS
        if not (
            low_x + 0.08 <= next_x <= high_x - 0.08
            and low_y + 0.08 <= next_y <= high_y - 0.08
        ):
            return "stand"
        if any(
            abs(next_x - obstacle.x) < obstacle.half_x + 0.18
            and abs(next_y - obstacle.y) < obstacle.half_y + 0.18
            for obstacle in OBSTACLES
        ):
            return "stand"
    if (
        not self_thought
        and state.nearest_obstacle_distance > 0.55
        and abs(state.target_bearing) < 0.22
        and requested in {"turn_left", "turn_right"}
    ):
        return "walk_forward"
    return requested


class G1Simulation:
    def __init__(self, scene: Path, task: str) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.task = task
        policy, config = ensure_g1_policy(Console())
        self.gait = G1GaitPolicy(self.model, policy, config)
        self.controller = MotionController(self.model, self.gait.gains)
        self.target = np.asarray(DEFAULT_GOAL, dtype=np.float64)
        self.goal_revision = 1
        goal_body = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "goal_marker"
        )
        if goal_body < 0:
            raise ValueError("The MuJoCo arena has no goal marker")
        self._goal_mocap_id = int(self.model.body_mocapid[goal_body])
        if self._goal_mocap_id < 0:
            raise ValueError("The goal marker must be a MuJoCo mocap body")
        self.reset()

    def reset(self) -> None:
        self._arrived = False
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = [*START, 0.78, 1.0, 0.0, 0.0, 0.0]
        self.data.mocap_pos[self._goal_mocap_id] = [*self.target, 0.0]
        for joint_name, value in self.gait.default_targets.items():
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
            )
            if joint_id >= 0:
                self.data.qpos[self.model.jnt_qposadr[joint_id]] = value
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.gait.reset()
        self.controller.reset_targets(self.gait.default_targets)
        self.controller.set_action("stand", 0.0)
        self._last_velocity_command = (0.0, 0.0, 0.0)
        self._applied_velocity = np.zeros(3, dtype=np.float32)
        mujoco.mj_forward(self.model, self.data)

    def set_goal(self, x: float, y: float) -> None:
        if not goal_is_safe(x, y):
            raise ValueError("Goal is outside the arena or too close to an obstacle")
        self.target[:] = [x, y]
        self.goal_revision += 1
        self._arrived = False
        self.data.mocap_pos[self._goal_mocap_id] = [x, y, 0.0]
        mujoco.mj_forward(self.model, self.data)

    def recover_upright(self) -> None:
        sim_time = float(self.data.time)
        position = self.data.qpos[:2].copy()
        self.reset()
        self.data.time = sim_time
        self.data.qpos[:2] = position
        mujoco.mj_forward(self.model, self.data)

    def observe(self, action: Motion) -> WorldState:
        position = self.data.qpos[:3].copy()
        roll, pitch, yaw = quaternion_to_euler(self.data.qpos[3:7])
        delta_target = self.target - position[:2]
        target_distance = float(np.linalg.norm(delta_target))
        if target_distance < 0.48:
            self._arrived = True
        target_angle = math.atan2(delta_target[1], delta_target[0])
        target_bearing = wrap_angle(target_angle - yaw)

        obstacles = []
        for item in OBSTACLES:
            dx, dy = item.x - position[0], item.y - position[1]
            clearance = math.hypot(
                max(abs(dx) - item.half_x, 0.0),
                max(abs(dy) - item.half_y, 0.0),
            )
            obstacles.append(
                ObstacleState(
                    name=item.name,
                    position=(item.x, item.y),
                    half_size=(item.half_x, item.half_y),
                    clearance=clearance,
                    bearing=wrap_angle(math.atan2(dy, dx) - yaw),
                )
            )
        nearest = min(obstacles, key=lambda item: item.clearance)
        nearest_distance = nearest.clearance
        obstacle_bearing = nearest.bearing
        low_x, high_x, low_y, high_y = BOUNDS
        wall_clearance = min(
            position[0] - low_x,
            high_x - position[0],
            position[1] - low_y,
            high_y - position[1],
        )
        if nearest_distance > 1.1 or abs(obstacle_bearing) > 1.0:
            obstacle_side = "none"
        elif abs(obstacle_bearing) < 0.28:
            obstacle_side = "ahead"
        elif obstacle_bearing > 0:
            obstacle_side = "left"
        else:
            obstacle_side = "right"

        route_phase = "arrived" if self._arrived else "approach_goal"

        return WorldState(
            sim_time=float(self.data.time),
            position=tuple(float(v) for v in position),
            orientation_rpy=(roll, pitch, yaw),
            linear_velocity=tuple(float(v) for v in self.data.qvel[:3]),
            angular_velocity=tuple(float(v) for v in self.data.qvel[3:6]),
            target_position=tuple(float(v) for v in self.target),
            target_distance=target_distance,
            target_bearing=target_bearing,
            nearest_obstacle_distance=nearest_distance,
            wall_clearance=float(wall_clearance),
            obstacle_side=obstacle_side,
            obstacles=obstacles,
            route_phase=route_phase,
            fallen=bool(position[2] < 0.65 or abs(roll) > 0.8 or abs(pitch) > 0.8),
            current_action=action,
            gait_phase=float(
                2 * math.pi * 1.45 * (self.data.time - self.controller.action_started)
            ),
            joint_positions={
                joint_name: float(self.data.qpos[qpos_adr])
                for _, qpos_adr, _, joint_name in self.controller._actuators
            },
            joint_velocities={
                joint_name: float(self.data.qvel[qvel_adr])
                for _, _, qvel_adr, joint_name in self.controller._actuators
            },
            commanded_velocity=self._last_velocity_command,
            commanded_joint_targets=dict(self.controller._commanded_targets),
            task=self.task,
        )

    def step(
        self,
        action: Motion,
        joint_offsets: dict[str, float] | None = None,
        velocity_command: tuple[float, float, float] | None = None,
    ) -> bool:
        previous_time = float(self.data.time)
        previous_position = self.data.qpos[:2].copy()
        self.controller.set_action(action, float(self.data.time))
        if joint_offsets is not None and velocity_command is not None:
            requested = self._safe_velocity(velocity_command)
            max_change = np.asarray([0.9, 0.9, 2.0], dtype=np.float32)
            max_change *= self.model.opt.timestep
            self._applied_velocity += np.clip(
                requested - self._applied_velocity, -max_change, max_change
            )
            command = self._applied_velocity
            gait_targets = self.gait.targets(self.data, command)
            targets = {
                name: gait_targets[name] + joint_offsets[name]
                for name in self.gait.default_targets
            }
        else:
            command = velocity_for(action)
            self._applied_velocity = command.copy()
            targets = dict(self.gait.targets(self.data, command))
            elapsed = float(self.data.time - self.controller.action_started)
            for name, offset in pose_offsets(action, elapsed).items():
                targets[name] += offset
        self._last_velocity_command = tuple(float(value) for value in command)
        self.controller.apply(self.data, targets)
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_step(self.model, self.data)
        if self.data.time <= previous_time or not np.all(np.isfinite(self.data.qpos)):
            self.reset()
            self.data.time = previous_time + self.model.opt.timestep
            self.data.qpos[:2] = previous_position
            mujoco.mj_forward(self.model, self.data)
            return True
        return False

    def _safe_velocity(self, requested: tuple[float, float, float]) -> np.ndarray:
        command = np.asarray(requested, dtype=np.float32)
        if command.shape != (3,) or not np.all(np.isfinite(command)):
            return np.zeros(3, dtype=np.float32)
        command = np.clip(command, [-0.22, -0.25, -0.70], [0.45, 0.25, 0.70])
        forward, lateral = float(command[0]), float(command[1])
        if math.hypot(forward, lateral) < 1e-4:
            return command
        yaw = quaternion_to_euler(self.data.qpos[3:7])[2]
        next_x = float(self.data.qpos[0]) + 0.45 * (
            forward * math.cos(yaw) - lateral * math.sin(yaw)
        )
        next_y = float(self.data.qpos[1]) + 0.45 * (
            forward * math.sin(yaw) + lateral * math.cos(yaw)
        )
        low_x, high_x, low_y, high_y = BOUNDS
        unsafe = not (
            low_x + 0.14 <= next_x <= high_x - 0.14
            and low_y + 0.14 <= next_y <= high_y - 0.14
        ) or any(
            abs(next_x - obstacle.x) < obstacle.half_x + 0.20
            and abs(next_y - obstacle.y) < obstacle.half_y + 0.20
            for obstacle in OBSTACLES
        )
        if unsafe:
            command[:2] = 0.0
        return command
