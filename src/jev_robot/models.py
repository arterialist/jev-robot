from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, Field

Motion = Literal[
    "direct",
    "stand",
    "walk_forward",
    "walk_backward",
    "walk_fast",
    "diagonal_left",
    "diagonal_right",
    "turn_left",
    "turn_right",
    "step_left",
    "step_right",
    "crouch",
    "squat",
    "bow",
    "wave",
    "wave_left",
    "wave_right",
    "arms_up",
    "arms_out",
    "reach_forward",
    "point_left",
    "point_right",
    "nod",
    "dance",
    "march_in_place",
    "kick_left",
    "kick_right",
    "hop",
    "recover",
]
ControlMode = Literal["direct", "primitives"]


class ObstacleState(BaseModel):
    name: str
    position: tuple[float, float]
    half_size: tuple[float, float]
    clearance: float
    bearing: float


class WorldState(BaseModel):
    sim_time: float
    position: tuple[float, float, float]
    orientation_rpy: tuple[float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    target_position: tuple[float, float]
    target_distance: float
    target_bearing: float
    nearest_obstacle_distance: float
    wall_clearance: float
    obstacle_side: Literal["left", "right", "ahead", "none"]
    obstacles: list[ObstacleState] = Field(default_factory=list)
    route_phase: Literal["approach_goal", "arrived"]
    fallen: bool
    current_action: Motion
    gait_phase: float = 0.0
    joint_positions: dict[str, float] = Field(default_factory=dict)
    joint_velocities: dict[str, float] = Field(default_factory=dict)
    commanded_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    commanded_joint_targets: dict[str, float] = Field(default_factory=dict)
    task: str

    def compact(self) -> dict[str, object]:
        return self.model_dump(mode="json", exclude={"task"})


class SystemTwoPlan(BaseModel):
    goal: str = Field(description="The next concrete objective for the robot")
    strategy: str = Field(description="A short spatial and physical strategy")
    risk_tolerance: float = Field(default=0.2, ge=0.0, le=1.0)
    task_complete: bool = False
    source: Literal["gemini", "fallback"] = "fallback"
    latency_ms: float = 0.0
    vision_used: bool = False
    mind_note: str = Field(
        default="Watching the scene and keeping the robot safe.",
        description="A short visible first-person status, not hidden reasoning",
    )


class ReflexDecision(BaseModel):
    action: Motion = "stand"
    confidence: float = Field(default=0.0, ge=0.0, le=1.0)
    emergency_probability: float = Field(default=0.0, ge=0.0, le=1.0)
    source: Literal["jev", "fallback"] = "fallback"
    latency_ms: float = 0.0
    observation_sim_time: float = 0.0
    probabilities: dict[str, float] = Field(default_factory=dict)
    velocity_command: tuple[float, float, float] = (0.0, 0.0, 0.0)
    joint_offsets: dict[str, float] = Field(default_factory=dict)
    joint_confidence: float = 0.0
