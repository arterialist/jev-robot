from __future__ import annotations

from dataclasses import dataclass

START = (0.0, -0.45)
DEFAULT_GOAL = (2.5, -0.45)
BOUNDS = (-0.8, 3.8, -1.7, 1.7)


@dataclass(frozen=True, slots=True)
class Obstacle:
    name: str
    x: float
    y: float
    half_x: float
    half_y: float
    height: float

    def as_dict(self) -> dict[str, float | str]:
        return {
            "name": self.name,
            "x": self.x,
            "y": self.y,
            "half_x": self.half_x,
            "half_y": self.half_y,
        }


OBSTACLES = (
    Obstacle("north gate", 0.85, 0.50, 0.18, 0.29, 0.28),
    Obstacle("inner block", 1.58, 0.62, 0.21, 0.23, 0.36),
    Obstacle("south block", 1.35, -1.30, 0.22, 0.20, 0.25),
    Obstacle("east gate", 2.58, 0.70, 0.23, 0.30, 0.30),
    Obstacle("south east", 2.95, -1.28, 0.20, 0.21, 0.27),
    Obstacle("north west", -0.20, 1.10, 0.17, 0.24, 0.23),
)


def goal_is_safe(x: float, y: float) -> bool:
    low_x, high_x, low_y, high_y = BOUNDS
    if not (low_x <= x <= high_x and low_y <= y <= high_y):
        return False
    return all(
        abs(x - obstacle.x) > obstacle.half_x + 0.30
        or abs(y - obstacle.y) > obstacle.half_y + 0.30
        for obstacle in OBSTACLES
    )
