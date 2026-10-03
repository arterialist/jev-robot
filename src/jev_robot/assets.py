from __future__ import annotations

import os
import subprocess
from pathlib import Path

from platformdirs import user_cache_path
from rich.console import Console

from jev_robot.arena import BOUNDS, DEFAULT_GOAL, OBSTACLES

UNITREE_REPO = "https://github.com/unitreerobotics/unitree_mujoco.git"
POLICY_REPO = "https://github.com/unitreerobotics/unitree_rl_lab.git"
POLICY_COMMIT = "4960b84732b0c2ec593dccbfe963fda1bcd7b1e3"
POLICY_DIR = "deploy/robots/g1_29dof/config/policy/velocity/v0"
SCENE_NAME = "scene_jev_robot.xml"
ROBOT_SCENE_NAME = "g1_29dof_jev.xml"


def ensure_g1_policy(console: Console) -> tuple[Path, Path]:
    root = user_cache_path("jev-robot") / "unitree_rl_lab"
    model_path = root / POLICY_DIR / "exported" / "policy.onnx"
    config_path = root / POLICY_DIR / "params" / "deploy.yaml"
    if model_path.exists() and config_path.exists():
        return model_path, config_path
    root.parent.mkdir(parents=True, exist_ok=True)
    if not (root / ".git").exists():
        console.print(f"[cyan]Fetching Unitree's G1 gait policy into {root}[/cyan]")
        subprocess.run(
            [
                "git",
                "clone",
                "--depth",
                "1",
                "--filter=blob:none",
                "--sparse",
                POLICY_REPO,
                str(root),
            ],
            check=True,
        )
    subprocess.run(
        ["git", "-C", str(root), "fetch", "--depth", "1", "origin", POLICY_COMMIT],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(root), "checkout", "--detach", POLICY_COMMIT],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(root), "sparse-checkout", "add", POLICY_DIR],
        check=True,
    )
    if not model_path.exists() or not config_path.exists():
        raise FileNotFoundError("Unitree G1 walking policy files are missing")
    return model_path, config_path


def ensure_g1_assets(console: Console) -> Path:
    configured = os.getenv("UNITREE_MUJOCO_ROOT")
    root = (
        Path(configured).expanduser()
        if configured
        else user_cache_path("jev-robot") / "unitree_mujoco"
    )
    g1_dir = root / "unitree_robots" / "g1"
    source_scene = g1_dir / "g1_29dof.xml"

    if not source_scene.exists():
        console.print(f"[cyan]Fetching official Unitree G1 assets into {root}[/cyan]")
        root.parent.mkdir(parents=True, exist_ok=True)
        if (root / ".git").exists():
            subprocess.run(
                ["git", "-C", str(root), "sparse-checkout", "set", "unitree_robots/g1"],
                check=True,
            )
        else:
            subprocess.run(
                [
                    "git",
                    "clone",
                    "--depth",
                    "1",
                    "--filter=blob:none",
                    "--sparse",
                    UNITREE_REPO,
                    str(root),
                ],
                check=True,
            )
            subprocess.run(
                ["git", "-C", str(root), "sparse-checkout", "set", "unitree_robots/g1"],
                check=True,
            )

    if not source_scene.exists():
        raise FileNotFoundError(f"Unitree G1 MJCF not found at {source_scene}")

    robot_xml = source_scene.read_text(encoding="utf-8")
    torso_tag = '<body name="torso_link" pos="0 0 0.019">'
    if robot_xml.count(torso_tag) != 1:
        raise ValueError("Cannot locate the G1 torso for its first-person camera")
    robot_xml = robot_xml.replace(
        torso_tag,
        torso_tag
        + '\n            <camera name="robot_fpv" pos="0.22 0 0.26" '
        'xyaxes="0 -1 0 0 0 1" fovy="78"/>',
        1,
    )
    (g1_dir / ROBOT_SCENE_NAME).write_text(robot_xml, encoding="utf-8")
    scene = g1_dir / SCENE_NAME
    scene.write_text(_scene_xml(), encoding="utf-8")
    return scene


def _scene_xml() -> str:
    obstacle_geoms = "\n    ".join(
        f'<geom name="{obstacle.name.replace(" ", "_")}" type="box" '
        f'pos="{obstacle.x} {obstacle.y} {obstacle.height / 2}" '
        f'size="{obstacle.half_x} {obstacle.half_y} {obstacle.height / 2}" '
        'material="obstacle_mat"/>'
        for obstacle in OBSTACLES
    )
    low_x, high_x, low_y, high_y = BOUNDS
    center_x = (low_x + high_x) / 2
    center_y = (low_y + high_y) / 2
    half_x = (high_x - low_x) / 2 + 0.25
    half_y = (high_y - low_y) / 2 + 0.25
    goal_x, goal_y = DEFAULT_GOAL
    return f"""<mujoco model="JEV robot arena">
  <include file="{ROBOT_SCENE_NAME}"/>
  <option timestep="0.002" integrator="implicitfast" iterations="20"/>
  <statistic center="{center_x} {center_y} 0.55" extent="4.0"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>
    <rgba haze="0.12 0.16 0.22 1"/>
    <global azimuth="-120" elevation="-18" offwidth="1280" offheight="720"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.12 0.18 0.28" rgb2="0.01 0.01 0.02" width="512" height="3072"/>
    <texture type="2d" name="floor_tex" builtin="checker" rgb1="0.16 0.18 0.22" rgb2="0.09 0.10 0.13" width="256" height="256"/>
    <material name="floor_mat" texture="floor_tex" texuniform="true" texrepeat="10 10" reflectance="0.12"/>
    <material name="target_mat" rgba="0.1 0.9 0.35 1" emission="0.25"/>
    <material name="obstacle_mat" rgba="0.9 0.15 0.12 1"/>
    <material name="wall_mat" rgba="0.18 0.43 0.52 1"/>
  </asset>
  <worldbody>
    <light pos="1 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="floor_mat" friction="1.2 0.01 0.001"/>
    <geom name="wall_west" type="box" pos="{low_x - 0.20} {center_y} 0.18" size="0.15 {half_y} 0.18" material="wall_mat"/>
    <geom name="wall_east" type="box" pos="{high_x + 0.20} {center_y} 0.18" size="0.15 {half_y} 0.18" material="wall_mat"/>
    <geom name="wall_north" type="box" pos="{center_x} {high_y + 0.20} 0.18" size="{half_x} 0.15 0.18" material="wall_mat"/>
    <geom name="wall_south" type="box" pos="{center_x} {low_y - 0.20} 0.18" size="{half_x} 0.15 0.18" material="wall_mat"/>
    {obstacle_geoms}
    <body name="goal_marker" mocap="true" pos="{goal_x} {goal_y} 0">
      <geom name="target" type="cylinder" pos="0 0 0.035" size="0.25 0.035" material="target_mat" contype="0" conaffinity="0"/>
      <site name="target_beacon" type="sphere" pos="0 0 0.45" size="0.055" rgba="0.2 1 0.45 0.65"/>
    </body>
  </worldbody>
</mujoco>
"""
