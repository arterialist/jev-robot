from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
from mujoco import viewer as mj_viewer
from rich.console import Console

from jev_robot.assets import ensure_g1_assets
from jev_robot.brains import CognitiveArchitecture
from jev_robot.simulation import G1Simulation, select_motor_action

DEFAULT_TASK = (
    "Reach the selected green goal without hitting red obstacles or arena walls. "
    "Hold position when I arrive."
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="jev-robot",
        description="Run the Unitree G1 with JEV reflexes and Gemini planning.",
    )
    parser.add_argument(
        "--task", default=DEFAULT_TASK, help="Natural-language task for System Two"
    )
    parser.add_argument(
        "--headless", action="store_true", help="Run without the MuJoCo viewer"
    )
    parser.add_argument(
        "--offline",
        action="store_true",
        help="Use local fallback decisions without API calls",
    )
    parser.add_argument(
        "--seconds", type=float, default=120.0, help="Simulation duration"
    )
    parser.add_argument(
        "--no-realtime",
        action="store_true",
        help="Run physics as fast as possible, mainly for headless smoke runs",
    )
    parser.add_argument(
        "--capture", type=Path, help="Write a final camera frame to a PNG file"
    )
    return parser


def run(args: argparse.Namespace) -> int:
    console = Console()
    scene = ensure_g1_assets(console)
    simulation = G1Simulation(scene, args.task)
    brain = CognitiveArchitecture(
        args.task,
        simulation.controller.joint_limits,
        simulation.gait.default_targets,
        offline=args.offline,
        on_event=console.log,
    )
    brain.start()

    if not args.offline:
        if not brain.has_jev:
            console.print(
                "[yellow]OPENROUTER_API_KEY is missing. System 1 is using the local fallback.[/yellow]"
            )
        if not brain.has_gemini:
            console.print(
                "[yellow]GEMINI_API_KEY is missing. System 2 is using the local fallback.[/yellow]"
            )

    console.print(f"[bold]Task:[/bold] {args.task}")
    console.print(
        f"[dim]MuJoCo {mujoco.__version__} | {simulation.model.nq} qpos | "
        f"{simulation.model.nu} actuators | "
        f"{1 / simulation.model.opt.timestep:.0f} Hz timestep target[/dim]"
    )

    viewer = None
    if not args.headless:
        try:
            viewer = mj_viewer.launch_passive(
                simulation.model,
                simulation.data,
                show_left_ui=False,
                show_right_ui=True,
            )
            viewer.cam.lookat[:] = [1.1, 0.0, 0.65]
            viewer.cam.distance = 3.5
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -18
        except Exception as exc:
            brain.close()
            if sys.platform == "darwin":
                console.print(
                    "[red]macOS viewer startup failed. Run with `uv run mjpython -m jev_robot`.[/red]"
                )
            raise RuntimeError("Could not launch MuJoCo viewer") from exc

    wall_started = time.monotonic()
    next_publish = 0.0
    next_status = 0.0
    fall_reported = False
    last_action = "stand"

    try:
        while simulation.data.time < args.seconds and (
            viewer is None or viewer.is_running()
        ):
            step_started = time.monotonic()
            decision = brain.motor_snapshot()
            _, plan = brain.snapshot()
            state = simulation.observe(decision.action)
            thought, _ = brain.thought_snapshot()

            joint_offsets = (
                decision.joint_offsets
                if decision.source == "jev"
                and len(decision.joint_offsets) == simulation.model.nu
                else None
            )
            last_action = (
                "direct"
                if joint_offsets is not None
                else "recover"
                if state.fallen
                else select_motor_action(state, decision.action)
            )
            if state.fallen and not fall_reported:
                console.log("Robot fell. JEV retains motor control; no automatic pose reset")
            fall_reported = state.fallen
            velocity_command = decision.velocity_command
            if state.route_phase == "arrived" and not thought:
                velocity_command = (0.0, 0.0, 0.0)
            if simulation.step(
                last_action, joint_offsets,
                velocity_command if joint_offsets is not None else None,
            ):
                console.log("MuJoCo instability guard restored the upright pose")

            if simulation.data.time >= next_publish:
                brain.publish(simulation.observe(last_action))
                next_publish = simulation.data.time + 0.02

            if simulation.data.time >= next_status:
                current = simulation.observe(last_action)
                reflex_rate, _, queue_depth, superseded = brain.metrics()
                console.print(
                    f"[dim]t={current.sim_time:5.1f}s action={last_action:<12} "
                    f"target={current.target_distance:4.2f}m obstacle={current.nearest_obstacle_distance:4.2f}m "
                    f"reflex_lag={max(0.0, current.sim_time - decision.observation_sim_time):4.2f}s "
                    f"jev={reflex_rate:3.1f}Hz queue={queue_depth} superseded={superseded} "
                    f"plan={plan.goal}[/dim]"
                )
                next_status = simulation.data.time + 2.0

            if viewer is not None:
                viewer.sync(state_only=True)

            if not args.no_realtime:
                remaining = simulation.model.opt.timestep - (
                    time.monotonic() - step_started
                )
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        console.print("\n[dim]Stopped.[/dim]")
    finally:
        brain.close()
        if viewer is not None:
            viewer.close()

    if args.capture:
        _capture(simulation, args.capture)
        console.print(f"Saved frame to {args.capture.resolve()}")

    final = simulation.observe(last_action)
    console.print(
        f"[bold green]Finished[/bold green] {final.sim_time:.1f}s simulated in "
        f"{time.monotonic() - wall_started:.1f}s wall time. Target distance: {final.target_distance:.2f}m."
    )
    return 0


def _capture(simulation: G1Simulation, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(simulation.model, height=720, width=1280)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [1.1, 0.0, 0.65]
    camera.distance = 3.5
    camera.azimuth = 135
    camera.elevation = -18
    renderer.update_scene(simulation.data, camera=camera)
    image = renderer.render()
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("PNG capture requires Pillow") from exc
    Image.fromarray(image).save(destination)
    renderer.close()


def main() -> None:
    raise SystemExit(run(build_parser().parse_args()))


if __name__ == "__main__":
    main()
