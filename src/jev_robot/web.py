from __future__ import annotations

import argparse
import asyncio
import io
import queue
import threading
import time
import webbrowser
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
from pydantic import BaseModel, Field
from rich.console import Console

from jev_robot.arena import BOUNDS, OBSTACLES, goal_is_safe
from jev_robot.assets import ensure_g1_assets
from jev_robot.brains import CognitiveArchitecture
from jev_robot.models import ControlMode
from jev_robot.simulation import G1Simulation, select_motor_action

DEFAULT_TASK = (
    "Reach the selected green goal without hitting the red obstacles or walls. "
    "Hold position when I arrive and respond to my next first-person thought."
)
STATIC_DIR = Path(__file__).parent / "static"


class ThoughtRequest(BaseModel):
    message: str = Field(max_length=1_000)


class GoalRequest(BaseModel):
    x: float
    y: float


class CameraRequest(BaseModel):
    view: str


class ControlModeRequest(BaseModel):
    mode: ControlMode


class RobotRuntime:
    def __init__(self, task: str, *, offline: bool = False) -> None:
        self.console = Console()
        self._events: deque[dict[str, Any]] = deque(maxlen=80)
        self._event_id = 0
        self._lock = threading.Lock()
        scene = ensure_g1_assets(self.console)
        self.simulation = G1Simulation(scene, task)
        self.brain = CognitiveArchitecture(
            task,
            self.simulation.controller.joint_limits,
            self.simulation.gait.default_targets,
            offline=offline,
            expect_vision=True,
            on_event=self._record_event,
        )
        self.offline = offline
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._render_thread: threading.Thread | None = None
        self._render_ready = threading.Event()
        self._render_pose: tuple[float, np.ndarray, np.ndarray] | None = None
        self._camera_view = "arena"
        self._commands: queue.SimpleQueue[tuple[str, tuple[float, float] | None]] = (
            queue.SimpleQueue()
        )
        self._frame: bytes | None = None
        self._frame_sequence = 0
        self._step_times: deque[float] = deque(maxlen=2000)
        self._sensory_times: deque[float] = deque(maxlen=200)
        self._frame_times: deque[float] = deque(maxlen=200)
        self._telemetry: dict[str, Any] = {}
        self._last_action = "stand"
        self._joint_source = "local"
        self._wall_started = time.monotonic()

    def start(self) -> None:
        self.brain.start()
        self._render_thread = threading.Thread(
            target=self._render_loop,
            name="mujoco-browser-render",
            daemon=True,
        )
        self._render_thread.start()
        self._thread = threading.Thread(
            target=self._physics_loop,
            name="mujoco-browser-stream",
            daemon=True,
        )
        self._thread.start()
        self._record_event("Browser control room online.")

    def close(self) -> None:
        self._stop.set()
        self._render_ready.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._render_thread:
            self._render_thread.join(timeout=3.0)
        self.brain.close()

    def set_thought(self, message: str) -> None:
        self.brain.set_thought(message)

    def set_goal(self, x: float, y: float) -> None:
        if not goal_is_safe(x, y):
            raise ValueError("Select a point inside the arena, clear of obstacles")
        self._commands.put(("goal", (x, y)))

    def reset_robot(self) -> None:
        self._commands.put(("reset", None))

    def set_camera(self, view: str) -> None:
        if view not in {"arena", "follow", "top", "fpv"}:
            raise ValueError("Unknown camera view")
        with self._lock:
            self._camera_view = view

    def set_control_mode(self, mode: str) -> None:
        if mode not in {"direct", "primitives"}:
            raise ValueError("Unknown control mode")
        self.brain.set_control_mode(mode)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._telemetry)

    def frame(self) -> tuple[int, bytes | None]:
        with self._lock:
            return self._frame_sequence, self._frame

    def _record_event(self, message: str) -> None:
        if message.startswith(("Self-thought", "Explicit self-thought")):
            kind = "self_thought"
        elif message.startswith("System 2"):
            kind = "system_two"
        elif message.startswith("System 1"):
            kind = "system_one"
        else:
            kind = "system"
        with self._lock:
            self._event_id += 1
            self._events.append(
                {
                    "id": self._event_id,
                    "kind": kind,
                    "time": time.time(),
                    "message": message,
                }
            )
        self.console.log(message)

    def _physics_loop(self) -> None:
        next_publish = 0.0
        next_render = 0.0
        next_telemetry = 0.0
        next_step_deadline = time.monotonic() + self.simulation.model.opt.timestep
        fall_reported = False

        while not self._stop.is_set():
            while True:
                try:
                    command, payload = self._commands.get_nowait()
                except queue.Empty:
                    break
                if command == "goal" and payload is not None:
                    x, y = payload
                    self.simulation.set_goal(x, y)
                    self.brain.set_thought("")
                    self.brain.start_new_goal(self.simulation.observe("stand"))
                    self._record_event(f"New goal selected at ({x:.2f}, {y:.2f}).")
                elif command == "reset":
                    sim_time = float(self.simulation.data.time)
                    self.simulation.reset()
                    self.simulation.data.time = sim_time
                    self.brain.start_new_goal(self.simulation.observe("stand"))
                    self._record_event("Robot pose reset at the arena start.")
            decision = self.brain.motor_snapshot()
            state = self.simulation.observe(decision.action)
            thought, _ = self.brain.thought_snapshot()
            joint_offsets = (
                decision.joint_offsets
                if decision.source == "jev"
                and len(decision.joint_offsets) == self.simulation.model.nu
                and self.brain.control_mode_snapshot() == "direct"
                else None
            )
            if joint_offsets is not None:
                self._last_action = "direct"
            else:
                self._last_action = (
                    "recover"
                    if state.fallen
                    else select_motor_action(state, decision.action, thought)
                )
            if state.fallen and not fall_reported:
                self._record_event("Robot fell. JEV retains motor control; Reset pose is manual.")
            fall_reported = state.fallen
            self._joint_source = "jev" if joint_offsets is not None else "local"
            velocity_command = decision.velocity_command
            if state.route_phase == "arrived" and not thought:
                velocity_command = (0.0, 0.0, 0.0)
            if self.simulation.step(
                self._last_action, joint_offsets,
                velocity_command if joint_offsets is not None else None,
            ):
                self._record_event(
                    "MuJoCo instability guard restored the upright pose."
                )
            self._step_times.append(time.monotonic())

            if self.simulation.data.time >= next_publish:
                self.brain.publish(self.simulation.observe(self._last_action))
                self._sensory_times.append(time.monotonic())
                next_publish += 0.02

            now = time.monotonic()
            if now >= next_render:
                with self._lock:
                    self._render_pose = (
                        float(self.simulation.data.time),
                        self.simulation.data.qpos.copy(),
                        self.simulation.data.mocap_pos.copy(),
                    )
                self._render_ready.set()
                next_render = now + 1 / 15

            if now >= next_telemetry:
                self._update_telemetry()
                next_telemetry = now + 0.1

            remaining = next_step_deadline - time.monotonic()
            if remaining > 0:
                self._stop.wait(remaining)
            next_step_deadline += self.simulation.model.opt.timestep
            if next_step_deadline < time.monotonic() - 0.1:
                next_step_deadline = time.monotonic()

    def _render_loop(self) -> None:
        render_data = mujoco.MjData(self.simulation.model)
        renderer = mujoco.Renderer(self.simulation.model, height=540, width=960)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [1.5, 0.0, 0.5]
        camera.distance = 6.0
        camera.azimuth = 135
        camera.elevation = -38
        try:
            while not self._stop.is_set():
                if not self._render_ready.wait(timeout=0.2):
                    continue
                self._render_ready.clear()
                with self._lock:
                    pose = self._render_pose
                    view = self._camera_view
                if pose is None:
                    continue
                render_data.time, render_data.qpos[:], render_data.mocap_pos[:] = pose
                mujoco.mj_forward(self.simulation.model, render_data)
                if view == "follow":
                    camera.lookat[:] = render_data.qpos[:3]
                    camera.distance = 2.7
                    camera.azimuth = 135
                    camera.elevation = -22
                elif view == "top":
                    camera.lookat[:] = [1.5, 0.0, 0.0]
                    camera.distance = 6.2
                    camera.azimuth = 90
                    camera.elevation = -85
                else:
                    camera.lookat[:] = [1.5, 0.0, 0.5]
                    camera.distance = 6.0
                    camera.azimuth = 135
                    camera.elevation = -38
                renderer.update_scene(
                    render_data,
                    camera="robot_fpv" if view == "fpv" else camera,
                )
                pixels = renderer.render()
                output = io.BytesIO()
                Image.fromarray(pixels).save(output, format="JPEG", quality=82)
                frame = output.getvalue()
                with self._lock:
                    self._frame = frame
                    self._frame_sequence += 1
                    self._frame_times.append(time.monotonic())
                self.brain.set_world_image(frame)
        finally:
            renderer.close()

    def _update_telemetry(self) -> None:
        decision, plan = self.brain.snapshot()
        state = self.simulation.observe(self._last_action)
        self_thought, thought_updated_at = self.brain.thought_snapshot()
        reflex_rate, plan_rate, queue_depth, superseded = self.brain.metrics()
        now = time.monotonic()
        for timestamps in (self._step_times, self._sensory_times):
            while timestamps and timestamps[0] < now - 2.0:
                timestamps.popleft()
        physics_hz = self._sample_rate(self._step_times)
        sensory_hz = self._sample_rate(self._sensory_times)
        with self._lock:
            while self._frame_times and self._frame_times[0] < now - 2.0:
                self._frame_times.popleft()
            render_fps = self._sample_rate(self._frame_times)
            events = list(self._events)
            camera_view = self._camera_view
        telemetry = {
            "connected": True,
            "wall_seconds": time.monotonic() - self._wall_started,
            "sim": state.model_dump(mode="json"),
            "decision": decision.model_dump(mode="json"),
            "motor_action": self._last_action,
            "motor_joint_source": self._joint_source,
            "control_mode": self.brain.control_mode_snapshot(),
            "goal_revision": self.simulation.goal_revision,
            "camera_view": camera_view,
            "arena": {
                "bounds": BOUNDS,
                "obstacles": [obstacle.as_dict() for obstacle in OBSTACLES],
            },
            "plan": plan.model_dump(mode="json"),
            "thought": {
                "message": self_thought,
                "updated_at": thought_updated_at,
            },
            "metrics": {
                "physics_hz": round(physics_hz),
                "physics_timestep_hz": round(1 / self.simulation.model.opt.timestep),
                "sensory_hz": round(sensory_hz),
                "render_fps": round(render_fps, 1),
                "system_one_hz": reflex_rate,
                "system_two_hz": plan_rate,
                "actuator_count": self.simulation.model.nu,
                "sim_speed": round(physics_hz * self.simulation.model.opt.timestep, 2),
                "queue_depth": queue_depth,
                "superseded": superseded,
                "reflex_lag": max(0.0, state.sim_time - decision.observation_sim_time),
            },
            "models": {
                "jev": {
                    "configured": self.brain.has_jev,
                    "active": decision.source == "jev",
                    "name": self.brain.system_one.model,
                    "error": self.brain.system_one.last_error,
                },
                "gemini": {
                    "configured": self.brain.has_gemini,
                    "active": plan.source == "gemini",
                    "name": self.brain.system_two.model,
                    "error": self.brain.system_two.last_error,
                },
                "offline": self.offline,
            },
            "events": events,
        }
        with self._lock:
            self._telemetry = telemetry

    @staticmethod
    def _sample_rate(timestamps: deque[float]) -> float:
        if len(timestamps) < 2:
            return 0.0
        return (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])


def create_app(runtime: RobotRuntime) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_: FastAPI):
        runtime.start()
        try:
            yield
        finally:
            runtime.close()

    app = FastAPI(title="JEV Robot Control Room", lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(
            STATIC_DIR / "index.html", headers={"Cache-Control": "no-store"}
        )

    @app.get("/api/state")
    async def state() -> JSONResponse:
        return JSONResponse(runtime.snapshot())

    @app.post("/api/thought")
    async def thought(request: ThoughtRequest) -> dict[str, bool]:
        runtime.set_thought(request.message)
        return {"accepted": True}

    @app.post("/api/goal")
    async def goal(request: GoalRequest) -> JSONResponse:
        try:
            runtime.set_goal(request.x, request.y)
        except ValueError as error:
            return JSONResponse({"error": str(error)}, status_code=422)
        return JSONResponse({"accepted": True})

    @app.post("/api/camera")
    async def camera(request: CameraRequest) -> JSONResponse:
        try:
            runtime.set_camera(request.view)
        except ValueError as error:
            return JSONResponse({"error": str(error)}, status_code=422)
        return JSONResponse({"accepted": True})

    @app.post("/api/control-mode")
    async def control_mode(request: ControlModeRequest) -> dict[str, bool]:
        runtime.set_control_mode(request.mode)
        return {"accepted": True}

    @app.post("/api/reset")
    async def reset() -> dict[str, bool]:
        runtime.reset_robot()
        return {"accepted": True}

    @app.get("/stream.mjpeg")
    async def stream() -> StreamingResponse:
        async def frames():
            last_sequence = -1
            while True:
                sequence, frame = runtime.frame()
                if frame and sequence != last_sequence:
                    last_sequence = sequence
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        + frame
                        + b"\r\n"
                    )
                await asyncio.sleep(1 / 30)

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.websocket("/ws")
    async def websocket(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                try:
                    payload = await asyncio.wait_for(
                        websocket.receive_json(), timeout=0.1
                    )
                    if payload.get("type") == "thought":
                        runtime.set_thought(str(payload.get("message", "")))
                    elif payload.get("type") == "goal":
                        try:
                            runtime.set_goal(float(payload["x"]), float(payload["y"]))
                        except (KeyError, TypeError, ValueError):
                            await websocket.send_json(
                                {"error": "Select a clear point inside the arena"}
                            )
                    elif payload.get("type") == "camera":
                        try:
                            runtime.set_camera(str(payload["view"]))
                        except (KeyError, ValueError):
                            await websocket.send_json({"error": "Unknown camera view"})
                    elif payload.get("type") == "control_mode":
                        try:
                            runtime.set_control_mode(str(payload["mode"]))
                        except (KeyError, ValueError):
                            await websocket.send_json({"error": "Unknown control mode"})
                    elif payload.get("type") == "reset":
                        runtime.reset_robot()
                except TimeoutError:
                    pass
                await websocket.send_json(runtime.snapshot())
        except WebSocketDisconnect:
            return

    return app


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the browser robot control room.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8000, type=int)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--no-open", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    runtime = RobotRuntime(args.task, offline=args.offline)
    app = create_app(runtime)
    url = f"http://{args.host}:{args.port}"
    if not args.no_open:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
        # Chrome may send localhost cookies larger than websockets' 8 KiB header limit.
        ws="wsproto",
        log_level="info",
        timeout_graceful_shutdown=2,
    )


if __name__ == "__main__":
    main()
