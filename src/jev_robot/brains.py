from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from collections import deque
from collections.abc import Callable

import httpx
from dotenv import find_dotenv, load_dotenv
from google import genai
from google.genai import errors as genai_errors
from google.genai import types
from pydantic import ValidationError

from jev_robot.models import ControlMode, Motion, ReflexDecision, SystemTwoPlan, WorldState
from jev_robot.primitives import PRIMITIVE_DESCRIPTIONS


class LatestLifoQueue[T]:
    """Bounded sensory-event stack that always hands consumers the newest event."""

    def __init__(self, maxsize: int = 256) -> None:
        self._queue: queue.LifoQueue[T] = queue.LifoQueue(maxsize=maxsize)
        self.superseded = 0

    def put(self, value: T) -> None:
        try:
            self._queue.put_nowait(value)
        except queue.Full:
            self._discard_pending()
            self._queue.put_nowait(value)

    def get_latest(self, timeout: float) -> T:
        newest = self._queue.get(timeout=timeout)
        while True:
            try:
                candidate = self._queue.get_nowait()
            except queue.Empty:
                return newest
            self.superseded += 1
            candidate_time = getattr(candidate, "sim_time", 0.0)
            newest_time = getattr(newest, "sim_time", 0.0)
            if candidate_time > newest_time:
                newest = candidate

    def _discard_pending(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
                self.superseded += 1
            except queue.Empty:
                return

    def qsize(self) -> int:
        return self._queue.qsize()


class JevReflexes:
    endpoint = "https://openrouter.ai/api/alpha/decisions"
    velocity_levels = {
        "forward": {
            "back_fast": (-0.22, "Walk backward at 0.22 m/s."),
            "back_slow": (-0.12, "Walk backward at 0.12 m/s."),
            "hold": (0.0, "No forward or backward travel."),
            "forward_slow": (0.15, "Walk forward at 0.15 m/s."),
            "forward_cruise": (0.30, "Walk forward at 0.30 m/s."),
            "forward_fast": (0.45, "Walk forward at 0.45 m/s."),
        },
        "lateral": {
            "right_fast": (-0.25, "Travel right at 0.25 m/s."),
            "right_slow": (-0.12, "Travel right at 0.12 m/s."),
            "hold": (0.0, "No sideways travel."),
            "left_slow": (0.12, "Travel left at 0.12 m/s."),
            "left_fast": (0.25, "Travel left at 0.25 m/s."),
        },
        "yaw": {
            "right_fast": (-0.70, "Turn right at 0.70 rad/s."),
            "right_slow": (-0.35, "Turn right at 0.35 rad/s."),
            "hold": (0.0, "Keep the current heading."),
            "left_slow": (0.35, "Turn left at 0.35 rad/s."),
            "left_fast": (0.70, "Turn left at 0.70 rad/s."),
        },
    }

    def __init__(
        self,
        api_key: str | None,
        model: str,
        joint_limits: dict[str, tuple[float, float]],
        neutral_positions: dict[str, float],
    ) -> None:
        self.api_key = api_key
        self.model = model
        self.joint_limits = joint_limits
        self.neutral_positions = neutral_positions
        self.client = httpx.Client(timeout=4.0, trust_env=False)
        self.last_error: str | None = None
        self._cooldown_until = 0.0

    def decide(
        self,
        state: WorldState,
        plan: SystemTwoPlan,
        self_thought: str = "",
        mode: ControlMode = "direct",
    ) -> ReflexDecision:
        if not self.api_key or time.monotonic() < self._cooldown_until:
            return self._fallback(state, self_thought)
        if mode == "primitives":
            return self._decide_primitives(state, plan, self_thought)

        started = time.perf_counter()
        joint_levels = {name: self._offset_levels(name) for name in self.joint_limits}
        joint_questions = {
            f"joint_{joint_name}": {
                "type": "score",
                "instructions": (
                    f"Choose a bounded offset for {joint_name}, one of my 29 G1 motors, "
                    "from the current 50 Hz gait and balance target. Current angle is "
                    f"{state.joint_positions[joint_name]:.3f} rad and velocity is "
                    f"{state.joint_velocities[joint_name]:.3f} rad/s. "
                    f"Neutral standing angle is {self.neutral_positions[joint_name]:.3f} rad. "
                    "The gait policy already generates coordinated steps from my numeric "
                    "forward, lateral, and turning commands. Use this offset to shape my "
                    "posture, arm gesture, or step without fighting leg balance. Zero is "
                    "appropriate when the gait policy should control this joint alone."
                ),
                "criteria": [
                    f"Add {angle:+.3f} radians to {joint_name}'s live gait target."
                    for angle in joint_levels[joint_name]
                ],
            }
            for joint_name in self.joint_limits
        }
        velocity_questions = {
            f"velocity_{axis}": {
                "type": "choice",
                "instructions": (
                    f"Choose my sustained {axis} body-frame velocity for the next control "
                    "interval. These three numeric channels drive a local 50 Hz walking "
                    "policy; they are not named movement primitives. Follow my explicit "
                    "first-person thought first, otherwise Gemini's current goal. To reach "
                    "a distant goal, actually move rather than selecting hold. Turn and "
                    "advance together when there is space. Slow or stop before a wall, "
                    "obstacle, or completed goal."
                ),
                "criteria": {
                    label: description
                    for label, (_, description) in levels.items()
                },
            }
            for axis, levels in self.velocity_levels.items()
        }
        payload = {
            "model": self.model,
            "state": {
                "robot": state.compact(),
                "joint_limits_radians": self.joint_limits,
                "balanced_joint_positions_radians": self.neutral_positions,
                "system_two_plan": plan.model_dump(mode="json"),
                "active_self_thought": self_thought
                or "No explicit self-thought. Continue autonomous intent.",
                "self_thought_priority": (
                    "This is the robot's current first-person thought. Treat it as "
                    "self-generated immediate intent unless it would cause a fall or collision."
                ),
                "decision_hierarchy": (
                    "The explicit first-person thought is immediate intent. Gemini supplies "
                    "the high-level goal and strategy. Choose numeric walking velocities "
                    "and a bounded residual for every one of the 29 joints. The local "
                    "walking policy handles step timing and balance at 50 Hz; PD torque "
                    "control runs at 500 Hz. Route phase reports arrival."
                ),
            },
            "questions": {
                "emergency": {
                    "type": "noul",
                    "instructions": "Is there an immediate fall, obstacle, or arena-wall collision risk that the motion commands must respond to?",
                    "criteria": {
                        "true": "I am falling, badly tilted, or dangerously close to an obstacle.",
                        "false": "I have adequate clearance and my body is stable.",
                    },
                },
                **velocity_questions,
                **joint_questions,
            },
        }
        try:
            response = self.client.post(
                self.endpoint,
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=payload,
            )
            response.raise_for_status()
            answers = response.json()["answers"]
            emergency = float(answers["emergency"]["noul"])
            velocity: list[float] = []
            confidences: list[float] = []
            for axis, levels in self.velocity_levels.items():
                answer = answers[f"velocity_{axis}"]
                label = answer["choice"]
                if label not in levels:
                    raise ValueError(f"Invalid {axis} velocity choice")
                velocity.append(levels[label][0])
                confidences.append(float(answer.get("confidence", 0.0)))
            offsets: dict[str, float] = {}
            for joint_name in self.joint_limits:
                answer = answers[f"joint_{joint_name}"]
                score = float(answer["score"])
                if not math.isfinite(score) or not 0.0 <= score <= 8.0:
                    raise ValueError(f"Invalid joint score for {joint_name}")
                probabilities = [
                    float(answer["probabilities"][str(index)])
                    for index in range(9)
                ]
                total = sum(probabilities)
                if not all(math.isfinite(value) and value >= 0 for value in probabilities):
                    raise ValueError(f"Invalid joint probabilities for {joint_name}")
                if not 0.95 <= total <= 1.05:
                    raise ValueError(f"Incomplete joint probabilities for {joint_name}")
                offsets[joint_name] = sum(
                    angle * probability
                    for angle, probability in zip(joint_levels[joint_name], probabilities)
                ) / total
                confidences.append(float(answer.get("confidence", 0.0)))
            self.last_error = None
            return ReflexDecision(
                action="direct",
                confidence=sum(confidences) / len(confidences),
                emergency_probability=emergency,
                source="jev",
                latency_ms=(time.perf_counter() - started) * 1000,
                observation_sim_time=state.sim_time,
                velocity_command=tuple(velocity),
                joint_offsets=offsets,
                joint_confidence=sum(confidences) / len(confidences),
            )
        except (httpx.HTTPError, KeyError, TypeError, ValueError) as error:
            return self._failed_decision(error, state, self_thought, started)

    def _decide_primitives(
        self, state: WorldState, plan: SystemTwoPlan, self_thought: str
    ) -> ReflexDecision:
        started = time.perf_counter()
        motions = PRIMITIVE_DESCRIPTIONS
        payload = {
            "model": self.model,
            "state": {
                "robot": state.compact(),
                "system_two_plan": plan.model_dump(mode="json"),
                "active_self_thought": self_thought
                or "No explicit self-thought. Continue autonomous intent.",
                "control_mode": "Choose one named movement; the local walking and joint-pose policies execute it.",
                "hop_once_available": True,
            },
            "questions": {
                "motion": {
                    "type": "choice",
                    "instructions": (
                        "Choose the next useful whole-body movement. Follow my first-person "
                        "thought when safe, otherwise Gemini's goal. React to falls and "
                        "obstacles. A small one-shot hop is available, but repeated "
                        "jumping is not stable."
                    ),
                    "criteria": motions,
                }
            },
        }
        try:
            response = self.client.post(
                self.endpoint,
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=payload,
            )
            response.raise_for_status()
            answer = response.json()["answers"]["motion"]
            action = answer["choice"]
            if action not in motions:
                raise ValueError("Unknown motion choice")
            if state.fallen:
                action = "recover"
            if action in {"wave", "wave_left", "wave_right"} and not any(
                word in self_thought.casefold() for word in ("wave", "greet", "hello")
            ):
                action = "stand"
            self.last_error = None
            return ReflexDecision(
                action=action,
                confidence=float(answer.get("confidence", 0.0)),
                source="jev",
                latency_ms=(time.perf_counter() - started) * 1000,
                observation_sim_time=state.sim_time,
                probabilities=answer.get("probabilities", {}),
            )
        except (httpx.HTTPError, KeyError, TypeError, ValueError) as error:
            return self._failed_decision(error, state, self_thought, started)

    def _failed_decision(
        self, error: Exception, state: WorldState, self_thought: str, started: float
    ) -> ReflexDecision:
        if isinstance(error, httpx.HTTPStatusError):
            status = error.response.status_code
            self.last_error = f"HTTP {status}"
            cooldown = 60.0 if status in {401, 403, 429} else 2.0
        else:
            self.last_error = "Invalid response"
            cooldown = 2.0
        self._cooldown_until = time.monotonic() + cooldown
        decision = self._fallback(state, self_thought)
        decision.latency_ms = (time.perf_counter() - started) * 1000
        return decision

    @staticmethod
    def _offset_levels(name: str) -> list[float]:
        if "hip" in name or "knee" in name or "ankle" in name:
            travel = 0.10
        elif "waist" in name:
            travel = 0.14
        else:
            travel = 0.38
        return [-travel, -0.75 * travel, -0.5 * travel, -0.25 * travel,
                0.0, 0.25 * travel, 0.5 * travel, 0.75 * travel, travel]

    @staticmethod
    def _fallback(state: WorldState, self_thought: str = "") -> ReflexDecision:
        thought = self_thought.casefold()
        if state.fallen:
            action = "recover"
        elif "jump" in thought:
            action = "stand"
        elif state.nearest_obstacle_distance < 0.25 and any(
            word in thought for word in ("forward", "straight", "go", "approach")
        ):
            action = "stand"
        elif directed_action := JevReflexes._thought_action(thought):
            action = directed_action
        elif state.route_phase == "arrived":
            action = "stand"
        elif state.route_phase == "approach_goal":
            if state.target_bearing > 0.22:
                action = "turn_left"
            elif state.target_bearing < -0.22:
                action = "turn_right"
            else:
                action = "walk_forward"
        elif state.nearest_obstacle_distance < 1.05 and state.obstacle_side in {
            "ahead",
            "left",
        }:
            action = "step_right"
        elif state.nearest_obstacle_distance < 1.05:
            action = "step_left"
        elif state.target_bearing > 0.22:
            action = "turn_left"
        elif state.target_bearing < -0.22:
            action = "turn_right"
        else:
            action = "walk_forward"
        return ReflexDecision(
            action=action,
            confidence=0.55,
            source="fallback",
            observation_sim_time=state.sim_time,
        )

    @staticmethod
    def _thought_action(thought: str) -> Motion | None:
        mappings = (
            (("stop", "hold", "stand", "wait"), "stand"),
            (("walk backward", "move backward", "back up", "reverse"), "walk_backward"),
            (("walk fast", "run", "hurry"), "walk_fast"),
            (("diagonal left", "forward left"), "diagonal_left"),
            (("diagonal right", "forward right"), "diagonal_right"),
            (("wave left", "left hand"), "wave_left"),
            (("wave right", "right hand"), "wave_right"),
            (("raise both arms", "arms up", "hands up"), "arms_up"),
            (("arms out", "spread my arms"), "arms_out"),
            (("reach forward", "reach out"), "reach_forward"),
            (("point left", "point with left"), "point_left"),
            (("point right", "point with right"), "point_right"),
            (("march", "walk in place"), "march_in_place"),
            (("kick left", "left kick"), "kick_left"),
            (("kick right", "right kick"), "kick_right"),
            (("hop",), "hop"),
            (("squat",), "squat"),
            (("bow",), "bow"),
            (("nod",), "nod"),
            (("dance", "sway"), "dance"),
            (("forward", "straight", "approach", "go"), "walk_forward"),
            (("turn left", "rotate left"), "turn_left"),
            (("turn right", "rotate right"), "turn_right"),
            (("step left", "move left", "sidestep left"), "step_left"),
            (("step right", "move right", "sidestep right"), "step_right"),
            (("crouch", "duck"), "crouch"),
            (("wave", "hello", "greet"), "wave"),
            (("recover", "get up"), "recover"),
        )
        for phrases, action in mappings:
            if any(phrase in thought for phrase in phrases):
                return action
        return None


class GeminiReasoner:
    def __init__(self, api_key: str | None, model: str, task: str) -> None:
        self.api_key = api_key
        self.model = model
        self.task = task
        self.last_error: str | None = None
        self.client = (
            genai.Client(
                api_key=api_key,
                http_options=types.HttpOptions(
                    client_args={"trust_env": False},
                    async_client_args={"trust_env": False},
                ),
            )
            if api_key
            else None
        )

    def plan(
        self,
        state: WorldState,
        previous: SystemTwoPlan,
        self_thought: str = "",
        image_bytes: bytes | None = None,
        control_mode: ControlMode = "direct",
    ) -> SystemTwoPlan:
        if not self.client:
            return self._fallback(state, self_thought)

        started = time.perf_counter()
        available_moves = ", ".join(PRIMITIVE_DESCRIPTIONS)
        motor_context = (
            "JEV sets continuous forward, sideways, and turning speeds plus bounded "
            "offsets for all 29 joints. A local 50 Hz gait policy makes coordinated "
            "steps and balances the robot; JEV does not choose named movements. "
            "Describe the desired direction, speed, clearance, and posture."
            if control_mode == "direct"
            else f"JEV chooses one named movement from {available_moves}. Unitree's "
            "walking policy or a local joint-pose overlay executes it. A small "
            "one-shot hop is available; repeated jumping is not stable."
        )
        prompt = f"""You are the slow System Two planner for a Unitree G1 humanoid in MuJoCo.

Task: {self.task}

Current state:
{json.dumps(state.compact(), indent=2)}

Previous plan:
{previous.model_dump_json(indent=2)}

Current first-person self-thought:
{self_thought or "No explicit self-thought. Continue autonomous intent."}

Control mode: {control_mode}. {motor_context}

Make a spatial and physical plan for the next 5 seconds using both the structured state and the attached MuJoCo camera view when present. An explicit self-thought is the robot's current highest-level intent. Replace any conflicting previous plan with it. Only fall and collision safety outrank the thought. Otherwise, reach the currently selected green target while avoiding the visible red obstacles and arena walls. The user may move this target at any time. Use the image for spatial context, but trust numeric target_distance and route_phase for arrival. Hold position once the target is reached unless the self-thought asks for movement. There is no predefined path, lap, or automatic reset. Write mind_note as a candid one-sentence first-person status, not private chain-of-thought. The robot can still fall or hit obstacles.
"""
        try:
            contents = [prompt]
            if image_bytes:
                contents.append(
                    types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg")
                )
            response = self.client.models.generate_content(
                model=self.model,
                contents=contents,
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema=SystemTwoPlan,
                    temperature=0.2,
                    automatic_function_calling=types.AutomaticFunctionCallingConfig(
                        disable=True
                    ),
                ),
            )
            if isinstance(response.parsed, SystemTwoPlan):
                plan = response.parsed
            else:
                plan = SystemTwoPlan.model_validate_json(response.text)
            self.last_error = None
            return plan.model_copy(
                update={
                    "source": "gemini",
                    "latency_ms": (time.perf_counter() - started) * 1000,
                    "vision_used": bool(image_bytes),
                }
            )
        except (genai_errors.APIError, ValidationError, TypeError, ValueError) as error:
            self.last_error = type(error).__name__
            return self._fallback(state, self_thought)

    @staticmethod
    def _fallback(state: WorldState, self_thought: str = "") -> SystemTwoPlan:
        if self_thought:
            return SystemTwoPlan(
                goal="Act on my current intent safely",
                strategy=(
                    f'Act on the thought "{self_thought}" while preserving '
                    "balance and collision clearance."
                ),
                mind_note=f'My current thought is: "{self_thought}"',
            )
        if state.route_phase == "arrived":
            return SystemTwoPlan(
                goal="Hold at the selected target",
                strategy="Stand at the green marker and wait for a new goal or thought.",
                task_complete=True,
            )
        return SystemTwoPlan(
            goal="Reach the green target",
            strategy="Face the target, sidestep nearby red obstacles, then advance.",
        )


class CognitiveArchitecture:
    def __init__(
        self,
        task: str,
        joint_limits: dict[str, tuple[float, float]],
        neutral_positions: dict[str, float],
        *,
        offline: bool = False,
        expect_vision: bool = False,
        on_event: Callable[[str], None] | None = None,
    ) -> None:
        load_dotenv(find_dotenv(usecwd=True))
        gemini_key = None if offline else os.getenv("GEMINI_API_KEY")
        openrouter_key = None if offline else os.getenv("OPENROUTER_API_KEY")
        self.has_gemini = bool(gemini_key)
        self.has_jev = bool(openrouter_key)
        self.system_one = JevReflexes(
            openrouter_key,
            os.getenv("JEV_MODEL", "~typesafe/jev-latest"),
            joint_limits,
            neutral_positions,
        )
        self.system_two = GeminiReasoner(
            gemini_key,
            os.getenv("GEMINI_MODEL", "gemini-3.8-flash"),
            task,
        )
        self.plan = SystemTwoPlan(
            goal="Reach the green target",
            strategy="Walk toward the target and avoid red obstacles.",
        )
        self.decision = ReflexDecision()
        self._system_one_in: LatestLifoQueue[WorldState] = LatestLifoQueue()
        self._system_two_in: LatestLifoQueue[WorldState] = LatestLifoQueue()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._replan = threading.Event()
        self._threads: list[threading.Thread] = []
        self._on_event = on_event or (lambda _: None)
        self._reflex_count = 0
        self._jev_completions: deque[float] = deque()
        self._gemini_completions: deque[float] = deque()
        self._started_at = time.monotonic()
        self._self_thought = ""
        self._thought_updated_at = 0.0
        self._min_decision_sim_time = 0.0
        self._world_image: bytes | None = None
        self._world_image_ready = threading.Event()
        self._expect_vision = expect_vision
        self._current_target_position: tuple[float, float] | None = None
        self._decision_generation = 0
        self._latest_state: WorldState | None = None
        self._control_mode: ControlMode = "direct"

    def start(self) -> None:
        self._threads = [
            *[
                threading.Thread(
                    target=self._reflex_loop,
                    name=f"jev-system-one-{index}",
                    daemon=True,
                )
                for index in range(3)
            ],
            threading.Thread(
                target=self._reasoning_loop, name="gemini-system-two", daemon=True
            ),
        ]
        for thread in self._threads:
            thread.start()

    def publish(self, state: WorldState) -> None:
        with self._lock:
            self._latest_state = state
        self._system_one_in.put(state)
        self._system_two_in.put(state)

    def snapshot(self) -> tuple[ReflexDecision, SystemTwoPlan]:
        with self._lock:
            return self.decision.model_copy(deep=True), self.plan.model_copy(deep=True)

    def motor_snapshot(self) -> ReflexDecision:
        with self._lock:
            return self.decision

    def request_replan(self) -> None:
        self._replan.set()

    def set_world_image(self, image_bytes: bytes) -> None:
        with self._lock:
            self._world_image = image_bytes
        self._world_image_ready.set()

    def start_new_goal(self, state: WorldState) -> None:
        with self._lock:
            self.decision = self.system_one._fallback(state, self._self_thought)
            self._min_decision_sim_time = state.sim_time
            self._decision_generation += 1
            self._current_target_position = state.target_position
        self.publish(state)
        self.request_replan()

    def set_thought(self, thought: str) -> None:
        thought = thought.strip()
        with self._lock:
            self._self_thought = thought
            self._thought_updated_at = time.time()
            self._decision_generation += 1
            if self._latest_state is not None:
                self.decision = self.system_one._fallback(self._latest_state, thought)
        self._replan.set()
        self._on_event(
            f"Self-thought adopted: {thought}"
            if thought
            else "Explicit self-thought released. Autonomous intent resumed."
        )

    def set_control_mode(self, mode: ControlMode) -> None:
        with self._lock:
            if mode == self._control_mode:
                return
            self._control_mode = mode
            self._decision_generation += 1
            if self._latest_state is not None:
                self.decision = self.system_one._fallback(
                    self._latest_state, self._self_thought
                )
                self._system_one_in.put(self._latest_state)
        self._replan.set()
        self._on_event(f"Control mode changed to {mode}.")

    def control_mode_snapshot(self) -> ControlMode:
        with self._lock:
            return self._control_mode

    def thought_snapshot(self) -> tuple[str, float]:
        with self._lock:
            return self._self_thought, self._thought_updated_at

    def metrics(self) -> tuple[float, float, int, int]:
        with self._lock:
            now = time.monotonic()
            while self._jev_completions and self._jev_completions[0] < now - 10.0:
                self._jev_completions.popleft()
            while self._gemini_completions and self._gemini_completions[0] < now - 30.0:
                self._gemini_completions.popleft()
            elapsed = min(10.0, max(1.0, now - self._started_at))
            reflex_rate = len(self._jev_completions) / elapsed
            plan_elapsed = min(30.0, max(1.0, now - self._started_at))
            plan_rate = len(self._gemini_completions) / plan_elapsed
        return (
            reflex_rate,
            plan_rate,
            self._system_one_in.qsize(),
            self._system_one_in.superseded,
        )

    def close(self) -> None:
        self._stop.set()
        self._replan.set()
        for thread in self._threads:
            thread.join(timeout=1.0)

    def _reflex_loop(self) -> None:
        while not self._stop.is_set():
            try:
                state = self._system_one_in.get_latest(timeout=0.2)
            except queue.Empty:
                continue
            with self._lock:
                plan = self.plan.model_copy(deep=True)
                self_thought = self._self_thought
                control_mode = self._control_mode
                generation = self._decision_generation
            decision = self.system_one.decide(state, plan, self_thought, control_mode)
            event: str | None = None
            with self._lock:
                if generation != self._decision_generation:
                    continue
                if decision.observation_sim_time < self._min_decision_sim_time:
                    continue
                if decision.observation_sim_time <= self.decision.observation_sim_time:
                    continue
                changed = (
                    decision.action != self.decision.action
                    or decision.source != self.decision.source
                )
                self.decision = decision
                self._reflex_count += 1
                if decision.source == "jev":
                    self._jev_completions.append(time.monotonic())
                if changed:
                    event = (
                        f"System 1 [{decision.source}] {decision.action} "
                        f"confidence={decision.confidence:.2f} "
                        f"latency={decision.latency_ms:.0f}ms "
                        f"joint_offsets={len(decision.joint_offsets)} "
                        f"velocity={decision.velocity_command}"
                    )
            if event:
                self._on_event(event)

    def _reasoning_loop(self) -> None:
        first = True
        next_plan_at = 0.0
        while not self._stop.is_set():
            try:
                state = self._system_two_in.get_latest(timeout=0.5)
            except queue.Empty:
                continue
            if not first:
                self._replan.wait(max(0.0, next_plan_at - time.monotonic()))
                self._replan.clear()
                if self._stop.is_set():
                    break
            first = False
            try:
                state = self._system_two_in.get_latest(timeout=0.01)
            except queue.Empty:
                pass
            if self._expect_vision and not self._world_image_ready.is_set():
                self._world_image_ready.wait(timeout=2.0)
            with self._lock:
                previous = self.plan.model_copy(deep=True)
                self_thought = self._self_thought
                control_mode = self._control_mode
                image_bytes = self._world_image
            started = time.monotonic()
            plan = self.system_two.plan(
                state, previous, self_thought, image_bytes, control_mode
            )
            next_plan_at = started + 5.0
            with self._lock:
                if control_mode != self._control_mode or self_thought != self._self_thought or (
                    self._current_target_position is not None
                    and state.target_position != self._current_target_position
                ):
                    continue
                self.plan = plan
                if plan.source == "gemini":
                    self._gemini_completions.append(time.monotonic())
            self._on_event(f"System 2 goal: {plan.goal}. {plan.strategy}")
