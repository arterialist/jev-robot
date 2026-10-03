"""Offline contracts for goal selection, fresh decisions, and bounded motors."""

from __future__ import annotations

import math
import queue
import threading
import unittest
from dataclasses import dataclass
from types import SimpleNamespace

import mujoco
import numpy as np

from jev_robot.arena import BOUNDS, DEFAULT_GOAL, OBSTACLES, goal_is_safe
from jev_robot.brains import CognitiveArchitecture, JevReflexes, LatestLifoQueue
from jev_robot.controller import MotionController
from jev_robot.models import ReflexDecision, WorldState
from jev_robot.simulation import G1Simulation, select_motor_action


def world_state(**updates: object) -> WorldState:
    values = {
        "sim_time": 1.0,
        "position": (0.0, -0.45, 0.78),
        "orientation_rpy": (0.0, 0.0, 0.0),
        "linear_velocity": (0.0, 0.0, 0.0),
        "angular_velocity": (0.0, 0.0, 0.0),
        "target_position": DEFAULT_GOAL,
        "target_distance": 2.5,
        "target_bearing": 0.0,
        "nearest_obstacle_distance": 2.0,
        "wall_clearance": 0.7,
        "obstacle_side": "none",
        "route_phase": "approach_goal",
        "fallen": False,
        "current_action": "stand",
        "task": "Reach the selected goal.",
    }
    values.update(updates)
    return WorldState.model_validate(values)


@dataclass
class Observation:
    sim_time: float


class ArenaTests(unittest.TestCase):
    def test_default_goal_is_reachable_point(self):
        self.assertTrue(goal_is_safe(*DEFAULT_GOAL))

    def test_goals_inside_obstacles_are_rejected(self):
        for obstacle in OBSTACLES:
            with self.subTest(obstacle=obstacle.name):
                self.assertFalse(goal_is_safe(obstacle.x, obstacle.y))

    def test_goals_outside_bounds_or_nonfinite_are_rejected(self):
        low_x, high_x, low_y, high_y = BOUNDS
        for point in ((low_x - 1, 0), (high_x + 1, 0), (0, low_y - 1),
                      (0, high_y + 1), (math.nan, 0), (math.inf, 0)):
            with self.subTest(point=point):
                self.assertFalse(goal_is_safe(*point))


class QueueTests(unittest.TestCase):
    def test_out_of_order_publication_returns_latest_simulation_time(self):
        stack = LatestLifoQueue[Observation]()
        for sim_time in (3.0, 1.0, 2.0):
            stack.put(Observation(sim_time))
        self.assertEqual(stack.get_latest(timeout=0.1).sim_time, 3.0)
        self.assertEqual(stack.superseded, 2)
        self.assertEqual(stack.qsize(), 0)

    def test_full_queue_keeps_new_observation(self):
        stack = LatestLifoQueue[Observation](maxsize=2)
        for sim_time in (1.0, 2.0, 3.0):
            stack.put(Observation(sim_time))
        self.assertEqual(stack.get_latest(timeout=0.1).sim_time, 3.0)
        self.assertEqual(stack.superseded, 2)

    def test_empty_queue_times_out(self):
        with self.assertRaises(queue.Empty):
            LatestLifoQueue[Observation]().get_latest(timeout=0.001)


class DecisionTests(unittest.TestCase):
    def test_arrival_holds_until_explicit_intent(self):
        state = world_state(route_phase="arrived")
        self.assertEqual(select_motor_action(state, "walk_forward"), "stand")
        self.assertEqual(select_motor_action(state, "walk_forward", "walk forward"),
                         "walk_forward")

    def test_wall_blocks_translation_but_allows_explicit_turn(self):
        state = world_state(position=(3.6, -0.45, 0.78))
        self.assertEqual(select_motor_action(state, "walk_forward"), "stand")
        self.assertEqual(select_motor_action(state, "turn_left", "turn left"),
                         "turn_left")

    def test_obstacle_blocks_forward_motion(self):
        state = world_state(position=(0.40, 0.50, 0.78))
        self.assertEqual(select_motor_action(state, "walk_forward"), "stand")

    def test_local_fallback_respects_hold_and_reports_source(self):
        decision = JevReflexes._fallback(world_state(), "hold")
        self.assertEqual(decision.action, "stand")
        self.assertEqual(decision.source, "fallback")

    def test_fallen_state_takes_priority_over_explicit_intent(self):
        decision = JevReflexes._fallback(world_state(fallen=True), "walk forward")
        self.assertEqual(decision.action, "recover")

    def test_older_inflight_response_cannot_overwrite_newer_response(self):
        brain = CognitiveArchitecture("Reach the goal", {}, {}, offline=True)
        self.addCleanup(brain.system_one.client.close)
        old_started = threading.Event()
        release_old = threading.Event()
        new_applied = threading.Event()

        def decide(state, plan, self_thought, mode):
            if state.sim_time == 1.0:
                old_started.set()
                release_old.wait(timeout=2)
                brain._stop.set()
            return ReflexDecision(action="walk_forward", source="jev",
                                  observation_sim_time=state.sim_time)

        brain.system_one.decide = decide
        brain._on_event = lambda message: new_applied.set()
        brain._threads = [threading.Thread(target=brain._reflex_loop, daemon=True)
                          for _ in range(2)]
        self.addCleanup(brain.close)
        self.addCleanup(release_old.set)
        for worker in brain._threads:
            worker.start()
        brain.publish(world_state(sim_time=1.0))
        self.assertTrue(old_started.wait(timeout=2), "First request did not start")
        brain.publish(world_state(sim_time=2.0))
        self.assertTrue(new_applied.wait(timeout=2), "New response was not applied")
        release_old.set()
        for worker in brain._threads:
            worker.join(timeout=2)
        self.assertEqual(brain.snapshot()[0].observation_sim_time, 2.0)

    def test_changed_intent_invalidates_inflight_response(self):
        brain = CognitiveArchitecture("Reach the goal", {}, {}, offline=True)
        self.addCleanup(brain.system_one.client.close)
        started = threading.Event()
        release = threading.Event()

        def decide(state, plan, self_thought, mode):
            started.set()
            release.wait(timeout=2)
            brain._stop.set()
            return ReflexDecision(action="walk_forward", source="jev",
                                  observation_sim_time=state.sim_time)

        brain.system_one.decide = decide
        worker = threading.Thread(target=brain._reflex_loop, daemon=True)
        brain._threads = [worker]
        self.addCleanup(brain.close)
        self.addCleanup(release.set)
        worker.start()
        brain.publish(world_state(sim_time=1.0))
        self.assertTrue(started.wait(timeout=2), "Request did not start")
        brain.set_thought("hold")
        release.set()
        worker.join(timeout=2)
        self.assertEqual(brain.snapshot()[0].action, "stand")
        self.assertEqual(brain.snapshot()[0].source, "fallback")


class MotorTests(unittest.TestCase):
    def test_pd_controller_limits_torque_and_joint_target_changes(self):
        model = mujoco.MjModel.from_xml_string('''
            <mujoco><compiler angle="radian"/><worldbody><body>
              <joint name="waist_yaw_joint" type="hinge" range="-0.5 0.5"/>
              <geom type="sphere" size="0.1" mass="1"/>
            </body></worldbody><actuator>
              <motor joint="waist_yaw_joint" ctrllimited="true" ctrlrange="-1 1"/>
            </actuator></mujoco>
        ''')
        controller = MotionController(model, {"waist_yaw_joint": (100.0, 1.0)})
        data = mujoco.MjData(model)
        previous = 0.0
        for _ in range(30):
            controller.apply(data, {"waist_yaw_joint": 100.0})
            target = controller._commanded_targets["waist_yaw_joint"]
            self.assertLessEqual(abs(target - previous), 0.04 + 1e-12)
            self.assertGreaterEqual(target, -0.48)
            self.assertLessEqual(target, 0.48)
            self.assertLessEqual(abs(float(data.ctrl[0])), 1.0)
            if previous == 0.0:
                self.assertAlmostEqual(target, 0.04)
            previous = target
        self.assertAlmostEqual(previous, 0.48)

    def test_direct_commands_clip_speeds_and_reject_nonfinite_values(self):
        simulation = G1Simulation.__new__(G1Simulation)
        simulation.data = SimpleNamespace(qpos=np.array([0, -0.45, 0.78, 1, 0, 0, 0]))
        np.testing.assert_allclose(simulation._safe_velocity((10.0, 10.0, 10.0)),
                                   [0.45, 0.25, 0.70])
        for command in ((math.nan, 0, 0), (0, math.inf, 0)):
            np.testing.assert_array_equal(simulation._safe_velocity(command), [0, 0, 0])

    def test_direct_command_preserves_turn_when_translation_is_blocked(self):
        simulation = G1Simulation.__new__(G1Simulation)
        simulation.data = SimpleNamespace(qpos=np.array([3.6, -0.45, 0.78, 1, 0, 0, 0]))
        np.testing.assert_allclose(simulation._safe_velocity((0.45, 0.0, 0.35)),
                                   [0, 0, 0.35])


if __name__ == "__main__":
    unittest.main()
