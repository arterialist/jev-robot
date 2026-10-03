"""Check the browser API without starting physics or making model calls."""

from __future__ import annotations

import queue
import threading
import unittest

from fastapi.testclient import TestClient

from jev_robot.web import RobotRuntime, create_app


class LocalRuntime(RobotRuntime):
    def __init__(self):
        self.started = False
        self.closed = False
        self.thought = ""
        self.mode = "direct"
        self._lock = threading.Lock()
        self._commands = queue.SimpleQueue()
        self._camera_view = "arena"

    def start(self):
        self.started = True

    def close(self):
        self.closed = True

    def snapshot(self):
        return {"offline": True, "control_mode": self.mode}

    def set_thought(self, message):
        self.thought = message

    def set_control_mode(self, mode):
        if mode not in {"direct", "primitives"}:
            raise ValueError("Unknown control mode")
        self.mode = mode


class BrowserApiTests(unittest.TestCase):
    def setUp(self):
        self.runtime = LocalRuntime()
        self.client = TestClient(create_app(self.runtime))
        self.client.__enter__()
        self.addCleanup(self.client.__exit__, None, None, None)

    def test_page_and_static_assets_ship(self):
        self.assertTrue(self.runtime.started)
        response = self.client.get("/")
        self.assertEqual(response.status_code, 200)
        self.assertIn("control-room.js", response.text)
        self.assertEqual(response.headers["cache-control"], "no-store")
        for path in ("control-room.js", "control-room.css"):
            self.assertEqual(self.client.get(f"/static/{path}").status_code, 200)

    def test_state_and_thought_contracts(self):
        self.assertTrue(self.client.get("/api/state").json()["offline"])
        response = self.client.post("/api/thought", json={"message": "hold"})
        self.assertEqual(response.json(), {"accepted": True})
        self.assertEqual(self.runtime.thought, "hold")

    def test_goal_validation_and_command_queue(self):
        response = self.client.post("/api/goal", json={"x": 0.85, "y": 0.50})
        self.assertEqual(response.status_code, 422)
        self.assertTrue(self.runtime._commands.empty())
        response = self.client.post("/api/goal", json={"x": 2.5, "y": -0.45})
        self.assertEqual(response.json(), {"accepted": True})
        self.assertEqual(self.runtime._commands.get_nowait(), ("goal", (2.5, -0.45)))

    def test_camera_and_control_mode_validation(self):
        self.assertEqual(self.client.post("/api/camera", json={"view": "invalid"}).status_code,
                         422)
        self.assertEqual(self.client.post("/api/camera", json={"view": "fpv"}).status_code,
                         200)
        self.assertEqual(self.runtime._camera_view, "fpv")
        self.assertEqual(self.client.post("/api/control-mode", json={"mode": "invalid"}).status_code,
                         422)
        response = self.client.post("/api/control-mode", json={"mode": "primitives"})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.runtime.mode, "primitives")

    def test_reset_remains_an_explicit_command(self):
        self.assertTrue(self.runtime._commands.empty())
        self.assertEqual(self.client.post("/api/reset").json(), {"accepted": True})
        self.assertEqual(self.runtime._commands.get_nowait(), ("reset", None))


if __name__ == "__main__":
    unittest.main()
