# Architecture

JEV robot combines slow remote model decisions with local locomotion and torque control. A remote response chooses what the robot should attempt, while the local gait policy and PD controller run between responses.

## Model loops

`CognitiveArchitecture` publishes each `WorldState` to separate bounded LIFO queues. Three JEV worker threads consume the newest available observation; one Gemini thread consumes state for planning. Each worker drains older pending observations rather than working through a FIFO backlog.

The state contains position, orientation, velocity, joint positions and velocities, target distance and bearing, obstacle geometry, wall clearance, arrival state, and the current action. JEV also receives Gemini's latest structured plan and the active thought. Gemini receives its previous plan, the state, and the active thought. In browser mode it also receives the current rendered JPEG.

A JEV result only replaces the current decision if its observation time is newer. Changing the goal, thought, or control mode increments a decision generation so an old request cannot overwrite the new intent. Gemini also rejects a completed plan if its thought, mode, or target is stale.

The explicit thought is an input supplied through the interface. The prompts present it as the robot's immediate first-person intent. It is not evidence that the model produced a thought autonomously. The visible `mind_note` is a short public status output, not hidden reasoning.

### JEV full control

The JEV client sends typed questions to `https://openrouter.ai/api/alpha/decisions`:

- Three Choice questions select body-frame forward speed, lateral speed, and yaw rate from fixed numeric levels.
- Twenty-nine Score questions return probabilities over nine bounded offsets for each joint.
- A Noul question reports immediate fall or collision risk.

The client computes each joint offset as the probability-weighted average of its candidate angles. It validates scores and probability totals before accepting the result. Full control therefore means numeric velocity selection and an offset for every joint, with a local gait policy still responsible for step timing and balance.

Velocity choices range from -0.22 to 0.45 m/s forward, -0.25 to 0.25 m/s lateral, and -0.70 to 0.70 rad/s yaw. Joint-offset bounds depend on the joint. These commands shape the current gait targets; they are not direct torque predictions.

### JEV primitives

In Primitives mode, JEV answers a Choice question to select a named movement from the movement library. The gait policy turns walking and turning actions into velocity commands. `pose_offsets` adds timed joint adjustments for gestures and posture changes. The library includes a small one-shot hop, but repeated hopping is unstable.

### Gemini planning

Gemini uses Google's `google-genai` SDK and the `SystemTwoPlan` Pydantic schema. Its output includes a goal, strategy, completion flag, and visible status note. Planning starts on a roughly five-second wall-clock schedule. A new thought, goal, or control mode requests an earlier replan.

In the browser, the renderer supplies the latest camera frame. Switching to FPV changes the frame Gemini receives, along with the displayed view. The standalone CLI has no live image input.

### Local fallbacks

Each model loop falls back independently when disabled, unconfigured, or unable to return a valid response. The local JEV fallback chooses movement actions from state and simple thought matching. The Gemini fallback describes arrival, the active thought, or a deterministic goal approach. An explicit thought immediately installs a local fallback decision while remote requests are pending.

Offline mode uses these fallbacks and makes no JEV or Gemini API calls. It still runs the actual MuJoCo model and ONNX gait policy. The browser reports configured keys, active response sources, and the most recent error type.

## Local motor path

`G1GaitPolicy` runs the Unitree ONNX velocity policy through ONNX Runtime's CPU execution provider. It receives body angular velocity, projected gravity, the velocity command, joint positions, joint velocities, and the previous action. It keeps five observations and concatenates their terms in Unitree's deployment order.

The policy updates joint targets at its configured 50 Hz rate. In Full control, JEV's offsets are added to those targets. In Primitives mode, the timed pose offsets are added instead. The controller clips target angles to model limits, limits their change to 0.04 radians per physics step, and applies per-joint PD gains. It clips the resulting torque to each actuator's control range.

MuJoCo uses a 2 ms timestep with `implicitfast`. The simulation clears applied external body forces each step. It does not overwrite root velocity or push the pelvis to create movement.

## Arena and motion limits

The arena has walls, six red box obstacles, and a movable green goal marker. The robot begins at the same fixed starting pose. Clicking the map changes the target without resetting the body. The goal validator rejects points outside the arena or within a margin of an obstacle.

Full control checks a short projection of the requested translation and suppresses it when the projection enters a wall margin or obstacle margin. Turning remains available. The named-movement path also checks its projected translation. These are local limits, not a route planner or a guarantee against contact.

Arrival latches when target distance drops below 0.48 m. Travel then stops unless an explicit thought asks for more movement. A new goal clears the arrival latch. A fall remains visible and does not normally reset the pose. Reset pose is an explicit browser command. The numerical-instability guard can restore the pose if simulated time stops advancing or joint positions become non-finite.

## Timing and telemetry

| Loop | Schedule | Meaning |
| --- | --- | --- |
| Physics | 2 ms simulated timestep | 500 steps per simulated second, with real-time pacing by default |
| Sensory publication | Every 20 ms of simulated time | 50 events per simulated second |
| Unitree gait | 20 ms simulated update | 50 joint-target updates per simulated second |
| JEV | Three concurrent workers | Completion rate depends on API latency and discarded responses |
| Gemini | Plan starts about every 5 wall-clock seconds | Can replan early on intent changes |
| Browser rendering | 15 wall-clock FPS target | Separate render thread and copied physics pose |
| Browser telemetry | Every 100 wall-clock ms | State, plans, commands, rates, and recent events |

The browser renderer has its own `MjData` and copies the current pose from the physics thread. It streams JPEG frames over MJPEG. The WebSocket carries controls and telemetry; HTTP endpoints provide equivalent controls and a state snapshot.

The dashboard reports measured rates, not only configured targets. Physics, sensory, and frame rates use recent timestamp samples spanning roughly two seconds. JEV uses a ten-second accepted-completion window, and Gemini uses a 30-second accepted-completion window. Decision age is the difference between the current simulation time and the applied decision's observation time, displayed in simulated milliseconds.

Three concurrent requests can increase throughput. They cannot make a remote model respond at the 500 Hz physics rate. The controller holds the latest accepted command while physics continues.

## Assets and upstream boundaries

`assets.py` fetches the G1 29-DOF MJCF and meshes from [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco). The model checkout follows the upstream default branch when first downloaded. The application creates a derived XML containing the FPV camera and generates the obstacle arena beside the downloaded model.

The gait policy and deployment configuration come from [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab), pinned to commit [`4960b84732b0c2ec593dccbfe963fda1bcd7b1e3`](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0). Only the policy directory is checked out. Cached assets are reused on later starts.

These external files are downloaded at runtime and retain their upstream licenses. The repository's own license does not relicense them. See [third-party notices](../THIRD_PARTY_NOTICES.md).

## Source map

| Module | Responsibility |
| --- | --- |
| `app.py` | CLI, native viewer, headless pacing, and PNG capture |
| `web.py` | Browser runtime, render thread, FastAPI endpoints, and WebSocket |
| `brains.py` | Model clients, fallback rules, concurrent queues, and stale-response checks |
| `models.py` | Typed state, decisions, plans, and movement names |
| `simulation.py` | MuJoCo state, goals, motor execution, and motion limits |
| `locomotion.py` | ONNX gait inference and movement velocity mappings |
| `controller.py` | Joint-target limits and clipped PD torque |
| `primitives.py` | Named movements and timed pose offsets |
| `arena.py` | Arena bounds, obstacle geometry, and goal validation |
| `assets.py` | Upstream downloads and generated scene XML |
| `static/` | Control-room HTML, CSS, and browser client |
