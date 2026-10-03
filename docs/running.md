# Running JEV robot

Start with the browser control room to explore the simulation. Use the command-line entry point for the native MuJoCo viewer, headless runs, or a final PNG capture.

## Install

You need Git and [uv](https://docs.astral.sh/uv/getting-started/installation/). The package requires Python 3.12 or newer.

```bash
git clone https://github.com/arterialist/jev-robot.git
cd jev-robot
uv sync --python 3.12
```

`uv sync` installs the project and dependencies into `.venv`. The first simulation run downloads the G1 assets and a pinned Unitree gait policy into the platform's user cache. This download also happens in offline mode. Later runs reuse the cached files.

## Browser control room

```bash
uv run jev-robot-web --offline
```

The command opens [http://127.0.0.1:8000](http://127.0.0.1:8000). Stop it with `Ctrl-C` in the terminal.

```bash
uv run jev-robot-web --offline --no-open --port 8080
uv run jev-robot-web --task "Reach the green target and hold position"
```

| Option | Default | Purpose |
| --- | --- | --- |
| `--host` | `127.0.0.1` | Address on which the server listens |
| `--port` | `8000` | HTTP port |
| `--task` | Reach the selected green goal, avoid obstacles and walls, then hold | Planner task |
| `--offline` | Off | Disable JEV and Gemini API calls |
| `--no-open` | Off | Skip automatic browser launch |

The server has no authentication. Its HTTP endpoints and WebSocket can change the goal, intent, camera, and pose. The default loopback address keeps it on your computer. Add access controls before exposing it on a network.

### Controls

Click a clear point on the map to set the green goal. A new goal clears the explicit thought and starts a new planning cycle. Reset pose restores the start pose while preserving the browser session's simulation time.

Full control uses JEV's three velocity choices and 29 joint offsets when a valid JEV response is active. Primitives uses named movements executed by the local controllers. Offline mode and request failures use deterministic local movement choices in either mode. The UI reports the actual source.

The thought field sets first-person intent for both model loops. A local fallback responds while the JEV request is in flight. Keyboard shortcuts work when the text field is unfocused:

| Key | Active thought |
| --- | --- |
| W | Go forward carefully |
| A / D | Turn left / right |
| Q / E | Step left / right |
| S | Stop and hold position |

These keys latch the thought; releasing the key does not clear it. Select Autonomous to release the thought and resume the selected goal. Hold replaces it with a stop instruction.

Camera choices are Arena, Follow, Top, and FPV. The FPV camera is attached to the torso near the face, so it pitches and rolls with the robot. Gemini receives the currently selected camera frame in browser mode. `LIVE + CAMERA` indicates a successful image-conditioned plan.

The movement library includes walking, faster and diagonal travel, turns, sidesteps, crouch, squat, bow, waves, arms up and out, reaching, pointing, nodding, dancing, marching, single-leg kicks, and a small hop. These are experimental movements. Repeated hopping is unstable, and a fall may require Reset pose.

## Enable remote models

```bash
cp .env.example .env
```

Fill in the local `.env` file:

```dotenv
OPENROUTER_API_KEY=your-openrouter-key
GEMINI_API_KEY=your-gemini-key
```

Then launch without `--offline`:

```bash
uv run jev-robot-web
```

Get an [OpenRouter API key](https://openrouter.ai/settings/keys) for JEV and a [Gemini API key](https://aistudio.google.com/apikey) for Gemini. The application loads `.env` from the current working directory or its parent directories. Existing shell variables take precedence. `.env` is gitignored.

| Environment variable | Code default | Purpose |
| --- | --- | --- |
| `OPENROUTER_API_KEY` | Empty | JEV through OpenRouter's Decisions API |
| `GEMINI_API_KEY` | Empty | Gemini through Google's GenAI SDK |
| `JEV_MODEL` | `~typesafe/jev-latest` | Decisions API model identifier |
| `GEMINI_MODEL` | `gemini-3.8-flash` | Gemini model identifier |

Provider availability can change. A model override must support the API used by its loop. JEV uses typed Decisions questions. Gemini uses structured JSON output and, in browser mode, image input.

You can enable either loop alone. Missing credentials or request errors activate that loop's local fallback. The browser shows the configuration, active source, and last error. API calls can incur charges, and the three JEV workers may make concurrent requests throughout an online run.

## Native viewer and headless runs

On macOS, use the supplied launcher:

```bash
./scripts/run-macos --offline
./scripts/run-macos --task "Reach the green target while keeping clear of the red blocks"
```

On other platforms with a desktop rendering backend:

```bash
uv run jev-robot --offline
```

The standalone viewer sends structured state to Gemini. Camera input is part of the browser runtime only. Close the viewer or press `Ctrl-C` to stop.

```bash
uv run jev-robot --headless --offline --seconds 5 --no-realtime
uv run jev-robot --headless --offline --seconds 5 --capture artifacts/g1.png
```

`--seconds` measures simulated time. `--no-realtime` removes the wall-clock pacing delay, so it is useful for headless smoke runs. PNG capture still creates a renderer and therefore needs a graphics backend.

| Option | Default | Purpose |
| --- | --- | --- |
| `--task` | Reach the selected green goal, avoid obstacles and walls, then hold | Planner task |
| `--headless` | Off | Run without the native viewer |
| `--offline` | Off | Disable both model APIs |
| `--seconds N` | `120` | Stop after N seconds of simulated time |
| `--no-realtime` | Off | Run without wall-clock pacing |
| `--capture PATH` | None | Save the final camera frame as PNG |

## Platform notes

macOS requires MuJoCo's `mjpython` launcher for the passive native viewer. `scripts/run-macos` also sets the fallback library path from uv's Python installation before launching. The MuJoCo wheel runs native CPU physics on Apple Silicon and Intel. Rendering uses the platform graphics stack; the project does not use GPU-accelerated dynamics.

The browser runtime also renders frames locally. A Linux desktop needs an available OpenGL context. On a Linux server, choose an appropriate MuJoCo rendering backend and install its system dependencies before starting the browser or requesting a capture. For example, `MUJOCO_GL=egl` selects EGL where the host has a working EGL driver. See [MuJoCo's Python documentation](https://mujoco.readthedocs.io/en/stable/python.html) for the supported backends.

The plain headless smoke command does not create a viewer or renderer. Run it first when a graphics setup fails. The project was developed on macOS. Its local Linux dev container has also passed an offline simulation with EGL rendering.

### GitHub Codespaces

The repository includes a dev container with Python 3.12, uv, Linux graphics libraries, and `MUJOCO_GL=egl`. It installs the locked dependencies and forwards port 8000. After opening the repository in Codespaces, run:

```bash
uv run jev-robot-web --offline --no-open --host 0.0.0.0
```

Open the forwarded port from the Ports panel and keep its visibility private. The browser UI works through that forwarded port; the native MuJoCo viewer is unavailable in Codespaces. Live models require your own credentials, supplied as Codespaces secrets or a local `.env` file.

The Linux dev-container image has been built locally, and its offline simulation passed with EGL rendering. A hosted Codespaces session has not been launched.

## Asset cache

The G1 MJCF and meshes come from `unitreerobotics/unitree_mujoco`. The walking policy comes from the pinned commit documented in [architecture](architecture.md#assets-and-upstream-boundaries). Both are stored under the `jev-robot` user cache selected by `platformdirs`.

To use an existing Unitree MuJoCo checkout, export its root before launching:

```bash
export UNITREE_MUJOCO_ROOT=/path/to/unitree_mujoco
uv run jev-robot-web --offline
```

The checkout must contain `unitree_robots/g1/g1_29dof.xml`. The application writes a derived robot XML with an FPV camera and a generated arena scene beside that file. This override does not change the separate gait-policy cache.

## Troubleshooting

| Symptom | First check |
| --- | --- |
| Startup cannot fetch assets | Confirm Git and network access. `--offline` still downloads uncached Unitree assets. |
| Native viewer fails on macOS | Use `./scripts/run-macos`, which launches through `mjpython`. |
| Browser opens but has no video | Check the terminal for renderer errors and verify the OpenGL backend. |
| Model status shows fallback | Confirm the key, model identifier, provider access, and reported error type. |
| Port 8000 is busy | Add `--port 8080` and open that port. |
| Robot keeps turning after a key press | The thought is latched. Select Autonomous or Hold. |
| Robot falls | Inspect the control trace; use Reset pose to restart its posture. |
| Rates are below their targets | Compare physics speed, rendering rate, model latency, and decision age in the UI. |
