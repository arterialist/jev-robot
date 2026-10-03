# Contributing

JEV robot is a simulation experiment with a browser control room. Help make its experiments reproducible. When describing model control, report what the code and recorded runs show.

## Set up a checkout

Fork the repository, clone your fork, and create a branch for the change. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and Python 3.12 or 3.13, then run:

```bash
uv sync --frozen
uv run jev-robot-web --offline
```

The first simulation run downloads the official Unitree model, meshes, and walking policy into your user cache. Those files keep their upstream licenses. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Live model calls are optional. Copy `.env.example` to `.env` and set your own keys when testing JEV or Gemini. Never commit `.env`, credentials, or private provider responses.

## Check a change

Run the offline tests before opening a pull request:

```bash
uv run --frozen python -m unittest discover -s tests -v
uv run --frozen jev-robot --headless --offline --seconds 2 --no-realtime
uv build --no-sources
```

On Linux, install EGL and OpenGL runtime libraries and set `MUJOCO_GL=egl` for headless rendering. On macOS, use `./scripts/run-macos` for the passive MuJoCo viewer. The browser demo runs with the regular Python launcher.

CI runs the offline tests with locked dependencies on Python 3.12 and 3.13 on Linux, plus Python 3.12 on macOS. It also runs a short Linux simulation and saves a rendered frame. The packaging job builds a wheel and source archive, then checks that the wheel contains the browser assets. A separate job builds the development container. The frame and package files are available as workflow artifacts. CI does not call JEV or Gemini.

If you change a controller decision, test the specific failure or behavior. For an interface change, check the controls in a browser and attach a screenshot or recording. Report the model mode, motor mode, launch command, and measurements needed to reproduce the result. Measure live model latency and performance in a live run.

## Submit a pull request

Keep each pull request focused. Explain the problem and the resulting behavior, then describe how you checked the change. Update the README when commands or public behavior change. Include `pyproject.toml` and the resulting `uv.lock` together when changing dependencies.

New GitHub Actions must use a full commit SHA with the release version in a comment. Give workflows only the permissions they need. Package publication and deployment require a separate maintainer decision.

For questions, use [Discussions](https://github.com/arterialist/jev-robot/discussions). Use the issue forms for bugs and feature requests. Report vulnerabilities through the process in [SECURITY.md](SECURITY.md).
