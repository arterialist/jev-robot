# Changelog

## 0.1.0 - 2026-10-03

Initial public prototype.

- Simulate Unitree's 29-joint G1 in a MuJoCo arena with movable goals and obstacles.
- Run JEV reflex decisions alongside Gemini planning, with independent local fallbacks.
- Apply continuous travel commands and joint offsets through a local ONNX gait policy and PD controller.
- Steer through the browser control room, inspect measured loop rates, and switch between Arena, Follow, Top, and FPV cameras.
- Run an offline browser demo or headless smoke simulation without model credentials.
- Include setup and architecture docs, real offline screenshots, regression checks, and GitHub workflows.

This release is a simulator prototype. It does not connect to physical robot hardware,
and a successful local demo does not establish live model performance.
