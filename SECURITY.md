# Security

Security fixes target the latest code on `main`. This prototype has no long-term support release or guaranteed response time.

## Report a vulnerability

Use [GitHub's private vulnerability reporting](https://github.com/arterialist/jev-robot/security/advisories/new). Include the affected commit, reproduction steps, impact, and a proposed fix if you have one. Do not post credentials or exploit details in a public issue.

If a credential has been exposed, revoke or rotate it at its provider before continuing the investigation. Removing a key from a later commit does not remove it from Git history.

## Local demo boundaries

The web server binds to `127.0.0.1` by default. Its controls are intended for a local simulation and do not provide authentication. Keep it local unless you add access controls appropriate for the deployment.

JEV and Gemini receive the robot state, task, and active self-thought during live runs. Gemini also receives the rendered camera frame. Use `--offline` when you want both model loops to run locally without model API calls. Offline mode still downloads the official Unitree simulation assets on the first run.

Keep API keys in a local `.env` or shell environment. Do not include them in issues, screenshots, recordings, browser traces, or commits.

This repository controls a simulated G1. Its steering guards and torque limits have not been validated for physical hardware.
