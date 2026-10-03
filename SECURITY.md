# Security

Security fixes target the latest code on `main`. This prototype has no long-term support release or guaranteed response time.

## Report a vulnerability

Use [GitHub's private vulnerability reporting](https://github.com/arterialist/jev-robot/security/advisories/new). Include the affected commit, reproduction steps, impact, and a proposed fix if you have one. Do not post credentials or exploit details in a public issue.

If a credential has been exposed, revoke or rotate it at its provider before continuing the investigation. Removing a key from a later commit does not remove it from Git history.

## Local demo boundaries

The web server binds to `127.0.0.1` by default and has no authentication. Keep it local unless you add access controls.

JEV and Gemini receive the robot state, task, and active self-thought during live runs. In browser mode, Gemini also receives the rendered camera frame. Use `--offline` to run both loops with local fallback decisions and no model API calls. The first offline run still downloads uncached Unitree assets.

Keep API keys in a local `.env` or shell environment. Do not include them in issues, screenshots, recordings, browser traces, or commits.

This repository controls a simulated G1. Its steering guards and torque limits have not been validated for physical hardware.
