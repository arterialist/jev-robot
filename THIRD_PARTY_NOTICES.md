# Third-party materials

The MIT license covers the original JEV robot project code and documentation.
Dependencies, robot descriptions, meshes, and walking policies retain their
own licenses. Unitree and model-provider names identify the software used;
they do not imply endorsement.

## Unitree G1 model and meshes

The application downloads the G1 MJCF and meshes from
[unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco).
That repository uses the
[BSD 3-Clause license](https://github.com/unitreerobotics/unitree_mujoco/blob/main/LICENSE),
with copyright belonging to Unitree Robotics. Downloaded files stay in the
user cache with the upstream license. This repository does not bundle them.
The application writes an additional scene and a copy of the model with a
first-person camera into that cache.

## Unitree G1 walking policy

The ONNX policy and deployment configuration come from
[unitreerobotics/unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab),
pinned in the code to commit `4960b84732b0c2ec593dccbfe963fda1bcd7b1e3`.
The upstream repository includes an
[Apache 2.0 license](https://github.com/unitreerobotics/unitree_rl_lab/blob/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/LICENCE).
These files are downloaded into the user cache and are not bundled here.

## Runtime and interface

MuJoCo, ONNX Runtime, FastAPI, Google's Gen AI SDK, and the other dependencies
listed in `pyproject.toml` are installed separately. See each distribution's
license before redistributing a combined application.

The browser interface requests Barlow Condensed and IBM Plex Mono through
Google Fonts. Their upstream distributions use the SIL Open Font License.
The repository does not bundle font files. System fonts serve as fallbacks.

## Screenshots

The screenshots in `artifacts/` and `docs/assets/` show this application's
MuJoCo simulation and interface. The simulated robot uses Unitree's G1 model.
The public README identifies captures made with local fallback decisions;
they are not evidence of a live JEV or Gemini API session.
