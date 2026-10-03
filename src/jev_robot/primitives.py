from __future__ import annotations

import math

from jev_robot.models import Motion


PRIMITIVE_DESCRIPTIONS = {
    "stand": "Balance in place and hold the current location.",
    "walk_forward": "Walk forward at a steady pace.",
    "walk_backward": "Walk backward slowly while checking rear clearance.",
    "walk_fast": "Walk forward faster through clear space.",
    "diagonal_left": "Walk forward and left together.",
    "diagonal_right": "Walk forward and right together.",
    "turn_left": "Rotate the whole body left in place.",
    "turn_right": "Rotate the whole body right in place.",
    "step_left": "Sidestep left without advancing.",
    "step_right": "Sidestep right without advancing.",
    "crouch": "Bend both knees into a shallow crouch and hold it.",
    "squat": "Repeat a shallow knee bend while keeping both feet planted.",
    "bow": "Lean the torso slightly forward in a bow while keeping balance.",
    "wave": "Wave the right arm to greet someone.",
    "wave_left": "Wave the left arm.",
    "wave_right": "Wave the right arm.",
    "arms_up": "Raise both arms overhead.",
    "arms_out": "Extend both arms sideways.",
    "reach_forward": "Extend both arms forward as if reaching.",
    "point_left": "Point the left arm forward and left.",
    "point_right": "Point the right arm forward and right.",
    "nod": "Nod the upper body gently; the G1 has no neck motor.",
    "dance": "Sway the waist and arms rhythmically while standing.",
    "march_in_place": "Lift alternating knees while staying near the current spot.",
    "kick_left": "Lift and extend the left leg briefly; balance may be lost.",
    "kick_right": "Lift and extend the right leg briefly; balance may be lost.",
    "hop": "Make one small hop, then return to a standing balance.",
    "recover": "Tuck limbs toward a recovery pose; manual Reset may still be needed.",
}


def pose_offsets(action: Motion, elapsed: float) -> dict[str, float]:
    """Joint offsets layered over the policy's live balance targets."""
    sway = math.sin(2 * math.pi * elapsed * 1.1)
    if action == "crouch":
        return _legs(-0.02, 0.20, -0.12)
    if action == "squat":
        return _legs(-0.02, 0.15 + 0.05 * sway, -0.11)
    if action == "bow":
        return {"waist_pitch_joint": 0.10}
    if action in {"wave", "wave_right", "wave_left"}:
        side = "left" if action == "wave_left" else "right"
        direction = 1.0 if side == "left" else -1.0
        return {
            f"{side}_shoulder_pitch_joint": -1.15,
            f"{side}_shoulder_roll_joint": direction * 0.28,
            f"{side}_elbow_joint": 0.32 + 0.17 * sway,
            f"{side}_wrist_roll_joint": direction * 0.35 * sway,
        }
    if action == "arms_up":
        return {
            "left_shoulder_pitch_joint": -1.25,
            "right_shoulder_pitch_joint": -1.25,
            "left_shoulder_roll_joint": 0.18,
            "right_shoulder_roll_joint": -0.18,
        }
    if action == "arms_out":
        return {
            "left_shoulder_roll_joint": 0.85,
            "right_shoulder_roll_joint": -0.85,
            "left_elbow_joint": -0.40,
            "right_elbow_joint": -0.40,
        }
    if action == "reach_forward":
        return {
            "left_shoulder_pitch_joint": -0.80,
            "right_shoulder_pitch_joint": -0.80,
            "left_elbow_joint": -0.40,
            "right_elbow_joint": -0.40,
        }
    if action in {"point_left", "point_right"}:
        side = "left" if action == "point_left" else "right"
        direction = 1.0 if side == "left" else -1.0
        return {
            f"{side}_shoulder_pitch_joint": -0.78,
            f"{side}_shoulder_roll_joint": direction * 0.36,
            f"{side}_elbow_joint": -0.55,
        }
    if action == "nod":
        return {"waist_pitch_joint": 0.16 * sway}
    if action == "dance":
        return {
            "waist_yaw_joint": 0.22 * sway,
            "waist_roll_joint": 0.08 * sway,
            "left_shoulder_roll_joint": 0.40 * sway,
            "right_shoulder_roll_joint": 0.40 * sway,
            "left_elbow_joint": 0.20 * sway,
            "right_elbow_joint": -0.20 * sway,
        }
    if action == "march_in_place":
        left = max(0.0, sway)
        right = max(0.0, -sway)
        return {
            "left_hip_pitch_joint": 0.18 * left,
            "left_knee_joint": 0.30 * left,
            "right_hip_pitch_joint": 0.18 * right,
            "right_knee_joint": 0.30 * right,
        }
    if action in {"kick_left", "kick_right"}:
        side = "left" if action == "kick_left" else "right"
        lift = max(0.0, math.sin(2 * math.pi * elapsed * 0.7))
        return {
            f"{side}_hip_pitch_joint": 0.38 * lift,
            f"{side}_knee_joint": 0.48 * lift,
            f"{side}_ankle_pitch_joint": -0.18 * lift,
        }
    if action == "recover":
        return _legs(-0.02, 0.20, -0.12)
    if action == "hop":
        if elapsed < 0.25:
            return _legs(-0.10, 0.40, -0.12)
        if elapsed < 0.48:
            return _legs(0.15, -0.35, 0.14)
    return {}


def _legs(hip: float, knee: float, ankle: float) -> dict[str, float]:
    return {
        f"{side}_hip_pitch_joint": hip
        for side in ("left", "right")
    } | {
        f"{side}_knee_joint": knee
        for side in ("left", "right")
    } | {
        f"{side}_ankle_pitch_joint": ankle
        for side in ("left", "right")
    }
