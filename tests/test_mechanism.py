"""Mechanism tests: forces, margins and motion ranges that decide whether the
MMU works, not just whether it prints. Run:  .venv/bin/python -m pytest -q tests
"""

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))

import build_mmu as B  # noqa: E402

FOLLOWER_WINDOW = math.degrees(math.asin(2.0 / B.CAM_NOSE_R)) + B.CAM_NOSE_HALF


def preload_force(compression_extra=0.0):
    od, free, _ = B.IDLER_PRELOAD_SPRING
    released = B.SPRING_GAP + B.SPRING_SEAT + B.FOLLOWER_POCKET
    length = released - B.IDLER_OVERTRAVEL - compression_extra
    return 2 * B.PRELOAD_RATE * (free - length)


def return_force():
    _, free = B.RETURN_SPRING
    length = (B.AXIS_CONTACT - B.IDLER_AXIS_IN_BLOCK + 6) - 31.5
    return 2 * B.RETURN_RATE * (free - length)


def test_grip_force_in_working_band():
    grip = preload_force() - return_force()
    assert 10 <= grip <= 30, f"grip {grip:.1f} N (want 10-30 N on 1.75 mm filament)"


def test_grip_tolerates_filament_diameter():
    """+/-0.05 mm filament changes the spring compression, not a hard stop."""
    lo, hi = preload_force(-0.05), preload_force(+0.05)
    assert abs(hi - lo) / preload_force() < 0.05


def test_gear_can_pull_harder_than_bowden_friction():
    holding = 0.45 / (B.GEAR_R / 1000)                 # N at 45 N.cm
    assert holding > 3 * 15, f"{holding:.0f} N"        # 15 N = worst Bowden drag


def test_servo_torque_margin():
    """Peak torque while the nose ramps onto the follower (at the target the
    contact force passes through the cam axis, so holding torque is ~0)."""
    force = preload_force() + return_force()
    torque = force * B.CAM_NOSE_R / 1000                # upper bound: full nose radius lever
    assert B.SERVO_TORQUE_NM / torque >= 2.5, f"margin {B.SERVO_TORQUE_NM / torque:.1f}x"


def test_servo_angles_inside_usable_range():
    lo, hi = B.SERVO_USABLE
    for a in B.CAM_LOBE_ANGLES + [B.CAM_NEUTRAL]:
        assert lo <= a <= hi, a


def test_lane_angles_have_margin_against_neighbours():
    angles = B.CAM_LOBE_ANGLES + [B.CAM_NEUTRAL]
    gaps = [abs(a - b) for i, a in enumerate(angles) for b in angles[i + 1:]]
    assert min(gaps) - FOLLOWER_WINDOW >= 5, \
        f"closest {min(gaps)} deg vs engagement window {FOLLOWER_WINDOW:.1f} deg"


def test_bowden_has_slack_over_full_z_travel():
    pts = np.array(B.BOWDEN_POINTS, float)
    length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    for dz in (0, -B.Z_TRAVEL):
        inlet = np.array(B.EXTRUDER_INLET, float) + [0, 0, dz]
        straight = float(np.linalg.norm(inlet - np.array(B.MMU_OUTLET)))
        assert length >= 1.1 * straight, f"dz={dz}: {length:.0f} vs {straight:.0f} mm"


def test_idle_lanes_clear_the_turning_gears():
    assert B.CHANNEL_FLOOR - (B.SHAFT_Z + B.GEAR_OD_R) >= 0.15
