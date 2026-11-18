"""Colour-change simulation as tests (sim/colour_change.py)."""

import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))

import colour_change as S  # noqa: E402

B = S.B


def test_idle_lanes_never_touch_the_shared_gears():
    assert S.idle_lane_clearance() >= 0.15


@pytest.mark.parametrize("material", list(S.MATERIALS))
def test_filament_bends_elastically_onto_the_gear(material):
    _, strain, yield_strain = S.filament_bending(material)
    assert strain <= 0.5 * yield_strain


@pytest.mark.parametrize("material", list(S.MATERIALS))
@pytest.mark.parametrize("lane", range(4))
def test_traction_beats_drag(material, lane):
    g = S.grip_and_drag(material, lane)
    assert g["traction"] >= S.REQUIRED_MARGIN * max(g["load"], g["unload"]), g
    assert g["printing"] * S.REQUIRED_MARGIN <= (g["traction"] + S.STOCK_EXTRUDER_PULL) / 2, g


def test_traction_survives_pessimistic_friction():
    g = S.grip_and_drag("PETG", 0, mu_gear=0.3, mu_ptfe=0.35)
    assert g["traction"] >= 1.2 * max(g["load"], g["unload"]), g


@pytest.mark.parametrize("lane", range(4))
def test_servo_error_tolerance(lane):
    assert S.single_lane_band(lane) >= S.SERVO_ERROR


def test_neutral_releases_everything():
    assert S.lanes_engaged(B.CAM_NEUTRAL) == []
    assert S.neutral_band() >= S.SERVO_ERROR


@pytest.mark.parametrize("lane", range(4))
def test_tip_can_only_go_forward_at_the_merge(lane):
    to_outlet, to_other = S.junction_angles(lane)
    assert to_outlet <= 35 and to_other >= 90


@pytest.mark.parametrize("pair", [(a, b) for a in range(4) for b in range(4) if a != b])
def test_full_colour_change(pair):
    st = S.simulate_change(*pair)
    assert not st.errors, st.errors
    assert st.t < 90, f"{st.t:.0f} s"


def test_landmarks_are_in_order():
    for lane in range(4):
        lm = S.landmarks(lane)
        order = ["park", "merge", "gate", "outlet", "ext_gear", "nozzle"]
        assert all(lm[a] < lm[b] for a, b in zip(order, order[1:])), lm
