"""Print-readiness tests. Run:  .venv/bin/python -m pytest -q tests

Geometry is measured on the exported STLs in their print orientation
(tests/printability.py). Rebuild the STLs first: cad/build_mmu.py.
"""

import math
import os
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))

import build_mmu as B  # noqa: E402
import check_mmu  # noqa: E402
import printability as P  # noqa: E402

PARTS = [n for n, _ in B.PARTS]

# Per-part minimum wall (mm). Defaults to P.MIN_WALL.
WALL_LIMIT = {
    "mmu_body": 1.0,      # 1.0-1.2 mm wedges beside the open filament groove
    "idler_block": 0.9,   # floor under the 3 mm pin bore
}


def _merger_exempt(p):
    """Knife edges where the four channels meet, and where the angled
    channels enter the rear face, are inherent to a merger."""
    near_merge = abs(p[0] - 55) < 8 and p[1] < 50
    rear_face = p[1] > 98
    return near_merge or rear_face


EXEMPT = {"merger": _merger_exempt}


def test_stls_are_current():
    for name in PARTS:
        assert os.path.exists(os.path.join(P.STL_DIR, f"{name}.stl")), name
    stale = {f[:-4] for f in os.listdir(P.STL_DIR) if f.endswith(".stl")} - set(PARTS)
    assert not stale, f"stale STLs: {stale}"


def test_every_part_has_a_print_orientation():
    assert set(P.PRINT_ORIENT) == set(PARTS)


def test_assembly_audit_passes():
    assert check_mmu.main() == 0


@pytest.mark.parametrize("name", PARTS)
def test_min_wall(name):
    limit = WALL_LIMIT.get(name, P.MIN_WALL)
    _, thin = P.min_wall(P.load(name), samples=12000, limit=limit)
    exempt = EXEMPT.get(name, lambda p: False)
    bad = [p for p in thin if not exempt(p)]
    assert not bad, f"{name}: walls < {limit} mm at {np.round(bad[:5], 1).tolist()}"


@pytest.mark.parametrize("name", [n for n in PARTS if not P.PRINT_ORIENT[n][1]])
def test_prints_without_support(name):
    m = P.load(name)
    area = P.overhang_area(m)
    assert area < 40, f"{name}: {area:.0f} mm^2 of overhang > {P.OVERHANG_DEG} deg"
    long = [b for b in P.bridges(m) if b[0] > P.MAX_BRIDGE]
    assert not long, f"{name}: bridges > {P.MAX_BRIDGE} mm: " \
        f"{[(round(b[0], 1), b[2].round(0).tolist()) for b in long]}"


@pytest.mark.parametrize("name", PARTS)
def test_fits_ender_bed_in_print_orientation(name):
    ext = P.load(name).extents
    assert np.all(ext <= [220, 220, 250]), ext


@pytest.mark.parametrize("what,hole,part", [
    ("idler block x in slot", B.IDLER_SLOT[0], B.IDLER_HALF[0]),
    ("idler block y in slot", B.IDLER_SLOT[1], B.IDLER_HALF[1]),
    ("623 in idler slot (width)", B.BEARING_SLOT[0], B.BEARING_HALF_W),
    ("623 in idler slot (dia)", B.BEARING_SLOT[1], B.BEARING_R),
    ("623 in body window (width)", B.BEARING_WINDOW[0], B.BEARING_HALF_W),
    ("623 in body window (dia)", B.BEARING_WINDOW[1], B.BEARING_R),
    ("MK8 gear length in pocket", B.GEAR_POCKET[0], B.GEAR_HALF_LEN),
    ("MK8 gear OD in pocket", B.GEAR_POCKET[1], B.GEAR_OD_R),
    ("MG996R width in window", B.SERVO_WINDOW[0], B.SERVO_HALF[0]),
    ("MG996R length in window", B.SERVO_WINDOW[1], B.SERVO_HALF[1]),
    ("clamp on 2020 bar", B.CLAMP_CHANNEL_HALF, B.BAR_HALF),
    ("camshaft between towers", B.TOWER_X - 5, 55),
    ("preload spring in its pocket", 2.2, B.IDLER_PRELOAD_SPRING[0] / 2),
])
def test_sliding_fit_clearance(what, hole, part):
    assert hole - part >= 0.2, f"{what}: {hole - part:.2f} mm/side (< 0.2)"


def test_feature_sizes_printable():
    assert 2 * B.BORE_FIL_R >= 1.75 + 0.4, "filament channel too tight"
    assert 2 * B.R_M2 >= 2 * P.NOZZLE + 1.0
    assert 2 * B.R_INSERT >= 4.0, "M3 heat-set insert hole"
    assert 2 * B.TAP_M6_R == pytest.approx(5.0) and 2 * B.TAP_M10_R == pytest.approx(9.0)


def test_merger_bends_are_gentle():
    run = -B.BODY_Y - B.MERGE_Y            # body front -> merge point
    worst = max(math.degrees(math.atan2(abs(x), run)) for x in B.LANE_X)
    assert worst <= 35, f"{worst:.1f} deg"


def test_bom_print_table_matches_cad():
    bom = open(os.path.join(ROOT, "BOM.md")).read()
    listed = set(re.findall(r"^\| `(\w+)`", bom, re.M))
    assert listed == set(PARTS), f"BOM vs CAD: {listed ^ set(PARTS)}"
    assert f"3 × {B.IDLER_PIN_LEN:g}" in bom, "idler pin length in BOM must match the block width"


def test_idler_grip_is_spring_loaded():
    """Grip force must come from a spring, not from rigid interference."""
    od, free, solid = B.IDLER_PRELOAD_SPRING
    released = B.SPRING_GAP + B.SPRING_SEAT + B.FOLLOWER_POCKET
    engaged = released - B.IDLER_OVERTRAVEL
    assert B.IDLER_GAP >= 1.0, "bearing must clear the filament when released"
    assert B.IDLER_OVERTRAVEL >= 1.0, "cam must compress the spring after contact"
    assert free > released > engaged > solid + 1.0


def test_gate_sensor_has_enough_travel():
    travel = B.GATE_LEVER_REST - (B.ZF - B.FIL_R)   # filament pushes the roller down
    assert travel >= 1.5, f"{travel:.2f} mm"
    assert B.GATE_LEVER_REST < B.ZF + B.MERGE_R, "roller must rest inside the channel"


def test_teardrop_is_circle_plus_roof():
    r = 3.0
    bb = B.tear_y(r, 0, 10).bounding_box()
    rc = r + B.HOLE_COMP
    assert bb.size.X == pytest.approx(2 * rc, abs=0.01), "no side horns"
    assert bb.max.Z == pytest.approx(rc * math.sqrt(2), abs=0.01)
    assert bb.min.Z == pytest.approx(-rc, abs=0.01)


def test_drive_and_cam_are_keyed():
    bom = open(os.path.join(ROOT, "BOM.md")).read()
    assert "D-shaft" in bom, "gears need a flat to bite on"
    assert "M3 × 4 heat-set" in bom, "grub screws go into inserts, not PETG"


def test_fit_coupon_is_printable_first():
    assert "fit_coupon" in PARTS
    assert "fit_coupon" in open(os.path.join(ROOT, "PREMORTEM.md")).read()
