#!/usr/bin/env python3
"""
Ender-MMU — parametric build123d source, v0.4 (Klipper + Happy Hare,
Type-A servo selector; see PLAN.md and cad/README_CAD.md).

Mechanism (PLAN.md §2):
  * 4 lanes run front-to-back (along Y) through the body at height ZF.
  * ONE NEMA 17 on the +X end turns a 5 mm drive shaft along X carrying
    FOUR MK8 drive gears — one under each lane (PicoMMU / ERCF style).
  * Each lane has a spring-lifted idler block (623 bearing) above its gear.
  * A horizontal camshaft along X (6 mm rod, MG996R on the -X end) has one
    lobe per lane at phases 0/45/90/135 deg. At servo angle a_i only lane
    i's lobe points down and presses its idler; 180 deg = all released.
  * In front, a merger joins the 4 lanes into one outlet (PC4-M10), with
    a gate sensor (microswitch) on the outlet channel. No cutter: Happy
    Hare forms the tip.
  * Control: Klipper on the Pi, a Raspberry Pi Pico as the MMU MCU.

Coordinates: body-local mm, origin at the body bottom centre, Z up,
front = -Y. World = body-local + (0, 0, Z0). Z0 = 473 (clamp top).

Run:  .venv/bin/python cad/build_mmu.py
Out:  stl/<part>.stl  +  viewer/manifest.json
"""

import json
import math
import os

from build123d import (Box, Cylinder, Plane, Polygon, Pos, Rot, Solid,
                       Vector, export_stl, extrude)

# ---------------------------------------------------------------- parameters
LANE_X = [-42, -14, 14, 42]          # lane 1..4 (T0..T3), left to right
BODY_X, BODY_Y, BODY_Z = 80, 35, 50  # half-X, half-Y, height
ZF = 30.0                            # filament axis height
FIL_R = 0.875                        # 1.75 mm filament
BORE_FIL_R = 1.1                     # 2.2 mm filament channel
MERGE_R = 1.25                       # 2.5 mm convergence (ream with a 2.5 mm drill)
BORE_PTFE_R = 2.05                   # 4.1 mm PTFE seat
TAP_M6_R = 2.5                       # 5.0 mm: tap M6x1 for PC4-M6
TAP_M10_R = 4.5                      # 9.0 mm: tap M10x1 for PC4-M10
HOLE_COMP = 0.1                      # added to horizontal hole radii; tune with fit_coupon
GEAR_R = 5.25                        # MK8 drive gear effective radius (93 e-steps)
GEAR_OD_R = 5.5
# Idle filament rests on the channel floor and must NOT touch its gear (the
# shaft turns for every lane). The engaged idler bends it down onto the gear.
CHANNEL_FLOOR = ZF - (BORE_FIL_R + HOLE_COMP)
GEAR_CLEAR = 0.2                     # tooth tips below the channel floor
SHAFT_Z = CHANNEL_FLOOR - GEAR_CLEAR - GEAR_OD_R
FIL_REST_Z = CHANNEL_FLOOR + FIL_R   # idle filament centre
FIL_GRIP_Z = SHAFT_Z + GEAR_R + FIL_R  # gripped filament centre
FIL_DEFLECT = FIL_REST_Z - FIL_GRIP_Z
FREE_SPAN = 24                       # unsupported filament length over the gear
BEARING_R = 5.0                      # 623ZZ idler, 3x10x4
# Spring-loaded idler: cam -> follower -> preload spring -> idler block.
IDLER_CLEAR = 1.0                    # released bearing above the idle filament
IDLER_GAP = IDLER_CLEAR + FIL_DEFLECT  # released bearing above the grip position
CAM_BASE_R = 7.0
CAM_NOSE_R = 10.0
CAM_LIFT = CAM_NOSE_R - CAM_BASE_R
IDLER_OVERTRAVEL = CAM_LIFT - IDLER_GAP  # preload spring compression when engaged
IDLER_PRELOAD_SPRING = (4.0, 10.0, 3.5)  # OD, free length, solid length (mm)
PRELOAD_RATE = 4.0                   # N/mm, firm preload springs (2 per idler)
RETURN_SPRING = (4.0, 10.0)          # OD, free length — light, lifts the block
RETURN_RATE = 0.3                    # N/mm
CAM_LOBE_ANGLES = [40, 75, 110, 145]  # servo angle that engages lane 1..4
CAM_NEUTRAL = 0                      # all idlers released while printing
SERVO_USABLE = (0, 160)              # stay inside what every MG996R reaches
SERVO_TORQUE_NM = 0.9                # MG996R ~9 kg.cm at 5-6 V
CAM_NOSE_HALF = 15                   # nose half-arc, deg
AXIS_CONTACT = FIL_GRIP_Z + FIL_R + BEARING_R  # idler axis with the filament on the gear
AXIS_REL = AXIS_CONTACT + IDLER_GAP         # idler axis, released
IDLER_AXIS_IN_BLOCK = 3.0            # block bottom -> bearing axis
IDLER_BLOCK_H = IDLER_AXIS_IN_BLOCK + 7.0   # 1.4 mm roof over the bearing slot
SPRING_SEAT = 1.5                    # preload seat depth in the block top
SPRING_GAP = 2.0                     # block top -> follower bottom, released
SPRING_Y = 9.0                       # preload + return springs and limit screws at y=+/-9
FOLLOWER_H, FOLLOWER_POCKET, RIDGE_H = 7.0, 4.5, 2.5
FOLLOWER_Z_REL = AXIS_REL - IDLER_AXIS_IN_BLOCK + IDLER_BLOCK_H + SPRING_GAP
CAM_Z = FOLLOWER_Z_REL + FOLLOWER_H + RIDGE_H + CAM_BASE_R
TOWER_X = 61                         # camshaft towers at +/-61
NEMA_FACE, NEMA_HOLE = 42, 31
PI_HOLE_SPACING = (58, 49)
MERGER_LEN = 100                     # body front -> outlet
MERGE_Y = -100                       # lanes meet here (body-local)
MERGE_PTFE_STOP = 15                 # PTFE in each lane stops this far before the merge
GATE_SENSOR_Y = -106                 # gate microswitch lever on the outlet channel
GATE_LEVER_REST = ZF + 1.0           # roller tip height with no filament (shim to set)
GATE_CHAMBER_TOP = ZF + 1.8
CARRIER_Y = -85                      # Pi + Pico carrier centre on the merger
CARRIER_HOLES = (46, 41)             # carrier -> merger insert offsets
MERGER_BOLT_X = 48                   # merger -> body bolts
SERVO_SPLINE_OFFSET = 10.0           # MG996R spline from case centre, along its length

# Fits (half-sizes, mm). Bought-part sizes are catalogue values.
GEAR_HALF_LEN = 5.5                  # MK8 gear is 11 long
GEAR_POCKET = (6.3, 6.0)             # x, y half-size of the gear pocket
IDLER_HALF = (6.0, 13.0)             # idler block / follower x, y
IDLER_SLOT = (6.3, 13.3)             # body slot x, y
BEARING_HALF_W = 2.0                 # 623 is 4 wide
BEARING_SLOT = (2.3, 5.6)            # in the idler block
BEARING_WINDOW = (2.6, 5.8)          # in the body
SERVO_HALF = (9.85, 20.35)           # MG996R case 19.7 x 40.7 (z, y as mounted)
SERVO_WINDOW = (10.25, 20.75)
BAR_HALF, CLAMP_CHANNEL_HALF = 10.0, 10.3
IDLER_PIN_LEN = 2 * IDLER_HALF[0]
SHAFT = "5 mm D-shaft"               # set screws bite on the flat

# Spool rack behind the printer (desk level). Spool axes along X.
SPOOL_DIA, SPOOL_W_MAX = 200, 75
ROLLER_LEN, ROLLER_R, ROLLER_Y, ROLLER_Z = 76, 14, 40, 30
CRADLE_INNER = ROLLER_LEN / 2 + 1    # wall inner face, half-span
CRADLE_WALL = 6
GUIDE_Y, GUIDE_Z = -79, 40           # filament guide (PC4-M6) in cradle coords
GUIDE_DIR = (0.0, -0.4472, 0.8944)   # guide points up toward the printer: less PTFE turning
CRADLE_X = [-150, -50, 50, 150]      # one cradle per lane
CRADLE_Y = 340                       # clear of the bed's rear travel
SPOOL_Z = ROLLER_Z + math.sqrt((SPOOL_DIA / 2 + ROLLER_R) ** 2 - ROLLER_Y ** 2)

# Stock extruder inlet (left end of the X gantry, gantry shown at z=300).
EXTRUDER_INLET = (-150, -10, 352)
Z_TRAVEL = 250                       # the extruder rides the gantry through this
MIN_PTFE_BEND_R = 40

R_M2, R_M25, R_M3, R_M4, R_M5 = 1.1, 1.35, 1.7, 2.2, 2.75
R_INSERT = 2.0                       # M3 heat-set insert, 4.0 mm hole
R_TAP_M3 = 1.25                      # 2.5 mm: M3 thread-forming into PETG

TOPBAR_TOP_Z = 465
CLAMP_H, CLAMP_CHANNEL = 28, 20
Z0 = TOPBAR_TOP_Z - CLAMP_CHANNEL + CLAMP_H   # body bottom, world = 473
CLAMP_X = 55

BASE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(BASE, "..", "05-manufacturing", "stl")
MANIFEST = os.path.join(BASE, "..", "06-viewer", "viewer", "manifest.json")


# ---------------------------------------------------------------- helpers
def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_between(r, p0, p1):
    a, b = Vector(*p0), Vector(*p1)
    return Solid.make_cylinder(r, (b - a).length, Plane(origin=a, z_dir=b - a))


def tear(r, p0, p1, up=1):
    """Horizontal hole p0 -> p1 with a 45 deg roof (teardrop), so it
    prints without support. up=-1 for parts printed upside down."""
    a, b = Vector(*p0), Vector(*p1)
    r = r + HOLE_COMP
    length = (b - a).length
    pl = Plane(origin=a, x_dir=(0, 0, up), z_dir=b - a)
    # Keep only the diamond above the tangent chord (x > r/sqrt2), else its
    # side corners stick out past the circle.
    c = r / math.sqrt(2)
    roof = (Pos(0, 0, length / 2) * Rot(0, 0, 45) * Box(2 * r, 2 * r, length)) & \
        (Pos(c + r, 0, length / 2) * Box(2 * r, 4 * r, length))
    return Solid.make_cylinder(r, length, pl) + pl * roof


def tear_x(r, x0, x1, y=0.0, z=0.0, up=1):
    return tear(r, (x0, y, z), (x1, y, z), up)


def tear_y(r, y0, y1, x=0.0, z=0.0, up=1):
    return tear(r, (x, y0, z), (x, y1, z), up)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(
        abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cbox(x0, x1, y0, y1, z0, z1, c=1.0):
    """Box with all edges chamfered — chamfer a plain box before any
    boolean, so OCCT never has to chamfer feature edges."""
    b = box(x0, x1, y0, y1, z0, z1)
    return b.chamfer(c, None, b.edges())


def hex_pocket(x, y, z0, af, depth):
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a)))
           for a in (0, 60, 120, 180, 240, 300)]
    return Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=depth)


# ---------------------------------------------------------------- parts
def mmu_body():
    """Drive block. Lanes along Y at ZF; drive shaft along X at SHAFT_Z
    with a gear pocket under each lane; idler slot above each gear; two
    camshaft towers on top; NEMA 17 on the +X face; servo mount on -X."""
    p = cbox(-BODY_X, BODY_X, -BODY_Y, BODY_Y, 0, BODY_Z, 1.5)
    for tx in (-TOWER_X, TOWER_X):
        p += cbox(tx - 5, tx + 5, -8, 8, BODY_Z - 1, CAM_Z + 8, 1.0)
        p -= tear_x(3.15, tx - 6, tx + 6, z=CAM_Z)         # 6 mm cam rod
    for x in LANE_X:
        p -= tear_y(BORE_FIL_R, -BODY_Y - 1, BODY_Y + 1, x=x, z=ZF)
        p -= tear_y(BORE_PTFE_R, 15, BODY_Y + 1, x=x, z=ZF)  # inlet PTFE
        p -= tear_y(TAP_M6_R, BODY_Y - 7, BODY_Y + 1, x=x, z=ZF)  # PC4-M6
        p -= box(x - GEAR_POCKET[0], x + GEAR_POCKET[0], -GEAR_POCKET[1], GEAR_POCKET[1],
                 SHAFT_Z - 7, 31.5)                         # gear pocket
        p -= box(x - BEARING_WINDOW[0], x + BEARING_WINDOW[0],
                 -BEARING_WINDOW[1], BEARING_WINDOW[1], ZF - 1, 31.5)
        p -= box(x - IDLER_SLOT[0], x + IDLER_SLOT[0], -IDLER_SLOT[1], IDLER_SLOT[1],
                 31.5, BODY_Z + 1)                          # idler slot
        p -= box(x - BORE_FIL_R - HOLE_COMP, x + BORE_FIL_R + HOLE_COMP,
                 -IDLER_SLOT[1], IDLER_SLOT[1], ZF, 31.5)   # open groove, no thin roof
        p -= box(x - BORE_FIL_R - HOLE_COMP, x + BORE_FIL_R + HOLE_COMP,
                 -FREE_SPAN / 2, FREE_SPAN / 2, SHAFT_Z, ZF)  # free span over the gear
    # Drive shaft, 625 bearing at -X end, coupler cavity + 625 at +X end.
    p -= tear_x(2.6, -BODY_X - 1, 57, z=SHAFT_Z)
    p -= tear_x(8.05, -BODY_X - 1, -BODY_X + 5, z=SHAFT_Z)
    p -= tear_x(8.05, 51, 56, z=SHAFT_Z)
    p -= tear_x(10.5, 56, BODY_X + 1, z=SHAFT_Z)
    p -= cyl_x(11.2, BODY_X - 2, BODY_X + 1, z=SHAFT_Z)    # NEMA pilot
    for dy in (-NEMA_HOLE / 2, NEMA_HOLE / 2):
        for dz in (-NEMA_HOLE / 2, NEMA_HOLE / 2):
            p -= tear_x(R_INSERT, BODY_X - 8, BODY_X + 1, y=dy, z=SHAFT_Z + dz)
    # Inserts: servo mount (-X face), merger (front face), clamps (bottom).
    for sy in (-18, 18):
        p -= tear_x(R_INSERT, -BODY_X - 1, -BODY_X + 8, y=sy, z=10)
    for sx in (-MERGER_BOLT_X, MERGER_BOLT_X):
        p -= tear_y(R_INSERT, -BODY_Y - 1, -BODY_Y + 8, x=sx, z=26)
    for sx in (-CLAMP_X, CLAMP_X):
        for sy in (-24, 24):
            p -= cyl_z(R_INSERT, -1, 8, x=sx, y=sy)
    return p


def idler_block():
    """Lower half of the spring-loaded idler: 623 bearing on a 3 mm pin
    (bearing protrudes 2 mm below). Return springs underneath lift it;
    preload springs on top push it down when the follower is pressed.
    Two M3 x 12 limit screws (thread-forming) tie it to the follower."""
    hx, hy = IDLER_HALF
    p = cbox(-hx, hx, -hy, hy, 0, IDLER_BLOCK_H, 0.6)
    p -= box(-BEARING_SLOT[0], BEARING_SLOT[0], -BEARING_SLOT[1], BEARING_SLOT[1],
             -1, IDLER_AXIS_IN_BLOCK + 5.6)
    p -= cyl_x(1.5, -7, 7, z=IDLER_AXIS_IN_BLOCK)
    for sy in (-SPRING_Y, SPRING_Y):
        p -= cyl_z(2.1, -1, 6, y=sy)                                   # return spring
        p -= cyl_z(2.2, IDLER_BLOCK_H - SPRING_SEAT, IDLER_BLOCK_H + 1, y=sy)  # preload seat
        p -= cyl_z(R_TAP_M3, 5, IDLER_BLOCK_H, y=sy)                   # limit screw
    return p


def idler_follower():
    """Upper half: the cam presses its ridge; it compresses the preload
    springs onto the block once the bearing touches the filament. The
    limit screws (heads on top) hold the springs pre-compressed."""
    hx, hy = IDLER_HALF
    p = cbox(-hx, hx, -hy, hy, 0, FOLLOWER_H, 0.6)
    p += box(-hx, hx, -2, 2, FOLLOWER_H - 0.1, FOLLOWER_H + RIDGE_H)
    for sy in (-SPRING_Y, SPRING_Y):
        p -= cyl_z(2.2, -1, FOLLOWER_POCKET, y=sy)                     # spring pocket
        p -= cyl_z(R_M3, -1, FOLLOWER_H + 1, y=sy)                     # screw clearance
    return p


def selector_camshaft():
    """Printed cam sleeve on the 6 mm rod (origin on the rod axis, built
    at servo angle 0). Lane i's nose points down at servo angle a_i."""
    p = cyl_x(5, -55, 55)
    for x, a in zip(LANE_X, CAM_LOBE_ANGLES):
        p += cyl_x(CAM_BASE_R, x - 4, x + 4)
        w = 2 * CAM_NOSE_R * math.sin(math.radians(CAM_NOSE_HALF))
        nose = (Pos(x, 0, 0) * Rot(-a, 0, 0)
                * Pos(0, 0, -(4 + CAM_NOSE_R) / 2) * Box(8, w, CAM_NOSE_R - 4))
        p += nose & cyl_x(CAM_NOSE_R, x - 4, x + 4)
    p -= cyl_x(3.1, -56, 56)
    for sx in (-28, 28):
        p += cyl_x(8, sx - 4, sx + 4)
        p -= cyl_x(3.1, sx - 5, sx + 5)
        p -= cyl_z(R_INSERT, 4.5, 9, x=sx)  # M3 x 4 heat-set insert
        p -= cyl_z(R_M3, 0, 5, x=sx)        # grub reaches the rod flat
    return p


def servo_coupler():
    """MG996R horn -> 6 mm cam rod. Origin on the axis at the horn face."""
    p = cyl_x(9.5, 0, 16)
    p -= cyl_x(3.1, 6, 17)
    for a in (45, 135, 225, 315):
        p -= cyl_x(R_M2, -1, 6, y=7 * math.cos(math.radians(a)),
                   z=7 * math.sin(math.radians(a)))
    p -= cyl_z(R_INSERT, 5.5, 10, x=11)     # M3 x 4 heat-set insert
    p -= cyl_z(R_M3, 0, 6, x=11)            # grub reaches the rod flat
    return p


def servo_mount():
    """Plate on the body -X face. The MG996R lies with its length along Y
    so the mount stays low; its spline (SERVO_SPLINE_OFFSET off the case
    centre) sits on the camshaft axis. Verify on the servo in hand (G7)."""
    x0, x1 = -BODY_X - 20, -BODY_X
    cy = -SERVO_SPLINE_OFFSET                          # case centre
    p = cbox(x0, x1, -42, 24, 0, CAM_Z + 14, 1.0)
    p -= box(x0 - 1, x1 + 1, cy - SERVO_WINDOW[1], cy + SERVO_WINDOW[1],
             CAM_Z - SERVO_WINDOW[0], CAM_Z + SERVO_WINDOW[0])
    for sy in (cy - 24.75, cy + 24.75):
        for sz in (-5, 5):
            p -= cyl_x(R_TAP_M3, x0 - 1, x1 + 1, y=sy, z=CAM_Z + sz)
    for sy in (-18, 18):
        p -= cyl_x(R_M3, x0 - 1, x1 + 1, y=sy, z=10)
        p -= cyl_x(3.1, x0 - 1, x0 + 10, y=sy, z=10)   # head counterbore
    return p


def merger():
    """4 lanes -> 1 outlet. Each lane is PTFE-lined (4.1 mm bore, tube
    pushed in from the rear face) until MERGE_PTFE_STOP before the merge,
    then a 2.5 mm channel to the outlet and the PC4-M10. A KW11 with a
    roller lever mounts from below; the roller rests in a widened part of
    the outlet channel and the filament pushes it down ~1.9 mm."""
    y0, y1 = -BODY_Y, -BODY_Y - MERGER_LEN
    p = cbox(-55, 55, y1, y0, 20, 42, 1.0)
    merge = Vector(0, MERGE_Y, ZF)
    for x in LANE_X:
        start = Vector(x, y0 + 0.5, ZF)
        stop = merge + (start - merge).normalized() * MERGE_PTFE_STOP
        p -= tear(MERGE_R, start, merge)
        p -= tear(BORE_PTFE_R, start, stop)
    p -= tear_y(MERGE_R, y1 - 1, MERGE_Y + 1, z=ZF)
    p -= tear_y(BORE_PTFE_R, y1 - 1, y1 + 23, z=ZF)      # PTFE seat
    p -= tear_y(TAP_M10_R, y1 - 1, y1 + 10, z=ZF)        # PC4-M10
    for sx in (-MERGER_BOLT_X, MERGER_BOLT_X):
        p -= box(sx - 4.5, sx + 4.5, y0 - 12, y0 - 6, 22, 43)  # bolt pocket
        p -= tear_y(R_M3, y0 - 7, y0 + 0.5, x=sx, z=26)
    gy = GATE_SENSOR_Y
    p -= box(-10, 10, gy - 4, gy + 3, 19, 25)              # KW11 pocket, from below
    p -= box(-1.5, 1.5, gy - 3.5, gy + 2.5, 24, ZF + MERGE_R)  # roller chamber
    for sx in (-4.75, 4.75):
        p -= cyl_z(R_M2, 24, 29, x=sx, y=gy - 0.5)         # M2 thread-forming
    hx, hy = CARRIER_HOLES
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            p -= cyl_z(R_INSERT, 36, 43, x=sx, y=CARRIER_Y + sy)
    return p


def fit_coupon():
    """Print first, same settings as the real parts: every critical
    horizontal hole. Adjust HOLE_COMP until the bought parts fit."""
    p = cbox(-40, 40, -6, 6, 0, 24, 0.8)
    for x, r in [(-28, 8.05), (-12, 2.6), (-2, 3.15), (7, R_INSERT),
                 (16, TAP_M6_R), (26, TAP_M10_R)]:
        p -= tear_y(r, -7, 7, x=x, z=10)
    p -= tear_y(BORE_PTFE_R, -7, 7, x=35, z=18)
    p -= tear_y(BORE_FIL_R, -7, 7, x=35, z=6)
    return p


def mcu_carrier():
    """MMU electronics bay floor, on top of the merger: Pico (Klipper MCU),
    a 40x60 perfboard with the TMC2209 + caps + JST headers, and the 6 V
    servo buck (zip-tied). The Pi + display live with the printer, not
    here. 4x M3 x 40 go through the cover and this plate into the merger."""
    p = cbox(-52.5, 52.5, -47.5, 47.5, 0, 4, 1.0)
    for sx in (-23.5, 23.5):                          # Pico holes 47 x 11.4
        for sy in (-5.7, 5.7):
            p += cyl_z(2.5, 3.9, 9, x=-24 + sx, y=26 + sy)
            p -= cyl_z(0.9, -1, 10, x=-24 + sx, y=26 + sy)
    for sx in (-17.5, 17.5):                          # perfboard holes 35 x 55
        for sy in (-27.5, 27.5):
            p += cyl_z(3.0, 3.9, 9, x=22 + sx, y=8 + sy)
            p -= cyl_z(1.1, -1, 10, x=22 + sx, y=8 + sy)
    for sx in (-14, 14):                              # buck: zip-tie slots
        p -= box(-24 + sx - 1.5, -24 + sx + 1.5, -24, -16, -1, 5)
    hx, hy = CARRIER_HOLES
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            p -= cyl_z(R_M3, -1, 5, x=sx, y=sy)
    return p


def electronics_cover():
    """Cover over the MMU electronics (origin at its top, printed top-down).
    Vents over the driver, cable exits toward the body (+Y) and -X."""
    h = 30
    p = cbox(-52.5, 52.5, -47.5, 47.5, -h, 0, 1.0)
    p -= box(-50.1, 50.1, -45.1, 45.1, -h - 1, -2)
    hx, hy = CARRIER_HOLES
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            p += cyl_z(4.0, -h, -1, x=sx, y=sy)
            p -= cyl_z(R_M3, -h - 1, 1, x=sx, y=sy)
    for i in range(5):                                # vents over the TMC2209
        p -= box(12 + 7 * i, 15 + 7 * i, -10, 26, -3, 1)
    p -= box(-20, 20, 44, 48, -h - 1, -h + 10)        # cables to the body
    p -= box(-53, -49, 10, 30, -h - 1, -h + 8)        # USB to the Pi
    return p


def spool_cradle():
    """One spool on two bearing rollers, behind the printer. The guide post
    in front takes a PC4-M6 (tapped) for the PTFE to the MMU inlet; the
    flared entry faces the spool."""
    wi, wt = CRADLE_INNER, CRADLE_WALL
    p = cbox(-wi - wt, wi + wt, GUIDE_Y - 6, ROLLER_Y + 15, 0, 4, 1.0)
    for s in (-1, 1):
        x0, x1 = (wi, wi + wt) if s > 0 else (-wi - wt, -wi)
        p += cbox(x0, x1, -ROLLER_Y - 15, ROLLER_Y + 15, 0, ROLLER_Z + 14, 1.0)
        for ry in (-ROLLER_Y, ROLLER_Y):
            p -= tear_x(4.1, x0 - 1, x1 + 1, y=ry, z=ROLLER_Z)   # 8 mm rod
    p += cbox(-8, 8, GUIDE_Y - 16, GUIDE_Y + 9, 0, GUIDE_Z + 14, 1.0)
    c, d = Vector(0, GUIDE_Y, GUIDE_Z), Vector(*GUIDE_DIR)
    p -= cyl_between(BORE_FIL_R + HOLE_COMP, c - d * 40, c + d * 40)   # 63 deg: no support
    top = (GUIDE_Z + 14 - GUIDE_Z) / GUIDE_DIR[2]
    p -= cyl_between(TAP_M6_R, c + d * (top - 7), c + d * (top + 5))  # PC4-M6, tap
    p -= Solid.make_cone(1.2, 4.5, 6, Plane(origin=c - d * 12.5, z_dir=-d))  # entry flare
    return p


def spool_roller():
    """Roller tube; a 608 is pressed flush into each end and runs on a
    fixed 8 mm rod. Straight bore = no unsupported ceiling."""
    p = cyl_z(ROLLER_R, 0, ROLLER_LEN)
    p -= cyl_z(11.0, -1, ROLLER_LEN + 1)
    return p


def topbar_clamp():
    """Saddle over the 2020 top bar. M5 + T-nut down into the bar's top
    slot (head recessed under the body), M5 into each side slot; the
    body bolts down onto the wings with 2x M3 from below."""
    p = cbox(-15, 15, -30, 30, 0, CLAMP_H, 1.0)
    p -= box(-16, 16, -CLAMP_CHANNEL_HALF, CLAMP_CHANNEL_HALF, -1, CLAMP_CHANNEL)
    for s in (-1, 1):
        p -= box(-16, 16, s * 15, s * 31, -1, CLAMP_CHANNEL)
        p -= tear_y(R_M5, s * 9, s * 16, z=10, up=-1)  # printed upside down
        p -= cyl_z(R_M3, CLAMP_CHANNEL - 1, CLAMP_H + 1, y=s * 24)
    p -= cyl_z(R_M5, CLAMP_CHANNEL - 1, CLAMP_H + 1)
    p -= cyl_z(4.6, CLAMP_H - 5, CLAMP_H + 1)
    return p


PARTS = [
    ("mmu_body", mmu_body),
    ("idler_block", idler_block),
    ("idler_follower", idler_follower),
    ("selector_camshaft", selector_camshaft),
    ("servo_coupler", servo_coupler),
    ("servo_mount", servo_mount),
    ("merger", merger),
    ("mcu_carrier", mcu_carrier),
    ("electronics_cover", electronics_cover),
    ("topbar_clamp", topbar_clamp),
    ("spool_cradle", spool_cradle),
    ("spool_roller", spool_roller),
    ("fit_coupon", fit_coupon),
]

# ---------------------------------------------------------------- assembly
# pos = world position of the part's local origin (mm), rot = Euler deg
# (XYZ), burst = explode offset at 100 %, qty = printed copies per STL.
_idler_z = Z0 + AXIS_REL - IDLER_AXIS_IN_BLOCK
ASSEMBLY = [
    ("mmu_body", (0, 0, Z0), (0, 0, 0), (0, 0, 0), 1,
     "MMU body — 4 lanes, drive shaft + 4 gear pockets, idler slots, cam towers"),
    *[("idler_block", (x, 0, _idler_z), (0, 0, 0), (0, 0, 45), 4,
       f"Idler block ×4 — 623 bearing, return springs (lane {i + 1})")
      for i, x in enumerate(LANE_X)],
    *[("idler_follower", (x, 0, Z0 + FOLLOWER_Z_REL), (0, 0, 0), (0, 0, 65), 4,
       f"Idler follower ×4 — cam ridge, preload springs, 2× M3 limit screws (lane {i + 1})")
      for i, x in enumerate(LANE_X)],
    ("selector_camshaft", (0, 0, Z0 + CAM_Z), (CAM_NEUTRAL, 0, 0), (0, 0, 80), 1,
     "Selector camshaft — lobes at 40/75/110/145°, shown at 0° (all released)"),
    ("servo_coupler", (-84, 0, Z0 + CAM_Z), (0, 0, 0), (-40, 0, 80), 1,
     "Servo coupler — MG996R horn to 6 mm cam rod"),
    ("servo_mount", (0, 0, Z0), (0, 0, 0), (-60, 0, 0), 1,
     "Servo mount — MG996R spline on the camshaft axis"),
    ("merger", (0, 0, Z0), (0, 0, 0), (0, -60, 0), 1,
     "Merger — 4 lanes to 1 outlet (PC4-M10)"),
    ("mcu_carrier", (0, CARRIER_Y, Z0 + 42), (0, 0, 0), (0, -60, 60), 1,
     "MMU electronics plate — Pico (Klipper MCU), TMC2209 perfboard, 6 V servo buck"),
    ("electronics_cover", (0, CARRIER_Y, Z0 + 76), (0, 0, 0), (0, -60, 110), 1,
     "Electronics cover — vents over the driver, cable exits"),
    ("topbar_clamp", (-CLAMP_X, 0, Z0 - CLAMP_H), (0, 0, 0), (0, 0, -50), 2,
     "Top-bar clamp ×2 — saddle on the 2020 top bar (left)"),
    ("topbar_clamp", (CLAMP_X, 0, Z0 - CLAMP_H), (0, 0, 0), (0, 0, -50), 2,
     "Top-bar clamp ×2 — saddle on the 2020 top bar (right)"),
    *[("spool_cradle", (cx, CRADLE_Y, 0), (0, 0, 0), (0, 60, 0), 4,
       f"Spool cradle ×4 — lane {i + 1}, rollers + PTFE guide") for i, cx in enumerate(CRADLE_X)],
    *[("spool_roller", (cx - ROLLER_LEN / 2, CRADLE_Y + ry, ROLLER_Z), (0, 90, 0), (0, 60, 30), 8,
       "Spool roller ×8 — 2× 608 on an 8 mm rod")
      for cx in CRADLE_X for ry in (-ROLLER_Y, ROLLER_Y)],
]

FRAME = [
    ("top_bar_2020", (330, 20, 20), (0, 0, 455), "Top bar — 2020 × 330"),
    ("z_upright_left", (20, 40, 420), (-155, 0, 235), "Z upright L — 2040 × 420"),
    ("z_upright_right", (20, 40, 420), (155, 0, 235), "Z upright R — 2040 × 420"),
    ("x_beam_2020", (345, 20, 20), (0, -30, 300), "X beam — 2020 × 345 (moves in Z)"),
    ("base_outline", (440, 410, 25), (0, 0, 12.5), "Base outline — 440 × 410"),
    ("bed_220", (220, 220, 3), (0, 0, 29.5), "Bed — 220 × 220"),
]


def _b(name, size, pos, color, label):
    return {"name": name, "kind": "box", "size": list(size),
            "pos": [round(v, 3) for v in pos], "color": color, "label": label}


def _c(name, r, length, pos, axis, color, label):
    """Cylinder (round bought part), axis 'x' | 'y' | 'z'."""
    return {"name": name, "kind": "cyl", "r": r, "len": length, "axis": axis,
            "pos": [round(v, 3) for v in pos], "color": color, "label": label}


def _t(name, points, color, label, r=2.0):
    """PTFE tube through these points (Catmull-Rom)."""
    return {"name": name, "kind": "tube", "r": r, "points": [list(p) for p in points],
            "color": color, "label": label}


LANE_COLORS = ["#e53935", "#43a047", "#1e88e5", "#fdd835"]
MMU_OUTLET = (0, -BODY_Y - MERGER_LEN - 10, Z0 + ZF)       # PC4-M10 tip


def hermite(p0, t0, p1, t1, k=1.2, n=40):
    """Curve p0 -> p1 leaving along t0 and arriving along t1 (the fitting
    axes), handle length k x chord. Returns n points."""
    a, b = Vector(*p0), Vector(*p1)
    L = (b - a).length * k
    ta, tb = Vector(*t0).normalized() * L, Vector(*t1).normalized() * L
    out = []
    for i in range(n):
        s = i / (n - 1)
        h = (2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2, s**3 - s**2)
        q = a * h[0] + ta * h[1] + b * h[2] + tb * h[3]
        out.append((round(q.X, 2), round(q.Y, 2), round(q.Z, 2)))
    return out


def guide_tip(i):
    """Tip of the PC4-M6 on lane i's spool guide (world)."""
    top = 14 / GUIDE_DIR[2]
    t = top + 10
    return (CRADLE_X[i], round(CRADLE_Y + GUIDE_Y + GUIDE_DIR[1] * t, 2),
            round(GUIDE_Z + GUIDE_DIR[2] * t, 2))


def feed_tube_points(i):
    """Spool guide (fitting tip, along GUIDE_DIR) -> MMU inlet (fitting
    tip, entering along -Y), lane i."""
    end = (LANE_X[i], BODY_Y + 10, Z0 + ZF)
    return hermite(guide_tip(i), GUIDE_DIR, end, (0, -1, 0), k=1.0)


_BOWDEN_VIA, _BOWDEN_VIA_DIR = (-100, -300, 500), (-1, 0, -0.5)   # >= 1.1x reach at Z min, R >= 45
BOWDEN_POINTS = [MMU_OUTLET] + hermite(MMU_OUTLET, (0, -1, 0), _BOWDEN_VIA, _BOWDEN_VIA_DIR)[1:] \
    + hermite(_BOWDEN_VIA, _BOWDEN_VIA_DIR, EXTRUDER_INLET, (0, 1, -0.5))[1:]


EXTRAS = [
    _b("feeder_motor", (40, 42, 42), (BODY_X + 20, 0, Z0 + SHAFT_Z), "#37474f",
       "NEMA 17 42-40 (purchased) — drives all 4 lanes through one shaft"),
    _c("drive_shaft", 2.5, 140, (-12, 0, Z0 + SHAFT_Z), "x", "#90a4ae",
       "5 mm D-shaft (purchased), retained by the gears in their pockets"),
    _c("coupler_5_5", 9.5, 25, (BODY_X - 12.5, 0, Z0 + SHAFT_Z), "x", "#b0bec5",
       "5–5 mm flexible coupler (purchased)"),
    *[_c(f"bearing_625_{s}", 8, 5, (px, 0, Z0 + SHAFT_Z), "x", "#cfd8dc",
         "625ZZ shaft bearing (purchased)") for s, px in (("L", -BODY_X + 2.5), ("R", 53.5))],
    *[_c(f"drive_gear_{i + 1}", GEAR_OD_R, 11, (x, 0, Z0 + SHAFT_Z), "x", "#c9a227",
         f"MK8 drive gear, lane {i + 1} (purchased), set screw on the D-flat")
      for i, x in enumerate(LANE_X)],
    *[_c(f"idler_623_{i + 1}", BEARING_R, 4, (x, 0, Z0 + AXIS_REL), "x", "#cfd8dc",
         f"623ZZ idler bearing, lane {i + 1} (purchased)") for i, x in enumerate(LANE_X)],
    _c("cam_rod", 3, 140, (-6, 0, Z0 + CAM_Z), "x", "#90a4ae",
       "6 mm steel cam rod with a filed flat (purchased)"),
    _b("selector_servo", (40, 40.7, 19.7), (-108, -SERVO_SPLINE_OFFSET, Z0 + CAM_Z), "#6a1b9a",
       "MG996R selector servo (purchased), length along Y, spline on the cam axis"),
    _b("pico", (51, 21, 4), (-24, CARRIER_Y + 26, Z0 + 42 + 9 + 2), "#1b5e20",
       "Raspberry Pi Pico (purchased) — Klipper MCU for the MMU"),
    _b("tmc2209_board", (40, 60, 22), (22, CARRIER_Y + 8, Z0 + 42 + 9 + 11), "#283593",
       "Perfboard: TMC2209 StepStick + 100 µF + JST headers (purchased)"),
    _b("servo_buck", (43, 21, 14), (-24, CARRIER_Y - 20, Z0 + 46 + 7), "#455a64",
       "24 → 6 V buck for the servo (purchased)"),
    _b("gate_sensor", (20, 6.5, 10), (0, GATE_SENSOR_Y, Z0 + 15), "#b71c1c",
       "KW11 roller gate sensor (purchased)"),
    _b("stock_extruder", (45, 40, 45), (EXTRUDER_INLET[0], EXTRUDER_INLET[1],
                                         EXTRUDER_INLET[2] - 28), "#37474f",
       "Stock Ender-3 V2 extruder (left end of the gantry, moves in Z)"),
    _b("pi_and_display", (110, 30, 80), (290, -120, 110), "#2e7d32",
       "Your Raspberry Pi 3B/4 + display (KlipperScreen), at the printer, USB to the board + Pico"),
    *[_c(f"spool_{i + 1}", SPOOL_DIA / 2, 70, (cx, CRADLE_Y, SPOOL_Z), "x",
         LANE_COLORS[i], f"1 kg spool, lane {i + 1} (T{i})") for i, cx in enumerate(CRADLE_X)],
    *[_c(f"roller_rod_{i}_{j}", 4, 2 * (CRADLE_INNER + CRADLE_WALL) + 4,
         (cx, CRADLE_Y + ry, ROLLER_Z), "x", "#90a4ae", "8 mm rod, 92 mm (purchased)")
      for i, cx in enumerate(CRADLE_X) for j, ry in enumerate((-ROLLER_Y, ROLLER_Y))],
    *[_t(f"feed_tube_{i + 1}", feed_tube_points(i), LANE_COLORS[i],
         f"PTFE 4/2, spool {i + 1} → MMU inlet {i + 1}") for i in range(4)],
    _t("bowden_to_extruder", BOWDEN_POINTS, "#eceff1",
       "PTFE 4/2, MMU outlet → stock extruder (cut to fit, gate G8)"),
]

COLORS = {
    "mmu_body": "#2f6db5", "idler_block": "#00897b", "idler_follower": "#4db6ac",
    "selector_camshaft": "#e65100", "fit_coupon": "#9e9e9e",
    "servo_coupler": "#fb8c00", "servo_mount": "#558b2f", "merger": "#5e35b1",
    "mcu_carrier": "#2e7d32", "electronics_cover": "#546e7a", "topbar_clamp": "#6d4c41",
    "spool_cradle": "#6d4c41", "spool_roller": "#8d6e63",
}


def placed(shape, pos, rot):
    return Pos(*pos) * Rot(*rot) * shape


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(os.path.dirname(MANIFEST), exist_ok=True)
    keep = set()
    for name, fn in PARTS:
        part = fn()
        if not part.is_valid:
            raise RuntimeError(f"{name}: invalid solid")
        bb = part.bounding_box()
        print(f"{name:18s} vol={part.volume:10.0f} mm^3  bbox=({bb.size.X:.1f} x "
              f"{bb.size.Y:.1f} x {bb.size.Z:.1f}) mm")
        export_stl(part, os.path.join(OUT, f"{name}.stl"))
        keep.add(f"{name}.stl")
    for fn in os.listdir(OUT):
        if fn.endswith(".stl") and fn not in keep:
            os.remove(os.path.join(OUT, fn))
            print(f"removed stale {fn}")
    manifest = {
        "project": "Ender-MMU v0.4 (Klipper + Happy Hare)",
        "units": "mm, Z up, front -Y, desk Z=0, top-bar top Z=465",
        "parts": [
            {"name": n, "file": f"{n}.stl", "pos": list(p), "rot": list(r),
             "burst": list(b), "qty": q, "color": COLORS[n], "label": lab}
            for n, p, r, b, q, lab in ASSEMBLY
        ],
        "frame": [{"name": n, "size": list(s), "pos": list(p), "label": lab}
                  for n, s, p, lab in FRAME],
        "extras": EXTRAS,
        "lanes": {"x": LANE_X, "z": Z0 + ZF, "cam_angles": CAM_LOBE_ANGLES,
                  "cam_neutral": CAM_NEUTRAL},
    }
    with open(MANIFEST, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"Wrote {len(PARTS)} STLs + manifest")


if __name__ == "__main__":
    main()
