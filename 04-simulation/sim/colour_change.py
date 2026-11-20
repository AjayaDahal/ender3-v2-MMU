#!/usr/bin/env python3
"""
Colour-change simulation for the Ender-MMU (geometry, kinematics, forces,
and the Happy Hare load/unload sequence).

What it models
  1. Idle lanes vs the shared, always-turning gear shaft (contact / drag).
  2. Filament bending when the idler presses it onto its gear (strain, force).
  3. Grip (gear traction) vs drag (spool + PTFE capstan friction) for both
     the unload pull and the load push, PLA and PETG.
  4. Servo sweep 0..160 deg: which lane is engaged at every angle.
  5. The merger junction: where a pushed tip can go.
  6. A 1-D event simulation of a full T(a) -> T(b) change for every pair:
     tip positions, sensor events, shared-path occupancy, idle-lane motion,
     and the time it takes.

What it does NOT prove: tip shape after forming, real friction coefficients,
spring-rate tolerance, print quality. Those are bench tests (PLAN.md §5).

Run:  .venv/bin/python sim/colour_change.py
"""

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "design"))
import build_mmu as B  # noqa: E402

# ---------------------------------------------------------------- materials
MATERIALS = {          # Young's modulus (MPa), yield strain, mu on PTFE
    "PLA": (3500, 0.025, 0.25),
    "PETG": (2100, 0.040, 0.30),
}
MU_GEAR = 0.4          # effective traction of a hobbed MK8 gear (conservative)
SPOOL_DRAG = 1.0       # N to turn a 1 kg spool on bearing rollers
TIP_DRAG = 0.5         # N for a free tip sliding into the PTFE (before the extruder grabs it)
STOCK_EXTRUDER_PULL = 20.0  # N a stock Ender-3 V2 extruder can pull without skipping
GEAR_TO_NOZZLE = 455   # stock Ender-3 V2: extruder gear -> nozzle (gate G5)
TOOLHEAD_PARK = 20     # Happy Hare parks the tip this far before the extruder gear
PARK_BEFORE_MERGE = 10  # idle tips park in their own merger lane, just before the merge
GATE_HOMING_MAX = 60   # Happy Hare gives up homing to the gate after this
SPEED_FAST, SPEED_HOME, SPEED_EXTRUDER = 80.0, 20.0, 25.0  # mm/s
SERVO_S_PER_60 = 0.17  # MG996R at 6 V
SERVO_ERROR = 5.0      # deg of servo position error each lane must tolerate
REQUIRED_MARGIN = 2.0


# ---------------------------------------------------------------- 1. idle lanes
def idle_lane_clearance():
    return B.CHANNEL_FLOOR - (B.SHAFT_Z + B.GEAR_OD_R)


# ---------------------------------------------------------------- 2. bending
def filament_bending(material):
    E, eps_yield, _ = MATERIALS[material]
    d, L, delta = 2 * B.FIL_R, B.FREE_SPAN, B.FIL_DEFLECT
    inertia = math.pi * d ** 4 / 64
    force = 48 * E * inertia * delta / L ** 3      # simply supported, centre load
    strain = 6 * d * delta / L ** 2
    return force, strain, eps_yield


# ---------------------------------------------------------------- 3. grip vs drag
def preload_force():
    od, free, _ = B.IDLER_PRELOAD_SPRING
    released = B.SPRING_GAP + B.SPRING_SEAT + B.FOLLOWER_POCKET
    return 2 * B.PRELOAD_RATE * (free - (released - B.IDLER_OVERTRAVEL))


def return_force():
    _, free = B.RETURN_SPRING
    block_bottom_engaged = B.AXIS_CONTACT - B.IDLER_AXIS_IN_BLOCK
    length = (block_bottom_engaged + 6) - 31.5      # spring pocket depth 6, slot floor 31.5
    return 2 * B.RETURN_RATE * (free - length)


def turning_angle(points):
    p = np.asarray(points, float)
    v = np.diff(p, axis=0)
    v = v[np.linalg.norm(v, axis=1) > 1e-9]
    v /= np.linalg.norm(v, axis=1)[:, None]
    cos = np.clip(np.einsum("ij,ij->i", v[:-1], v[1:]), -1, 1)
    return float(np.sum(np.arccos(cos)))


def merger_bend(lane):
    return math.atan2(abs(B.LANE_X[lane]), -B.BODY_Y - B.MERGE_Y)


def grip_and_drag(material, lane, mu_gear=MU_GEAR, mu_ptfe=None):
    _, _, mu = MATERIALS[material]
    mu = mu if mu_ptfe is None else mu_ptfe
    bend_force, _, _ = filament_bending(material)
    normal = preload_force() - return_force() - bend_force
    traction = mu_gear * normal
    feed = turning_angle(B.feed_tube_points(lane))
    bowden = turning_angle(B.BOWDEN_POINTS) + merger_bend(lane)
    # Load: pull from the spool through the feed tube while pushing a free
    # tip through the merger + Bowden (capstan model on every bend).
    load = SPOOL_DRAG * math.exp(mu * feed) + TIP_DRAG * math.exp(mu * bowden)
    # Unload: pull the filament back out of the Bowden; the slack pushed
    # back into the feed tube costs little.
    unload = 0.5 * math.exp(mu * bowden) + 0.3
    # Printing: Happy Hare syncs the MMU gear to the extruder (lane held
    # engaged), so both pull; the stock extruder alone also has to drive its
    # own Bowden to the hotend.
    printing = SPOOL_DRAG * math.exp(mu * (feed + bowden))
    return dict(normal=normal, traction=traction, load=load, unload=unload,
                printing=printing, feed_deg=math.degrees(feed),
                bowden_deg=math.degrees(bowden))


# ---------------------------------------------------------------- 4. servo sweep
RIDGE_HALF = 2.0


def lanes_engaged(servo_deg):
    ridge_top = B.CAM_Z - B.CAM_BASE_R
    out = []
    for i, lobe in enumerate(B.CAM_LOBE_ANGLES):
        d = servo_deg - lobe
        for phi in np.linspace(d - B.CAM_NOSE_HALF, d + B.CAM_NOSE_HALF, 31):
            r = math.radians(phi)
            if abs(B.CAM_NOSE_R * math.sin(r)) <= RIDGE_HALF and \
                    B.CAM_Z - B.CAM_NOSE_R * math.cos(r) < ridge_top:
                out.append(i)
                break
    return out


def single_lane_band(lane, step=0.25):
    """How far the servo can be off its target and still engage only this lane."""
    target = B.CAM_LOBE_ANGLES[lane]
    band = 0.0
    while lanes_engaged(target - band - step) == [lane] and \
            lanes_engaged(target + band + step) == [lane]:
        band += step
        if band > 60:
            break
    return band


def neutral_band(step=0.25):
    band = 0.0
    while lanes_engaged(B.CAM_NEUTRAL + band + step) == [] and band < 60:
        band += step
    return band


def servo_sweep(step=0.25):
    lo, hi = B.SERVO_USABLE
    return [(a, lanes_engaged(a)) for a in np.arange(lo, hi + step, step)]


def engagement_intervals(sweep):
    ivals = {}
    for a, lanes in sweep:
        for i in lanes:
            lo, hi = ivals.get(i, (a, a))
            ivals[i] = (min(lo, a), max(hi, a))
    return ivals


# ---------------------------------------------------------------- 5. junction
def junction_angles(lane):
    """Angle between lane's heading at the merge point and (a) the outlet,
    (b) the opening of every other lane (pointing back toward the body)."""
    def unit(v):
        v = np.asarray(v, float)
        return v / np.linalg.norm(v)
    head = unit((0 - B.LANE_X[lane], B.MERGE_Y + B.BODY_Y))
    outlet = unit((0, -1))
    to_outlet = math.degrees(math.acos(np.clip(head @ outlet, -1, 1)))
    others = [math.degrees(math.acos(np.clip(head @ unit((B.LANE_X[j], -B.BODY_Y - B.MERGE_Y)),
                                             -1, 1)))
              for j in range(4) if j != lane]
    return to_outlet, min(others)


# ---------------------------------------------------------------- 6. sequence
def landmarks(lane):
    """Positions along lane's path, mm from its MMU gear contact."""
    to_merge = B.BODY_Y + math.hypot(B.LANE_X[lane], -B.BODY_Y - B.MERGE_Y)
    gate = to_merge + (B.MERGE_Y - B.GATE_SENSOR_Y)
    outlet = to_merge + (B.MERGE_Y - (-B.BODY_Y - B.MERGER_LEN)) + 10
    bowden = float(np.sum(np.linalg.norm(np.diff(np.array(B.BOWDEN_POINTS, float), axis=0),
                                         axis=1)))
    ext_gear = outlet + bowden
    return dict(park=to_merge - PARK_BEFORE_MERGE, merge=to_merge, gate=gate, outlet=outlet,
                ext_gear=ext_gear, nozzle=ext_gear + GEAR_TO_NOZZLE)


@dataclass
class State:
    tip: list                       # tip position per lane (mm from its gear)
    servo: float = B.CAM_NEUTRAL
    t: float = 0.0
    log: list = field(default_factory=list)
    errors: list = field(default_factory=list)


def _gate_triggered(st):
    return any(st.tip[i] >= landmarks(i)["gate"] for i in range(4))


def _check_shared(st, step):
    inside = [i for i in range(4) if st.tip[i] > landmarks(i)["merge"]]
    if len(inside) > 1:
        st.errors.append(f"{step}: lanes {inside} both past the merge point")


def _servo_to(st, target, step):
    if target == st.servo:
        return
    path = np.linspace(st.servo, target, int(abs(target - st.servo) * 4) + 2)
    touched = sorted({i for a in path for i in lanes_engaged(a)})
    st.t += abs(target - st.servo) / 60 * SERVO_S_PER_60 + 0.2
    st.log.append(f"{step}: servo {st.servo:.0f} -> {target:.0f} deg "
                  f"(stepper idle; lanes touched in passing: {touched or 'none'})")
    st.servo = target


def _gear_move(st, lane, dist, speed, step):
    """Move the shared gear shaft. Only the engaged lane's filament moves;
    idle lanes would move only if they touched a gear."""
    engaged = lanes_engaged(st.servo)
    if engaged != [lane]:
        st.errors.append(f"{step}: gear moved with lanes {engaged} engaged, expected [{lane}]")
    if idle_lane_clearance() <= 0:
        for i in range(4):
            if i != lane:
                st.errors.append(f"{step}: idle lane {i + 1} dragged by its gear")
    st.tip[lane] += dist
    st.t += abs(dist) / speed
    _check_shared(st, step)


def _home_to_gate(st, lane, direction, step):
    """Move until the gate sensor changes state (Happy Hare gate homing)."""
    lm = landmarks(lane)
    want = direction > 0
    moved = 0.0
    while _gate_triggered(st) != want:
        _gear_move(st, lane, direction * 0.5, SPEED_HOME, step)
        moved += 0.5
        if moved > GATE_HOMING_MAX:
            st.errors.append(f"{step}: gate never {'triggered' if want else 'cleared'}")
            return
    st.log.append(f"{step}: gate {'triggered' if want else 'cleared'} at "
                  f"{st.tip[lane]:.1f} mm (gate at {lm['gate']:.1f})")


def simulate_change(a, b):
    """T(a) -> T(b). Lane a is loaded to the nozzle, the others parked."""
    tip = [landmarks(i)["park"] for i in range(4)]
    tip[a] = landmarks(a)["nozzle"]
    parked = list(tip)
    st = State(tip=tip)
    la, lb = landmarks(a), landmarks(b)
    # Unload a: toolhead (extruder) to just before its gear, then the MMU.
    _servo_to(st, B.CAM_LOBE_ANGLES[a], "select old")
    st.tip[a] = la["ext_gear"] - TOOLHEAD_PARK
    st.t += (la["nozzle"] - st.tip[a]) / SPEED_EXTRUDER
    st.log.append(f"unload toolhead: tip {a + 1} at {st.tip[a]:.0f} (before the extruder gear)")
    fast = st.tip[a] - la["gate"] - 30                     # stop 30 mm short of the gate
    _gear_move(st, a, -fast, SPEED_FAST, "unload bowden")
    _home_to_gate(st, a, -1, "unload home")
    _gear_move(st, a, -(st.tip[a] - la["park"]), SPEED_FAST, "park old")
    if not (0 < st.tip[a] <= la["merge"] - PARK_BEFORE_MERGE + 1e-9):
        st.errors.append(f"old tip parked at {st.tip[a]:.1f}, outside its own lane")
    # Load b.
    _servo_to(st, B.CAM_LOBE_ANGLES[b], "select new")
    _home_to_gate(st, b, +1, "load to gate")
    fast = lb["ext_gear"] - st.tip[b] - TOOLHEAD_PARK
    _gear_move(st, b, fast, SPEED_FAST, "load bowden")
    st.tip[b] = lb["nozzle"]
    st.t += (lb["nozzle"] - (lb["ext_gear"] - TOOLHEAD_PARK)) / SPEED_EXTRUDER
    st.log.append(f"load toolhead: tip {b + 1} at the nozzle ({st.tip[b]:.0f})")
    _servo_to(st, B.CAM_NEUTRAL, "release")
    for i in range(4):
        if i not in (a, b) and st.tip[i] != parked[i]:
            st.errors.append(f"idle lane {i + 1} moved to {st.tip[i]:.1f}")
    return st


def all_changes():
    return {(a, b): simulate_change(a, b) for a in range(4) for b in range(4) if a != b}


# ---------------------------------------------------------------- report
def main():
    ok = True

    def line(cond, text):
        nonlocal ok
        ok &= bool(cond)
        print(f"[{'PASS' if cond else 'FAIL'}] {text}")

    print("== 1. Idle lanes vs the turning shaft")
    c = idle_lane_clearance()
    line(c >= 0.15, f"idle filament clears its gear teeth by {c:.2f} mm")

    print("== 2. Filament bending onto the gear")
    for m in MATERIALS:
        f, s, y = filament_bending(m)
        line(s <= 0.5 * y, f"{m}: {B.FIL_DEFLECT:.2f} mm over {B.FREE_SPAN} mm -> "
             f"strain {s * 100:.2f} % (yield {y * 100:.1f} %), bend force {f:.1f} N")

    print("== 3. Grip vs drag (traction margin)")
    for m in MATERIALS:
        for lane in range(4):
            g = grip_and_drag(m, lane)
            need = max(g["load"], g["unload"])
            line(g["traction"] >= REQUIRED_MARGIN * need,
                 f"{m} lane {lane + 1}: normal {g['normal']:.1f} N, traction {g['traction']:.1f} N"
                 f" vs load {g['load']:.1f} N / unload {g['unload']:.1f} N"
                 f" (bends feed {g['feed_deg']:.0f} deg, bowden {g['bowden_deg']:.0f} deg)"
                 f" -> {g['traction'] / need:.1f}x")
            shared = (g["traction"] + STOCK_EXTRUDER_PULL) / 2
            line(g["printing"] * REQUIRED_MARGIN <= shared,
                 f"{m} lane {lane + 1}: print-time drag {g['printing']:.1f} N vs synced"
                 f" gear + extruder {shared:.1f} N each side")
    print("   sensitivity (PLA, worst lane): traction margin for mu_gear x mu_ptfe")
    for mg in (0.3, 0.4, 0.6):
        row = []
        for mp in (0.15, 0.25, 0.35):
            g = grip_and_drag("PLA", 0, mg, mp)
            row.append(f"{g['traction'] / max(g['load'], g['unload']):.1f}x")
        print(f"     mu_gear {mg}: " + "  ".join(f"mu_ptfe {mp}: {r}" for mp, r in
                                                zip((0.15, 0.25, 0.35), row)))

    print("== 4. Servo sweep")
    sweep = servo_sweep()
    for i, (lo, hi) in sorted(engagement_intervals(sweep).items()):
        band = single_lane_band(i)
        line(band >= SERVO_ERROR,
             f"lane {i + 1}: target {B.CAM_LOBE_ANGLES[i]} deg, only this lane engaged within"
             f" +/-{band:.1f} deg (engaged {lo:.1f}..{hi:.1f})")
    line(neutral_band() >= SERVO_ERROR,
         f"neutral {B.CAM_NEUTRAL} deg: nothing engaged within +{neutral_band():.1f} deg")
    print("   (between targets two lanes touch briefly: harmless because the gear"
          " stepper is idle while the selector moves - checked in section 6)")

    print("== 5. Merger junction")
    for lane in range(4):
        out, other = junction_angles(lane)
        line(out <= 35 and other >= 90,
             f"lane {lane + 1}: {out:.0f} deg to the outlet, >= {other:.0f} deg to any other lane")

    print("== 6. Full colour changes (all 12 pairs)")
    runs = all_changes()
    for (a, b), st in runs.items():
        line(not st.errors, f"T{a} -> T{b}: {st.t:.1f} s" + (f"  {st.errors}" if st.errors else ""))
    print("\nExample T0 -> T1:")
    for entry in runs[(0, 1)].log:
        print("   ", entry)
    lm = landmarks(0)
    print("\nLane 1 landmarks (mm from its gear):",
          {k: round(v, 1) for k, v in lm.items()})
    print(f"\n{'ALL PASS' if ok else 'FAILURES ABOVE'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
