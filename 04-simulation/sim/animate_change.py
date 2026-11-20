#!/usr/bin/env python3
"""
Animate a T0 (red) -> T1 (green) colour change inside the MMU, driven by
the same geometry and kinematics as cad/build_mmu.py and sim/colour_change.py.

Panels
  * Top view: body + merger, the four lanes, filament tips, gate sensor.
  * Front section (along the shafts): drive shaft with its 4 gears, the
    idlers and followers, and the camshaft lobes pressing them.
  * Side sections through lane 1 and lane 2: gear rotation, cam nose,
    follower -> preload springs -> idler block -> bearing -> filament bent
    onto the gear.

Run:  .venv/bin/python sim/animate_change.py      -> renders/colour_change.gif
"""

import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation, patches  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import colour_change as S  # noqa: E402

B = S.B
OUT = os.path.join(HERE, "..", "..", "06-viewer", "renders", "colour_change.gif")
FPS, T_END = 12, 20.0
SPIN_SCALE = 0.12          # gear spin slowed for visibility
OLD, NEW = 0, 1
LM = [S.landmarks(i) for i in range(4)]
VIEW_END = [LM[i]["outlet"] + 4 for i in range(4)]   # beyond this the tip is in the Bowden

# ---------------------------------------------------------------- timeline
# (t0, t1, caption, servo angle at t1, {lane: (tip_from, tip_to)}, shaft note)
P_OLD, P_NEW = LM[OLD]["park"], LM[NEW]["park"]
FAR = 400.0                # "somewhere in the Bowden" for the visible animation
PHASES = [
    (0.0, 2.5, "Printing T0 (red): lane 1 engaged, gear stepper synced to the extruder",
     B.CAM_LOBE_ANGLES[OLD], {OLD: (FAR, FAR + 25)}, "forward (synced)"),
    (2.5, 4.0, "Tip forming in the hotend (Happy Hare) - MMU waits",
     B.CAM_LOBE_ANGLES[OLD], {}, "stopped"),
    (4.0, 7.5, "Unload: shaft runs BACKWARDS, red is pulled out of the Bowden and merger",
     B.CAM_LOBE_ANGLES[OLD], {OLD: (FAR, LM[OLD]["gate"] - 0.5)}, "reverse"),
    (7.5, 8.5, "Gate sensor cleared -> park red 16 mm behind it, in its own lane",
     B.CAM_LOBE_ANGLES[OLD], {OLD: (LM[OLD]["gate"] - 0.5, P_OLD)}, "reverse (slow)"),
    (8.5, 10.5, "Selector: servo 40 -> 75 deg, cam lifts lane 1 and presses lane 2 (gear stopped)",
     B.CAM_LOBE_ANGLES[NEW], {}, "stopped"),
    (10.5, 12.0, "Load: shaft runs FORWARD, green leaves its park position toward the merge",
     B.CAM_LOBE_ANGLES[NEW], {NEW: (P_NEW, LM[NEW]["gate"] + 0.5)}, "forward (homing)"),
    (12.0, 15.0, "Gate sensor triggered -> fast Bowden move to the extruder",
     B.CAM_LOBE_ANGLES[NEW], {NEW: (LM[NEW]["gate"] + 0.5, FAR)}, "forward (fast)"),
    (15.0, 17.0, "Extruder grabs green and loads it to the nozzle, gear synced",
     B.CAM_LOBE_ANGLES[NEW], {NEW: (FAR, FAR + 15)}, "forward (synced)"),
    (17.0, 20.0, "Printing T1 (green). Idle lanes never moved: their filament clears the turning gears",
     B.CAM_LOBE_ANGLES[NEW], {NEW: (FAR + 15, FAR + 40)}, "forward (synced)"),
]


def _smooth(u):
    return u * u * (3 - 2 * u)


def state_at(t):
    """Servo angle, per-lane tip position, shaft angle, caption at time t."""
    tips = [LM[i]["park"] for i in range(4)]
    tips[OLD] = FAR
    servo = B.CAM_LOBE_ANGLES[OLD]
    shaft = 0.0
    caption, note = PHASES[0][2], PHASES[0][5]
    for t0, t1, cap, servo_to, moves, nt in PHASES:
        if t < t0:
            break
        u = min(1.0, (t - t0) / (t1 - t0))
        servo_from = servo
        servo = servo_from + (servo_to - servo_from) * _smooth(u)
        for lane, (a, b) in moves.items():
            new = a + (b - a) * u
            shaft += (new - a) / B.GEAR_R
            tips[lane] = new
        caption, note = cap, nt
        if t < t1:
            break
    return servo, tips, shaft, caption, note


# ---------------------------------------------------------------- mechanism
def cam_radius(psi_deg, lobe_dir_deg):
    d = (psi_deg - lobe_dir_deg + 180) % 360 - 180
    return B.CAM_NOSE_R if abs(d) <= B.CAM_NOSE_HALF else B.CAM_BASE_R


def follower_push(servo, lane):
    """How far the cam pushes lane's follower down (mm)."""
    rest_top = B.CAM_Z - B.CAM_BASE_R
    low = rest_top
    lobe_dir = servo - B.CAM_LOBE_ANGLES[lane]
    for psi in np.linspace(-40, 40, 321):
        r = cam_radius(psi, lobe_dir)
        y = r * math.sin(math.radians(psi))
        if abs(y) <= 2.0:
            low = min(low, B.CAM_Z - r * math.cos(math.radians(psi)))
    return max(0.0, rest_top - low)


def idler_state(servo, lane):
    d = follower_push(servo, lane)
    block = min(d, B.IDLER_GAP)
    deflect = min(max(block - B.IDLER_CLEAR, 0.0), B.FIL_DEFLECT)
    return dict(push=d, block=block, deflect=deflect, spring=d - block,
                axis=B.AXIS_REL - block,
                follower_bottom=B.FOLLOWER_Z_REL - d,
                fil_z=B.FIL_REST_Z - deflect)


# ---------------------------------------------------------------- top-view path
def lane_xy(lane, s):
    x = B.LANE_X[lane]
    if s <= B.BODY_Y:
        return x, -s
    m = LM[lane]["merge"]
    if s <= m:
        u = (s - B.BODY_Y) / (m - B.BODY_Y)
        return x * (1 - u), -B.BODY_Y + (B.MERGE_Y + B.BODY_Y) * u
    return 0.0, B.MERGE_Y - (s - m)


def lane_polyline(lane, s0, s1, step=2.0):
    if s1 <= s0:
        return np.zeros((0, 2))
    ss = np.append(np.arange(s0, s1, step), s1)
    return np.array([lane_xy(lane, s) for s in ss])


# ---------------------------------------------------------------- drawing
def draw_top(ax, tips, gate_on, fed, zoom=False):
    ax.add_patch(patches.Rectangle((-B.BODY_X, -B.BODY_Y), 2 * B.BODY_X, 2 * B.BODY_Y,
                                   fc="#dfe7f2", ec="#2f6db5", lw=1.2))
    ax.add_patch(patches.Rectangle((-55, -B.BODY_Y - B.MERGER_LEN), 110, B.MERGER_LEN,
                                   fc="#e6e0f2", ec="#5e35b1", lw=1.2))
    ax.add_patch(patches.Rectangle((B.BODY_X, -21), 40, 42, fc="#cfd8dc", ec="#37474f"))
    ax.text(B.BODY_X + 20, 0, "NEMA 17", ha="center", va="center", fontsize=7, rotation=90,
            clip_on=True)
    ax.add_patch(patches.Rectangle((-B.BODY_X - 48, -30), 28, 40, fc="#e1bee7", ec="#6a1b9a"))
    ax.text(-B.BODY_X - 34, -10, "servo", ha="center", va="center", fontsize=7, rotation=90,
            clip_on=True)
    ax.plot([-B.BODY_X, B.BODY_X], [0, 0], color="#78909c", lw=2.5, zorder=1)
    ax.text(-75, 3, "drive shaft", fontsize=6.5, color="#546e7a", clip_on=True)
    for i, x in enumerate(B.LANE_X):
        ax.add_patch(patches.Rectangle((x - 5.5, -5.5), 11, 11, fc="#c9a227", ec="#8d6e00", zorder=2))
        path = lane_polyline(i, -B.BODY_Y, LM[i]["outlet"])
        ax.plot(path[:, 0], path[:, 1], color="#b0bec5", lw=5, solid_capstyle="round", zorder=1)
    for i in range(4):
        tip = min(tips[i], VIEW_END[i])
        line = lane_polyline(i, -95, tip)
        col = B.LANE_COLORS[i]
        ax.plot(line[:, 0], line[:, 1], color=col, lw=2.4, zorder=3)
        marks = [lane_xy(i, s) for s in np.arange(-95 + fed[i] % 12, tip, 12)]
        if marks:
            m = np.array(marks)
            ax.plot(m[:, 0], m[:, 1], "o", ms=2.2, color="white", zorder=4)
        if tips[i] < VIEW_END[i]:
            x, y = lane_xy(i, tips[i])
            ax.plot([x], [y], "o", ms=5, color=col, mec="black", mew=0.6, zorder=5)
    gy = B.GATE_SENSOR_Y
    ax.add_patch(patches.Rectangle((-10, gy - 4), 20, 7, fc="#2e7d32" if gate_on else "#b71c1c",
                                   ec="black", lw=0.6, alpha=0.8, zorder=4))
    if zoom:
        ax.set_xlim(-22, 22)
        ax.set_ylim(-122, -78)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title("zoom: merge + gate sensor", fontsize=7)
        return
    ax.text(14, gy, f"gate sensor: {'FILAMENT' if gate_on else 'empty'}", fontsize=7, va="center")
    ax.annotate("to Bowden / extruder", (0, -146), (0, -158), ha="center", fontsize=7,
                arrowprops=dict(arrowstyle="<-", lw=0.8))
    ax.text(0, 88, "from the 4 spools", ha="center", fontsize=7)
    for i, x in enumerate(B.LANE_X):
        ax.text(x, 50, f"T{i}", ha="center", fontsize=8, color=B.LANE_COLORS[i], weight="bold")
    ax.set_xlim(-140, 130)
    ax.set_ylim(-165, 95)
    ax.set_aspect("equal")
    ax.set_title("Top view: lanes, merger, gate sensor", fontsize=9)
    ax.axis("off")


def draw_front(ax, servo, shaft):
    ax.add_patch(patches.Rectangle((-B.BODY_X, 0), 2 * B.BODY_X, B.BODY_Z, fc="#eef3f9",
                                   ec="#2f6db5", lw=1))
    ax.plot([-B.BODY_X, B.BODY_X + 25], [B.SHAFT_Z] * 2, color="#78909c", lw=3)
    ax.plot([-B.BODY_X - 30, 66], [B.CAM_Z] * 2, color="#78909c", lw=2.5)
    ax.add_patch(patches.Rectangle((B.BODY_X, B.SHAFT_Z - 21), 40, 42, fc="#cfd8dc", ec="#37474f"))
    ax.text(B.BODY_X + 20, B.SHAFT_Z, "NEMA\n17", ha="center", va="center", fontsize=7)
    cx, cy = -B.BODY_X - 38, B.CAM_Z
    ax.add_patch(patches.Circle((cx, cy), 12, fc="#e1bee7", ec="#6a1b9a"))
    a = math.radians(servo)
    ax.plot([cx, cx + 11 * math.sin(a)], [cy, cy - 11 * math.cos(a)], color="#4a148c", lw=2)
    ax.text(cx, cy - 18, f"servo {servo:5.1f} deg", ha="center", fontsize=7)
    for i, x in enumerate(B.LANE_X):
        st = idler_state(servo, i)
        col = B.LANE_COLORS[i]
        # gear (edge-on): teeth stripes move with the shaft
        ax.add_patch(patches.Rectangle((x - 5.5, B.SHAFT_Z - B.GEAR_OD_R), 11, 2 * B.GEAR_OD_R,
                                       fc="#c9a227", ec="#8d6e00"))
        phase = (shaft * SPIN_SCALE * B.GEAR_OD_R) % 2.0
        for k in range(-6, 7):
            z = B.SHAFT_Z + k * 2.0 + phase - 1
            if abs(z - B.SHAFT_Z) < B.GEAR_OD_R:
                ax.plot([x - 5.5, x + 5.5], [z, z], color="#8d6e00", lw=0.5)
        ax.add_patch(patches.Circle((x, st["fil_z"]), B.FIL_R, fc=col, ec="black", lw=0.5, zorder=5))
        ax.add_patch(patches.Rectangle((x - 2, st["axis"] - B.BEARING_R), 4, 2 * B.BEARING_R,
                                       fc="#cfd8dc", ec="#607d8b", zorder=4))
        blk = st["axis"] - B.IDLER_AXIS_IN_BLOCK
        ax.add_patch(patches.Rectangle((x - 6, blk), 12, B.IDLER_BLOCK_H, fc="#00897b", alpha=0.55,
                                       ec="#00695c"))
        fb = st["follower_bottom"]
        ax.add_patch(patches.Rectangle((x - 6, fb), 12, B.FOLLOWER_H, fc="#4db6ac", alpha=0.75,
                                       ec="#00695c"))
        ax.add_patch(patches.Rectangle((x - 6, fb + B.FOLLOWER_H), 12, B.RIDGE_H, fc="#4db6ac",
                                       ec="#00695c"))
        low = B.CAM_Z - B.CAM_BASE_R - st["push"]
        ax.add_patch(patches.Rectangle((x - 4, low), 8, B.CAM_Z - low + B.CAM_BASE_R,
                                       fc="#ff9800" if st["push"] > 0.2 else "#ffcc80", ec="#e65100"))
        label = "GRIP" if st["deflect"] >= B.FIL_DEFLECT - 1e-6 else (
            "pressing" if st["push"] > 0.2 else "free")
        ax.text(x, -9, f"T{i}\n{label}", ha="center", fontsize=7, color=col, weight="bold")
    ax.set_xlim(-135, 125)
    ax.set_ylim(-16, 78)
    ax.set_aspect("equal")
    ax.set_title("Front section: one shaft, 4 gears, camshaft picks ONE idler", fontsize=9)
    ax.axis("off")


def draw_side(ax, servo, shaft, lane, fed_moving):
    st = idler_state(servo, lane)
    col = B.LANE_COLORS[lane]
    half = B.FREE_SPAN / 2
    for y0, y1 in ((-30, -half), (half, 30)):
        ax.add_patch(patches.Rectangle((y0, B.CHANNEL_FLOOR - 6), y1 - y0, 6, fc="#dfe7f2",
                                       ec="#2f6db5", lw=0.8))
    g = patches.Circle((0, B.SHAFT_Z), B.GEAR_OD_R, fc="#c9a227", ec="#8d6e00")
    ax.add_patch(g)
    ang = shaft * SPIN_SCALE
    for k in range(6):
        a = ang + k * math.pi / 3
        ax.plot([0, B.GEAR_OD_R * math.sin(a)], [B.SHAFT_Z, B.SHAFT_Z + B.GEAR_OD_R * math.cos(a)],
                color="#5d4037", lw=1)
    ax.add_patch(patches.Circle((0, B.SHAFT_Z), 2.5, fc="#78909c", ec="black", lw=0.5))
    ys = np.linspace(-30, 30, 121)
    sag = st["deflect"] * np.clip(1 - np.abs(ys) / half, 0, 1) ** 1.6
    ax.plot(ys, B.FIL_REST_Z - sag, color=col, lw=4.5, solid_capstyle="butt", zorder=5)
    if fed_moving:
        for yy in np.arange(-30 + fed_moving % 6, 30, 6):
            ax.plot([yy], [B.FIL_REST_Z - st["deflect"] * max(0, 1 - abs(yy) / half) ** 1.6], "o",
                    ms=1.8, color="white", zorder=6)
    ax.add_patch(patches.Circle((0, st["axis"]), B.BEARING_R, fc="#cfd8dc", ec="#607d8b", zorder=4))
    blk = st["axis"] - B.IDLER_AXIS_IN_BLOCK
    ax.add_patch(patches.Rectangle((-13, blk), 26, B.IDLER_BLOCK_H, fc="#00897b", alpha=0.45,
                                   ec="#00695c", zorder=3))
    fb = st["follower_bottom"]
    ax.add_patch(patches.Rectangle((-13, fb), 26, B.FOLLOWER_H, fc="#4db6ac", alpha=0.8, ec="#00695c"))
    ax.add_patch(patches.Rectangle((-2, fb + B.FOLLOWER_H), 4, B.RIDGE_H, fc="#4db6ac", ec="#00695c"))
    for sy in (-B.SPRING_Y, B.SPRING_Y):
        z0, z1 = blk + B.IDLER_BLOCK_H, fb
        zz = np.linspace(z0, z1, 13)
        xx = sy + np.array([0 if k in (0, 12) else (1.5 if k % 2 else -1.5) for k in range(13)])
        ax.plot(xx, zz, color="#37474f", lw=0.9)
    lobe = servo - B.CAM_LOBE_ANGLES[lane]
    psi = np.linspace(-180, 180, 721)
    r = np.array([cam_radius(p, lobe) for p in psi])
    ax.fill(r * np.sin(np.radians(psi)), B.CAM_Z - r * np.cos(np.radians(psi)),
            fc="#ff9800", ec="#e65100", alpha=0.9)
    ax.add_patch(patches.Circle((0, B.CAM_Z), 3, fc="#78909c", ec="black", lw=0.5))
    state = ("GRIP: filament pressed onto the gear" if st["deflect"] >= B.FIL_DEFLECT - 1e-6
             else "released: filament clears the gear")
    ax.text(0, 4, state, ha="center", fontsize=7.5, color=col, weight="bold")
    ax.text(0, 0, f"spring compression {st['spring']:.1f} mm", ha="center", fontsize=7)
    ax.set_xlim(-32, 32)
    ax.set_ylim(-2, 75)
    ax.set_aspect("equal")
    ax.set_title(f"Side section, lane T{lane}", fontsize=9, color=col)
    ax.axis("off")


def main():
    fig = plt.figure(figsize=(13, 8), dpi=80)
    gs = fig.add_gridspec(2, 3, width_ratios=[1.4, 1, 1], height_ratios=[1, 1.05])
    ax_top = fig.add_subplot(gs[:, 0])
    pos = ax_top.get_position()
    ax_zoom = fig.add_axes([pos.x0 + 0.60 * pos.width, pos.y0 + 0.02 * pos.height,
                            0.38 * pos.width, 0.30 * pos.height])
    ax_front = fig.add_subplot(gs[0, 1:])
    ax_s0 = fig.add_subplot(gs[1, 1])
    ax_s1 = fig.add_subplot(gs[1, 2])
    title = fig.text(0.5, 0.965, "", ha="center", fontsize=12, weight="bold")
    status = fig.text(0.5, 0.932, "", ha="center", fontsize=9, color="#37474f")
    n = int(T_END * FPS)
    prev = {"tips": None}

    def frame(k):
        t = k / FPS
        servo, tips, shaft, caption, note = state_at(t)
        fed = [tips[i] for i in range(4)]
        moving = [prev["tips"] is not None and abs(tips[i] - prev["tips"][i]) > 1e-6
                  for i in range(4)]
        prev["tips"] = list(tips)
        gate_on = any(LM[i]["gate"] <= tips[i] for i in range(4))
        for ax in (ax_top, ax_zoom, ax_front, ax_s0, ax_s1):
            ax.clear()
        draw_top(ax_top, tips, gate_on, fed)
        draw_top(ax_zoom, tips, gate_on, fed, zoom=True)
        draw_front(ax_front, servo, shaft)
        draw_side(ax_s0, servo, shaft, OLD, fed[OLD] if moving[OLD] else 0)
        draw_side(ax_s1, servo, shaft, NEW, fed[NEW] if moving[NEW] else 0)
        title.set_text(caption)
        touching = [i for i in range(4) if idler_state(servo, i)["push"] > 0.2]
        extra = ("   |   two idlers touch in passing - shaft stopped, nothing moves"
                 if len(touching) > 1 else "")
        status.set_text(f"t = {t:4.1f} s (compressed)   |   drive shaft: {note}   |   "
                        f"selector servo: {servo:5.1f} deg   |   gate sensor: "
                        f"{'filament' if gate_on else 'empty'}{extra}")
        return []

    anim = animation.FuncAnimation(fig, frame, frames=n, interval=1000 / FPS)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    anim.save(OUT, writer=animation.PillowWriter(fps=FPS))
    for t in (1.0, 6.0, 9.5, 11.5, 18.0):
        frame(int(t * FPS))
        fig.savefig(OUT.replace(".gif", f"_t{t:04.1f}.png"))
    print(f"wrote {OUT} ({n} frames)")


if __name__ == "__main__":
    main()
