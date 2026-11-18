#!/usr/bin/env python3
"""
Audit the Ender-MMU STLs AND the assembly (v0.3).

Per STL: watertight, volume > 0, fits the 220x220x250 build volume,
STL volume matches the build123d source.
Assembly (parts placed exactly as in viewer/manifest.json):
  - no two printed parts (or a part and the frame) overlap
  - every lane path is open from inlet to outlet (no solid on the path)
  - the filament channel is enclosed (wall around it) outside the
    drive windows
  - mechanism geometry: gear top / idler bearing meet the filament,
    each servo angle engages exactly one lane, neutral engages none
  - envelope: MMU fits between the uprights and <= 85 mm above the bar

Run:  .venv/bin/python cad/check_mmu.py     (exit 0 only if all pass)
"""

import itertools
import math
import os
import struct
import sys

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)
import build_mmu as B  # noqa: E402
from build123d import Vector  # noqa: E402

STL_DIR = os.path.join(BASE, "..", "stl")
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    (count,) = struct.unpack_from("<I", data, 80)
    if 84 + count * 50 == len(data):
        arr = np.frombuffer(data[84:], dtype=np.uint8).reshape(count, 50)
        return arr[:, :48].copy().view("<f4").reshape(count, 4, 3)[:, 1:, :]
    verts = [[float(v) for v in ln.split()[1:4]]
             for ln in data.decode("ascii", "replace").splitlines()
             if ln.strip().startswith("vertex")]
    return np.array(verts, dtype=float).reshape(-1, 3, 3)


def mesh_stats(tris):
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    vol = abs(float(np.sum(np.einsum("ij,ij->i", v0, np.cross(v1, v2))) / 6))
    size = tris.reshape(-1, 3).max(0) - tris.reshape(-1, 3).min(0)
    _, ids = np.unique(np.round(tris.reshape(-1, 3), 3), axis=0, return_inverse=True)
    ids = ids.reshape(-1, 3)
    e = np.sort(np.concatenate([ids[:, [0, 1]], ids[:, [1, 2]], ids[:, [2, 0]]]), axis=1)
    _, counts = np.unique(e, axis=0, return_counts=True)
    return vol, size, int(np.sum(counts != 2))


def catmull_rom(p, n=24):
    """Uniform Catmull-Rom through the points (same curve the viewer draws)."""
    p = np.vstack([2 * p[0] - p[1], p, 2 * p[-1] - p[-2]])
    out = []
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        for t in np.linspace(0, 1, n, endpoint=False):
            out.append(0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    out.append(p[-2])
    return np.array(out)


def min_bend_radius(pts):
    a, b, c = pts[:-2], pts[1:-1], pts[2:]
    ab, bc, ca = (np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1),
                  np.linalg.norm(a - c, axis=1))
    area2 = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    with np.errstate(divide="ignore"):
        r = ab * bc * ca / (2 * area2)
    return float(np.nanmin(np.where(area2 < 1e-9, np.inf, r)))


def main():
    shapes = {n: fn() for n, fn in B.PARTS}

    print("== STLs")
    for name, _ in B.PARTS:
        vol, size, bad = mesh_stats(read_stl(os.path.join(STL_DIR, f"{name}.stl")))
        check(f"{name}: watertight", bad == 0, f"{bad} bad edges" if bad else "")
        check(f"{name}: fits 220x220x250", bool(np.all(size <= [220, 220, 250])),
              f"{size[0]:.0f}x{size[1]:.0f}x{size[2]:.0f}")
        rel = abs(vol - shapes[name].volume) / shapes[name].volume
        check(f"{name}: STL volume = source", rel < 0.01, f"{rel * 100:.2f}%")

    print("== Assembly interference")
    placed = [(f"{n}@{p[0]:.0f},{p[1]:.0f}", B.placed(shapes[n], p, r))
              for n, p, r, _, _, _ in B.ASSEMBLY]
    frame = [(n, B.Pos(*p) * B.Box(*s)) for n, s, p, _ in B.FRAME
             if n in ("top_bar_2020", "z_upright_left", "z_upright_right", "x_beam_2020")]
    clashes = []
    for (na, a), (nb, b) in itertools.chain(
            itertools.combinations(placed, 2), itertools.product(placed, frame)):
        ba, bb = a.bounding_box(), b.bounding_box()
        if (ba.min.X > bb.max.X or bb.min.X > ba.max.X or ba.min.Y > bb.max.Y
                or bb.min.Y > ba.max.Y or ba.min.Z > bb.max.Z or bb.min.Z > ba.max.Z):
            continue
        common = a & b
        v = common.volume if common is not None else 0.0
        if v > 1.0:
            clashes.append(f"{na} x {nb}: {v:.0f} mm^3")
    check("no part/part or part/frame overlap", not clashes, "; ".join(clashes))

    print("== Filament path")
    solids = [s for _, s in placed]
    zf = B.Z0 + B.ZF
    y_in, y_out = B.BODY_Y, -B.BODY_Y - B.MERGER_LEN
    for i, x in enumerate(B.LANE_X):
        pts = [(x, y, zf) for y in np.linspace(y_in - 0.5, -B.BODY_Y, 40)]
        pts += [(x + (0 - x) * t, -B.BODY_Y + (B.MERGE_Y + B.BODY_Y) * t, zf)
                for t in np.linspace(0, 1, 30)]
        pts += [(0, y, zf) for y in np.linspace(B.MERGE_Y, y_out + 0.5, 15)]
        blocked = [p for p in pts if any(s.is_inside(Vector(*p)) for s in solids)]
        check(f"lane {i + 1}: path open inlet -> outlet", not blocked,
              f"blocked at {blocked[:2]}" if blocked else f"{len(pts)} points")
        body = placed[0][1]
        walls = [(x, y, zf + dz) for y in (15, -15, -25) for dz in (-1.6, 2.0)]
        walls += [(x, 25, zf + dz) for dz in (-2.6, 3.2)]  # PTFE seat (teardrop roof)
        missing = [p for p in walls if not body.is_inside(Vector(*p))]
        check(f"lane {i + 1}: channel enclosed in body", not missing,
              f"no wall at {missing[:2]}" if missing else "")
    merger = dict(placed)[[n for n, _ in placed if n.startswith("merger")][0]]
    walls = [(0, -97, zf + dz) for dz in (-1.8, 2.2)]
    walls += [(0, -118, zf + dz) for dz in (-2.6, 3.2)]  # PTFE seat (teardrop roof)
    check("merger outlet channel enclosed",
          all(merger.is_inside(Vector(*p)) for p in walls))
    lever = Vector(0, B.GATE_SENSOR_Y - 0.5, zf - 2.5)
    check("gate sensor lever slot reaches the outlet channel",
          not merger.is_inside(lever))

    print("== Mechanism")
    gear_top = B.SHAFT_Z + B.GEAR_R
    check("drive gear meets the gripped filament", abs(gear_top - (B.FIL_GRIP_Z - B.FIL_R)) < 1e-6,
          f"gear top {gear_top:.3f}")
    check("idle filament clears its gear teeth by >= 0.15 mm",
          B.CHANNEL_FLOOR - (B.SHAFT_Z + B.GEAR_OD_R) >= 0.15,
          f"{B.CHANNEL_FLOOR - (B.SHAFT_Z + B.GEAR_OD_R):.2f} mm")
    check("released idler clears the idle filament by >= 1 mm",
          (B.AXIS_REL - B.BEARING_R) - (B.FIL_REST_Z + B.FIL_R) >= 1.0 - 1e-9)
    check("cam lift closes the gap and compresses the preload spring >= 1 mm",
          B.IDLER_OVERTRAVEL >= 1.0, f"overtravel {B.IDLER_OVERTRAVEL:.2f}")
    check("follower never bottoms on the block (gap when engaged >= 0.3)",
          B.SPRING_GAP - B.IDLER_OVERTRAVEL >= 0.3)
    od, free, solid = B.IDLER_PRELOAD_SPRING
    rel_len = B.SPRING_GAP + B.SPRING_SEAT + B.FOLLOWER_POCKET
    eng_len = rel_len - B.IDLER_OVERTRAVEL
    check("preload spring is pre-compressed when released", free > rel_len,
          f"free {free} vs {rel_len:.1f}")
    check("preload spring stays >= 1 mm off solid when engaged", eng_len - solid >= 1.0,
          f"{eng_len:.1f} vs solid {solid}")
    ridge = 2.0
    ridge_top = B.CAM_Z - B.CAM_BASE_R
    check("follower ridge touches the cam base circle when released",
          abs(B.FOLLOWER_Z_REL + B.FOLLOWER_H + B.RIDGE_H - ridge_top) < 1e-6)

    def engaged(servo, lobe):
        d = servo - lobe
        for phi in np.linspace(d - B.CAM_NOSE_HALF, d + B.CAM_NOSE_HALF, 61):
            r = math.radians(phi)
            if abs(B.CAM_NOSE_R * math.sin(r)) <= ridge and \
                    B.CAM_Z - B.CAM_NOSE_R * math.cos(r) < ridge_top:
                return True
        return False

    for j, a in enumerate(B.CAM_LOBE_ANGLES):
        got = [i + 1 for i, lobe in enumerate(B.CAM_LOBE_ANGLES) if engaged(a, lobe)]
        check(f"servo {a} deg engages only lane {j + 1}", got == [j + 1], f"engaged {got}")
    got = [i + 1 for i, lobe in enumerate(B.CAM_LOBE_ANGLES) if engaged(B.CAM_NEUTRAL, lobe)]
    check(f"servo {B.CAM_NEUTRAL} deg (neutral) engages none", not got, f"engaged {got}")

    print("== Envelope")
    mmu = [s for (n, s) in placed if n.split("@")[0] in
           ("mmu_body", "idler_block", "idler_follower", "selector_camshaft", "servo_coupler",
            "servo_mount", "merger", "mcu_carrier", "electronics_cover")]
    zmax = max(s.bounding_box().max.Z for s in mmu)
    xmin = min(s.bounding_box().min.X for s in mmu)
    xmax = max(s.bounding_box().max.X for s in mmu)
    motor = next(e for e in B.EXTRAS if e["name"] == "feeder_motor")
    servo = next(e for e in B.EXTRAS if e["name"] == "selector_servo")
    xmax = max(xmax, motor["pos"][0] + motor["size"][0] / 2)
    xmin = min(xmin, servo["pos"][0] - servo["size"][0] / 2)
    check("MMU <= 85 mm above the top bar", zmax - B.TOPBAR_TOP_Z <= 85,
          f"{zmax - B.TOPBAR_TOP_Z:.0f} mm")
    check("MMU fits between the uprights (|x| < 145)", xmin > -145 and xmax < 145,
          f"x {xmin:.0f} .. {xmax:.0f}")

    print("== PTFE routing")
    for e in B.EXTRAS:
        if e["kind"] != "tube":
            continue
        pts = catmull_rom(np.array(e["points"], float), 4)
        r = min_bend_radius(pts)
        check(f"{e['name']}: bend radius >= {B.MIN_PTFE_BEND_R} mm",
              r >= B.MIN_PTFE_BEND_R, f"min {r:.0f} mm")
        hit = [n for n, s in placed for p in pts[8:-8:6] if s.is_inside(Vector(*p))]
        check(f"{e['name']}: does not pass through printed parts", not hit,
              f"hits {sorted(set(hit))}" if hit else "")
    for i in range(4):
        p = B.feed_tube_points(i)
        check(f"feed tube {i + 1} ends at MMU inlet {i + 1}",
              abs(p[-1][0] - B.LANE_X[i]) < 1e-6 and abs(p[-1][2] - zf) < 1e-6)
    check("Bowden starts at the merger outlet", tuple(B.BOWDEN_POINTS[0]) == B.MMU_OUTLET)
    lengths = {e["name"]: float(np.sum(np.linalg.norm(np.diff(np.array(e["points"]), axis=0),
                                                      axis=1)))
               for e in B.EXTRAS if e["kind"] == "tube"}
    print("INFO PTFE lengths (mm):", {k: round(v) for k, v in lengths.items()})

    n_fail = results.count(False)
    print(f"\n{len(results) - n_fail}/{len(results)} checks passed, {n_fail} failed.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
