#!/usr/bin/env python3
"""
Shaded preview renders of the assembly (no Blender needed): reads
viewer/manifest.json + stl/, writes renders/*.png for the README.

Run:  .venv/bin/python tools/render_preview.py
"""

import json
import math
import os

import matplotlib
import numpy as np
import trimesh
from PIL import Image, ImageOps

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
STL = os.path.join(ROOT, "stl")
OUT = os.path.join(ROOT, "renders")
MAN = json.load(open(os.path.join(ROOT, "viewer", "manifest.json")))
LIGHT = np.array([-0.4, -0.7, 0.8])
LIGHT /= np.linalg.norm(LIGHT)
MMU_PARTS = {"mmu_body", "idler_block", "idler_follower", "selector_camshaft", "servo_coupler",
             "servo_mount", "merger", "mcu_carrier", "electronics_cover", "topbar_clamp"}


def hex_rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)])


def placed_parts(burst=0.0, only=None, ghost=()):
    cache, out = {}, []
    for p in MAN["parts"]:
        if only and p["name"] not in only:
            continue
        if p["file"] not in cache:
            cache[p["file"]] = trimesh.load(os.path.join(STL, p["file"]))
        m = cache[p["file"]].copy()
        rot = trimesh.transformations.euler_matrix(*[math.radians(a) for a in p["rot"]], "sxyz")
        m.apply_transform(rot)
        m.apply_translation(np.array(p["pos"]) + burst * np.array(p["burst"]))
        col = hex_rgb(p["color"])
        out.append((m, np.append(col, 0.18) if p["name"] in ghost else col))
    return out


def extra_meshes(names=None, skip=()):
    out = []
    for e in MAN["extras"]:
        if (names and e["name"] not in names) or any(e["name"].startswith(s) for s in skip):
            continue
        col = hex_rgb(e["color"])
        if e["kind"] == "box":
            m = trimesh.creation.box(extents=e["size"])
            m.apply_translation(e["pos"])
        elif e["kind"] == "cyl":
            m = trimesh.creation.cylinder(radius=e["r"], height=e["len"], sections=28)
            if e["axis"] == "x":
                m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            elif e["axis"] == "y":
                m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
            m.apply_translation(e["pos"])
        else:
            pts = np.array(e["points"], float)
            segs = []
            for a, b in zip(pts[:-1], pts[1:]):
                if np.linalg.norm(b - a) > 1e-6:
                    segs.append(trimesh.creation.cylinder(radius=e["r"], segment=[a, b], sections=8))
            m = trimesh.util.concatenate(segs)
        out.append((m, col))
    return out


def frame_meshes():
    out = []
    for f in MAN["frame"]:
        m = trimesh.creation.box(extents=f["size"])
        m.apply_translation(f["pos"])
        out.append((m, np.array([0.62, 0.66, 0.70])))
    return out


def render(meshes, fname, elev, azim, lims, size=(14, 9), title=None, dpi=110, max_edge=6.0):
    fig = plt.figure(figsize=size, dpi=dpi)
    ax = fig.add_subplot(projection="3d")
    polys, colors = [], []
    for m, col in meshes:
        # Small triangles sort correctly in matplotlib's painter's algorithm.
        edge = max_edge if max(m.extents) < 300 else max_edge * 5
        v, f = trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=edge, max_iter=8)
        m = trimesh.Trimesh(v, f, process=False)
        tri = m.triangles
        n = m.face_normals
        shade = 0.38 + 0.62 * np.clip(n @ LIGHT, 0, 1)
        polys.append(tri)
        rgb = np.clip(col[None, :3] * shade[:, None] + 0.05, 0, 1)
        alpha = np.full((len(tri), 1), col[3] if len(col) == 4 else 1.0)
        colors.append(np.hstack([rgb, alpha]))
    pc = Poly3DCollection(np.concatenate(polys), facecolors=np.concatenate(colors),
                          edgecolors="none", linewidths=0)
    ax.add_collection3d(pc)
    (x0, x1), (y0, y1), (z0, z1) = lims
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_zlim(z0, z1)
    ax.set_box_aspect((x1 - x0, y1 - y0, z1 - z0))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.patch.set_facecolor("white")
    if title:
        fig.suptitle(title, fontsize=15, weight="bold", y=0.93)
    plt.subplots_adjust(0, 0, 1, 1)
    path = os.path.join(OUT, fname)
    fig.savefig(path, facecolor="white", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)
    img = Image.open(path).convert("RGB")
    box = ImageOps.invert(img).getbbox()
    if box:
        pad = 20
        img.crop((max(0, box[0] - pad), max(0, box[1] - pad),
                  min(img.width, box[2] + pad), min(img.height, box[3] + pad))).save(path)
    print("wrote", path)


def main():
    os.makedirs(OUT, exist_ok=True)
    everything = frame_meshes() + placed_parts() + extra_meshes(skip=("pi_and",))
    render(everything, "01_overview.png", 22, -58,
           ((-260, 260), (-330, 430), (0, 560)),
           title="Ender-3 V2 + 4-colour MMU (Klipper + Happy Hare)")
    mmu = (placed_parts(only=MMU_PARTS)
           + extra_meshes(skip=("spool", "roller_rod", "feed_tube", "pi_and", "stock_extruder",
                                "bowden")))
    top_bar = [(m, c) for m, c in frame_meshes()[:1]]
    render(top_bar + mmu, "02_mmu_closeup.png", 24, -52,
           ((-140, 130), (-150, 60), (440, 560)), title="MMU on the top bar")
    render(top_bar + placed_parts(burst=1.0, only=MMU_PARTS)
           + extra_meshes(names={"feeder_motor", "selector_servo", "drive_shaft"}),
           "03_mmu_exploded.png", 26, -55,
           ((-200, 190), (-260, 90), (380, 680)), title="Exploded: every printed MMU part")
    no_cover = MMU_PARTS - {"electronics_cover", "mcu_carrier", "merger", "topbar_clamp"}
    render(placed_parts(only=no_cover, ghost={"mmu_body", "servo_mount"})
           + extra_meshes(names={"drive_shaft", "coupler_5_5", "cam_rod", "feeder_motor",
                                 "selector_servo", "bearing_625_L", "bearing_625_R",
                                 *[f"drive_gear_{i}" for i in range(1, 5)],
                                 *[f"idler_623_{i}" for i in range(1, 5)]}),
           "04_drive_train.png", 38, -35,
           ((-135, 125), (-140, 50), (470, 545)), title="Drive train: one NEMA 17, one shaft, four gears")
    rack = (placed_parts(only={"spool_cradle", "spool_roller"})
            + extra_meshes(names={*[f"spool_{i}" for i in range(1, 5)]}, skip=())
            + extra_meshes(names={f"roller_rod_{i}_{j}" for i in range(4) for j in range(2)}))
    render(rack, "05_spool_rack.png", 25, -60,
           ((-220, 220), (220, 460), (0, 250)), title="Spool cradles with PTFE guides")


if __name__ == "__main__":
    main()
