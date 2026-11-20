"""Printability analysis shared by tests/test_printability.py.

Everything is measured on the exported STLs in the orientation each part is
actually printed in (PRINT_ORIENT), so the numbers match what the slicer sees.
"""

import math
import os

import numpy as np
import trimesh

BASE = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(BASE, "..", "05-manufacturing", "stl")

NOZZLE = 0.4
MIN_WALL = 1.2          # 3 perimeters at 0.4 mm
OVERHANG_DEG = 45       # steeper than this from vertical needs support or a bridge
MAX_BRIDGE = 10.0       # mm, safe PETG bridge length

# Print orientation (Euler XYZ deg) and whether supports are planned.
PRINT_ORIENT = {
    "mmu_body": ((0, 0, 0), False),         # teardrop holes, open gear pockets
    "idler_block": ((0, 0, 0), False),
    "idler_follower": ((0, 0, 0), False),
    "fit_coupon": ((0, 0, 0), False),
    "selector_camshaft": ((0, 0, 0), True),  # lobe noses
    "servo_coupler": ((0, -90, 0), False),   # standing on the horn face
    "servo_mount": ((0, 90, 0), False),      # on its 50x75 face
    "merger": ((0, 0, 0), False),
    "mcu_carrier": ((0, 0, 0), False),
    "electronics_cover": ((180, 0, 0), False),  # top face on the bed
    "topbar_clamp": ((180, 0, 0), False),    # upside down, slab on the bed
    "spool_cradle": ((0, 0, 0), False),
    "spool_roller": ((0, 0, 0), False),      # standing on end
}


def load(name):
    mesh = trimesh.load(os.path.join(STL_DIR, f"{name}.stl"))
    rot, _ = PRINT_ORIENT[name]
    m = trimesh.transformations.euler_matrix(*[math.radians(a) for a in rot], "sxyz")
    mesh.apply_transform(m)
    mesh.apply_translation(-mesh.bounds[0])
    return mesh


def min_wall(mesh, samples=6000, seed=1, limit=MIN_WALL):
    """Ray-cast inward from points on the surface; the distance to the next
    surface is the local wall thickness. Returns (min, points below limit)."""
    pts, idx = trimesh.sample.sample_surface_even(mesh, samples, seed=seed)
    normals = mesh.face_normals[idx]
    origins = pts - normals * 1e-3
    locs, ray_idx, tri_idx = mesh.ray.intersects_location(
        origins, -normals, multiple_hits=False)
    # Only count opposing faces: a ray leaving through an adjacent face near
    # an edge or chamfer is a corner, not a thin wall.
    opposing = np.einsum("ij,ij->i", normals[ray_idx], mesh.face_normals[tri_idx]) < -0.7
    d = np.full(len(pts), np.inf)
    d[ray_idx[opposing]] = np.linalg.norm(
        locs[opposing] - origins[ray_idx[opposing]], axis=1)
    thin = pts[d < limit]
    return float(d.min()), thin


def overhang_area(mesh):
    """Area (mm^2) of lifted downward faces steeper than OVERHANG_DEG from
    vertical, excluding flat ceilings (those are bridges, see bridges())."""
    n = mesh.face_normals
    limit = -math.cos(math.radians(90 - OVERHANG_DEG - 1))  # 1 deg slack for 45 deg roofs
    down = (n[:, 2] < limit) & (n[:, 2] > -0.999)
    lifted = mesh.triangles_center[:, 2] > 1.0   # bottom-edge chamfers are fine
    return float(mesh.area_faces[down & lifted].sum())


def bridges(mesh):
    """Flat lifted ceilings, grouped into connected regions. Returns a list
    of (span_mm, area_mm2, centre) where span is the shorter XY extent."""
    flat = np.where((mesh.face_normals[:, 2] < -0.999)
                    & (mesh.triangles_center[:, 2] > 0.2))[0]
    if not len(flat):
        return []
    adj = mesh.face_adjacency
    keep = np.isin(adj, flat).all(axis=1)
    groups = trimesh.graph.connected_components(adj[keep], nodes=flat)
    out = []
    for g in groups:
        v = mesh.triangles[g].reshape(-1, 3)
        ext = v.max(0) - v.min(0)
        out.append((float(min(ext[0], ext[1])), float(mesh.area_faces[g].sum()),
                    v.mean(0)))
    return out
