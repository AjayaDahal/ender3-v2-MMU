"""Render the assembled Ender-MMU on the Ender-3 V2 frame in Blender.

Reads viewer/manifest.json (written by cad/build_mmu.py), so the renders
always match the CAD layout. Paths are relative to this file.

Run headless:  blender -b -P render_mmu.py
Test first:    TEST=1 blender -b -P render_mmu.py   (50 %, 2 views)
One view:      ONLY=04_exploded blender -b -P render_mmu.py
"""
import json
import math
import os

import bpy
from mathutils import Euler, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(HERE, "stl")
OUT_DIR = os.path.join(HERE, "renders")
with open(os.path.join(HERE, "viewer", "manifest.json")) as f:
    MAN = json.load(f)
os.makedirs(OUT_DIR, exist_ok=True)

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE_NEXT"
scene.render.resolution_x, scene.render.resolution_y = 1920, 1080
scene.render.image_settings.file_format = "PNG"
world = bpy.data.worlds.new("World")
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.96, 0.96, 0.96, 1)
scene.world = world


def hex_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))


_mats = {}


def mat(color, rough=0.55):
    if color not in _mats:
        m = bpy.data.materials.new(color)
        m.use_nodes = True
        bsdf = m.node_tree.nodes["Principled BSDF"]
        bsdf.inputs["Base Color"].default_value = (*hex_rgb(color), 1)
        bsdf.inputs["Roughness"].default_value = rough
        _mats[color] = m
    return _mats[color]


def euler(rot_deg):
    return Euler([math.radians(a) for a in rot_deg], "XYZ")


placed = []
for i, p in enumerate(MAN["parts"]):
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.wm.stl_import(filepath=os.path.join(STL_DIR, p["file"]))
    obj = bpy.context.active_object
    obj.name = f"{p['name']}_{i}"
    obj.scale = (0.001,) * 3
    bpy.ops.object.transform_apply(scale=True)
    bpy.ops.object.shade_smooth_by_angle(angle=0.5236)
    obj.rotation_euler = euler(p["rot"])
    obj.data.materials.append(mat(p["color"]))
    placed.append((obj, Vector(p["pos"]) * 0.001, Vector(p["burst"]) * 0.001))


def box(name, size, pos, color, rough=0.6):
    bpy.ops.mesh.primitive_cube_add(size=1, location=Vector(pos) * 0.001)
    obj = bpy.context.active_object
    obj.name = name
    obj.dimensions = Vector(size) * 0.001
    obj.data.materials.append(mat(color, rough))


def cyl(name, r, length, pos, axis, color):
    rot = {"x": (0, math.pi / 2, 0), "y": (math.pi / 2, 0, 0), "z": (0, 0, 0)}[axis]
    bpy.ops.mesh.primitive_cylinder_add(radius=r * 0.001, depth=length * 0.001,
                                        location=Vector(pos) * 0.001, rotation=rot)
    bpy.context.active_object.name = name
    bpy.context.active_object.data.materials.append(mat(color, 0.4))


def tube(name, points, r, color):
    data = bpy.data.curves.new(name, "CURVE")
    data.dimensions, data.bevel_depth = "3D", r * 0.001
    spline = data.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for sp, p in zip(spline.points, points):
        sp.co = (*(Vector(p) * 0.001), 1)
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(mat(color, 0.3))


for f in MAN["frame"]:
    box(f["name"], f["size"], f["pos"], "#9aa3ad", 0.8)
for e in MAN["extras"]:
    if e["kind"] == "cyl":
        cyl(e["name"], e["r"], e["len"], e["pos"], e["axis"], e["color"])
    elif e["kind"] == "tube":
        tube(e["name"], e["points"], e["r"], e["color"])
    else:
        box(e["name"], e["size"], e["pos"], e["color"], 0.5)
bpy.ops.mesh.primitive_plane_add(size=10, location=(0, 0, -0.001))
bpy.context.active_object.data.materials.append(mat("#e6e6e6", 1.0))


def aim(obj, loc, target):
    obj.location = loc
    obj.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


for name, energy, loc in (("key", 900, (2.5, -3.0, 4.0)), ("fill", 300, (-3.0, 2.0, 3.0))):
    bpy.ops.object.light_add(type="AREA")
    light = bpy.context.active_object
    light.name, light.data.energy, light.data.size = name, energy, 3.0
    aim(light, loc, (0, 0, 0.3))

# Front is -Y: cameras on the -Y side see X=0 (left upright) on the image left.
VIEWS = [
    ("01_isometric", (-0.95, -1.15, 0.95), (0, 0, 0.35), False),
    ("02_front", (0, -0.60, 0.55), (0, -0.04, 0.50), False),
    ("03_top", (0, 0, 1.70), (0, 0, 0.40), False),
    ("04_exploded", (0.95, -1.15, 0.95), (0, 0, 0.42), True),
    ("05_mmu_closeup", (0.32, -0.45, 0.78), (0, -0.04, 0.50), False),
]
if os.environ.get("TEST"):
    scene.render.resolution_percentage = 50
    VIEWS = VIEWS[:2]
if os.environ.get("ONLY"):
    VIEWS = [v for v in VIEWS if os.environ["ONLY"] in v[0]]
for name, loc, target, exploded in VIEWS:
    for obj, base, burst in placed:
        obj.location = base + (burst if exploded else Vector())
    bpy.ops.object.camera_add()
    cam = bpy.context.active_object
    cam.data.lens = 50
    aim(cam, loc, target)
    scene.camera = cam
    scene.render.filepath = os.path.join(OUT_DIR, name + ".png")
    bpy.ops.render.render(write_still=True)
    print("rendered", name)
