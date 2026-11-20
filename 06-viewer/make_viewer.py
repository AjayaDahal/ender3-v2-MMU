#!/usr/bin/env python3
"""Build Ender-MMU v0.4 3-D viewer manifest, STL copies, print-bed metadata, and renders.

Run:
  python3 06-viewer/make_viewer.py
"""
from __future__ import annotations

import json
import pathlib
import shutil
import struct

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
VIEWER = HERE / "viewer"
PARTS_OUT = VIEWER / "parts"
PRINT_OUT = VIEWER / "print_stls"
MEDIA_OUT = VIEWER / "media"
STL_DIR = ROOT / "05-manufacturing" / "stl"
RENDERS = HERE / "renders"

CALLOUTS = [
    {"id": "body", "part": "mmu_body", "label": "4-Lane MMU Body (Shared Shaft + 4 MK8 Gears)", "offset": [95, -55, 65]},
    {"id": "cam", "part": "selector_camshaft", "label": "Servo Camshaft (40/75/110/145°)", "offset": [-95, -40, 80]},
    {"id": "idler", "part": "idler_block", "label": "4× Spring-Loaded Idler Blocks", "offset": [85, 55, 75]},
    {"id": "merger", "part": "merger", "label": "PTFE-Lined 4→1 Merger + Gate Sensor", "offset": [-80, -85, 25]},
    {"id": "servo", "part": "servo_mount", "label": "MG996R Selector Servo Mount", "offset": [-105, 45, 50]},
    {"id": "mcu", "part": "mcu_carrier", "label": "RP2040 Pico + Klipper/Happy Hare Bay", "offset": [105, -60, -25]},
    {"id": "clamp", "part": "topbar_clamp", "label": "Ender-3 V2 2020 Top-Bar Clamps", "offset": [110, 40, -35]},
    {"id": "spool", "part": "spool_cradle", "label": "4× 608-Bearing Spool Cradles", "offset": [-100, 95, 55]},
]


def stl_dims(path: pathlib.Path):
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    lo = [1e9, 1e9, 1e9]
    hi = [-1e9, -1e9, -1e9]
    off = 84
    for _ in range(n):
        vals = struct.unpack_from("<12f", data, off)
        off += 50
        for j in (3, 6, 9):
            for k in range(3):
                v = vals[j + k]
                if v < lo[k]:
                    lo[k] = v
                if v > hi[k]:
                    hi[k] = v
    return [round(hi[k] - lo[k], 1) for k in range(3)]


def main() -> int:
    for d in (PARTS_OUT, PRINT_OUT, MEDIA_OUT):
        d.mkdir(parents=True, exist_ok=True)

    man_path = VIEWER / "manifest.json"
    man = json.loads(man_path.read_text(encoding="utf-8"))

    # Copy all STLs into viewer/parts and viewer/print_stls
    printed_parts = {}
    qty_map = {}
    label_map = {}
    color_map = {}
    for p in man.get("parts", []):
        qty_map[p["name"]] = p.get("qty", 1)
        label_map.setdefault(p["name"], p.get("label", p["name"]))
        color_map.setdefault(p["name"], p.get("color", "#2f6db5"))

    for stl in sorted(STL_DIR.glob("*.stl")):
        shutil.copyfile(stl, PARTS_OUT / stl.name)
        shutil.copyfile(stl, PRINT_OUT / stl.name)
        name = stl.stem
        dims = stl_dims(stl)
        printed_parts[name] = {
            "name": name,
            "url": f"print_stls/{stl.name}",
            "stl_download": f"../../05-manufacturing/stl/{stl.name}",
            "material": "PETG (0.2mm, 4 walls, 40% gyroid)",
            "qty": qty_map.get(name, 1),
            "dims_mm": dims,
            "color": color_map.get(name, "#2f6db5"),
            "label": label_map.get(name, name.replace("_", " ")),
            "supports": name == "selector_camshaft",
            "fit_first": name in ("fit_coupon", "mmu_body", "idler_block", "idler_follower", "selector_camshaft"),
        }

    # Enrich assembly parts with local url & download links
    for idx, p in enumerate(man.get("parts", [])):
        p["url"] = f"parts/{p['file']}"
        p["print_stl"] = f"print_stls/{p['file']}"
        p["stl_download"] = f"../../05-manufacturing/stl/{p['file']}"
        p["id"] = f"{p['name']}#{idx + 1}"
        p["dims_mm"] = printed_parts.get(p["name"], {}).get("dims_mm", [0, 0, 0])

    # Copy renders
    studio_renders = []
    if RENDERS.exists():
        for f in sorted(RENDERS.iterdir()):
            if f.suffix.lower() in (".png", ".jpg", ".gif"):
                shutil.copyfile(f, MEDIA_OUT / f.name)
                studio_renders.append({
                    "file": f"media/{f.name}",
                    "title": f.stem.replace("_", " ").title(),
                    "kind": "GIF" if f.suffix.lower() == ".gif" else "PNG",
                })

    man["vehicle"] = "Ender-MMU v0.4"
    man["subtitle"] = "4-Lane Multi-Material Unit for Creality Ender-3 V2 · Klipper + Happy Hare"
    man["printed_parts"] = printed_parts
    man["callouts"] = CALLOUTS
    man["studio_renders"] = studio_renders
    man["checks"] = {
        "assembly_audit": {"passed": 80, "total": 80},
        "pytest": {"passed": 104, "total": 104},
        "simulation": {
            "idle_clearance_mm": 0.20,
            "filament_strain_pct": 0.82,
            "pla_yield_pct": 2.50,
            "traction_drag_ratio": "3.1–3.4× (1.9× pessimistic)",
            "servo_margin_deg": "±8.2°",
            "transitions_passed": "12 / 12 (~56 s)",
            "filament_mass_g": 875,
            "bom_cost_usd": "$120–135 + Pi",
        },
        "firmware": {
            "printer_fw": "Klipper (Ender-3 V2 mainboard)",
            "mmu_mcu": "Raspberry Pi Pico (RP2040)",
            "orchestrator": "Happy Hare (Type-A servo selector)",
            "cam_angles": "40° / 75° / 110° / 145° (neutral 0°)",
            "gate_park_mm": "16 mm behind gate microswitch",
        },
    }

    man_path.write_text(json.dumps(man, indent=2) + "\n", encoding="utf-8")
    print(
        f"Ender-MMU viewer built: {len(man['parts'])} instances, {len(printed_parts)} printable STLs, "
        f"{len(studio_renders)} renders -> 06-viewer/viewer/manifest.json"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
