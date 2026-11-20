# CAD notes — Ender-MMU v0.4

`cad/build_mmu.py` (build123d) writes every STL in `stl/` and
`viewer/manifest.json` (assembly used by the viewer, renders and checks).
`cad/check_mmu.py` audits STLs **and** the assembly and must print `0 failed`.

## Checks

- STLs: watertight, fit 220×220×250, volume = source.
- Assembly: no part/part or part/frame overlap.
- Filament path open on every lane through the merger; channel walls present;
  gate-sensor lever slot reaches the outlet channel.
- Mechanism: gear meets the filament; released bearing clears it by
  `IDLER_GAP`; the cam lift compresses the preload springs (pre-compressed,
  never solid, follower never bottoms); each servo angle engages exactly one
  lane, 0° (neutral) engages none.
- Envelope: ≤ 85 mm above the top bar, between the uprights.
- Printability (`tests/`): wall thickness, overhangs, bridges, fits, teardrop
  shape, BOM consistency.

Bought parts are boxes in the manifest, not checked as solids.

## Key parameters

| Name | Value | Meaning |
|---|---|---|
| `LANE_X` | −42, −14, 14, 42 | lanes |
| `ZF` / `SHAFT_Z` / `CAM_Z` | 30 / 23.875 / 62.875 | filament, drive shaft, cam rod heights (derived) |
| `IDLER_GAP` / `CAM_LIFT` | 1.5 / 3.0 | release clearance / cam lift → 1.5 mm spring overtravel |
| `IDLER_PRELOAD_SPRING` | 4 OD × 10, solid 3.5 | firm preload springs, 2 per idler |
| `HOLE_COMP` | 0.1 | added to horizontal hole radii — tune with `fit_coupon` |
| `CAM_LOBE_ANGLES` / `CAM_NEUTRAL` | 40/75/110/145 / 0 | servo angle per gate / release |
| `MERGE_Y` / `GATE_SENSOR_Y` | −100 / −106 | lanes meet / gate sensor |
| `Z0` | 473 | body bottom (world) |

## Holes

Heat-set M3 4.0 mm; M3 thread-forming 2.5; PC4-M6 5.0 (tap M6×1);
PC4-M10 9.0 (tap M10×1); filament 2.2; PTFE seats 4.1.

## Simplifications

Gears, bearings, springs, rods not modelled as solids (pockets sized for
them). MG996R spline offset / ear pattern are catalogue values (gate G7).
Only outer boxes are chamfered (`cbox`), before any boolean.
