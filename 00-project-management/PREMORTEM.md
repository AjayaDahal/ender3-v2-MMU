# Premortem — Ender-MMU v0.4

*Imagine it's three months from now. The parts are printed, the MMU is on
the printer, and it doesn't work. Why?* Each failure story now has a design
answer; **[test]** marks the ones `tests/test_printability.py` or
`cad/check_mmu.py` enforce.

## Verdict

**Ready for a staged print, not a blind full print.** Print in this order and
stop if a step fails:

1. `fit_coupon` — tune `HOLE_COMP` until the 625 bearing, 5 mm D-shaft, 6 mm
   rod, M3 insert, PC4-M6/M10 (tapped), PTFE and filament fit.
2. Calibration set: `mmu_body`, one `idler_block` + `idler_follower`,
   `selector_camshaft` — check grip and release by hand (PLAN.md §5).
3. Everything else.

What tests can't prove: real spring rates, PETG creep, servo torque, Klipper
on your board. Those are bench steps, listed below.

## Failure stories → fixes

| # | What went wrong | Fix in the design | Guard |
|---|---|---|---|
| 1 | Lanes slip or grind | **Spring-loaded idler**: cam → `idler_follower` → 2 firm preload springs → `idler_block` → bearing. Bearing clears the filament by 1.5 mm when released; the cam's 3 mm lift closes that and compresses the springs 1.5 mm (~5 N each). Two M3 limit screws keep them pre-compressed. | **[test]** gap, overtravel, pre-compression, solid-length margin, follower never bottoms |
| 2 | Gate sensor never triggers | Roller-lever KW11 rests 1 mm above the channel axis in a widened chamber; the filament pushes it down **1.9 mm** (shim the switch to set the rest height) | **[test]** travel ≥ 1.5 mm, roller inside the channel |
| 3 | Gears walk on the shaft | **5 mm D-shaft**; set screws on the flat | **[test]** BOM |
| 4 | Tip snags at the merge | Each merger lane is **PTFE-lined** to 15 mm before the merge; 2.5 mm reamed convergence; ≤ 33° bends | **[test]** bend angle, path open, walls |
| 5 | Holes print tight or oval | All horizontal holes are teardrops with `HOLE_COMP`; `fit_coupon` to tune it | **[test]** teardrop shape, coupon exists |
| 6 | Grub screws strip | Camshaft bosses and coupler take **M3 × 4 heat-set inserts**; file a flat on the 6 mm rod | **[test]** BOM |
| 7 | Servo can't reach or hold a lane | MG996R lies along Y with its spline on the cam axis (offset modelled); Happy Hare keeps it powered while engaged | **[check]** envelope; bench G7 |
| 8 | Body warps / supports trapped | Body and merger print **support-free** (teardrops, pockets open into the slots); brim in BOM notes | **[test]** overhang + bridge |
| 9 | Spool tangles from unload slack | Free-spinning rollers + a loose loop (no PTFE on the last ~600 mm before each inlet); optional Happy Hare espooler later | operational |
| 10 | Klipper won't run on the board | Gate G1 (STM32F103 vs GD32F303) before flashing; keep stock firmware | gate |
| 11 | Thin walls / unprintable skins | Found and fixed: 0.04 mm membranes, 0.4 mm spring walls, teardrop side horns, 0.5 mm follower roofs | **[test]** ray-cast wall ≥ 1.2 mm (body 1.0, idler 0.9 with reasons) |
| 12 | Parts don't fit together | ≥ 0.2 mm/side on every sliding fit | **[test]** |
| 13 | BOM doesn't match the CAD | Print table, pin length, D-shaft, inserts checked against the CAD | **[test]** |
| 14 | No way to get filament in / incomplete system | Spool cradles (2 rollers on 608s + PC4-M6 guide), PTFE feed tubes to each inlet, Bowden to the stock extruder, covered electronics bay; the Pi + display stay at the printer | **[check]** tubes end at the fittings |
| 15 | PTFE kinks / Bowden pulls tight at low Z | Routes are tangent to each fitting, min bend radius ≥ 40 mm; Bowden has ≥ 10 % slack at Z 0 and Z max | **[check]** bend radius, **[test]** slack |
| 16 | Grip too weak to pull filament, or servo stalls | Grip 10–30 N from 2.5 N/mm preload springs; servo torque ≥ 4× the cam load; holding force ≥ 3× worst Bowden drag | **[test]** test_mechanism.py |
| 17 | Servo can't reach a lane | Angles 40/75/110/145, neutral 0 — all inside 0–160°, ≥ 5° outside the neighbour's engagement window | **[test]** |
| 18 | Loading one lane drags the other three | Found by the simulation: idle filaments sat 0.58 mm into the teeth of the shared, turning gears. Gears now sit 0.2 mm below the channel floor; the idler bends the filament 0.45 mm onto its gear over a 24 mm free span (0.8 % strain, elastic) | **[sim]** |
| 19 | Gear slips / stock extruder skips | Found by the simulation: 570° of PTFE bends. Angled spool guides (feed 98°), a shorter-turning Bowden route (195°), 4 N/mm preload springs, gear synced to the extruder while printing → traction ≥ 3× load drag | **[sim]** |
| 20 | Gate homing gives up / tips collide at the merge | Found by the simulation: tips parked 100 mm before the shared gate sensor. Now park 16 mm behind it in their own lane; all 12 changes run with one tip in the shared path at a time | **[sim]** |
| 21 | Bowden pulls tight at the bottom of Z | Found by the tests: the shorter route was 2 mm short at Z min. Route lengthened to 571 mm (≥ 1.1× reach at Z min) | **[test]** |

## Why the STLs looked like blocks

Function-first rectangular parts with features cut inside, and every bought
part drawn as a plain box in the viewer. Cosmetic surfacing (fillets, ribs,
vendor models) is still to do and doesn't affect function.

## Run the checks

```bash
.venv/bin/python cad/build_mmu.py
.venv/bin/python cad/check_mmu.py        # assembly + mechanism
.venv/bin/python -m pytest -q tests      # printability, fits, BOM
```
