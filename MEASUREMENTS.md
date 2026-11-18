# Ender-3 V2 measurements + gates

**Machine: Creality Ender-3 V2 (24 V).** `VERIFIED` = spec sheet; `FAMILY` =
Ender-3 family value; `TO VERIFY` = measure before printing what depends on it.

## 1. Machine + frame

| Item | Value | Status |
|---|---|---|
| Build volume | 220 × 220 × 250 | VERIFIED |
| PSU | Mean Well LRS-350-24, 14.6 A | VERIFIED |
| Top crossbar | 2020 × 330 — **MMU mount (2 saddle clamps, M5 T-nuts)** | FAMILY (G2/G3) |
| Z uprights | 2040 × 420, inner clear ±145 | FAMILY |
| Extruder | Bowden, MK8, on the left end of the X gantry (moves in Z) | VERIFIED |
| Bowden | 4/2 PTFE, ~380–420 | TO VERIFY |

**Datum:** top-bar top Z 465, clamp slab 8 mm → **MMU body bottom Z 473**.
Front = −Y. **Envelope** (checked): x −128…+120, y −135…+35, ≤ 83 mm above
the top bar.

> **Mains safety:** the PSU tap is next to the mains input in the base.
> Unplug, wait a minute, insulate the tap (ring terminals + heat-shrink).

## 2. Board

| Item | Value | Status |
|---|---|---|
| Mainboard | 4.2.2 or 4.2.7; MCU **STM32F103RET6 or GD32F303** | TO VERIFY (G1) |
| Drivers | 4.2.7 TMC2225; 4.2.2 TMC2208 or HR4988/A4988 | TO VERIFY (G1) |

## 3. Filament path (from the CAD; Happy Hare calibrates the rest)

| Segment | mm |
|---|---|
| Lane gear → merger outlet | 147.4 / 136.5 / 136.5 / 147.4 (lanes 1–4) |
| Lane gear → merger entry (body front) | 35 — park tips ~15 past the gear |
| Merger outlet → extruder | ~450 PTFE (cut to fit, gate G8) |

## 4. Gates

- [ ] **G1** — Board version, MCU chip (STM32 vs GD32), driver type.
- [ ] **G2** — Top bar 330; MMU (~250 wide) fits between the uprights.
- [ ] **G3** — Top bar 20 × 20, slot 6 mm (clamp channel 20.6).
- [ ] **G4** — Klipper baseline: single-colour cube as good as on Marlin.
- [ ] **G5** — Extruder rotation_distance + PID done; Bowden length noted.
- [ ] **G6** — 24 V tap with a meter, 3 A fuse, peak draw measured.
- [ ] **G7** — MG996R in hand: ear holes 49.5 × 10, spline 10 mm off centre; angle calibration.
- [ ] **G8** — Outlet PTFE works at Z 0 and Z 250 (no kink, no tension).
