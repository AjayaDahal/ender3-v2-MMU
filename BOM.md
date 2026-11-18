# BOM — Ender-MMU v0.4 (Klipper, 4 lanes)

US online-retail estimates, Nov 2026. Already owned: Raspberry Pi 3B/4, display.

## Buy — electronics + motion

| Qty | Part | Note | Est. |
|---|---|---|---|
| 1 | Raspberry Pi Pico (RP2040) + micro-USB cable | Klipper MCU for the MMU | $6 |
| 1 | TMC2209 StepStick + heatsink | UART mode | $7 |
| 1 | 1 kΩ resistor, 100 µF 35 V cap | UART link, driver supply | $0.5 |
| 1 | NEMA 17, 42-40, ≥ 45 N·cm, 5 mm shaft | gear stepper | $10 |
| 4 | MK8 drive gear 40T, 5 mm bore (~11 mm) | one per lane | $8 |
| 1 | 5 mm **D-shaft** 150 mm, 2× 625ZZ, 5–5 flexible coupler | gear set screws bite on the flat | $10 |
| 4 | 623ZZ + 3 × 12 dowel pin | idlers (pin = block width) | $4 |
| 8 | Compression spring 4 OD × 10, **light** (~0.3 N/mm) | idler return, under each block | $2 |
| 8 | Compression spring 4 OD × 10, **firm** (~4 N/mm), solid ≤ 3.5 | idler preload, ~14 N per spring engaged | $3 |
| 8 | M3 × 12 socket head | idler limit screws (follower → block) | — |
| 1 | 6 mm steel rod 150 mm, file a flat | cam rod | $3 |
| 1 | MG996R metal-gear servo | selector | $5 |
| 1 | KW11 microswitch with **roller** lever + 2× M2 × 10 + shims | gate sensor (roller rests 1 mm above the channel axis) | $1.5 |
| 1 | Buck 24 → 6 V ≥ 3 A (fits 43 × 21) | servo, inside the MMU electronics bay | $5 |
| 1 | 40 × 60 perfboard, 2× 8-pin female headers | TMC2209 carrier | $2 |
| 1 | 3 A fuse + holder | 24 V tap | $2 |
| 2 | USB cables (Pi → Ender board, Pi → Pico) | | $5 |
| — | Wire, JST-XH, Dupont | | $5 |
| 16 | 608ZZ | spool rollers, 2 per roller | $12 |
| 8 | 8 mm steel rod × 92 mm | roller axles | $8 |

## Buy — filament path + fasteners

| Qty | Part | Est. |
|---|---|---|
| 8 / 1 | PC4-M6 (4 MMU inlets + 4 spool guides) / PC4-M10 (outlet), tap M6×1 / M10×1 | $14 |
| 3 m | PTFE 4/2 (Capricorn for PETG): 4× ~530 feed tubes, ~570 Bowden to the extruder, merger liners 2× 63 + 2× 52 | $8 |
| 1 | 2.5 mm drill (ream the merger channel) | $1 |
| 16 | M3 heat-set inserts (4.0 mm holes) | $3 |
| 3 | M3 × 4 heat-set inserts + M3 × 4 grub screws | camshaft (2), coupler (1) | $2 |
| — | M3 screws: 4×8 (NEMA), 4×10 (clamps), 2×30 (servo mount), 2×16 (merger), 4×40 (cover + carrier → merger), 4×12 (servo ears) | $5 |
| — | M2 × 6 ×8 (Pico, horn), M2.5 × 6 ×4 (perfboard) | $2 |
| 6 | M5 spring T-nut + M5 × 10 | $3 |

**Bought total ≈ $120–135** (Pi + display owned; the Pi is powered by its own supply at the printer).

## Print — PETG, 0.2 mm, 4 walls, 40 % gyroid

| Part | Qty | Size (mm) |
|---|---|---|
| `fit_coupon` | 1 | 80 × 12 × 24 — **print first**, tune `HOLE_COMP` |
| `mmu_body` | 1 | 160 × 70 × 71 |
| `idler_block` | 4 | 12 × 26 × 10 |
| `idler_follower` | 4 | 12 × 26 × 9.5 |
| `selector_camshaft` | 1 | 110 × 18 × 19 |
| `servo_coupler` | 1 | 19 dia × 16 |
| `servo_mount` | 1 | 20 × 66 × 77 |
| `merger` | 1 | 110 × 100 × 22 |
| `mcu_carrier` | 1 | 105 × 95 × 9 |
| `electronics_cover` | 1 | 105 × 95 × 30 (print top-down) |
| `topbar_clamp` | 2 | 30 × 60 × 28 |
| `spool_cradle` | 4 | 90 × 140 × 48 |
| `spool_roller` | 8 | 28 dia × 76 (print standing) |

≈ 1,570 cm³ solid → **~875 g PETG at 40 %** (the 4 spool cradles are ~440 g of it). Print with a brim; body and merger
need no supports, the camshaft needs supports under its lobes.
Print order: `fit_coupon` → calibration set (PLAN.md §5) → the rest.
