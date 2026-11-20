# Plan — Ender-MMU v0.4 (Klipper + Happy Hare)

## 1. Why Klipper

v0.3 kept Marlin, which forced a bump-switch "pulse code", a fixed 42 s
blind wait and no feedback. Klipper removes all three: the MMU is just more
Klipper hardware, Happy Hare sequences every change from real sensor and
motor state, and a failed load pauses the print. Cost: reflashing the Ender
board (reversible) and losing the stock DWIN screen UI (replaced by
KlipperScreen on your display).

## 2. Architecture

```
Raspberry Pi 3B/4 + display — at the printer: Klipper, Moonraker, Mainsail,
   │                          Happy Hare, KlipperScreen
   ├─ USB → Ender-3 V2 board (Klipper firmware): X/Y/Z/E, heaters, fans
   └─ USB → Pico in the MMU electronics bay (covered, vented)
              ├─ TMC2209 (UART) → NEMA 17 → D-shaft, 4 MK8 gears
              ├─ MG996R (6 V buck) → camshaft → spring-loaded idlers
              │     40/75/110/145° = gate 0-3, 0° = all released
              └─ KW11 roller gate sensor on the merger outlet
spools ×4 on roller cradles (desk, behind) → PTFE → MMU inlets → gears
  → merger (PTFE-lined lanes) → PTFE ~530 mm → stock extruder → hotend
```

The mechanism: spring-loaded idlers (cam → follower → preload springs →
bearing), a gate sensor with 1.9 mm of lever travel, PTFE-lined merger,
spool cradles with guides, and a covered electronics bay. `cad/check_mmu.py`
and `tests/` check geometry, fits, PTFE bend radii, grip force, servo torque
and Bowden slack over the full Z travel.

## 3. A tool change (Happy Hare)

Slicer emits `T1` → Happy Hare: park over the tower, form the tip (or let
the slicer do it), extruder retracts out of the hotend → selector to the old
lane, gear stepper pulls it back until the gate sensor clears, then to its
park position → selector to the new lane, feed to the gate sensor, then the
calibrated Bowden length fast → extruder + gear load together to the nozzle →
selector released → wipe tower purge. Every step waits for real completion;
any mismatch pauses the print.

Expected change time for this class of MMU: ~20–30 s plus purge.

## 4. Risks (ranked)

1. **Tip doesn't clear the merger** → jam. Tune tip forming; channels are
   2.2 mm with ≤ 33° bends.
2. **Lane slip** (idler preload) → Happy Hare detects it at the gate sensor;
   tune `IDLER_GAP` / `CAM_NOSE_R` (CAD) and the preload spring rate.
3. **Servo can't hold against the idler springs** → MG996R ≥ 9 kg·cm, kept
   powered while a lane is engaged.
4. **Board MCU variant** (STM32F103 vs GD32F303) → wrong Klipper build.
   Gate G1.
5. **Slack at the spool side** (~1 m pushed back per unload) → free rollers,
   no PTFE on the last ~600 mm before each inlet so slack loops freely.

## 5. Phases (don't skip)

1. **Klipper on the plain Ender** (no MMU): flash the board, install
   Klipper/Moonraker/Mainsail/KlipperScreen on the Pi, use the official
   Ender-3 V2 config, tune e-steps (rotation_distance), PID, and print a
   single-colour calibration cube as good as on Marlin. Keep the stock
   Marlin `firmware.bin` to go back.
2. **Print `fit_coupon`, then the calibration set:** `mmu_body`, one
   `idler_block` + `idler_follower` with its springs and limit screws,
   `selector_camshaft`. Turn the cam by hand: the lane grips at its angle
   (springs compress ~1.5 mm) and the bearing lifts clear at 0°. Push
   filament through every lane and the `merger`.
3. **Bench the MMU MCU:** flash the Pico, wire TMC2209 + servo + gate
   sensor, install Happy Hare (Servo_Selector type), calibrate servo angles,
   gear rotation_distance, gate positions.
4. **Mount + calibrate:** Bowden length, toolhead, tip forming per filament.
5. **Two-colour test:** 10 changes, 100 %. **Four-colour:** 40 changes.

## 6. Cost

~$100–115 bought parts excluding the Pi and display (BOM.md), vs Coprint
ChromaSet Lite 4-colour $329.
