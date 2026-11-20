# Ender-MMU v0.2 — Review: what must be fixed

## Resolution status (v0.3, 2026-11-17)

The v0.2 files are archived in `.old_v02/`. v0.3 redesigns the mechanism, and
`cad/check_mmu.py` now checks the assembly (53/53 pass).

| Items | Resolution |
|---|---|
| A1, A2, A7 | Pi does the long moves through a free-wheeling extruder (`M84 E`); cutter removed, tips shaped by the slicer; slack handling in PLAN.md §5 |
| A3, A4, C3 | Drive gear under every lane on one shaft; camshaft parallel to the lane row; lanes stay separate until the merger |
| A5 | Timing recomputed (~36 s Pi, `G4 S42`, ~70 s per change) and logged by the daemon |
| A6 | One set of path constants in `pi_mmu.py`, mirrored in MEASUREMENTS.md §6 and the G-code |
| C1, C2, C4–C7, C9, C11 | Parts rebuilt; interference, path-open, wall-present and envelope checks added |
| C8 | Servo mount places the spline 10 mm off the case centre (verify, G7) |
| C10 | Insert / tap-drill hole sizes |
| C12 | Gate G8 |
| G1–G3, G5–G9 | New G-code: relative hop/return, no purge on the bed, no Z trigger move, 145 mm chunks, fast + slow load, `{if}` template, SEMM settings, start/end blocks |
| G4 (Marlin limits) | Gate G9 |
| §4 Marlin | FIRMWARE.md §1 corrected |
| F1–F11 | `pi_mmu.py` rewritten (release counting + glitch filter, exact wave-chain steps, new pins, EN pull-up, MS1/MS2, servo calibration, FAULT state, persisted state, log file, systemd unit, split power rails) |
| §6, §7 | Docs rewritten; `render_mmu.py` reads the manifest with relative paths; debug scripts removed; `requirements.txt`, `.gitignore` added |

Still open (needs the machine or hardware): every gate in MEASUREMENTS.md §7,
bench calibration, and re-rendering `renders/` (Blender isn't installed here).

**v0.4 (2026-11-18):** moved to Klipper + Happy Hare. The Marlin-specific
v0.3 pieces (bump switch, `pi_mmu.py`, tool-change G-code) are archived in
`.old_v03/`, so the G- and F-items above no longer apply to the active design.

---

Review date: 2026-11-17. Scope: every file in the repo (docs, CAD, check script,
G-code, Pi daemon, viewer, render script). **Nothing has been fixed yet — this is
a punch list only.**

Findings come from reading the source and, where noted, checking the generated
STLs (cross-sections via `cad/check_mmu.py` helpers). `check_mmu.py` passing 70/70
does **not** catch any of the items below; it only checks watertightness, build
volume, a few bores and the cam lobes, never the assembly.

Severity:
- **P0 — blocks function.** The machine cannot work as designed until fixed.
- **P1 — will fail or damage something** on the bench / first print.
- **P2 — wrong or misleading.** Docs, stale numbers, inconsistencies.
- **P3 — housekeeping.**

---

## 1. Architecture / filament-path logic (P0)

### A1. The stock extruder cannot retract 940 mm — P0
[slicer/toolchange_marlin.gcode](slicer/toolchange_marlin.gcode#L43), [FIRMWARE.md](FIRMWARE.md), [MEASUREMENTS.md](MEASUREMENTS.md) §6

`G1 E-940` assumes the Bowden extruder can pull the tip all the way back to the
MMU cutter. The extruder only grips filament while filament is in its gear: once
the tip passes the gear (~tube + 55 ≈ 455 mm), further retract does nothing. The
remaining ~485 mm must be pulled by the MMU feeder, which the sequence never does
(the Pi doesn't know a change is coming until the pulses, which come *after* the
retract).

### A2. The cutter position and the sequence contradict each other — P0
[PLAN.md](PLAN.md) §3, [FIRMWARE.md](FIRMWARE.md) §3, [firmware/pi_mmu.py](firmware/pi_mmu.py#L234)

The sequence retracts the tip **past** the cutter, then fires the cutter. If the
tip is already upstream of the blade, the cut hits nothing (or makes a loose stub
inside the combiner, the jam the design is trying to avoid). If you cut while
the filament still runs through the cutter, ~900 mm of old filament is left
downstream and has to be pushed through the nozzle as purge. Decide what the
cutter is for (cutting off the deformed tip, or separating the lane) and reorder
the sequence to match. Without nozzle-side tip shaping (ramming/cooling moves),
the tip will still be a blob.

### A3. One drive gear can't feed four lanes 108 mm apart — P0
[PLAN.md](PLAN.md) §2, [cad/build_mmu.py](cad/build_mmu.py#L333), [cad/README_CAD.md](cad/README_CAD.md) "What was simplified" #2

The plan says the cam "presses ONE lane's idler onto the drive gear" (PicoMMU
Type-A). In the CAD the lanes sit in a row at x = −54/−18/18/54 and there is a
single NEMA 17 + BMG drive gear in a separate bracket at (55, −70), downstream
of the point where the lanes merge. PicoMMU-style designs need a drive gear (or
a long shared shaft with gears) **at every lane**, or a selector that moves the
filament to one gear. As drawn, an unloaded lane's tip is parked upstream of the
merge and nothing can push it to the feeder, so a new lane can never load.

### A4. A rotating cam doesn't map onto a straight row of lanes — P0
[cad/build_mmu.py](cad/build_mmu.py#L236) `selector_cam`, `selector_plate`

The cam turns about the vertical axis with lobes at 0/45/90/135°, radius 31 mm.
The lanes are in a straight line along X, up to 54 mm from the axis. Lobes at
45/90/135° point at nothing, and no lobe can reach x = ±54. The "sliding gate"
plate has guide slots but nothing in the body for it to slide on, and no
mechanism connects cam rotation to plate travel. Redesign the selector
kinematics first (e.g. a horizontal camshaft parallel to the lane row, or a
linear/rotary selector whose lane spacing matches its geometry).

### A5. Change time is ~2–3 min, not ~20 s — P0
[README.md](README.md), [PLAN.md](PLAN.md) §3/§5, [firmware/pi_mmu.py](firmware/pi_mmu.py#L88-L89)

- Pi side: cut 0.6 s + retract 60 mm @ 20 mm/s (3 s) + select 0.8 s + feed
  485 mm @ 12 mm/s (**40 s**) ≈ **45 s**, and it starts ~1 s after the last
  pulse (`QUIET_S`). The G-code only dwells `G4 S20`, so the stock extruder
  starts `E485` while the MMU is still feeding.
- Marlin side (stock Ender-3 V2 max feedrates: Z ≈ 5 mm/s, E ≈ 25 mm/s):
  `G0 Z200 F1200` is clamped to 5 mm/s (≈40 s up + 40 s down from low layers),
  `E-940` ≈ 38 s, `E485` ≈ 20+ s.
- Rough total: 150–180 s per change at low layers. A 4-colour print with
  hundreds of changes is not realistic. Recompute the timing budget, then
  either set the dwell from the real Pi time or shorten the path/feeds.

### A6. Filament-path distances are inconsistent / undefined — P1
[MEASUREMENTS.md](MEASUREMENTS.md) §6, [firmware/pi_mmu.py](firmware/pi_mmu.py#L88), [slicer/toolchange_marlin.gcode](slicer/toolchange_marlin.gcode#L60)

- `485` is used for two different segments: Pi `FEED_MM` "MMU outlet → extruder
  gear", and G-code `E485` "MMU outlet → nozzle". Extruder gear → nozzle is
  ~455 mm (tube + 55), so `E485` overshoots by ~30 mm.
- Retract-to-cutter adds `feeder → combiner outlet (35)`, but the cutter is
  *downstream* of the combiner, so that 35 mm shouldn't be in the sum.
- Where idle lanes park their tips is never defined, so the Pi's feed length
  (parked tip → feeder → outlet → extruder gear) is unknown.
- 60 mm "retract old lane so the tip clears the combiner" leaves ~3 mm margin
  against the 57 mm outlet → gate distance. Too tight.
- No first-load procedure (loading all 4 lanes to their park position, loading
  the first tool at print start).

### A7. Slack / backdrive during the long retract — P1
Pushing ~900 mm of filament back toward the spool with no rewind and an
engaged-but-disabled BMG (3:1, NEMA detent) loads the stock extruder heavily
(skipping) and buckles the 350–500 mm spool → inlet PTFE. Not addressed
anywhere.

---

## 2. CAD geometry (P0/P1) — verified against `cad/build_mmu.py` and the STLs

### C1. The body is hollow through the filament path — P0
[cad/build_mmu.py](cad/build_mmu.py#L160-L180)

The v0.2 lightening pocket `box(-70, 70, -30, 30, -1, 45)` removes everything
below z = 45 in the centre. STL sections at z = 30 and z = 40 have **no material
inside |x| < 69, |y| < 29** (checked). Consequences:
- The selector cavity "floor at z = 30" doesn't exist. The cam sits on the top bar.
- The outlet bore at z = 30 (`cyl_y(..., -46, 0, z=30)`) runs through empty
  space, so there's no PTFE bore at all.
- The horizontal inlet channel (z 44–48.2) is open on its underside, and the
  x = ±54 inlet bores open into the pocket.
- The feeder recess (y −46…−30, z 25–55) meets the pocket at y = −30, which
  leaves a big hole through the front wall.

### C2. Outlet PC4-M10 counterbore is deleted by the feeder recess — P0
[cad/build_mmu.py](cad/build_mmu.py#L174-L179)

Counterbore at x = 0, z = 25–35, y −46…−38 lies entirely inside the recess
(x ±25, z 25–55, y −46…−30), so the outlet fitting has nothing to seat in.

### C3. Inlet channel merges all four lanes before the selector — P0
[cad/build_mmu.py](cad/build_mmu.py#L172-L173)

`box(-58, 58, ...)` is one continuous slot joining all four inlet bores. With
no gating upstream of it, the lanes share one path before any selection
happens.

### C4. Cutter blade slot is parallel to the filament — P0
[cad/build_mmu.py](cad/build_mmu.py#L387-L388) `cutter_base`

The PTFE bore runs along Y (v0.1 fix) and the blade slot `box(-0.75, 0.75, -16, 16, ...)`
also runs along Y. The blade would slit the filament lengthwise instead of
cutting across it. The slot must be across the bore (along X). README_CAD's
"blade slot across the gap" is wrong.

### C5. Cutter arm has no blade slot and no blade screw — P0
[cad/build_mmu.py](cad/build_mmu.py#L370-L371)

The arm box is y −10…10, but the blade slot is cut at y 10…32 and the clamp
hole at y = 16. Both are entirely outside the part, so the subtractions do
nothing (STL Y extent confirmed −10…10). The arm also swings horizontally
(vertical horn axis) while the blade is meant to drop vertically into the base
slot. The kinematics need redesign.

### C6. Parts that interfere in the assembly — P0/P1
[cad/build_mmu.py](cad/build_mmu.py#L563-L620) `ASSEMBLY`
- **Selector plate** (120 × 40, z = Z0+43…53) overlaps solid body top slab
  outside the 66 mm cavity, and overlaps the cam (z 43–45).
- **Selector plate covers the cam axis.** No centre hole, so the servo shaft rod
  from the bridge can't reach the cam.
- **Top-bar clamp** at Z0−25 (z 440–465) with a 20 mm channel: the clamp's top
  slab (z 460–465) occupies the same space as the top bar (z 445–465). Either
  the clamp lifts the body 5 mm or it can't be assembled.
- **Clamp bolt pattern mismatch.** The body's M5 holes are at x = ±60, but the
  clamp centred there has its top M5 holes at x = ±60 ± 14, and nothing at its
  centre.
- **Pi carrier rotated the wrong way.** `rot (90, 0, 0)` maps local +Z to world
  −Y, so the plate (y 41–45) and standoffs (to y 33) sit **inside** the body
  rear wall, while the Pi envelope floats at y = 60. Needs −90°. Also the carrier
  bolt holes land at Z0+30 while the body holes are at Z0+45 (15 mm off).
- **Bump finger vs. switch bracket vs. upright.** The finger at (−140, 0, 295–320)
  overlaps the bracket (x −177.5…−132.5, z 300–330), and its −X tab (x −155…−147)
  is inside the left upright (x −165…−145).
- **Microswitch lever points −X** (back into the bracket/upright). The finger
  approaches from +X, so the lever must face +X.
- **Feeder bracket** filament bore lands at world x = 73. **Combiner** bores are
  at Z0+11 while the body outlet is at Z0+30. **Cutter** bore is at Z0+12.5. None
  of these line up, and the combiner and feeder are not attached to anything
  (they float in front of the top bar).

### C7. Inlet PTFE can't enter: the servo bridge covers it — P0
[cad/build_mmu.py](cad/build_mmu.py#L495-L530) `servo_bridge`

Inlet blocks occupy Z0+68…98. The bridge plate is solid at y = 0 from Z0+103,
which leaves 5 mm of clearance above the PC4-M6 fittings, and the PTFE comes in
vertically from the top. The x = ±18 and ±54 inlets are under solid plate (the
lightening slots are at |y| 10–20).

### C8. Selector servo shaft is off-axis — P1
`servo_bridge` puts the shaft hole at the centre of the ±24 / ±5 mount pattern.
On an MG996R the output spline is offset ~10 mm from the centre of the ear
holes, so the shaft won't be coaxial with the cam. Fold into gate G7.

### C9. Switch bracket fully encircles the Z upright — P1
[cad/build_mmu.py](cad/build_mmu.py#L425-L450)

A closed 20.6 × 42 channel can only be fitted by sliding it down the upright,
which means removing the top bar and the gantry. The left X-gantry carriage
(V-wheels, X motor, extruder, X endstop) also rides that upright, so a bracket
around it at trigger height is directly in the gantry's path. It needs a one-face
T-nut bracket, and the mount location should be re-evaluated (top bar, gantry
end plate) once G4 is measured.

### C10. Holes sized for clearance where threads/inserts are intended — P1
Body Pi-carrier holes ("tap / insert"), lid holes ("4× M3 inserts"), and
bridge→lid→body bolt holes all use `R_M3 = 1.7` (3.4 mm clearance). M3 heat-set
inserts need ~4.0–4.2 mm, and an M3 tap needs 2.5 mm. The BOM lists 12 inserts
that have no correctly sized holes. The PC4-M6/M10 bores (6.0/10.0 mm) are also
major diameter: too big to tap (M6 tap drill 5.0, M10 tap drill 8.5).

### C11. Leftover / dead features — P2
- Body + lid M3 "servo mount screws" at (75, ±20) are from the v0 servo
  position; the servo now lives on the bridge.
- `selector_cam` shaft bore is exactly 6.0 mm for a 6 mm rod (zero clearance).
- Envelope check (b) stacks only body + lid (68 mm). The real stack is body +
  lid + inlet blocks + bridge + servo ≈ 152 mm above the bar, so the documented
  "180 × 90 × 85 envelope" is not met.

### C12. Bowden length vs. Z travel not considered — P1
The extruder is on the left end of the X gantry and moves 250 mm in Z. A fixed
450 mm PTFE from the top bar (z ≈ 465) to the extruder may be taut or kinked at
low Z. Verify at Z = 0 and Z = max (add to gate G5).

---

## 3. Slicer / Marlin G-code (P0/P1)

[slicer/toolchange_marlin.gcode](slicer/toolchange_marlin.gcode), [FIRMWARE.md](FIRMWARE.md) §2

### G1. Z restore runs in relative mode — P0
Every block does `G91` → `M83` → `G1 E485` → **`G0 Z[layer_z]`**. Under `G91`
that moves Z **up by** `layer_z` instead of to it ([line 61](slicer/toolchange_marlin.gcode#L61)
and the same line in every block). Needs `G90` before the Z move.

### G2. Purge lands at X5 Y200 on the bed, not in the tower — P0
XY is never restored, so `G1 E25` runs at the bump-switch park position at
layer height and leaves a blob on the bed/print. Purge must happen at the
tower (or in a purge bucket), and XY must be restored.

### G3. Prints taller than the trigger Z will crash — P1
`G0 Z200` moves **down** into the part if `layer_z > 200`. The max multicolour
print height must be documented and enforced (or the trigger mechanism changed).

### G4. Single 940/485 mm extrude moves may be rejected — P1
Marlin's `PREVENT_LENGTHY_EXTRUDE` / `EXTRUDE_MAXLENGTH` (200 mm in upstream
default configs) rejects single E moves longer than the limit. Verify the
Ender-3 V2 stock value, or split into chunks.

### G5. Load speed exceeds hotend melt rate — P1
`G1 E485 F1200` (20 mm/s) runs the last ~50 mm into a hot MK8 nozzle at
~48 mm³/s, far beyond its ~8–10 mm³/s. That will grind or skip. Use a fast load
to just above the nozzle, then a slow final load.

### G6. Nozzle dwells on the print during the long retract — P1
`E-940` runs before any Z lift, so the hot nozzle sits on the part for ~40 s.
Z-hop first.

### G7. Leaves the printer in relative-E mode — P1
Blocks end with `G90` but `M83` stays active. If the slicer emits absolute E
(`M82`), the rest of the print breaks. Either require "Use relative E distances"
in the slicer or restore the mode / `G92 E0`.

### G8. Slicer integration is under-specified — P1
- The tool-change field holds **one** template. Four pasted blocks can't be
  selected per change. PrusaSlicer/OrcaSlicer macros support
  `{if next_extruder==0}…{elsif}…{endif}`, so no post-processor is needed.
  Labels like "T0 → T1" are misleading because pulses depend only on the *next*
  tool.
- No mention of Single Extruder Multi Material mode, wipe tower ramming, cooling
  moves, or the parking/loading distances PrusaSlicer will *also* perform around
  the custom G-code. They will fight the custom retract/load.
- Start G-code has the 7-pulse home but no initial load of the first tool.
- Marlin with `EXTRUDERS 1` will echo "Invalid extruder" for the `T1..T3` the
  slicer emits. Harmless, but should be documented or stripped.

### G9. Spurious triggers — P2
Any print move to X < 3 at Z ≈ trigger height, or homing at that height, will
press the switch and be counted. Document this or interlock it.

---

## 4. Marlin firmware recommendations (P1/P2)

[FIRMWARE.md](FIRMWARE.md) §1
- **`SINGLENOZZLE` with `EXTRUDERS 1`** fails Marlin's sanity check (SINGLENOZZLE
  requires ≥ 2 extruders). Drop it, or document the real config.
- **PlatformIO env name** `STM32F103RET6_creality` is the old 2.0.x name. Current
  Marlin uses `STM32F103RE_creality`. Some newer V2 boards ship with a **GD32F303**
  MCU; add that to gate G1.
- Board/driver claims vary between files ("4.2.2 board" in the README table vs.
  "believed 4.2.7"). Some 4.2.2 boards have HR4988/A4988, not TMC2208. Consolidate
  in one place.
- Stock firmware's `ADVANCED_PAUSE_FEATURE` / M600 status: marked as fact, but
  unverified. Mark it TO VERIFY.

---

## 5. Pi daemon — [firmware/pi_mmu.py](firmware/pi_mmu.py) (P0/P1)

### F1. Release bounce is counted as an extra press — P0
[line 143](firmware/pi_mmu.py#L143)
The debounce only compares against the previous *falling* edge. A switch bounce
on **release** (~400 ms after the press) produces a falling edge > 80 ms after
the last count, so it's counted. That turns lane 2 into lane 3 or 4, which is
exactly risk #1 in PLAN.md. Use `pi.set_glitch_filter()` and/or count on a
stable press-and-release cycle. The docs/G-code say "counts on release", but the
code counts on press.

### F2. Stall detection can never fire — P1
[line 161–200](firmware/pi_mmu.py#L161-L200)
The wait loop breaks at `expected`, so `took > expected * 1.5` is never true. A
time-based stall check on an open-loop stepper can't detect a stall anyway.
`_stall()` blinks for 5 s and returns, not "stop and wait" as documented.

### F3. Feed length is time-based, not step-counted — P1
`wave_send_repeat` + `time.sleep(0.05)` polling means the step count is off by
up to ~250 steps (~0.6 mm) per feed, plus the Python scheduling on stop. The
"pulse train is jitter-free" claim is true, but the *count* is not exact. Use
`wave_chain` with an exact loop count.

### F4. GPIO 14/15 are the Pi's UART (serial console) — P1
STEP/DIR on TXD/RXD will toggle during boot if the serial console or login is
enabled, which steps the motor. Move the pins, or document disabling the serial
console.

### F5. TMC2209 EN floats low at boot — P1
GPIO 16 defaults to an input with a pull-down, so the driver is **enabled** from
power-on until the daemon starts. Add an external pull-up on EN.

### F6. TMC2209 microstepping not set — P1
Standalone TMC2209 with MS1 = MS2 = low is **8** microsteps, not 16. The
415 steps/mm assumes 16, so wiring must tie MS1/MS2 accordingly. Document this
in the wiring table.

### F7. Servo pulse range vs. angle — P1
`SELECTOR_US = 1000/1250/1500/1750` assumes 1000–2000 µs = 180°. Most MG996R
units need ~500–2500 µs for ~180°, so 1000–2000 µs is ~90°. Lane angles will
be roughly half of what's intended. Calibrate and document.

### F8. Pulses during a change are silently dropped — P1
[line 237](firmware/pi_mmu.py#L237)
If a new pulse train arrives while busy, it's logged and discarded, and the
lane state desyncs with no alarm. Also `_pulses` is read/reset across threads
without a lock (line 281).

### F9. Hold-ignore logic is incomplete — P2
If the switch is still held for 1–3 s, the loop proceeds to trigger a change
while the switch is pressed.

### F10. Behaviour differs from docs — P2
- No log file; docs say `/var/log/mmu.log`.
- "7 pulses = home / unload-all" doesn't unload in code.
- Docstring says servos use "hardware PWM", but `set_servo_pulsewidth` is
  DMA-timed. `set_PWM_frequency()` on servo pins is pointless.
- Runs as root (`sudo python3`), which isn't needed with `pigpiod`.
- No systemd unit and no lane-state persistence across restarts.

### F11. 5 V power budget understated — P1
An MG996R stalls at ~2.5 A and a Pi 4 can draw ~3 A, which exceeds a 5 V / 3 A
buck. Generic LM2596 modules realistically give ~2 A continuous, so expect
brown-out resets when the selector stalls. The docs also mix 24 V and 5 V amps
("0.8 A + 0.5 A + 0.6 A"). Redo the budget per rail, and consider a separate
servo supply.

---

## 6. Documentation consistency (P2)

| Item | Where | Problem |
|---|---|---|
| Cost | [PLAN.md](PLAN.md) §8 vs [BOM.md](BOM.md) | $55–75 vs ~$79 ($55–90). The BOM sum excludes the Pi. |
| Bearings | [BOM.md](BOM.md) | 4× 608 listed, but 8 rollers printed with 1 bearing seat each, so 8 are needed. |
| Missing BOM items | [BOM.md](BOM.md) | Per-lane idlers/springs/pins (see A3), M2.5 Pi screws/standoffs, TMC2209 heatsink + bulk cap, 608 axle rod, shaft coupler/bushing, wiring for the ~0.5 m switch run. |
| Envelope | [MEASUREMENTS.md](MEASUREMENTS.md) §2 | Says 180 × 90 × 85, but the real stack is ~152 mm tall (see C11). |
| Cam label | [cad/build_mmu.py](cad/build_mmu.py#L576) / `viewer/manifest.json` | "4 lobes at 90°". Should be 45°. |
| Check count | [cad/README_CAD.md](cad/README_CAD.md) | 66/66 in one section, 70/70 elsewhere. |
| check_mmu docstring | [cad/check_mmu.py](cad/check_mmu.py) | "Audit the v0 STLs", "(d) lobes at 90 deg", "measured from STL vertices". All stale. README claims "interference checks" that don't exist. |
| Part dims | `build_mmu.py` docstrings vs README_CAD/BOM | cam 12 vs 15 tall, feeder 60×50×40 vs 60×53×42, cutter arm 12 vs 14, cutter base 15 vs 16.5. |
| Version naming | [README.md](README.md) | Folder is v0.2, README says "v0 CAD", title says v1. |
| "Coherent path" | [cad/README_CAD.md](cad/README_CAD.md) | Claims the filament path is coherent. It isn't (C1–C6). |
| Stale render | [renders/04_exploded.png](renders/04_exploded.png) | Pre-orientation-fix burst directions (README_CAD admits it). |
| "Hardware PWM" | `pi_mmu.py`, README_CAD | Contradicts FIRMWARE.md (correctly says DMA servo pulses). |
| Mains safety | [MEASUREMENTS.md](MEASUREMENTS.md) §5 | The 24 V PSU tap is inside the base next to mains terminals. Add an unplug / insulation warning. |

---

## 7. Tooling / housekeeping (P3)

- [render_mmu.py](render_mmu.py) hard-codes `/home/hatch/workspace/ender-mmu/...`,
  which doesn't exist on this machine. Use paths relative to the script.
- `render_mmu.py` duplicates the `ASSEMBLY` / `EXTRAS` layout by hand instead
  of reading `viewer/manifest.json`, so the two drift (the Pi-carrier rotation bug
  is in both).
- [cad/debug_chamfer.py](cad/debug_chamfer.py) patches the chamfer helpers with
  lambdas that lack the `keep` argument, so it now raises `TypeError` when run.
  All `cad/debug_*.py` use `sys.path.insert(0, "cad")` (cwd-dependent). Delete
  them or move them to `tools/`.
- Run instructions point at `~/workspace/tools/vtol-venv/bin/python`. There's no
  `requirements.txt` / pinned `build123d` version.
- `docs/` is empty.
- No `.gitignore` / no regeneration script tying `build_mmu.py` → `check_mmu.py`
  → renders together.
- The viewer needs an HTTP server (`fetch('manifest.json')` and `../stl/` fail
  on `file://`). Document `python3 -m http.server` from the repo root.
- `check_mmu.py` should gain the assembly checks that would have caught section 2:
  pairwise part interference from the manifest poses, filament-path continuity
  (bore-to-bore alignment), and features-actually-cut checks (e.g. cutter-arm
  slot).

---

## Suggested fix order

1. **Redesign the mechanism on paper first** (A1–A4): where each lane is
   driven, how selection works, where the cut happens, who pulls the filament
   back past the extruder. Everything else depends on this.
2. Redo the timing budget (A5) and the path lengths (A6) from that design.
3. Rebuild the CAD to match (C1–C12), and add assembly/interference/path checks
   to `check_mmu.py` before trusting any "PASS".
4. Fix the G-code bugs G1, G2 and G7 even for bench tests. They are cheap and
   would damage a print.
5. Fix the Pi daemon: F1 (miscount), F4/F5 (boot-time stepping), F6/F7
   (calibration assumptions), F11 (power).
6. Clean up docs and tooling (sections 6–7).
