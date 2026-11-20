# Firmware — Klipper on the Ender, the Pico MMU MCU, Happy Hare

Verify each step against the current docs; versions move:
[Klipper](https://www.klipper3d.org/Installation.html) ·
[Happy Hare v4 docs](https://moggieuk.github.io/Happy-Hare-Doc/) ·
[KIAUH](https://github.com/dw-0/kiauh).

## 1. Klipper on the Ender-3 V2 (phase 1, no MMU)

1. **Back up**: keep the stock Marlin `firmware.bin` (to go back, flash it
   the same way).
2. **Pi**: Raspberry Pi OS Lite (64-bit), then KIAUH → install Klipper,
   Moonraker, Mainsail (or Fluidd), and **KlipperScreen** for your display.
3. **Board firmware**: read the header of Klipper's
   `config/printer-creality-ender3-v2-2020.cfg` — it gives the exact
   `make menuconfig` choices for the 4.2.2/4.2.7 board (STM32F103, 28 KiB
   bootloader, serial on USART1). If the chip under the heatsink says
   **GD32F303**, check the Klipper docs / community notes for that variant
   before flashing (gate G1). Copy the built `klipper.bin` to the SD card
   with a **new file name** (the bootloader skips names it has seen), power on.
4. **Config**: start from that same file as `printer.cfg`, set the `serial:`
   from `ls /dev/serial/by-id/`, tune rotation_distance for the extruder,
   PID-tune hotend and bed, print a single-colour cube.

The stock DWIN screen isn't supported by mainline Klipper — unplug it and
use KlipperScreen on your display.

## 2. Pico = MMU MCU

Flash Klipper for RP2040 (`make menuconfig`: Raspberry Pi RP2040, USB),
hold BOOTSEL, copy `klipper.uf2`. Connect it to the Pi by USB (also powers it).

| Pico pin | Function |
|---|---|
| gpio10 | TMC2209 STEP (gear stepper) |
| gpio11 | TMC2209 DIR |
| gpio12 | TMC2209 EN |
| gpio9 | TMC2209 PDN_UART (single-wire UART; 1 kΩ from gpio9 to PDN_UART) |
| gpio15 | MG996R signal (hardware PWM) |
| gpio16 | Gate sensor KW11 (NO to GND, use `^` pull-up) |
| gpio17 | spare: extruder-entry sensor (optional, later) |
| GND | common with the 24 V, 6 V and Pi grounds |

TMC2209 MS1/MS2 set the UART address (both low = 0) when UART is used.

## 3. Happy Hare

Install per the docs (`git clone` + `./install.sh -i`). Answer the installer:

| Question | Answer |
|---|---|
| MMU type | **PicoMMU / Type-A with Servo_Selector, shared gear stepper** (Generic/Custom if PicoMMU's geometry doesn't fit) |
| Gates | **4** |
| MCU | the Pico's `/dev/serial/by-id/...` |
| Gear stepper pins | step gpio10, dir gpio11, enable !gpio12, uart gpio9 |
| Servo pin | gpio15 |
| Gate sensor | gpio16 (`^gpio16`) — shared "gate" sensor at the merger |
| Encoder | none |

Then in Happy Hare's config / calibration:
- **Selector angles**: gate 0–3 = **40 / 75 / 110 / 145°**, release/neutral **0°**
  (all inside 0–160° so any MG996R reaches them; calibrate the exact
  pulse widths with Happy Hare's servo commands).
- Gear `rotation_distance`: start ≈ **33** (MK8 gear, 2π × 5.25), calibrate.
- Gate parking: the gate sensor is shared (after the merge), so tips park in
  their own merger lane **16 mm behind the gate sensor** (10 mm before the
  merge). Set Happy Hare's gate parking distance to ~16 mm.
- Bowden length (gate sensor → extruder gear): ~610 mm; calibrate with
  Happy Hare's Bowden calibration.
- **Selector moves with the gear stepper idle** (Happy Hare's normal
  behaviour): between two lane angles the next lobe briefly touches its idler.
- **Sync the gear stepper to the extruder while printing** (Happy Hare
  sync-to-extruder). The lane stays engaged, so the stock extruder doesn't
  drag 1.1 m of PTFE on its own (simulation: 4–5 N drag, 15 N available).
- Tip forming: start with Happy Hare's standalone tip forming; tune per filament.
- Gate sensor: shim the KW11 so its roller rests ~1 mm above the channel
  axis; check with `QUERY_ENDSTOPS`/Happy Hare sensor status that it toggles
  as filament passes.

## 4. Slicer (PrusaSlicer / OrcaSlicer)

Follow Happy Hare's *Slicer Setup* page. In short: 4 extruders, single
nozzle / SEMM, wipe tower on, tool-change G-code = `T[next_extruder]`, and
the print-start G-code passes the initial tool and tool list to Happy Hare's
start macro. Either the slicer or Happy Hare forms tips — not both.

## 5. Power

| Rail | Load | Supply |
|---|---|---|
| 24 V | TMC2209 + NEMA 17 | PSU tap, 3 A fuse |
| 6 V | MG996R (stall ~2.5 A) | buck inside the MMU electronics bay, ≥ 3 A |

The Pi runs from its own 5 V supply at the printer, with your display. The
MMU bay gets 24 V (fused) for the driver and the servo buck, and USB from the
Pi for the Pico. All grounds common.
