# Flashing the RX — Arduino CLI / IDE (⚠️ Advanced users only)

> **One wrong partition setting WIPES your config + logs.** Read every line. The RX and TX use *different* settings — use this file only for the **RX** (the board in the buggy).

**Board:** HT-CT62 module = **ESP32-C3**. Install the **`esp32` by Espressif** board package (Boards Manager).
**Sketch:** `Source/V2_Integration_Rx/`

---

## Critical settings — RX (get these exactly right)

| Setting | Value | Why it matters |
|---|---|---|
| Board FQBN | `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` | ESP32-C3, CDC off (below), **Custom** partition scheme (below) |
| **Partition Scheme** | **`custom`** — Arduino IDE: **"Custom"** | The RX sketch ships its **own `partitions.csv`**. **"Custom" means "use the sketch's own table"** — it does **not** invent a new one, so the flashed layout is *byte-identical* to leaving it unset (verified by hashing the generated table). What it fixes is the **size check** — see the size note below. ⚠️ **NEVER select "Huge App"** or any other named scheme — those *replace* your table, **halve SPIFFS, relocate it, and WIPE config + logs.** |
| **USB CDC On Boot** | **Disabled** (`CDCOnBoot=default`) | GPIO 18/19 are the USB D-/D+ pins **and** the GPS UART (Serial1). Enabling CDC makes USB claim those pins → **GPS silently dies.** |
| Flash Size | 4 MB | |
| Flash Mode / Freq | QIO / 80 MHz (defaults) | |
| **OTA** | **None** | RX is flashed over USB; the OTA partition is **deliberately dropped** in `partitions.csv` to give the app + SPIFFS more room. Do not add an OTA scheme. |

**The RX `partitions.csv` (for reference — do not edit):** `app0` 2.0 MB @ `0x10000` · `spiffs` 1.875 MB @ `0x210000` · **no OTA** (sums to 4 MB).

---

## Compile + flash (arduino-cli)
```
# COMPILE
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom  <path>/Source/V2_Integration_Rx

# UPLOAD
arduino-cli upload  --fqbn esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom  --port <COMx>  <path>/Source/V2_Integration_Rx
```
(Arduino IDE equivalent: Board = "ESP32C3 Dev Module", **USB CDC On Boot = Disabled**, **Partition Scheme = "Custom"**.)

---

## About the reported program size (this trips everyone up)

The RX's real app slot is **2.0 MB**, set by the sketch's own `partitions.csv`. But arduino-cli doesn't read the size limit from that file — it reads it from whichever **Partition Scheme** you selected.

| Partition Scheme | Flashed layout | Size limit the tool checks against | Reported usage |
|---|---|---|---|
| *(not set)* | your `partitions.csv` ✅ | **1.25 MB — wrong** | **~97% (false alarm)** |
| **`custom`** ✅ | your `partitions.csv` ✅ (identical) | 16 MB | ~7% |
| `huge_app` ❌ | **replaced — WIPES config + logs** | 3 MB | — |

**Why this matters:** with the scheme unset, the tool **refuses to compile once the firmware passes 1,310,720 bytes** — even though the chip still has ~800 KB free. The build simply starts failing with "sketch too big" for no real reason.

**`PartitionScheme=custom` fixes it.** Its partitions field is empty, so the build still falls back to *your* `partitions.csv` — the generated table is byte-for-byte identical (verified by hash). Only the limit the tool measures against changes.

> **Read this twice: "Custom" is safe. "Huge App" is the wipe trap.** They sound similar and do opposite things.

---

## Gotchas (read before you panic)
- **If you compile without `PartitionScheme=custom` and see "97% of program storage" — that is a FALSE reading.** See the size section above. **Do NOT "fix" it by selecting Huge App** — that IS the wipe trap. Select **Custom** instead.
- **Verify the board by its MAC before flashing** (RX vs TX vs any other ESP32-C3) — flashing the wrong board is on you.
- **Any partition-scheme change wipes SPIFFS** → you must re-pair, re-enter settings, and re-run compass calibration afterward.
- **Libraries** (install via Library Manager): RadioLib, TinyGPS++, Adafruit AW9523. See the `#include`s at the top of the sketch for the exact set.
- **No compass library is needed — do not install one.** `Compass.ino` drives both supported magnetometers through raw `Wire` register writes: the **QMC5883L** at `0x0D` (Beitian BN-880, HGLRC M100 Pro) and the **QMC5883P** at `0x2C` (HGLRC M100-5883). Installing a QMC5883L library changes nothing, and if your compass is dead it will send you looking in the wrong place — particularly on an **M100-5883**, whose part is a QMC5883**P** and which no QMC5883L library would drive anyway.
