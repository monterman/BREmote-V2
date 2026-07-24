# Flashing the RX — Arduino CLI / IDE (⚠️ Advanced users only)

> **One wrong partition setting WIPES your config + logs.** Read every line. The RX and TX use *different* settings — use this file only for the **RX** (the board in the buggy).

**Board:** HT-CT62 module = **ESP32-C3**. Install the **`esp32` by Espressif** board package (Boards Manager).
**Sketch:** `Source/V2_Integration_Rx/`

---

## Critical settings — RX (get these exactly right)

| Setting | Value | Why it matters |
|---|---|---|
| Board FQBN | `esp32:esp32:esp32c3:CDCOnBoot=default` | ESP32-C3, CDC off (below) |
| **Partition Scheme** | **DO NOT SET — leave it alone** | The RX sketch ships its **own `partitions.csv`** and the build picks it up automatically. ⚠️ **NEVER select "Huge App"** (or pass any `PartitionScheme`) — it *overrides* the custom table, **halves SPIFFS, relocates it, and WIPES config + logs.** |
| **USB CDC On Boot** | **Disabled** (`CDCOnBoot=default`) | GPIO 18/19 are the USB D-/D+ pins **and** the GPS UART (Serial1). Enabling CDC makes USB claim those pins → **GPS silently dies.** |
| Flash Size | 4 MB | |
| Flash Mode / Freq | QIO / 80 MHz (defaults) | |
| **OTA** | **None** | RX is flashed over USB; the OTA partition is **deliberately dropped** in `partitions.csv` to give the app + SPIFFS more room. Do not add an OTA scheme. |

**The RX `partitions.csv` (for reference — do not edit):** `app0` 2.0 MB @ `0x10000` · `spiffs` 1.875 MB @ `0x210000` · **no OTA** (sums to 4 MB).

---

## Compile + flash (arduino-cli)
```
# COMPILE — note: NO PartitionScheme flag on the RX
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=default  <path>/Source/V2_Integration_Rx

# UPLOAD
arduino-cli upload  --fqbn esp32:esp32:esp32c3:CDCOnBoot=default  --port <COMx>  <path>/Source/V2_Integration_Rx
```
(Arduino IDE equivalent: Board = "ESP32C3 Dev Module", **USB CDC On Boot = Disabled**, **Partition Scheme = leave default / do not change**.)

---

## Gotchas (read before you panic)
- **"97% of program storage" is a FALSE reading.** arduino-cli size-checks against the default 1.25 MB app slot, but the real slot is **2.0 MB (≈61% used)**. **Do NOT "fix" it by selecting Huge App** — that IS the wipe trap.
- **Verify the board by its MAC before flashing** (RX vs TX vs any other ESP32-C3) — flashing the wrong board is on you.
- **Any partition-scheme change wipes SPIFFS** → you must re-pair, re-enter settings, and re-run compass calibration afterward.
- **Libraries** (install via Library Manager): RadioLib, TinyGPS++, Adafruit AW9523 + the QMC5883L compass lib. See the `#include`s at the top of the sketch for the exact set.
