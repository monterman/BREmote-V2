# Flashing the TX — Arduino CLI / IDE (⚠️ Advanced users only)

> The RX and TX use **different** settings. Use this file only for the **TX** (the handheld remote). The TX setting that's mandatory here (**Huge App**) is the exact one you must **never** use on the RX.

**Board:** HT-CT62 module = **ESP32-C3**. Install the **`esp32` by Espressif** board package.
**Sketch:** `Source/V2_Integration_Tx/`

---

## Critical settings — TX (get these exactly right)

| Setting | Value | Why it matters |
|---|---|---|
| Board FQBN | `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app` | ESP32-C3, CDC off + Huge App |
| **Partition Scheme** | **Huge App (3 MB, No OTA)** | **MANDATORY.** NimBLE (BLE) pushes the TX binary to **~1.46 MB**, over the default 1.25 MB app slot → **"Sketch too big"** without it. The TX has **no** source `partitions.csv`, so the FQBN flag is what decides. |
| **USB CDC On Boot** | **Disabled** (`CDCOnBoot=default`) | Same reason as the RX — GPIO 18/19 are the GPS UART (Serial1). Enabling CDC kills the GPS. |
| Flash Size | 4 MB | |
| Flash Mode / Freq | QIO / 80 MHz (defaults) | |
| **OTA** | **None** | Huge App is a single 3 MB app with **no OTA partition**; the TX is USB-flashed. |

---

## Compile + flash (arduino-cli)
```
# COMPILE — PartitionScheme=huge_app is REQUIRED on the TX
arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app"  <path>/Source/V2_Integration_Tx

# UPLOAD
arduino-cli upload  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app"  --port <COMx>  <path>/Source/V2_Integration_Tx
```
(Arduino IDE equivalent: Board = "ESP32C3 Dev Module", **USB CDC On Boot = Disabled**, **Partition Scheme = Huge APP (3MB No OTA/1MB SPIFFS)**.)

---

## Gotchas
- **Without `huge_app` → "Sketch too big"** — the BLE binary overflows the default slot. Always include the flag on the TX.
- **BLE needs battery power at boot.** If USB-C is connected while the TX boots, BLE init is blocked (ESP32-C3 hardware limitation — not fixable in firmware). Boot on battery, then connect USB if needed.
- **Verify the board by its MAC before flashing** — don't flash the RX with the TX build (wrong partition = trouble).
- **Libraries** (Library Manager): RadioLib, **NimBLE-Arduino**, TinyGPS++, Adafruit AW9523. See the `#include`s for the exact set.
