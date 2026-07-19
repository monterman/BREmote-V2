# Forum Post Draft — foil.zone thread #298
**Status:** Draft — review before posting. DO NOT post until the telemetry fix + robust parser are field-tested and pushed to GitHub.
**Target:** foil.zone, thread #298 (BREmote / tow buggy remote thread)
**Date:** 2026-07-19

---

## Post body

Hey all,

Big update on the V2.5-Evo fork — the VESC UART telemetry issue that's bitten a few of us is finally solved, and I want to save everyone the rabbit hole I went down.

**VESC UART telemetry — real root cause found and fixed.**

If you've ever had "no VESC data" over UART that made no sense — wiring checks out, config checks out, the VESC talks fine over Bluetooth, but the remote shows nothing — here's what it was on my end, and it's worth knowing:

The on-board `?vescraw` diagnostic was sending its VESC request with a **wrong CRC**. A VESC **silently drops any packet with a bad CRC** — no error, no reply, at any baud — so the diagnostic always printed "NO BYTES" even against a perfectly healthy VESC. That false negative had me chasing dead-hardware ghosts (RX board, mux, cables, VESC firmware) for hours. The *actual* telemetry path always computed the CRC correctly — only the diagnostic frame was wrong.

Bench-proven with an FTDI straight to the VESC: the correct GET_VALUES request `02 01 04 40 84 03` returns full live data on both VESC firmware **6.05 and 6.06**. Two fixes shipped: corrected the diagnostic CRC, and hardened the telemetry parser to validate replies by the **echoed command + mask** instead of an exact byte-length — so it stays compatible across VESC firmware **3.x–7.x** (VESC only ever appends new fields at the end of the struct).

Also removed an in-ride telemetry freeze: an earlier throttle-gate skipped the VESC poll whenever throttle was held, which froze the readout to dashes on a continuous-throttle vehicle like a tow buggy. Gone.

**If you're setting this up:** VESC App Config → *App to Use = PPM and UART*, UART baud **115200**, and wire to the VESC's actual **UART TX/RX/GND** — not the CAN or COMM/BLE pins, those are separate ports. Works with VESC FW 3.x through 7.x, no per-version selection needed.

---

**BLE live telemetry (unchanged, still solid).**

The TX advertises as `BRemote-TX-XX` over BLE and speaks NUS + the VESC Tool binary protocol. Open VESC Tool on iOS/Android, scan, connect — live gauges appear: FET temp, motor amps, duty, voltage, RPM, power. Activate via `bt_enabled = 2`, the boot gesture (hold Throttle + LEFT toggle), or a DRV5032 hall on GPIO 9. One constraint: boot on battery — USB-C during boot blocks BLE init (ESP32-C3 hardware limitation, not fixable in firmware).

---

**Return-To-Me (RTM): implemented, water-test pending.**

Full RTM (GPS + compass heading, safety gates, approach convergence) passed code review and bench testing. Still finishing outdoor + motor field tests before I call it done — there's a known near-target behaviour I'm tuning. Not "set and forget" yet.

**Follow-Me (FM): mode *selection* works; autonomous *following* is designed but NOT shipped.**

You can cycle and set the FM mode on the TX (F0/F1/F2/F3 = Off / Near-Right / Behind / Near-Left, default **Near-Right**) and it's sent to the RX. But the autonomous *following* control law — the buggy actually trailing you after you release the rope — is **not implemented yet**. Selecting a mode does not make the buggy follow. The design is written and it's the next build; I'm calling this out so nobody flashes this expecting FM to drive. When it's real and field-tested, I'll post.

---

**Hardware: HT-CT62 confirmed** — Heltec HT-CT62 integrates ESP32-C3 + SX1262 LoRa + BLE on one module. No separate BT hardware needed.

GitHub: https://github.com/monterman/BREmote-V2.git

---

*Edit notes before posting:*
- *Check thread #298 for the last post — avoid repeating anything already covered*
- *foilIQ / Waveshare peripheral: don't mention yet, nothing to show*
- *Hold until telemetry fix + robust parser are field-tested and pushed*
