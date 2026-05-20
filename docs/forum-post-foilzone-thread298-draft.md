# Forum Post Draft — foil.zone thread #298
**Status:** Draft — review before posting
**Target:** foil.zone, thread #298 (BREmote / tow buggy remote thread)
**Date:** 2026-05-20

---

## Post body

Hey all,

Sharing a major update on the V2.5-Evo fork — a lot has landed since the last post.

**BLE live telemetry is in master and field-confirmed.**

The TX now advertises as `BRemote-TX-XX` (last byte of BT MAC) over BLE. It speaks NUS + VESC Tool binary protocol (COMM_GET_VALUES, auto-detected). Open VESC Tool on iOS or Android, scan, connect — live gauges appear immediately: FET temp, motor amps, duty, voltage, RPM, power. Free app, no config needed.

Three ways to activate BLE:
- Set `bt_enabled = 2` via the web config for always-on
- Boot gesture: hold Throttle + LEFT toggle — BLE activates for the session, display shows `bt`
- Hall sensor expansion: add a DRV5032 to GPIO 9 (P_MAG) for magnet-based activation without touching the remote

One constraint: boot on battery. USB-C during boot blocks BLE init — hardware limitation of the ESP32-C3, not fixable in firmware.

---

**VESC telemetry root cause fixed (SW55).**

For anyone hitting flickery or silent VESC data: the culprit is USB-C. The ESP32-C3's native USB peripheral shares GPIO 18/19 with Serial1, which carries both GPS and VESC through the hardware MUX. Any USB cable plugged in during field use silences UART completely. Unplug before riding.

Two code fixes shipped in the same sprint:
- GPS MUX was taking the bus and never returning it — VESC starved on every GPS poll cycle. Fixed with `setUartMux(0)` at exit of both `getGPSLoop()` and `configureGPS()`.
- Stale error flag in `receiveFromVESC()` was poisoning reads after a single bad byte. Removed — CRC handles validation.

Both TX and RX compile clean: 39% / 40% flash (huge_app partition, ESP32-C3).

---

**RTM and FM bench-complete, water test pending.**

Full RTM implementation (GPS+compass layered heading, 10 safety gates, convergence check) passed static code review. Waiting on a proper outdoor + motor test before calling it field-ready. If anyone has water access and wants to alpha test — DM me.

---

**Hardware note: HT-CT62 confirmed**

Heltec HT-CT62 integrates ESP32-C3 + SX1262 LoRa + BLE on one module. No separate Bluetooth hardware needed.

GitHub: https://github.com/monterman/BREmote-V2.git

---

*Edit notes before posting:*
- *Check thread #298 for last post — avoid repeating anything already covered*
- *foilIQ / Waveshare peripheral: don't mention yet, nothing to show*
