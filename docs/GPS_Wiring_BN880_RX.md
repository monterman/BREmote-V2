# BN-880 → RX Wiring (GPS + Compass)

**Applies to:** V2.5-Evo RX (HT-CT62 / ESP32-C3) with a **BN-880** GPS module.
**Why the BN-880:** it carries a **QMC5883L magnetometer**. The RX needs a compass for
**Return-to-Me** and **Follow-Me** heading — a BN-220 has no compass and cannot run those modes.

> This covers the **module-to-board** wiring only. Waterproof connectors and pack wiring are
> deliberately not covered — everyone uses different connectors, so pick your own and keep the
> signal mapping below.

---

## Pin map — the six wires

The RX board's pads are labelled on the silkscreen — **wire label-to-label**, and the GPIO numbers
below are only for reference when you need them.

All six wires go to the **`UART 1.1` header** (the right-hand one — see below) except SDA/SCL,
which are separate I2C pads.

| BN-880 pad | RX board pad | GPIO | Notes |
|---|---|---|---|
| **VCC** | `V+` on `UART 1.1` | — | **5 V** — put that header's selector on `5V`, not `3V3` |
| **GND** | `GD` on `UART 1.1` | — | Common ground with the RX |
| **TX** | **`RX`** on `UART 1.1` | 19 · `P_U1_RX` | **crossed** |
| **RX** | **`TX`** on `UART 1.1` | 18 · `P_U1_TX` | **crossed** |
| **SCL** | `SCL` | 1 · `P_I2C_SCL` | straight |
| **SDA** | `SDA` | 2 · `P_I2C_SDA` | straight |

**The UART crosses; I2C does not.** Module **TX → board RX**, module **RX → board TX**. The compass
lines go straight across: SCL→SCL, SDA→SDA.

### Which header — `UART 1.1`, the right-hand one

The board has **two UART headers**. They are *not* two independent serial ports: they are one
hardware UART (`Serial1`) shared between two connectors by a 74HC4052 analog mux, which the firmware
flips back and forth. That is what the `.0` and `.1` in the silkscreen mean — **UART1, mux channel 0
and mux channel 1**.

```
   UART 1.0                UART 1.1
    5V   3V3                5V   3V3      <- supply selector (GPS: 5V)
   V+  RX  TX  GD          V+  RX  TX  GD
   └──── VESC ────┘        └──── GPS ─────┘
```

| Header | Mux channel | Peripheral |
|---|---|---|
| `UART 1.0` | 0 | **VESC** |
| `UART 1.1` | 1 | **GPS** — wire the BN-880 here |

Straight from the firmware (`Source/V2_Integration_Rx/System.ino`, `setUartMux()`):

```
//   channel 0 (VESC): MUX_0 = LOW,  MUX_1 = LOW
//   channel 1 (GPS):  MUX_0 = HIGH, MUX_1 = LOW
```

The board files agree. In `Electronics/github_public/Rx/Rx/Rx_V2-2.sch` the two 1×4 headers carry
nets named `UART_1_0_RX` / `UART_1_0_TX` (header `JP5`) and `UART1_1_RX` / `UART1_1_TX` (header
`JP9`) — the silkscreen is naming the net, and the net is naming the mux channel.

Each header's supply selector is a **3-pin jumper** (`JP7` for `1.0`, `JP10` for `1.1`): the middle
pin feeds that header's `V+` pad, and you bridge it to one side or the other. For the BN-880,
bridge it to the **5 V** side.

⚠️ **Wire the GPS into `UART 1.0` and it will never work** — that connector is on the VESC channel,
so the module's bytes are switched away from the UART. The symptom is identical to a TX/RX swap:
nothing at any baud from `?gpsbaud`. Confirm you are on the **right-hand header** before you start
swapping data wires.

### Power the BN-880 from 5 V — not 3.3 V

Each UART header has its own **`5V` / `3V3`** selector. On `UART 1.1` it goes on **`5V`**.

The BN-880 is a 5 V module: it has an onboard regulator that drops 5 V to the 3.3 V its u-blox core
and QMC5883L need. Run it from the 3.3 V rail instead and you sit right on the regulator's dropout —
which does not fail cleanly. You get a module that boots, enumerates, sometimes even reports sats,
then browns out under the current draw of an acquisition burst. That reads as flaky GPS, not as a
power problem, and it will waste a day.

The reference build takes **5 V off the VESC's 5 V output**. That is the known-good arrangement.

**The data lines are still 3.3 V logic**, even with a 5 V supply — the module's UART TX and its I2C
lines run at the internal 3.3 V rail. No level shifter is needed between the BN-880 and the
ESP32-C3, and nothing on the board sees 5 V on a signal pin.

> The `3V3` position on the selector exists for modules that want 3.3 V — the HGLRC M100 series,
> for example, accepts 3.3–5 V. It is not the right setting for a BN-880.

> ⚠️ **Crossed or straight? Test it — don't assume.** The pads are silkscreened `RX` and `TX`, but
> boards differ in what those labels mean: some name the **signal the pin carries** (so you cross),
> others name **what you connect there** (so you wire straight, because the swap was already done on
> the board). The original BREmote wiring notes describe a **direct RX→RX / TX→TX** connection.
>
> Resolve it in two minutes, with no risk — swapping UART data lines cannot damage anything, they are
> both 3.3 V logic:
>
> 1. Wire it **crossed** first, on the **`UART 1.1`** header (module `TX` → board `RX`). That is the
>    standard convention.
> 2. Power up and run **`?gpsbaud`** — a listen-only scan that reports whether *any* bytes are
>    arriving. It needs no sky and no fix, so it answers the wiring question on its own.
> 3. **Bytes at some baud → the orientation is right.** Now run `?printgps` outdoors to confirm sats.
> 4. **Nothing at any baud → swap just the two data wires** (`RX`→`RX`, `TX`→`TX`) and run
>    **`?gpsbaud`** again.
> 5. Whichever way returns bytes is correct. Note it down — the same rule applies to the VESC UART on
>    the other header (test that one with `?vescraw`, the raw byte-dump probe, then `?vescping`).

```
        BN-880 module                    RX board — UART 1.1 header (GPS)
   ┌──────────────────────┐                 ┌──────────────────────────────┐
   │                      │                 │                              │
   │  VCC  ───────────────┼─── red ────────►│  V+   selector on 5V         │
   │  GND  ───────────────┼─── black ──────►│  GD                          │
   │                      │                 │                              │
   │  TX   ───────────────┼─────────╲       │                              │
   │                      │          ╳──────┤  RX    (GPIO 19)      UART   │
   │  RX   ◄──────────────┼─────────╱       │  TX    (GPIO 18)      UART   │
   │                      │  TX→RX, RX→TX   │                              │
   │  SCL  ───────────────┼─── straight ───►│  SCL   (GPIO 1)       I2C    │
   │  SDA  ───────────────┼─── straight ───►│  SDA   (GPIO 2)       I2C    │
   │                      │                 │                              │
   └──────────────────────┘                 └──────────────────────────────┘
       GPS + QMC5883L                         compass answers at 0x0D
```

Wire colours vary between vendors — **go by the silkscreen labels on the module**, not by colour.

> ### ⚠️ Where are SDA / SCL on the RX? — unresolved, read this before you order parts
>
> **They are not on either UART header, and we have not been able to locate them.** This is an open
> gap in the documentation, stated plainly rather than guessed at.
>
> What we do know, from the board files in `Electronics/github_public/Rx/`:
>
> - The GPIO numbers are certain: **`1` = SCL, `2` = SDA**. The firmware uses them, and the
>   schematic confirms they land on the module's `32K_XN` (GPIO 1) and `FSPIQ` (GPIO 2) pins.
> - On the `Rx_V2-2` schematic the `SDA` and `SCL` nets reach **only** three things: `IC1` (the
>   AW9523 I/O expander), their pull-up resistors `R24` / `R28`, and the ESP32-C3 module.
>   **No connector or header sits on those nets.** Every 1×4 and 1×3 header on that sheet carries
>   UART, ESC or power — none carries I2C.
>
> The straightforward reading is that **this board revision does not break I2C out to a labelled
> connector**, and that a compass has to be tapped from a via or test point. We are not asserting
> that as fact: the `Electronics/` folder may not match every board in the wild, and a later
> revision may well have added a connector.
>
> **Before you solder, check your own board** — both sides — for pads marked `SDA` / `SCL`, and
> confirm with a meter against GPIO 1 and GPIO 2. If you find them, please open an issue with a
> photo; it closes this gap for everyone.
>
> Note that the GPS half of the BN-880 works fine over the UART alone. Only the **compass** needs
> I2C — and without a compass the RX cannot do Return-to-Me or Follow-Me heading.

---

## After wiring — three things, in this order

### 1. Tell the firmware which module you fitted
The RX must be told it has a BN-880, or it will not configure the module correctly:

```
?set gps_chip_type 1        # 0 = BN-220, 1 = BN-880, 2/3 = M10
?save
```
Reboot after saving.

### 2. Check the build setting that shares these pins
The ESP32-C3 USB peripheral claims **GPIO 18/19** — the same pins the GPS uses. If you compile it
yourself, **USB CDC On Boot must be Disabled** (`Tools → USB CDC On Boot → Disabled`, or
`:CDCOnBoot=default` on the arduino-cli FQBN). The firmware refuses to build otherwise. Prebuilt
`.bin` files are already correct.

### 3. Verify before you trust it
```
?gpsbaud      # listen-only: are any bytes arriving at all? (answers the wiring question)
?printgps     # sats, fix and position = the link is working end to end
?magtest      # magnetometer health + how much your wiring disturbs it
?compasscal   # full calibration — rotate the buggy through a complete horizontal circle
```
No bytes at any baud is a **UART orientation** problem — swap the two data wires and retest (see the
note above). Compass not found at `0x0D` is almost always **SDA/SCL swapped** or a bad ground.

---

## Mounting — this matters more than people expect

The magnetometer sits in the same module as the GPS, and **motor phase wires and battery leads
throw the heading off by 100°+ at throttle**. On the RX a bad heading is a Follow-Me fault, not a
cosmetic problem.

- Mount the BN-880 **as far from the battery and phase wires as the build allows**.
- Keep it away from ferrous hardware and anything carrying high current.
- Run `?magtest` **in place, on the real build** — not on the bench — to see the actual disturbance.
- **Re-run `?compasscal` after any change** to module, position, or mounting. A stored calibration
  does not carry over: different module, different mounting, different hard/soft-iron offsets.

---

## Related

- [GPS configuration, baud rates and dynamic model →](GPS.md)
- [GPS troubleshooting →](hardware/gps-troubleshooting.md)
- [Follow-Me guide →](FOLLOW_ME_GUIDE.md)
- [RX flashing guide →](FLASHING_RX_ARDUINO.md)
