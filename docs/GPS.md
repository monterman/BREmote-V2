# GPS on BREmote — what changed, and why it matters

**V2.5-Evo, 2026-07-29 → 07-31.** Applies to both TX and RX. Hardware-verified on both.

For wiring, LED meanings and step-by-step troubleshooting, see
[`docs/hardware/gps-troubleshooting.md`](hardware/gps-troubleshooting.md). This document
explains the *design* — what was wrong, what changed, and what you need to know if you fit a
different GPS module.

---

## The short version

The firmware used to configure the GPS **blind**. It sent settings and never checked whether the
module accepted them. It now checks every single write, auto-detects which u-blox generation
you have, and refuses to transmit until it knows what speed the module is talking at.

Three things came out of that:

1. A receiver was found running the **wrong navigation mode** — silently, for an unknown length
   of time.
2. u-blox **M10** modules were never being configured at all.
3. The old baud-negotiation could, over many reboots, **disable the GPS receiver entirely**.

---

## 1. Why `dynModel = Sea` matters more than it sounds

Every u-blox receiver runs a **dynamic platform model** — an assumption about what it's bolted
to. That assumption feeds its internal navigation filter and decides which solutions it's willing
to believe.

These modules ship in **Portable**, the permissive default:

| | Portable (factory) | **Sea** (what we set) |
|---|---|---|
| Max horizontal velocity | 310 m/s (694 mph) | **25 m/s (56 mph)** |
| Max vertical velocity | 50 m/s | **5 m/s** |
| Max altitude | 12,000 m | **500 m** |

Portable exists so the same chip can work in an aircraft. On a tow buggy it means the receiver
will **accept and report physically impossible solutions as valid**.

**This is not theoretical.** On 2026-07-24 a logger using the same u-blox family on the same
buggy emitted **254 km/h and 4,800 m altitude** — reported as *high-confidence* fixes, with 5–7
satellites and HDOP under 3. Nothing downstream could tell those apart from real data, because
by every quality metric the receiver publishes, they looked real.

Under Sea, those readings are not "less likely" — they are **outside the filter's accepted state
space and cannot be emitted at all.**

### Why this is a safety feature, not an accuracy feature

Be clear about what Sea does and doesn't do:

- **It does not make a good fix more accurate.** With 8 satellites and HDOP 1.2, the difference
  is sub-metre at most.
- **It cuts off the tail.** The distribution's *centre* doesn't move; its *outliers* stop existing.

On the RX that matters twice over, because the RX GPS feeds **Return-to-Me and Follow-Me** — the
modes where the buggy drives itself. A bogus fix there isn't a wrong number on a screen; it's a
wrong idea about where you are. There's also a second-order effect: a suspicious fix trips
`gps_rejected`, which **blocks RTM arming**. So outliers didn't just risk bad behaviour — they
caused the remote to refuse to arm for no visible reason.

### ⚠️ The altitude caveat — read this if you're not at sea level

**`dynModel = Sea` is valid to 500 m altitude.** Above that, the receiver may reject perfectly
good fixes, because your real position sits outside the model.

- The Great Lakes are 75–183 m. Lake Michigan is 176 m — roughly **3× margin**. Fine.
- **A mountain lake above 500 m is not fine.** Lake Tahoe (1,897 m), Lake Titicaca (3,812 m), any
  alpine reservoir — Sea is the wrong model there.

**If you ride above 500 m, change `dynModel` to 4 (Automotive)**: 6,000 m ceiling, 100 m/s
horizontal, 15 m/s vertical. Still far tighter than Portable, and still kills the 254 km/h class
of garbage.

This is **hard-coded on purpose**, not a config field — adding one would have changed the config
struct size, which forces a settings wipe on the next flash. Changing it means editing
`setNav5Sea` in `GPS.ino` (byte index 8 is the dynModel value) and recompiling.

---

## 2. BN-220/BN-880 and M10 speak different languages

This is the part that took the most work, and it's the reason a straightforward "just send the
config" approach quietly fails.

**u-blox M10 removed the legacy configuration messages.** Not deprecated — *removed*. Its entire
`UBX-CFG` class is five messages: `CFG-CFG`, `CFG-RST`, `CFG-VALDEL`, `CFG-VALGET`, `CFG-VALSET`.

The firmware was sending five messages that **do not exist on M10**:

| Legacy message (u-blox 6/7/8) | Sets | On M10 |
|---|---|---|
| `CFG-PRT` | UART baud rate | ❌ rejected |
| `CFG-RATE` | measurement rate | ❌ rejected |
| `CFG-MSG` | NMEA sentence filter | ❌ rejected |
| `CFG-NAV5` | **dynModel** | ❌ rejected |
| `CFG-GNSS` | constellations | ❌ rejected |

So on a genuine M10 the firmware applied **no rate, no NMEA filter, and no dynModel** — the
module sat in Portable, and nothing said so.

### How it's solved: the rejection *is* the detection

u-blox guarantees every well-formed `UBX-CFG` message is answered by either `ACK-ACK` or
`ACK-NAK`, carrying the class and id of the message being answered. That contract is identical
on u-blox 6, 7, 8, M9 and M10.

So the firmware sends `CFG-NAV5` and reads the answer:

- **ACK** → u-blox 6/7/8. Legacy dialect. Carry on.
- **NAK** → "I don't speak that." Automatically re-send the same setting via **`CFG-VALSET`**,
  the modern key/value interface.
- **silence** → unknown module. Send blind (old behaviour), **report it**, never fail.

**A NAK means "wrong dialect", not "bad hardware."** You don't have to tell the firmware which
chip you fitted. A BN-220, BN-880, NEO-M8N, NEO-M9N, MAX-M10S or HGLRC M100 all self-configure
from the same image.

The modern equivalents, verified against the u-blox M10 interface description:

| Setting | Legacy | Modern key |
|---|---|---|
| dynModel | `CFG-NAV5` | `CFG-NAVSPG-DYNMODEL` `0x20110021` (Sea = 5) |
| rate | `CFG-RATE` | `CFG-RATE-MEAS` `0x30210001` |
| baud | `CFG-PRT` | `CFG-UART1-BAUDRATE` `0x40520001` |
| NMEA filter | `CFG-MSG` | `CFG-MSGOUT-NMEA_ID_*_UART1` |

---

## 3. Baud: the failure that bricks the receiver

The remote and the GPS must agree on a speaking speed, and neither knows the other's at startup.

The old approach was to **shout the baud command at 115200, then at 9600**, and hope one landed.
That works — until it doesn't, for a reason that isn't obvious:

**u-blox counts UART framing errors, and past roughly 100 it switches off its own receiver.**
Sending data at the wrong baud doesn't look like a mistake to the module — it looks like
gibberish, and gibberish is counted.

The critical detail: **that counter does not reset when the ESP32 reboots.** It resets only when
the *GPS module* loses power. So ~28 wrong-baud bytes per boot looks harmless in isolation and
accumulates across a bench session of reflashes until the receiver latches off.

**This happened during development, on real hardware.** NMEA still streaming out, every command
ignored, looking exactly like dead hardware.

### The fix: listen before you speak

A u-blox module streams NMEA continuously from power-on. So the firmware now **listens** across
115200 / 38400 / 9600 / 57600 / 19200 and waits to hear `$G…`. That identifies the baud with
**zero transmission**.

Only then does it speak — and if the module needs moving to a faster speed, the command goes out
**at the module's own baud**, so it's parsed correctly instead of sprayed at a guess.

The old dual-baud dance still exists as a **last-resort fallback**, used only when listening
hears nothing at all. On a working module it never runs.

### Symptoms and recovery

You have this failure if the GPS **reports position but rejects all configuration**:

```
GPS config [UNVERIFIED - module sends no ACK]: dynModel=Sea no-ACK | GSV no-ACK | ...
```

and `?gpsbaud` shows the signature — **NMEA present, UBX dead**:

```
  baud     NMEA
  9600     yes
  -> UBX input: DEAD — module is NOT accepting UBX
```

**Recovery: unplug USB, then switch the unit off. Wait a couple of seconds. Power back on.**

A reboot, a reset, or a re-flash will **not** clear it — the GPS module has to actually lose
power. That's also why no serial command can fix it: the module has stopped listening.

> The ~100-error threshold is documented by u-blox; the "stays dead until power loss" part is
> what we **observed on the bench**. u-blox documents an automatic re-enable after about a
> second. Both can be true — a module still being sprayed with wrong-baud traffic would
> re-disable about as fast as it recovers. The recovery procedure is correct under either model.

---

## 4. Baud vs update rate — why 9600 isn't enough

At 8N1 every byte costs 10 bits on the wire. GGA + RMC is roughly 142 bytes per fix:

| Rate | Needs | At 9600 | At 115200 |
|---|---|---|---|
| 1 Hz | ~1,420 bit/s | ✅ 15% | ✅ 1% |
| 5 Hz | ~7,100 bit/s | ⚠️ **74%** — no headroom, drops sentences | ✅ 6% |
| 10 Hz | ~14,200 bit/s | ❌ **impossible** | ✅ 12% |

The firmware warns at boot when the configured rate doesn't fit the link:

```
TX GPS: !! 9600 baud is only 27% clear of what 5 Hz needs — sentences WILL drop.
        Run '?gpssetup' to move the module to 115200 permanently.
```

GSV, GLL and VTG are switched off because TinyGPS++ never parses them — GSV alone can be 6–10
sentences per epoch with multiple constellations. GSA is deliberately left enabled.

---

## 5. Two independent checks, not one

The whole design rests on not trusting a single signal:

1. **The ACK** proves the module *accepted* the write.
2. **`?gpscfg`** reads the setting back *out of the module* and proves what it's actually running.

These fail differently. An ACK can't catch a setting that a later write or a warm restart undid.
A readback can't tell you which write was responsible. You want both.

This is not hypothetical either — **the readback is what caught the original bug**, minutes after
it was first written, on a receiver that had reported success.

---

## 6. Commands

| Command | What it does | Blocks |
|---|---|---|
| `?gpscfg` | Reads `dynModel` back out of the module. Works on both dialects. | ~3 s |
| `?gpsbaud` | Listen-only baud scan + UBX-alive check | ~2–6 s |
| `?gpssetup` | **One-time full setup**: find, raise to 115200, configure ACK-verified, save into the module, verify | ~15–20 s |
| `?gpsraw [sec]` | Raw NMEA dump — use when nothing else responds | 5 s |
| `?gpsreinit` | Re-run GPS init without rebooting | ~2 s |
| `?gpscoldreset` | Clear the satellite cache, force fresh acquisition | instant |

**Run `?gpssetup` once per assembled unit.** It writes the settings into the *module's own*
non-volatile memory, so boot only has to verify rather than reconfigure.

> The baud lives in the GPS module, **not** in the remote's config. Adding a field to the config
> struct would change its size and force a settings wipe — pairing, calibration, everything. The
> module has its own memory, so it costs nothing there, and the setting follows the module if you
> move it to another unit.

⚠️ These are **bench commands**. They block the main loop — on the RX that stops RTM/FM and VESC
polling while they run. On the TX they're USB-only (the TX disables serial on a battery boot).

---

## 7. Fitting a different GPS

**It should just work.** Plug it in, power on, run `?gpscfg`, expect `dynModel : 5 (Sea)`.

The firmware finds the module at whatever baud it ships on and configures it in whichever dialect
it speaks. Set `gps_chip_type` (0 = BN-220, 1 = BN-880, 2/3 = M10) and reboot.

Two things to check before buying:

**Voltage.** The TX supplies **3.3 V only**. Modules needing 3.6 V+ (e.g. HGLRC M100 Pro at
3.6–5.5 V) will have corrupted UART on the TX. Measure the red wire on your board before
assuming.

**Compass — RX only.** The RX needs a magnetometer for RTM/FM heading. A BN-880 has one; a BN-220
does not. If you swap modules, the RX driver speaks **QMC5883L at address 0x0D**, and you must
**re-run `?compasscal`** afterwards — different module, different mounting, different hard/soft
iron offsets. A stored calibration does not carry over, and on the RX a bad heading is a
Follow-Me fault.

---

## Appendix — testing BLE: use a clean battery boot

Recorded 2026-08-02 after this cost real time.

**On the TX, BLE cannot come up at all while it sits on the charge screen — and USB puts it
there.** `setup()` calls `checkCharger()` *before* `initTasks()`, and `checkCharger()` loops for
as long as USB is supplying power. The BLE task is created by `initTasks()`, so until you send
`?exitchg` (or unplug), that task does not exist yet and `?state` will correctly report
`BLE: OFF`. A scanning central will meanwhile keep finding the *cached* advertisement and
reporting `connect failed`.

That is not a fault and not a configuration problem — the remote simply has not reached that
point in boot. Nothing about `bt_enabled` is involved.

**So: do not judge BLE state while the unit is USB-tethered and being reset over serial.** A
`?state` query taken during that window can report `BLE: OFF` on a remote whose BLE is enabled
and working: init is deferred (`bleInitTask`), a serial reset drops the stack mid-negotiation,
and a scanning central will meanwhile keep finding the *cached* advertisement and reporting
`connect failed`. Every one of those symptoms is an artefact of the test setup, not a fault.

**Valid test:** power both units on battery, USB disconnected, and leave them alone. Expect the
BLE dots to flash on both, connect within roughly ten seconds, then telemetry to flow.

The same caution applies to the GPS commands — `?gpscfg`, `?gpsbaud` and `?gpssetup` are
USB-only by design, and repeatedly resetting a unit to re-read them changes the thing being
measured.
