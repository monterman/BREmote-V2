# BREmote TX — prebuilt firmware

Flash these directly with **esptool** — no Arduino IDE, no toolchain, no compiling.

Board: **HT-CT62 (ESP32-C3)** · partition scheme `huge_app` · app offset **`0x10000`**

## Which one do I want?

| File | SW | What it is |
|---|---|---|
| `BREmote-TX-SW27R2-haptics.bin` | 27 | **Try this and tell me what you think.** Everything in `ubx-checksum` plus far fewer vibrations — see below. Not yet the recommended build, because how it feels is a judgement I cannot make from a build log. |
| `BREmote-TX-SW27-ubx-checksum.bin` | 27 | **Start here.** Current. Everything in `gps-verified` **plus** the UBX checksum fix below. |
| ~~`BREmote-TX-SW27-haptics.bin`~~ | 27 | ⚠️ **Withdrawn — do not use.** The vibration cull was applied in the wrong place: it silenced the *uncommanded* stops (RTM max-runtime, GPS stale, throttle-release timeout, and the RX fault-stop) as well as the deliberate ones, so the buggy could stop on its own with no buzz at all. `SW27R2-haptics` above replaces it and restores exactly those. |
| `BREmote-TX-SW27-gps-verified.bin` | 27 | The previous build, **kept deliberately**. Field-proven — this is the one that has actually been ridden. Every GPS config write is ACK-verified, auto-detects u-blox M8 vs M9/M10, never transmits at an unconfirmed baud. Adds `?gpscfg`, `?gpsbaud`, `?gpssetup`. |
| `BREmote-TX-SW26R2-rtm-working.bin` | **26R2** | Known-good historical build from 2026-06-05. RTM working; Follow-Me not yet matured. Fallback if something newer misbehaves. ⚠️ **Different `SW_VERSION` — see below.** |

### What changed in `ubx-checksum` (2026-08-15)

`ubxPoll()` was accepting a GPS config reply as soon as it had the payload, **without ever reading
the two checksum bytes**. Any byte sequence in the stream that looked like a valid UBX header was
believed. That function decides whether your module speaks the legacy u-blox 6/7/8 dialect or the
M9/M10 one, and `?gpssetup` writes in whichever dialect it reports — so a false verdict means the
module is sent commands it cannot parse and **configuration fails silently**.

Not theoretical: the RX had the identical bug and reported a BN-880 (an M8) as "M9/M10" on three
consecutive runs. Fixed on the RX in `1f2ba8c` (2026-08-02); this is the same fix applied to the TX.

**All three SW27 builds — `haptics`, `ubx-checksum` and `gps-verified` — are `SW_VERSION` 27, so
moving between them does NOT touch your settings.** Flash any of them either way freely — no
re-pairing, no re-calibration. Only `SW26R2` is a different version; see below.

> ⚠️ **`ubx-checksum` has not been bench-tested yet.** It compiles clean and the change is confined
> to the GPS config path — nothing in throttle, steering, PWM or failsafe is touched — but no one has
> run it on hardware. `gps-verified` remains the field-proven build. If anything looks off, drop back
> to it; the two are interchangeable.
>
> Verifying it takes a minute: `?gpscfg` should name the correct dialect for your module, and
> `?gpssetup` should complete.

> The RX folder has an intermediate `pre-gpsbaud` build. The TX equivalent was withdrawn
> and is deliberately not published.

### What `haptics` changes (2026-08-17)

A rider holding the remote while concentrating on a wave does not decode vibration patterns — they
feel *a buzz*. Twenty-five buzz events across seven patterns is not a language, it is noise. Six
events are gone:

**Removed — you get no buzz when YOU do it:**
- Disarming RTM by steering or by the magnet toggle, disarming FM the same two ways, or selecting F0.
  You just did it, and the display already shows the stop. This was the most frequent buzz in the
  system.

**Kept — everything that tells you something you did not already know:**
- Arm confirm. You cannot watch the display while riding.
- Weak signal, radio failsafe, water ingress (E7). These must cut through and are never suppressed.
- Magnet "release now" prompts — a blind gesture needs to be told when to let go.
- Arm window expired — the system changing state without you.

**One long buzz means the system stopped without you asking** — RTM hitting its runtime limit, the
TX losing its GPS fix, the throttle-release timeout expiring, or the RX faulting and stopping
Follow-Me. It also fires when an **arm is refused**, which is the same class of information: the
mode you just asked for is not running.

> ⚠️ **The first `haptics` build (`SW27-haptics`, withdrawn) got this backwards.** The cull was
> applied inside the two shared functions that end an engagement — but those serve the automatic
> stops as well as the deliberate ones, so *every* uncommanded stop went silent while the buzz
> survived only on arm refusals. `SW27R2-haptics` classifies each of the ten call sites explicitly
> and restores all five automatic stops.

Two delivery bugs are fixed alongside it. A stop buzz now genuinely preempts: it is raised as a
pending request that is promoted ahead of every other pattern and cuts a multi-pulse pattern short
between pulses, instead of being silently overwritten by whatever was already playing (Pattern 3
runs four seconds — long enough to swallow a stop buzz entirely). And the weak-signal warning no
longer marks itself as "given" when the collision guard stopped it from ever playing, so a signal
drop that coincides with another buzz is still reported.

## Flash it

```bash
esptool --chip esp32c3 --port COM<N> write-flash 0x10000 BREmote-TX-SW27-ubx-checksum.bin
```

**Find your port by MAC, not by COM number** — COM numbers move between reboots, and the TX and
RX are the same chip:

```bash
esptool --chip esp32c3 --port COM<N> read-mac
```

## Before you flash

**Back up your config.** `?conf` prints a Base64 blob; `?setconf <blob>` then `?applyconf`
restores it. Throttle calibration and pairing are the expensive things to lose.

**Check the partition table hasn't moved** — if it matches, your settings survive:

```bash
esptool --chip esp32c3 --port COM<N> verify-flash 0x8000 <build>.partitions.bin
```

These are **app-only** images (`0x10000`). They do not touch the partition table or SPIFFS, so
the flash itself never erases your settings.

> ⚠️ **But rolling back to `SW26R2` DOES reset your config, and that is not the flash's doing.**
> The firmware compares the stored `SW_VERSION` against its own on boot; when they differ it
> rewrites the config to defaults. `SW26R2` is version **26**, the current build is **27**, so
> going backwards **wipes throttle calibration, pairing and every setting**.
>
> **Back up first — `?conf` — and expect to restore with `?setconf <blob>` + `?applyconf` and
> re-pair.** Moving *forward* again from 26R2 to 27 wipes it a second time, for the same reason.
>
> *(This paragraph previously claimed `SW_VERSION` was 27 across all builds and that switching
> between them was safe. It was wrong, and it was wrong in the direction that costs you a
> calibration.)*

## After flashing

The TX sits on the **charge screen** while USB is connected — that is normal, and GPS init has
not run yet. Send **`?exitchg`** to boot through. Then:

```
?gpscfg      →  expect  dynModel : 5 (Sea)
```

If it says anything else, see [`docs/hardware/gps-troubleshooting.md`](../../../docs/hardware/gps-troubleshooting.md).

> Full-flash `.merged.bin` images are deliberately **not** published here. They are 4 MB each and
> overwrite the whole chip including your settings — the wrong tool for updating a working remote.
