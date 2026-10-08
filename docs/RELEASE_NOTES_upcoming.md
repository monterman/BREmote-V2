# Upcoming release — notes for riders

> **🌙 nightly firmware.** These notes describe the **nightly** firmware on the
> [`fm-stations` branch](https://github.com/monterman/BREmote-V2/tree/fm-stations), not the firmware
> on master. Nightly = bleeding edge, under water testing, use at your own risk. Binaries are in the
> `latest-FM-stations` folders. Its commits move to master gradually after more water testing.

Remote (TX) SW 27 and buggy (RX) SW 36, both V2.5-Evo.

## Before you flash

1. **Download all your logs from the buggy.** The log format changes to 3; after the flash the
   buggy can't open files from older firmware (the PC log reader on the branch still can).
2. **Back up your settings** on both boards ([how](../README.md#-backing-up-your-settings--read-this-before-you-flash)).
   Your stored settings are kept by this flash, but a backup costs a minute.
3. **Flash the remote and the buggy together.** The new screens need both.
4. Bench-test wheels up before the water.

## What's new

**Follow-Me screens.** One dot hopping = waiting. Two steady dots = following. The dot column
filling down = the buggy is coming to you. The numbers are metres between the remote and the
buggy. [See them animated, with a glance quiz.](https://monterman.github.io/BREmote-V2/followme-screens.html)

**Auto-return.** Stop, and the buggy parks and waits while you surf, swim or pump back. Squeeze
the trigger and it comes back to you. Pausing or letting go never cancels it. It ends when it
arrives, on a fault (St), or when you switch the remote off and on.

**Arrival is safe at full throttle.** When a returning buggy reaches you, steering is yours again
at once, but the throttle stays limited until you let go of the trigger once.

**Magnet tap** (`mag_mode` 4). Tap to arm Follow-Me, even on the rope; tap again while following
to step the station. Hold 2.5 s with the trigger released to start the manual return-to-me.

**Gestures need a released trigger.** The return gestures only start with the trigger fully
released, so steering a returning buggy can't start or cancel anything by accident. A magnet tap
still arms Follow-Me on the rope. A 1 s LEFT hold during the "rn" window cancels the return-to-me
back to full manual.

**Front stations F4 / F5\*.** Ahead-right and ahead-left, for downwind runs. Never dead ahead,
about 13 m (43 ft) to the side, extra distance forward, and a protection cone around your path that
the buggy never cuts through. *\* In development and testing, not yet water-tested. The buggy side
is in; picking F4/F5 from the remote comes in a later remote update.*

**Serial console on the buggy's WiFi page** (RX only). From your phone at the dock or the beach:
a dropdown of the commands the buggy allows over WiFi, a free-text box, and one-tap buttons for
everyday jobs (Save, `?get`, North check, compass align, `?diag`, `?conf`, `?printgps`,
`?logstat`). Save checks for free space first, so a nearly full buggy can't silently lose your
settings.

**Throttle-input fault protection.** If the remote's trigger sensor stops answering (water, a stuck
sensor bus), the throttle goes to zero and the screen blinks St. It used to hold the last value.

**Motor needs a fresh throttle command.** The buggy only runs the motor on a fresh throttle packet,
not just a live radio link.

**Fewer buzzes.**

| Buzz | Means |
|---|---|
| two short taps | armed |
| one long + St | stopped, refused or a fault |
| five short | link lost |
| five long + E 7 | water in the buggy |
| two short, out of the blue | weak signal or low battery |

No buzz for distance, a station step or the start of following. The distance-warning buzz is gone.

**Second VESC in the logs.** Log level 4 now records both VESCs.

## Changed defaults

| Setting | Board | New default |
|---|---|---|
| `fm_display_mode` | remote | **2** — numbers show metres to the buggy (was 1, your speed) |
| `fm_warn_distance_m` | remote | 150 m, now only the bar's full scale (no buzz) |
| `fm_return_mode` | buggy | **1** — auto-return on |
| Setup wizard battery | — | 12S |

Boards with a stored value keep it; defaults apply to new or reset boards. The
[setup wizard](https://monterman.github.io/BREmote-V2/setup-wizard.html) gives you a full config
for your setup. Review it before you ride.

## Safety rules behind all of this

Manual always wins. "St" is the one not-working signal. The screen shows only what the buggy
confirms. Auto-return waits for the rider, and "waiting" means it will work. Arrival keeps the
throttle capped until one full release. Details in the
[Follow-Me guide](FOLLOW_ME_GUIDE.md#2-safety-philosophy).

## Thanks

Ludwig, BREmote's author ([lbre.de/BREmote](https://lbre.de/BREmote/),
[setup video](https://youtu.be/r6JIZEq3aTU), [github.com/Luddi96/BREmote-V2](https://github.com/Luddi96/BREmote-V2)),
and Heiguga for the Follow-Me ideas and the front stations.
