# Changelog

## 2026-07-25 — GPS heading fix (Alpha, pending on-water testing)

**Recommended: reflash both TX and RX.**

### The bug
The receiver has **one serial line shared between the GPS and the VESC**, switched back and forth by a multiplexer. The switching had the priority backwards: it sat on the VESC and only glanced at the GPS for 10 milliseconds, twice a second.

The GPS talks in bursts, so that window almost always landed in the silence between bursts. **The receiver was catching about 2% of what the GPS said** — roughly one course update every 25 seconds.

With no live course, Follow-Me fell back to the compass. The compass is snapshotted while the motor is idle and can't refresh while you're holding the trigger, so it went stale mid-run — **and the buggy steered the wrong way with nothing detecting it.**

The 10 ms delay turned out to be leftover code guarding something that had been deleted long ago. The multiplexer actually switches in under 3 microseconds.

### The fix
**Priority swapped: the line now rests on the GPS, and the VESC is the visitor.** Measured on hardware:

| | Before | After |
|---|---|---|
| GPS sentences per second | 0.1 | **84.8** |
| GPS fix | none | **holds a fix, 23 ms old** |
| VESC telemetry success | 72% | **100%** |
| Worst-case loop time | 239 ms | **29 ms** |

It also uses **fewer** multiplexer switches than before, not more.

### Also fixed
- **Follow-Me now refuses to steer on a dead heading.** A GPS course that stops changing is no longer treated as live just because its timestamp keeps updating, and a frozen course no longer silently hands steering to the compass. If the compass and GPS disagree by more than 45°, neither is trusted and the buggy holds straight.
- **New safety net:** if Follow-Me stops actually following — beyond twice the engage distance and not closing — it stops and hands control back.
- **Signal bar was reading low on a healthy link.** The scale bottomed out ~18 dB too early, which also caused false weak-signal buzzes.
- **Distance display:** the decimal dot now always means a decimal. It used to mean "×100" above 100 m, so 170 m showed as `1.7` and read as 1.7 m. Now `17` is 17 m, `1.7` is 1.7 m, and 100 m or more scrolls **FAR**. Metres only this version.
- **Follow-Me engage distance is now a real setting** — measure your tow rope and set at least a metre beyond it. Minimum 8 m, enforced. A value shorter than your rope would let Follow-Me engage while you're still being towed.
- **Log downloads over WiFi were missing five columns** (including signal strength) and reported **motor RPM 10× too low**. Both export paths now match.
- **Logging levels added** (`log_level`, 0–4). Level 4 records deep diagnostics — GPS throughput, whether the course is actually changing, I²C errors, worst loop time — so a session can be diagnosed from its log.
- **Compile fix:** the RX build reported "97% of program storage" and would have refused to build with ~800 KB still free. Use `PartitionScheme=custom` — see the RX flashing guide. Your partition layout is unchanged; only the size check was wrong.

### Still alpha
**This is fixed but not yet proven on the water.** The measurements above are bench and bucket tests. On-water validation is next. Test at your own risk, keep manual control in reach, and report anything you see.

Your saved settings are **not** wiped by this update.
