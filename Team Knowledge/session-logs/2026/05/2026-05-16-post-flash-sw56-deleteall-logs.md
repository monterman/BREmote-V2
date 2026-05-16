# 2026-05-16 — Post-Flash: SW56 Animation Fix + Delete-All Logs

**Branch:** `feature/bluetooth` (HTML changes cherry-picked to `master`)
**Continues from:** [[2026-05-16-lora-packet-expansion-display-fix]]

---

## What happened

### RX compile error — duplicate `buffer_get_float32_auto` (commit `0f0d0f1`)

After flashing attempt, RX failed to compile:

```
error: 'float buffer_get_float32_auto(...)' was declared 'extern' and later 'static'
```

Root cause: `vesc_buffer.h` already declares `buffer_get_float32_auto` as extern, and `vesc_buffer.cpp` already implements it with the correct exponent bias 126. Our duplicate `static` definition in `VESC.ino` conflicted on linkage. Removed the duplicate — one-line fix.

### Both TX + RX compiled clean and flashed

Confirmed by user: "both clean and flashed."

### Display first-boot animation — still slow at last frame

User reported: first unlock after boot still shows sluggish last frame. Waited 20 seconds before unlocking — still slow. 2nd and 3rd unlocks smooth.

**Root cause:** WiFi AP (running since boot) was fully active during `unlockAnimation()` on the first unlock. WiFi stack background tasks (DHCP, beacons) preempt Core 0 animation frames — making the last frame stutter. After first unlock, `webCfgNotifyTxUnlocked()` stops the AP, so 2nd/3rd unlocks have no WiFi contention.

### SW56: WiFi AP stopped before animation (commits `f38ea31`)

**Hall.ino:** moved `webCfgNotifyTxUnlocked()` to before `unlockAnimation()`.  
**WebConfig.ino:** `webCfgNotifyTxUnlocked()` now calls `webCfgStopService()` directly (synchronous) instead of setting a deferred flag.

User clarified they don't mind the full painted square being visible while AP shuts down. With SW56, the sequence is:
1. Confirm unlock → AP stops synchronously (~200-300ms, pre-animation)
2. Arrow animation plays clean (180ms, no WiFi tasks)
3. Full square holds (~750ms delays) → ready to ride

**Status:** compiled, NOT yet flashed to TX.

### Delete-all-logs (commits `df87157`, `1662cef`, `80e089f`, `6ee3832`)

**Problem:** No way to delete all SPIFFS log files at once. User had been asking 2 days.

**Firmware (RX):**
- `deleteAllLogFiles()` in Logger.ino — iterates SPIFFS, removes all `.log` files, skips active log
- `?deleteallogs` serial command registered in System.ino
- `POST /api/logs/delete_all` web endpoint in WebConfigEngine.h

**Web tool (GitHub Pages):**
- **Delete All** button added to Logs panel (next to Refresh List, red danger style, confirms before running)
- `?deleteallogs` added to Logging quick-commands dropdown
- Both HTML changes cherry-picked to `master` and live immediately

**Status:** firmware NOT yet flashed to RX.

### Team rule locked in memory

**Serial command sync rule:** every new `?command` must ship simultaneously in:
1. Firmware handler + registration
2. Quick-commands dropdown in the web tool HTML
3. Relevant UI panel button (if applicable)
4. HTML always cherry-picked to `master` immediately

---

## Pending (carry forward)

- [ ] Flash TX — SW56 animation fix; test first-boot unlock
- [ ] Flash RX — `?deleteallogs` command; test from web tool
- [ ] Confirm VESC Tool BLE gauges show live voltage, duty, RPM (unverified since packet expansion flash)
- [ ] `sendAuxCommand()` — strobe/find-me not wired to any button gesture yet
- [ ] Animation timing decision — user OK with WiFi stopping before animation (SW56); no further action needed unless feel of pre-pause is unacceptable after flash

## Cross-links

- [[2026-05-16-lora-packet-expansion-display-fix]] — same day, earlier context: 8→19 byte packet expansion + display retry fix
- [[2026-05-14-sw55-vesc-fix-display-polish-github-push]] — previous session
