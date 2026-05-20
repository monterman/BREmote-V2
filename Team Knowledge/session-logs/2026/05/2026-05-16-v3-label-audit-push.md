# 2026-05-16 — V3 Label Audit + Push

**Branch:** `feature/bluetooth`
**Continues from:** [[2026-05-16-post-flash-sw56-deleteall-logs]]

---

## What happened

### V3 → V2.5-Evo bulk replacement (commit `13640ec`)

Previous audit identified 29 occurrences of bare `V3` (word-boundary matched) in source comment labels and header strings across TX+RX. Final count after replacement: **31 occurrences in 11 files** (two extra found at replacement time).

**Files changed:**

| File | Replacements |
|---|---|
| `Source/V2_Integration_Rx/BREmote_V2_Rx.h` | 1 |
| `Source/V2_Integration_Rx/GPS.ino` | 3 |
| `Source/V2_Integration_Rx/Logger.ino` | 7 |
| `Source/V2_Integration_Rx/SPIFFS.ino` | 1 |
| `Source/V2_Integration_Rx/System.ino` | 7 |
| `Source/V2_Integration_Rx/VESC.ino` | 1 |
| `Source/V2_Integration_Tx/BREmote_V2_Tx.h` | 7 |
| `Source/V2_Integration_Tx/Display.ino` | 1 |
| `Source/V2_Integration_Tx/Init.ino` | 1 |
| `Source/V2_Integration_Tx/Radio.ino` | 1 |
| `Source/V2_Integration_Tx/GPS.ino` | 1 |

**Excluded:** `Source/V2_Integration_Rx/vesc_datatypes.h` — contains `FOC_SENSOR_MODE_HFI_V3`, a VESC protocol enum identifier that must not be renamed.

Verified: no `\bV3\b` remains in any `.ino`, `.h`, or `.cpp` file outside `vesc_datatypes.h`.

### Pushed to remote

`feature/bluetooth` pushed: `7cb2a99..13640ec`

---

## Pending (carry forward)

- [ ] Flash TX — SW56 animation fix; test first-boot unlock
- [ ] Flash RX — `?deleteallogs` command; test from web tool
- [ ] Confirm VESC Tool BLE gauges show live voltage, duty, RPM
- [ ] `sendAuxCommand()` — strobe/find-me not wired to any button gesture yet

## Cross-links

- [[2026-05-16-post-flash-sw56-deleteall-logs]] — same day, earlier context: SW56 + delete-all-logs implementation
- [[2026-05-16-lora-packet-expansion-display-fix]] — same day, even earlier: 8→19 byte packet expansion
