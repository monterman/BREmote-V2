# BREmote V2.5-Evo — Dot Display Reference

## Hardware
Two 5×7 LED matrices side by side = 10 columns × 7 rows = 70 red LEDs
Driver: HT16K33 at I2C address 0x70

## Zone Map
| Zone | Location | Description |
|---|---|---|
| Digits | C0-C2 (left) + C4-C6 (right), R0-R4 | Two character display |
| Gap | C3, R0-R4 | Normally dark — separates digits; R4 used as decimal dot during RTM distance display |
| GPS status | C7 R0 only | NEW in V2.5-Evo — see below |
| Temperature | C8, R0-R4 | VESC temp, fills bottom→top |
| Signal | C9, R0-R4 | LoRa signal, fills bottom→top |
| R5 proximity bar | R5, C0-C9 | RTM/FM proximity indicator — NEW in P9 (see below) |
| Battery | R6, all cols | VESC battery, fills left→right |

## GPS Status Dot (C7 R0) — V2.5-Evo New Feature
Only visible when gps_en = 1 in SPIFFS config.

| State | Display | Meaning |
|---|---|---|
| Solid green | ● | Valid GPS fix — ready for RTM/Follow-Me |
| Slow blink 1s | ○ ● ○ ● | Acquiring fix — waiting for satellites |
| Fast blink 250ms | ●●●● | GPS rejected — spoofing or bad signal alert |
| Off | (dark) | GPS disabled (gps_en = 0) |

## Decimal Dot (C3 R4) — Distance Display
When `rtm_display_mode = 0` (distance) and the value is ≥100m (metric) or ≥100ft (imperial), the gap column pixel at R4 lights as a decimal point for fractional km or miles:
- `3.5` → left digit "3", decimal dot at C3 R4, right digit "5" (e.g. 3.5 km or 3.5 mi)
- `12` → left digit "1", no decimal dot, right digit "2" (e.g. 12 m or 12 ft — whole value)
- Set via `displayBuffer[5] |= (1u << 3)` **after** calling `displayDigits()` (displayDigits clears R4)

## Vibration Feedback Patterns — V2.5-Evo P8
| Pattern | Feel | Trigger |
|---|---|---|
| 2 short (150ms) | Gentle | Low VESC battery warning (Pattern 1) |
| 5 short (150ms) | Urgent | LoRa connection lost OR GPS rejected (Pattern 2) |
| 5 long (500ms) | Emergency | Water detected inside buggy — E71 (Pattern 3) |
| 2 fast short (80ms) | Quick confirm | RTM arm / RTM disarm confirmed (Pattern 4) — NEW in P8 |

## Gesture Map — V2.5-Evo P8 (Breaking Change from P7)
| Gesture | Action | Notes |
|---|---|---|
| LEFT tap | Record tap (starts 3s combo window) | Sets up combo |
| RIGHT tap | Record tap (starts 3s combo window) | Sets up combo |
| LEFT hold 2s | Cycle telemetry display mode | Simple hold, no combo required |
| RIGHT hold 2s | Reserved — no action | Was: cycle display |
| RIGHT tap → LEFT hold 5s | Arm RTM (Return-to-Me) | Requires gps_en=1 and rtm_enabled=1 |
| LEFT tap → RIGHT hold 5s | Cycle FM mode (F0→F1→F2→F3) | Requires gps_en=1 and fm_override_enabled=1 |
| Boot: hold LEFT | Pairing mode | Unchanged |

**Notes:**
- Lock feature removed in P8. System always boots unlocked.
- Throttle must be at 0 for long-press actions to fire.
- Tap window: 3s (`COMBO_WINDOW_MS`). Tap must occur before the hold begins.

## Display Modes (LEFT toggle hold 2s cycles through)
| Code | Full name | Shows | Available when |
|---|---|---|---|
| tH | Throttle | 0-99% trigger pull | Always |
| Ub | Internal battery | Remote voltage ×10 (e.g. 42 = 4.2V) | Always |
| tE | Temperature | VESC temp °C | VESC connected |
| 5P | Speed | km/h, knots or mph per speed_src setting | GPS enabled |
| bA | VESC battery | Remaining % | VESC connected |

## RTM / FM Active Display — V2.5-Evo P8 + P9
When `rtm_tx_active == true`, normal display is replaced by RTM info display.
Controlled by `rtm_display_mode` SPIFFS field:

| Mode | Shows | Notes |
|---|---|---|
| 0 (default) | Distance to TX | Unit-aware via `dist_unit` (see Distance Unit section below) |
| 1 | Speed | Same source as normal speed display (speed_src) |
| 2 | Alternating | Switches between distance and speed every 2.5s |

Distance is computed on RX side and sent via `telemetry.rtm_distance` (index 5).
Encoding: 0-99 = tenths of meter (0.0–9.9m); 100-254 = value-90 meters (100=10m, 199=109m).
TX decodes this value then routes it through `displayDistanceInUnits()` for unit conversion.

## Distance Unit Display (P9 New — dist_unit SPIFFS field)
All distance values on the TX display (RTM mode 0) are converted using `dist_unit`:

| dist_unit | Unit | Range A | Range B (decimal dot active) | Minimum |
|---|---|---|---|---|
| 0 (default) | Metres / km | 1–99 m → whole metres ("01"–"99") | 100+ m → X.X km ("1.0"–"9.9") | <1 m → "00" |
| 1 | Feet / miles | 4–99 ft → whole feet ("04"–"99") | 100+ ft → X.X mi ("0.1"–"9.9") | <4 ft → "00" |

- Internal math is always metres. Unit conversion is display-layer only.
- Decimal dot (C3 R4) lights only in Range B (fractional km or miles).
- `displayDistanceInUnits(float dist_m)` in Display.ino handles all cases.

## Full-Screen Confirmation Messages (P9 New — fontCompact3x7)
One-time blocking flashes rendered using the compact 3×7 font (all 10 columns × all 7 rows).
Source font: `docs/Dot_Matrix_Display_10x7_Render.html` — canonical pixel layout reference.

| Message | Columns | Trigger |
|---|---|---|
| *(removed)* `A rM` | — | Arm confirm replaced by unlock animation + `rn` 2s blink (removed in P9 Bug4) |
| `St` | large-font `displayDigits(LET_S, LET_T)` — not compact font | RTM disengages (any exit path) or pre-arm rejected |
| `FM 0` – `FM 3` | F(3) + M(3) + space(1) + 0-3(3) = 10 | FM mode cycled or FM arm confirmed |
| `E 7` | E(3) + space(1) + 7(3) = 7 columns | Water ingress error code E71 (display shows "E 7" — the "1" does not fit; blinks 250ms on/off, non-blocking) |

RTM exit shows `St` via `displayDigits(LET_S, LET_T)` (large-font, 2s). `showFullScreenMessage()` is still used for `FM 0`–`FM 3` and `E 7` (water ingress). FreeRTOS vibration task continues running during any blocking display hold.

## R5 Proximity Bar (P9 New)
Row R5 (`displayBuffer[6]`) is used as a proximity bar during RTM or FM.
Blink pattern: 1000 ms on / 500 ms off.

**RTM bar (square-root curve):**
- At RTM engage: `rtm_arm_dist_m` is captured from current `telemetry.rtm_distance`.
- Each update: `pixels = round(sqrt(current_m / arm_m) × 10)`, clamped 0–10.
- Bar fills C0→C9 (left to right). Shrinks from right as buggy approaches.
- At 100% distance (just armed): all 10 pixels lit.
- At 25% distance: 5 pixels lit (sqrt(0.25) = 0.5 × 10 = 5).
- At 0% (hard stop): 0 pixels — bar gone just before RTM disengages.

![RTM Proximity Bar Animation](rtm_bar_animation.gif)

**FM bar (Priority 10 — linear fill; center-expanding version pending):**
- Current implementation: left-to-right linear fill. Full bar (10 pixels) = buggy close. Empty = buggy ≥30m away.
- **Implemented** (2026-04-30): center-expanding from C4-C5 sweet spot. 2 pixels at C4+C5 = ideal follow distance. Bar expands outward symmetrically as buggy gets closer. Dark when ≥30 m away. Full bar (C0-C9) = buggy right next to user.

![FM Proximity Bar Animation](fm_bar_animation.gif)

**`rtm_arm_dist_m` variable:**
- RAM only — never written to SPIFFS.
- Captured at RTM engage (both single-squeeze and double-squeeze paths).
- Reset to `0.0f` inside `rtmDisengage()` on every exit path.
- If `0.0f` at bar render time, bar is suppressed (avoids divide-by-zero).

## Status / Mode Codes (TX dot display)
| Code | Meaning |
|---|---|
| rn | RTM armed (waiting for squeeze) — static 1.5s flash ×2 on arm; blinks 500ms during arm window |
| AR | *(deprecated)* First squeeze detected (double-squeeze mode removed) |
| RY | *(deprecated)* Ready for second squeeze (double-squeeze mode removed) |
| St | RTM disengaged or pre-arm rejected — large-font `displayDigits(LET_S, LET_T)`, 2s |
| FM 0 / FM 1 / FM 2 / FM 3 | FM mode confirmed — full-screen 2s flash (fontCompact3x7) |
| -- | ET error or no-data — auto-clears after 3s |

## Error Codes (TX dot display)
| Code | Meaning |
|---|---|
| EP | Not paired |
| EC | Not calibrated |
| E- | Error (code too large for display) |
| E71 | Water ingress detected (triggers 5 long buzzes) |
| E5V | Config version mismatch — re-flash or clear SPIFFS |
| U5b | USB serial mode active |
| XX | Power saver mode — remote going to sleep |

## New SPIFFS Fields — V2.5-Evo P8 (TX-side)
| Field | Range | Default | Description |
|---|---|---|---|
| `rtm_display_mode` | 0-2 | 0 | RTM/FM active display: 0=distance, 1=speed, 2=alternating 2.5s |
| `fm_warn_distance_m` | 50-1000m | 150 | TX-RX distance for FM proximity warning vibration |
| `rtm_steer_exit_on_input` | 0-1 | 1 | 1=steering exits RTM; 0=blend/correction mode |
| `rtm_max_runtime_s` | 0-300s | 0 | Max RTM runtime; 0=disabled (safety gates handle scenarios) |

## New SPIFFS Fields — V2.5-Evo P9 (TX-side)
| Field | Range | Default | Description |
|---|---|---|---|
| `dist_unit` | 0-1 | 0 | Distance display unit: 0=Metres/km, 1=Feet/miles. Internal math always in metres. |

**No sizeof change:** `dist_unit` fills the 2-byte tail padding left by P8.1. sizeof(confStruct) stays 128. No SPIFFS reset on first P9 flash — old configs read 0 here (tail padding was zero), which is the correct Metres default.
