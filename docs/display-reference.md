# BREmote V3 — Dot Display Reference

## Hardware
Two 5×7 LED matrices side by side = 10 columns × 7 rows = 70 red LEDs
Driver: HT16K33 at I2C address 0x70

## Zone Map
| Zone | Location | Description |
|---|---|---|
| Digits | C0-C2 (left) + C4-C6 (right), R0-R4 | Two character display |
| Gap | C3, R0-R4 | Normally dark — separates digits; R4 used as decimal dot during RTM distance display |
| GPS status | C7 R0 only | NEW in V3 — see below |
| Temperature | C8, R0-R4 | VESC temp, fills bottom→top |
| Signal | C9, R0-R4 | LoRa signal, fills bottom→top |
| Unused | R5, all cols | Reserved for future indicators |
| Battery | R6, all cols | VESC battery, fills left→right |

## GPS Status Dot (C7 R0) — V3 New Feature
Only visible when gps_en = 1 in SPIFFS config.

| State | Display | Meaning |
|---|---|---|
| Solid green | ● | Valid GPS fix — ready for RTM/Follow-Me |
| Slow blink 1s | ○ ● ○ ● | Acquiring fix — waiting for satellites |
| Fast blink 250ms | ●●●● | GPS rejected — spoofing or bad signal alert |
| Off | (dark) | GPS disabled (gps_en = 0) |

## Decimal Dot (C3 R4) — RTM Distance Display
When RTM is active and `rtm_display_mode = 0` (distance), the gap column pixel at R4 lights as a decimal point:
- `3.5` → left digit "3", decimal dot at C3 R4, right digit "5"
- `12` → left digit "1", no decimal dot, right digit "2"
- Set via `displayBuffer[5] |= (1 << 3)` after calling `displayDigits()`

## Vibration Feedback Patterns — V3 P8
| Pattern | Feel | Trigger |
|---|---|---|
| 2 short (150ms) | Gentle | Low VESC battery warning (Pattern 1) |
| 5 short (150ms) | Urgent | LoRa connection lost OR GPS rejected (Pattern 2) |
| 5 long (500ms) | Emergency | Water detected inside buggy — E71 (Pattern 3) |
| 2 fast short (80ms) | Quick confirm | RTM arm / RTM disarm confirmed (Pattern 4) — NEW in P8 |

## Gesture Map — V3 P8 (Breaking Change from P7)
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

## RTM / FM Active Display — V3 P8 New Feature
When `rtm_tx_active == true`, normal display is replaced by RTM info display.
Controlled by `rtm_display_mode` SPIFFS field:

| Mode | Shows | Notes |
|---|---|---|
| 0 (default) | Distance to TX | X.X metres when <10m (decimal dot in C3 gap R4); XX metres when ≥10m |
| 1 | Speed | Same source as normal speed display (speed_src) |
| 2 | Alternating | Switches between distance and speed every 2.5s |

Distance is computed on RX side and sent via `telemetry.rtm_distance` (index 5).
Encoding: 0-99 = tenths of meter (0.0–9.9m); 100-254 = value-90 meters (100=10m, 199=109m).

## Status / Mode Codes (TX dot display)
| Code | Meaning |
|---|---|
| rn | RTM armed (waiting for squeeze) or RTM active — static 1.5s flash |
| F0 / F1 / F2 / F3 | FM mode selected: F0=off, F1=behind, F2=near right, F3=near left |
| -- | Unknown error or ET event — auto-clears after 3s |
| AR | First squeeze detected (double-squeeze mode) |
| RY | Ready for second squeeze |
| Stp | RTM cooldown / disengaged |

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

## New SPIFFS Fields — V3 P8 (TX-side)
| Field | Range | Default | Description |
|---|---|---|---|
| `rtm_display_mode` | 0-2 | 0 | RTM/FM active display: 0=distance, 1=speed, 2=alternating 2.5s |
| `fm_warn_distance_m` | 50-1000m | 150 | TX-RX distance for FM proximity warning vibration |
| `rtm_steer_exit_on_input` | 0-1 | 1 | 1=steering exits RTM; 0=blend/correction mode |
| `rtm_max_runtime_s` | 0-300s | 0 | Max RTM runtime; 0=disabled (safety gates handle scenarios) |
