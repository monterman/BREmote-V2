# BREmote V3 — Dot Display Reference

## Hardware
Two 5×7 LED matrices side by side = 10 columns × 7 rows = 70 red LEDs
Driver: HT16K33 at I2C address 0x70

## Zone Map
| Zone | Location | Description |
|---|---|---|
| Digits | C0-C2 (left) + C4-C6 (right), R0-R4 | Two character display |
| Gap | C3, R0-R4 | Always dark — separates digits |
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

## Vibration Feedback Patterns
| Pattern | Feel | Trigger |
|---|---|---|
| 2 short buzzes | Gentle | Low VESC battery warning |
| 5 short buzzes | Urgent | LoRa connection lost OR GPS rejected during autonomous mode |
| 5 long buzzes | Emergency | Water detected inside buggy (E71) |

## Display Modes (right toggle hold 2s cycles through)
| Code | Full name | Shows | Available when |
|---|---|---|---|
| tH | Throttle | 0-99% trigger pull | Always |
| Ub | Internal battery | Remote voltage ×10 (e.g. 42 = 4.2V) | Always |
| tE | Temperature | VESC temp °C | VESC connected |
| 5P | Speed | km/h, knots or mph per speed_src setting | GPS enabled |
| bA | VESC battery | Remaining % | VESC connected |

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
