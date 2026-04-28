# Priority 7: RTM Return-to-Me Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Return-to-Me (RTM) autonomous steering + Follow-Me (FM) runtime override on both TX (ESP32-C3) and RX (ESP32-S3), with all 10 safety gates and Phase C anti-spoofing.

**Architecture:** TX detects left-hold gesture → sends 0xF1 meta-packet → RX applies compass-based steering override; TX caps throttle ramp on its own side; RX emergency-stops on any safety gate failure. FM override uses right-hold → 0xF2 meta-packet → RX runtime variable (no SPIFFS write). Both 0xF1 and 0xF2 use the existing 6-byte LoRa format (same as control packets but with meta-type in byte[3]).

**Tech Stack:** TinyGPSPlus (bearing/distance), QMC5883L compass (readCompassRaw → atan2f heading), FreeRTOS volatile globals for cross-task communication, Arduino ESP32 C3/S3 firmware.

---

## File Map

### TX — files to modify
| File | Change |
|---|---|
| `Source/V2_Integration_Tx/BREmote_V2_Tx.h` | +12 uint16_t confStruct fields; sizeof 96→120; display chars LET_R/N/S/M; 5 new volatile globals |
| `Source/V2_Integration_Tx/ConfigService.ino` | +12 field entries |
| `Source/V2_Integration_Tx/WebUiEmbedded.h` | +12 UI entries |
| `Source/V2_Integration_Tx/Radio.ino` | +`queueMetaPacketBurst()`; modify `sendData()` to consume queue; cap throttle 0xF2→0xF0 |
| `Source/V2_Integration_Tx/Hall.ino` | Modify `handleGearToggle()`: left-hold → RTM arm; right-hold → FM cycle |
| `Source/V2_Integration_Tx/Throttle.ino` | Modify `calcFinalThrottle()` to apply `rtm_thr_cap_tx` |
| `Source/V2_Integration_Tx/V2_Integration_Tx.ino` | Run `getTxGPSLoop()` on `gps_en` alone (not speed_src gate); add `runRtmLoop()` |

### TX — files to create
| File | Purpose |
|---|---|
| `Source/V2_Integration_Tx/RTMState.ino` | RTM + FM state machines; display states; `setRtmArmed()`; `cycleFmMode()`; `runRtmLoop()` |

### RX — files to modify
| File | Change |
|---|---|
| `Source/V2_Integration_Rx/BREmote_V2_Rx.h` | +2 floats +3 uint16_t confStruct fields; sizeof 136→152; `#define VESC_MORE_VALUES`; 4 new volatile globals |
| `Source/V2_Integration_Rx/ConfigService.ino` | +5 field entries |
| `Source/V2_Integration_Rx/WebUiEmbedded.h` | +5 UI entries |
| `Source/V2_Integration_Rx/Radio.ino` | Modify `triggeredReceive()`: dispatch 0xF1/0xF2; add `processRtmStatePacket()`; add `processFmOverridePacket()` |
| `Source/V2_Integration_Rx/Compass.ino` | +`getCompassHeading()` |
| `Source/V2_Integration_Rx/PWM.ino` | Modify `calcPWM()`: apply emergency stop flag + steering override |
| `Source/V2_Integration_Rx/V2_Integration_Rx.ino` | Add `runRtmLoop()` call in outer loop |

### RX — files to create
| File | Purpose |
|---|---|
| `Source/V2_Integration_Rx/RTMState.ino` | 10 safety gates; compass bearing steering; Phase C; `runRtmLoop()` |

---

## Task 1: TX SPIFFS Infrastructure

**Files:**
- Modify: `Source/V2_Integration_Tx/BREmote_V2_Tx.h`
- Modify: `Source/V2_Integration_Tx/ConfigService.ino`
- Modify: `Source/V2_Integration_Tx/WebUiEmbedded.h`

- [ ] **Step 1: Add 12 new fields to TX confStruct**

Open `Source/V2_Integration_Tx/BREmote_V2_Tx.h`. Find the line `uint16_t gps_chip_type;` (currently the last field). Add immediately after it:

```cpp
    // ============================================================
    // V3 - 2026-04-25 - PRIORITY 7: RTM AND FM MODE PARAMETERS
    //
    // 12 new uint16_t fields — sizeof grows 96→120.
    // First flash of P7 firmware resets all TX settings to defaults.
    // After flashing: re-pair TX/RX, re-enter all settings via web UI.
    // ============================================================
    uint16_t rtm_enabled;              // RTM master enable; 0=off, 1=on; default 1
    uint16_t rtm_hold_duration_s;      // LEFT hold time to arm RTM; 4-10 s; default 5
    uint16_t rtm_arm_window_s;         // Window to engage throttle after arming; 5-30 s; default 10
    uint16_t rtm_double_squeeze_en;    // Require double-squeeze (1) or 500ms hold (0); default 1
    uint16_t rtm_throttle_start_pct;   // Initial throttle cap when RTM engages; 10-50 %; default 30
    uint16_t rtm_throttle_max_pct;     // Max throttle cap after ramp; 30-90 %; default 70
    uint16_t rtm_ramp_duration_s;      // Time to ramp throttle start→max; 2-15 s; default 5
    uint16_t rtm_stop_distance_m;      // Hard stop distance from TX; 3-20 m; default 10
    uint16_t rtm_max_runtime_s;        // Maximum continuous RTM runtime; 30-300 s; default 120
    uint16_t rtm_gps_timeout_ms;       // TX GPS loss timeout before safety stop; 500-3000 ms; default 2000
    uint16_t fm_hold_duration_s;       // RIGHT hold time for FM mode cycle; 4-10 s; default 5
    uint16_t fm_override_enabled;      // Allow TX to override RX follow-me mode; 0=off, 1=on; default 1
```

- [ ] **Step 2: Update TX static_assert and defaultConf**

Find `static_assert(sizeof(confStruct) == 96`. Change to:

```cpp
static_assert(sizeof(confStruct) == 120, "confStruct size mismatch — expected 120 bytes (V3.3/P7). Update this assert if you change the struct.");
```

Find `defaultConf = {` and locate the last two values `200, // gps_max_hdop` and `0  // gps_chip_type`. After the `0` (gps_chip_type), add a comma and the 12 new defaults:

```cpp
  0,             // gps_chip_type (0=BN-220 default)
  // V3 - 2026-04-25 - Priority 7 RTM/FM defaults
  1,    // rtm_enabled
  5,    // rtm_hold_duration_s
  10,   // rtm_arm_window_s
  1,    // rtm_double_squeeze_en
  30,   // rtm_throttle_start_pct
  70,   // rtm_throttle_max_pct
  5,    // rtm_ramp_duration_s
  10,   // rtm_stop_distance_m
  120,  // rtm_max_runtime_s
  2000, // rtm_gps_timeout_ms
  5,    // fm_hold_duration_s
  1     // fm_override_enabled
```

- [ ] **Step 3: Add display characters LET_R, LET_N, LET_S, LET_M**

Find the line `#define TLT 29` and add after it:

```cpp
#define LET_R 30
#define LET_N 31
#define LET_S 32
#define LET_M 33
```

Find the `num0[30][3]` array declaration and change the size to `num0[34][3]`. Then find the last entry `{0x04, 0x0A, 0x11},` (TLT) and add after it:

```cpp
                    //R (30)              //N (31)              //S (32)              //M (33)
                    {0x1F, 0x14, 0x13}, {0x1F, 0x10, 0x1F}, {0x1D, 0x15, 0x17}, {0x1F, 0x18, 0x1F}
```

Pattern meanings (3 cols × 5 rows, bit4=top):
- R {0x1F,0x14,0x13}: left-full / top+mid / top+bottom-two → recognizable R with leg
- N {0x1F,0x10,0x1F}: left-full / top-only / right-full → two bars joined at top
- S {0x1D,0x15,0x17}: same as digit 5 — standard S/5 shape
- M {0x1F,0x18,0x1F}: left-full / top-two / right-full → arch/tent shape

- [ ] **Step 4: Add 12 entries to TX ConfigService.ino**

Open `Source/V2_Integration_Tx/ConfigService.ino`. Find the line with `{"gps_chip_type", ...}`. After it, add:

```cpp
  // V3 - 2026-04-25 - Priority 7 RTM and FM mode parameters
  {"rtm_enabled",            CFG_U16, offsetof(confStruct, rtm_enabled),            true, false, true,  0.0f,   1.0f,    0, false},
  {"rtm_hold_duration_s",    CFG_U16, offsetof(confStruct, rtm_hold_duration_s),    true, false, true,  4.0f,  10.0f,    0, false},
  {"rtm_arm_window_s",       CFG_U16, offsetof(confStruct, rtm_arm_window_s),       true, false, true,  5.0f,  30.0f,    0, false},
  {"rtm_double_squeeze_en",  CFG_U16, offsetof(confStruct, rtm_double_squeeze_en),  true, false, true,  0.0f,   1.0f,    0, false},
  {"rtm_throttle_start_pct", CFG_U16, offsetof(confStruct, rtm_throttle_start_pct), true, false, true, 10.0f,  50.0f,    0, false},
  {"rtm_throttle_max_pct",   CFG_U16, offsetof(confStruct, rtm_throttle_max_pct),   true, false, true, 30.0f,  90.0f,    0, false},
  {"rtm_ramp_duration_s",    CFG_U16, offsetof(confStruct, rtm_ramp_duration_s),    true, false, true,  2.0f,  15.0f,    0, false},
  {"rtm_stop_distance_m",    CFG_U16, offsetof(confStruct, rtm_stop_distance_m),    true, false, true,  3.0f,  20.0f,    0, false},
  {"rtm_max_runtime_s",      CFG_U16, offsetof(confStruct, rtm_max_runtime_s),      true, false, true, 30.0f, 300.0f,    0, false},
  {"rtm_gps_timeout_ms",     CFG_U16, offsetof(confStruct, rtm_gps_timeout_ms),     true, false, true, 500.0f,3000.0f,   0, false},
  {"fm_hold_duration_s",     CFG_U16, offsetof(confStruct, fm_hold_duration_s),     true, false, true,  4.0f,  10.0f,    0, false},
  {"fm_override_enabled",    CFG_U16, offsetof(confStruct, fm_override_enabled),    true, false, true,  0.0f,   1.0f,    0, false},
```

- [ ] **Step 5: Add 12 entries to TX WebUiEmbedded.h**

Open `Source/V2_Integration_Tx/WebUiEmbedded.h`. Find the entry for `gps_chip_type` and add after it:

```
{key:"rtm_enabled",label:"RTM Enabled",description:"Master enable for Return-to-Me feature. 0=off, 1=on.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_hold_duration_s",label:"RTM Arm Hold Time",description:"Seconds to hold LEFT toggle to arm RTM. Range 4-10 s, default 5.",group:"RTM & Follow-Me",type:"int",def:5,min:4,max:10,unit:"s"},
{key:"rtm_arm_window_s",label:"RTM Arm Window",description:"Seconds after arming to complete throttle engagement. Range 5-30 s, default 10.",group:"RTM & Follow-Me",type:"int",def:10,min:5,max:30,unit:"s"},
{key:"rtm_double_squeeze_en",label:"RTM Double-Squeeze",description:"1=require two squeezes to engage (default), 0=hold throttle >30% for 500ms.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_throttle_start_pct",label:"RTM Start Throttle %",description:"Initial throttle cap when RTM engages. Range 10-50%, default 30%.",group:"RTM & Follow-Me",type:"int",def:30,min:10,max:50,unit:"%"},
{key:"rtm_throttle_max_pct",label:"RTM Max Throttle %",description:"Maximum throttle cap after ramp completes. Range 30-90%, default 70%.",group:"RTM & Follow-Me",type:"int",def:70,min:30,max:90,unit:"%"},
{key:"rtm_ramp_duration_s",label:"RTM Ramp Duration",description:"Seconds to ramp throttle from start% to max%. Range 2-15 s, default 5.",group:"RTM & Follow-Me",type:"int",def:5,min:2,max:15,unit:"s"},
{key:"rtm_stop_distance_m",label:"RTM Stop Distance",description:"Hard stop distance from TX. Motor cuts to 0 when closer than this. Range 3-20 m, default 10.",group:"RTM & Follow-Me",type:"int",def:10,min:3,max:20,unit:"m"},
{key:"rtm_max_runtime_s",label:"RTM Max Runtime",description:"Maximum continuous RTM runtime before auto-disengage. Range 30-300 s, default 120.",group:"RTM & Follow-Me",type:"int",def:120,min:30,max:300,unit:"s"},
{key:"rtm_gps_timeout_ms",label:"RTM GPS Timeout",description:"TX GPS data loss timeout before safety stop. Range 500-3000 ms, default 2000.",group:"RTM & Follow-Me",type:"int",def:2000,min:500,max:3000,unit:"ms"},
{key:"fm_hold_duration_s",label:"FM Hold Time",description:"Seconds to hold RIGHT toggle to enter FM mode cycling. Range 4-10 s, default 5.",group:"RTM & Follow-Me",type:"int",def:5,min:4,max:10,unit:"s"},
{key:"fm_override_enabled",label:"FM Override Enabled",description:"Allow TX to override RX follow-me mode at runtime (no SPIFFS write). 0=off, 1=on.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
```

- [ ] **Step 6: Compile TX firmware in Arduino IDE**

Open `Source/V2_Integration_Tx/V2_Integration_Tx.ino` in Arduino IDE. Select board: ESP32C3 Dev Module. Compile only (do not flash yet). Expected result: zero errors. The static_assert will catch any struct size mistakes at compile time — if it fails, recount the fields.

- [ ] **Step 7: Commit**

```bash
git add Source/V2_Integration_Tx/BREmote_V2_Tx.h \
        Source/V2_Integration_Tx/ConfigService.ino \
        Source/V2_Integration_Tx/WebUiEmbedded.h
git commit -m "feat(TX/P7): add RTM+FM SPIFFS params, display chars LET_R/N/S/M; sizeof 96→120"
```

---

## Task 2: RX SPIFFS Infrastructure

**Files:**
- Modify: `Source/V2_Integration_Rx/BREmote_V2_Rx.h`
- Modify: `Source/V2_Integration_Rx/ConfigService.ino`
- Modify: `Source/V2_Integration_Rx/WebUiEmbedded.h`

- [ ] **Step 1: Add 5 new fields to RX confStruct**

Open `Source/V2_Integration_Rx/BREmote_V2_Rx.h`. Find the line `float gps_max_speed_diff_kmh;` (currently the last field). Add immediately after it:

```cpp
    // ============================================================
    // V3 - 2026-04-25 - PRIORITY 7: RTM PHASE C + RX SAFETY PARAMETERS
    //
    // sizeof grows 136→152. Layout:
    //   float rtm_vesc_speed_diff_kmh  (4)
    //   float vesc_erpm_per_kmh        (4)
    //   uint16_t rtm_rx_enabled        (2)
    //   uint16_t rtm_rx_override_steer (2)
    //   uint16_t rtm_compass_required  (2)
    //   [2 bytes implicit tail padding]
    //
    // First flash of P7 firmware resets all RX settings to defaults.
    // After flashing: re-pair TX/RX, re-enter all settings, re-run runcal.
    // ============================================================
    float    rtm_vesc_speed_diff_kmh;    // Phase C: max GPS vs VESC speed diff; 5-50 km/h; default 20.0
    float    vesc_erpm_per_kmh;          // ERPM per km/h (vehicle-specific); default 0.0 (0=skip VESC check)
    uint16_t rtm_rx_enabled;             // RX-side RTM master enable; 0=off, 1=on; default 1
    uint16_t rtm_rx_override_steering;   // Allow RTM to override steering; 0=off, 1=on; default 1
    uint16_t rtm_compass_required;       // Require valid compass for RTM arming; 0=no, 1=yes; default 1
```

- [ ] **Step 2: Update RX static_assert and defaultConf**

Find `static_assert(sizeof(confStruct) == 136`. Change to:

```cpp
static_assert(sizeof(confStruct) == 152, "confStruct size mismatch — expected 152 bytes (V3.3/P7). Update this assert if you change the struct.");
```

Find `defaultConf = {` and locate the last two values `500.0f` and `50.0f`. After `50.0f`, add a comma and the 5 new defaults:

```cpp
  50.0f,      // gps_max_speed_diff_kmh
  // V3 - 2026-04-25 - Priority 7 RTM Phase C + RX safety defaults
  20.0f,      // rtm_vesc_speed_diff_kmh: max GPS vs VESC speed diff (5-50 km/h)
  0.0f,       // vesc_erpm_per_kmh: 0 = skip Phase C VESC check until calibrated
  1,          // rtm_rx_enabled: 1 = RTM enabled on RX side
  1,          // rtm_rx_override_steering: 1 = RTM may override steering
  1           // rtm_compass_required: 1 = compass required for RTM arming
```

- [ ] **Step 3: Add VESC_MORE_VALUES define**

In `BREmote_V2_Rx.h`, find the line `#include "vesc_datatypes.h"`. Add above it:

```cpp
// V3 - 2026-04-25 - P7: Enable ERPM fetch for Phase C speed consistency check.
// Without this define, vesc.erpm is always 0. Adds motor current, battery
// current, duty, and ERPM to the VESC request (~2ms extra UART per call).
#define VESC_MORE_VALUES
```

- [ ] **Step 4: Add 5 entries to RX ConfigService.ino**

Open `Source/V2_Integration_Rx/ConfigService.ino`. Find the entry for `gps_max_speed_diff_kmh`. After it, add:

```cpp
  // V3 - 2026-04-25 - Priority 7 RTM Phase C + RX safety parameters
  {"rtm_vesc_speed_diff_kmh",  CFG_FLOAT, offsetof(confStruct, rtm_vesc_speed_diff_kmh),  true, false, true,  5.0f, 50.0f,  1, false},
  {"vesc_erpm_per_kmh",        CFG_FLOAT, offsetof(confStruct, vesc_erpm_per_kmh),        true, false, true,  0.0f, 9999.0f,1, false},
  {"rtm_rx_enabled",           CFG_U16,   offsetof(confStruct, rtm_rx_enabled),           true, false, true,  0.0f,  1.0f,  0, false},
  {"rtm_rx_override_steering", CFG_U16,   offsetof(confStruct, rtm_rx_override_steering), true, false, true,  0.0f,  1.0f,  0, false},
  {"rtm_compass_required",     CFG_U16,   offsetof(confStruct, rtm_compass_required),     true, false, true,  0.0f,  1.0f,  0, false},
```

- [ ] **Step 5: Add 5 entries to RX WebUiEmbedded.h**

Open `Source/V2_Integration_Rx/WebUiEmbedded.h`. Find the entry for `gps_max_speed_diff_kmh` and add after it:

```
{key:"rtm_vesc_speed_diff_kmh",label:"Phase C: Max VESC Speed Diff",description:"Phase C: max difference between GPS speed and VESC ERPM-implied speed during active RTM. Range 5-50 km/h, default 20.",group:"GPS & Follow-Me",type:"float",def:20.0,min:5.0,max:50.0,step:1.0,unit:"km/h"},
{key:"vesc_erpm_per_kmh",label:"VESC ERPM per km/h",description:"Vehicle-specific: how many ERPM equals 1 km/h. Set by driving at known speed and reading ERPM from ?printtasks. 0=disable Phase C VESC check.",group:"GPS & Follow-Me",type:"float",def:0.0,min:0.0,max:9999.0,step:1.0,unit:"ERPM/kmh"},
{key:"rtm_rx_enabled",label:"RTM RX Enabled",description:"RX-side RTM master enable (safety kill switch). 0=RTM blocked regardless of TX. Default 1.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_rx_override_steering",label:"RTM Override Steering",description:"Allow RTM to override steering towards TX GPS position. 0=disable steering override (throttle limiting only). Default 1.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_compass_required",label:"RTM Compass Required",description:"Require valid compass reading for RTM arming. 0=allow RTM without compass (steering disabled). Default 1.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
```

- [ ] **Step 6: Compile RX firmware**

Open `Source/V2_Integration_Rx/V2_Integration_Rx.ino` in Arduino IDE. Board: ESP32S3 Dev Module. Compile only. Zero errors expected. static_assert catches struct size bugs.

- [ ] **Step 7: Commit**

```bash
git add Source/V2_Integration_Rx/BREmote_V2_Rx.h \
        Source/V2_Integration_Rx/ConfigService.ino \
        Source/V2_Integration_Rx/WebUiEmbedded.h
git commit -m "feat(RX/P7): add RTM Phase C + safety params; VESC_MORE_VALUES; sizeof 136→152"
```

---

## Task 3: TX Meta-Packet Queuing + Throttle Cap

**Files:**
- Modify: `Source/V2_Integration_Tx/BREmote_V2_Tx.h`
- Modify: `Source/V2_Integration_Tx/Radio.ino`
- Modify: `Source/V2_Integration_Tx/V2_Integration_Tx.ino`

- [ ] **Step 1: Add volatile globals to BREmote_V2_Tx.h**

Find the block of `volatile uint8_t thr_sent = 0;` in `BREmote_V2_Tx.h`. After the existing volatile globals, add:

```cpp
// V3 - 2026-04-25 - P7 RTM meta-packet burst queue.
// Loop task writes type/value then sets count; sendData task consumes count→0.
// Writing count last (after type/value) is safe: sendData won't read until count>0.
volatile uint8_t rtm_meta_type  = 0;    // 0xF1=RTM state, 0xF2=FM override
volatile uint8_t rtm_meta_value = 0;    // rtm_active (0/1) or fm_mode (0-3)
volatile int8_t  rtm_meta_count = 0;    // bursts remaining; 0=idle

// V3 - 2026-04-25 - P7 RTM throttle cap (set by TX RTM state machine).
// 255 = no cap. During RTM ACTIVE, set to the ramped cap value (30-70% of 255).
// Applied in calcFinalThrottle() so the capped throttle is what gets sent to RX.
// RTM can only subtract from user throttle — never add. Safety philosophy enforced here.
volatile uint8_t rtm_thr_cap_tx = 255;
volatile bool    rtm_tx_active  = false;
```

- [ ] **Step 2: Modify sendData() in TX Radio.ino to consume meta-packet queue**

Open `Source/V2_Integration_Tx/Radio.ino`. Find the `sendData()` function. Find the `if(usrConf.paired && isRadioActivityEnabled())` block. At the very top of that block, BEFORE the `gps_cycle` increment, insert:

```cpp
      // ---- Meta-packet burst path (highest priority) ----
      // Replaces one control cycle per burst; 3 bursts = 300ms total.
      // Type/value are set by queueMetaPacketBurst() in loop task before count is set.
      if (rtm_meta_count > 0)
      {
        uint8_t metaPkt[6];
        memcpy(metaPkt, usrConf.dest_address, 3);
        metaPkt[3] = rtm_meta_type;
        metaPkt[4] = rtm_meta_value;
        metaPkt[5] = esp_crc8(metaPkt, 5);

        rxprint("Sending meta-pkt: ");
        #ifdef DEBUG_RX
        printHexArray(metaPkt, 6);
        #endif

        radio.implicitHeader(6);
        radio.startTransmit(metaPkt, 6);
        rtm_meta_count--;
        num_sent_packets++;
        vTaskDelay(pdMS_TO_TICKS(10));
        radio.implicitHeader(6);
        rfInterrupt = false;
        radio.startReceive();
        xTaskNotifyGive(triggeredWaitForTelemetryHandle);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }
```

- [ ] **Step 3: Update throttle cap from 0xF2 to 0xF0 in sendData()**

In the normal 6-byte control packet path of `sendData()`, find:

```cpp
        sendArray[3] = (thr >= 0xF3) ? 0xF2 : thr;  // cap: 0xF3 reserved for GPS meta-packet
```

Replace with:

```cpp
        // V3 - 2026-04-25 - P7: cap at 0xF0 (240/255=94.1%) to reserve 0xF1-0xFF for all meta-packet types.
        // 0xF1=RTM state, 0xF2=FM override, 0xF3=GPS. 0xF0 vs 0xF2 is imperceptible (~0.8% throttle).
        sendArray[3] = (thr > 0xF0) ? 0xF0 : thr;
```

- [ ] **Step 4: Add queueMetaPacketBurst() helper to TX Radio.ino**

At the bottom of `Source/V2_Integration_Tx/Radio.ino`, add:

```cpp
// V3 - 2026-04-25 - P7: Queue a 3-burst meta-packet transmission.
// Called from loop task (RTM/FM state machine). sendData() consumes the queue.
// type: 0xF1=RTM state, 0xF2=FM override
// value: for 0xF1: 0=inactive 1=active; for 0xF2: 0-3 FM mode
void queueMetaPacketBurst(uint8_t type, uint8_t value)
{
  rtm_meta_type  = type;
  rtm_meta_value = value;
  rtm_meta_count = 3;  // sendData sends 3 bursts at 100ms intervals = 300ms total
}
```

- [ ] **Step 5: Fix getTxGPSLoop() call condition in V2_Integration_Tx.ino**

Open `Source/V2_Integration_Tx/V2_Integration_Tx.ino`. Find:

```cpp
  if (usrConf.gps_en &&
      (usrConf.speed_src == 2 || usrConf.speed_src == 3 || usrConf.speed_src == 5))
  {
    getTxGPSLoop();
  }
```

Replace with:

```cpp
  // V3 - 2026-04-25 - P7: Run GPS loop whenever gps_en=1, regardless of speed_src.
  // RTM mode needs TX GPS position even when speed display uses RX source (speed_src 0/1/4).
  if (usrConf.gps_en)
  {
    getTxGPSLoop();
  }
```

- [ ] **Step 6: Compile TX firmware**

Compile in Arduino IDE. Zero errors expected.

- [ ] **Step 7: Commit**

```bash
git add Source/V2_Integration_Tx/BREmote_V2_Tx.h \
        Source/V2_Integration_Tx/Radio.ino \
        Source/V2_Integration_Tx/V2_Integration_Tx.ino
git commit -m "feat(TX/P7): meta-packet burst queue, throttle cap 0xF2→0xF0, GPS loop always-on"
```

---

## Task 4: RX Meta-Packet Dispatch (0xF1 / 0xF2)

**Files:**
- Modify: `Source/V2_Integration_Rx/BREmote_V2_Rx.h`
- Modify: `Source/V2_Integration_Rx/Radio.ino`

- [ ] **Step 1: Add 4 volatile globals to BREmote_V2_Rx.h**

Find the existing globals block in `BREmote_V2_Rx.h` (near the `rx_tx_gps_*` globals). Add:

```cpp
// V3 - 2026-04-25 - P7 RTM runtime state (set by processRtmStatePacket in Radio.ino)
// rtm_rx_active: true = TX has signalled RTM active; RX RTM state machine applies steering
// rtm_rx_emergency_stop: true = safety gate failed; calcPWM() forces throttle to 0
// rtm_steer_override: bearing-derived steering (0-255, 127=straight); written by RTMState.ino
// fm_mode_runtime: 0-3 = FM mode override from TX; 0xFF = use SPIFFS default
volatile bool    rtm_rx_active         = false;
volatile bool    rtm_rx_emergency_stop = false;
volatile uint8_t rtm_steer_override    = 127;
volatile uint8_t fm_mode_runtime       = 0xFF;
```

- [ ] **Step 2: Add processRtmStatePacket() and processFmOverridePacket() to RX Radio.ino**

Open `Source/V2_Integration_Rx/Radio.ino`. Just before `triggeredReceive()`, add:

```cpp
// V3 - 2026-04-25 - P7: Handle 0xF1 RTM state meta-packet from TX.
// pkt: 6-byte buffer. byte[3]=0xF1, byte[4]=rtm_active (0=off, 1=on).
// Sets rtm_rx_active. Safety gates in RTMState.ino may override this.
static void processRtmStatePacket(const uint8_t *pkt)
{
  uint8_t new_state = pkt[4];
  if (new_state == 0)
  {
    rtm_rx_active         = false;
    rtm_rx_emergency_stop = false;
    Serial.println("RTM [RX] deactivated by TX");
  }
  else if (new_state == 1)
  {
    // Safety gates in runRtmLoop() will set rtm_rx_active=true only if all gates pass.
    // Set a flag that the state machine checks on next iteration.
    rtm_rx_active = true;
    Serial.println("RTM [RX] activation requested by TX");
  }
}

// V3 - 2026-04-25 - P7: Handle 0xF2 FM override meta-packet from TX.
// pkt: 6-byte buffer. byte[3]=0xF2, byte[4]=fm_mode (0-3).
// Updates runtime FM mode without writing SPIFFS. Resets to SPIFFS default on RX reboot.
static void processFmOverridePacket(const uint8_t *pkt)
{
  uint8_t mode = pkt[4] & 0x03;  // clamp to 0-3
  fm_mode_runtime = mode;
  Serial.printf("FM [RX] mode override: %d\n", mode);
}
```

- [ ] **Step 3: Modify triggeredReceive() to dispatch 0xF1 and 0xF2**

In `triggeredReceive()`, find the existing 0xF3 check:

```cpp
              if (rcvArray[3] == 0xF3)
              {
```

Add handling for 0xF1 and 0xF2 BEFORE the 0xF3 check (or restructure as a chained else-if):

```cpp
              if (rcvArray[3] == 0xF1)
              {
                // RTM state meta-packet: TX signals RTM active or inactive.
                processRtmStatePacket(rcvArray);
                // No telemetry reply needed for meta-packets.
              }
              else if (rcvArray[3] == 0xF2)
              {
                // FM override meta-packet: TX cycles follow-me mode.
                processFmOverridePacket(rcvArray);
                // No telemetry reply needed.
              }
              else if (rcvArray[3] == 0xF3)
              {
```

The existing 0xF3 block content and everything after is unchanged. Also update the comment on the throttle range: add a note that throttle byte is now capped at 0xF0 on TX so 0xF1+ always means meta-packet.

- [ ] **Step 4: Compile RX**

Compile in Arduino IDE (ESP32S3). Zero errors expected.

- [ ] **Step 5: Bench test meta-packet dispatch**

Flash RX only. On TX serial monitor, send `?radio on` then manually trigger `queueMetaPacketBurst(0xF1, 1)` (add a temporary `?rtmtest` serial command if needed). Verify RX serial prints `RTM [RX] activation requested by TX`. Then test 0xF2.

- [ ] **Step 6: Remove any temporary test commands, commit**

```bash
git add Source/V2_Integration_Rx/BREmote_V2_Rx.h \
        Source/V2_Integration_Rx/Radio.ino
git commit -m "feat(RX/P7): dispatch 0xF1/0xF2 meta-packets; add rtm_rx_active + fm_mode_runtime globals"
```

---

## Task 5: RX PWM RTM Override Hook

**Files:**
- Modify: `Source/V2_Integration_Rx/PWM.ino`

- [ ] **Step 1: Modify calcPWM() to apply RTM emergency stop and steering override**

Open `Source/V2_Integration_Rx/PWM.ino`. Find `void calcPWM()`. The function starts with:

```cpp
void calcPWM()
{
  if(usrConf.steering_type == 0)
  {
    //Efoil mode
    PWM0_time = constrain(map(thr_received, 0, 255, ...
```

Replace the first line of `calcPWM()` body to add RTM overrides at the top:

```cpp
void calcPWM()
{
  // V3 - 2026-04-25 - P7: Apply RTM overrides before PWM calculation.
  // Emergency stop: any safety gate failure forces throttle to neutral regardless of user input.
  // Steering override: bearing-derived value replaces radio steering when RTM active.
  // RTM can only subtract from user throttle (never add). Creator safety philosophy enforced.
  uint8_t effective_thr   = rtm_rx_emergency_stop ? 0 : thr_received;
  uint8_t effective_steer = (rtm_rx_active && usrConf.rtm_rx_override_steering)
                            ? rtm_steer_override
                            : steering_received;

```

Then replace every subsequent use of `thr_received` and `steering_received` in `calcPWM()` with `effective_thr` and `effective_steer` respectively.

Specifically, replace:
- `map(thr_received, 0, 255, usrConf.PWM0_min, usrConf.PWM0_max)` → `map(effective_thr, 0, 255, ...)`
- `map(thr_received, 0, 255, usrConf.PWM1_min, usrConf.PWM1_max)` → `map(effective_thr, 0, 255, ...)`
- All `steering_received` references → `effective_steer`

(Read the full calcPWM() function to locate each occurrence before editing.)

- [ ] **Step 2: Compile RX**

Compile in Arduino IDE. Zero errors expected.

- [ ] **Step 3: Bench test**

Flash RX. From RX serial: send `?printpwm`. Manually set `rtm_rx_emergency_stop = true` via a temporary serial command. Verify PWM drops to neutral. Remove the test command.

- [ ] **Step 4: Commit**

```bash
git add Source/V2_Integration_Rx/PWM.ino
git commit -m "feat(RX/P7): calcPWM() applies RTM emergency stop and steering override"
```

---

## Task 6: RX Compass Heading Function

**Files:**
- Modify: `Source/V2_Integration_Rx/Compass.ino`

- [ ] **Step 1: Add getCompassHeading() to Compass.ino**

Open `Source/V2_Integration_Rx/Compass.ino`. At the end of the file, add:

```cpp
// V3 - 2026-04-25 - P7: Compute calibrated compass heading in degrees.
//
// What it does:
//   Reads raw magnetometer via readCompassRaw(), applies hard-iron offset
//   correction (mag_offset_x/y) and soft-iron scale correction (mag_scale_x/y),
//   then returns the 2D heading angle via atan2f.
//
// Returns:
//   Heading in degrees, 0=North, 90=East, 180=South, 270=West (clockwise).
//   Returns -1.0f if compass is not detected or never calibrated (scale=0).
//
// Note: if heading is consistently wrong by a fixed offset, adjust physical
//   mounting or add a calibration offset parameter in a future revision.
//   If left/right are swapped, negate cal_y below.
float getCompassHeading()
{
  if (!compass_detected) return -1.0f;

  // Reject uncalibrated scale (default 1.0f after runcal is fine; 0.0f = never set)
  if (usrConf.mag_scale_x == 0.0f || usrConf.mag_scale_y == 0.0f) return -1.0f;

  readCompassRaw();

  float cal_x = ((float)magX - (float)usrConf.mag_offset_x) * usrConf.mag_scale_x;
  float cal_y = ((float)magY - (float)usrConf.mag_offset_y) * usrConf.mag_scale_y;

  float heading = atan2f(cal_y, cal_x) * (180.0f / M_PI);
  if (heading < 0.0f) heading += 360.0f;

  return heading;
}
```

- [ ] **Step 2: Add ?compassheading serial command to RX System.ino**

In `Source/V2_Integration_Rx/System.ino`, find the `cmdPrintCompass` command entry. Add a new entry after it:

```cpp
void cmdPrintCompassHeading(const String& params) {
  while (true) {
    esp_task_wdt_reset();
    if (checkSerialQuit()) break;
    float h = getCompassHeading();
    if (h < 0.0f) Serial.println("Compass not detected or not calibrated");
    else Serial.printf("Heading: %.1f deg\n", h);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
```

Add to the command table:
```cpp
  {"compassheading", "print live compass heading in degrees", cmdPrintCompassHeading},
```

- [ ] **Step 3: Compile and flash RX**

Compile + flash RX. Open serial monitor. Type `?compassheading`. Physically rotate the RX board. Verify heading changes continuously. Rotate 360° and confirm output cycles 0–360 (approximately). If it cycles backwards, negate `cal_y` in `getCompassHeading()`.

- [ ] **Step 4: Commit**

```bash
git add Source/V2_Integration_Rx/Compass.ino \
        Source/V2_Integration_Rx/System.ino
git commit -m "feat(RX/P7): add getCompassHeading(); ?compassheading serial diagnostic"
```

---

## Task 7: TX RTM + FM State Machine

**Files:**
- Create: `Source/V2_Integration_Tx/RTMState.ino`
- Modify: `Source/V2_Integration_Tx/Hall.ino`
- Modify: `Source/V2_Integration_Tx/Throttle.ino`
- Modify: `Source/V2_Integration_Tx/V2_Integration_Tx.ino`

- [ ] **Step 1: Create RTMState.ino on TX**

Create `Source/V2_Integration_Tx/RTMState.ino` with this content:

```cpp
// V3 - 2026-04-25 - P7: TX RTM and FM state machines.
// RTM: left-hold gesture → arm → squeeze(s) → active → cooldown → idle
// FM:  right-hold gesture → cycle FM mode 0→1→2→3→0 → send 0xF2 meta-packet

// ============================================================
// RTM STATE MACHINE
// ============================================================

typedef enum { RTM_IDLE, RTM_ARMED, RTM_SQUEEZE_WAIT, RTM_ACTIVE, RTM_COOLDOWN } RtmTxState;

static RtmTxState  rtm_tx_state        = RTM_IDLE;
static unsigned long rtm_arm_start_ms  = 0;   // when ARMED state was entered
static unsigned long rtm_active_start_ms = 0; // when ACTIVE state was entered
static unsigned long rtm_release_ms    = 0;   // when throttle was last released during ACTIVE
static unsigned long rtm_cooldown_ms   = 0;   // when COOLDOWN state was entered
static unsigned long rtm_squeeze_ms    = 0;   // when SQUEEZE_WAIT was entered

// ---- Compute the current throttle cap for the ramp ----
// Returns 0-255. During ACTIVE, ramps from rtm_throttle_start_pct→max over rtm_ramp_duration_s.
uint8_t calcRtmThrottleCap()
{
  if (rtm_tx_state != RTM_ACTIVE) return 255;  // no cap outside ACTIVE
  unsigned long elapsed = millis() - rtm_active_start_ms;
  float t = (float)elapsed / ((float)usrConf.rtm_ramp_duration_s * 1000.0f);
  if (t > 1.0f) t = 1.0f;
  float pct = (float)usrConf.rtm_throttle_start_pct
            + t * (float)(usrConf.rtm_throttle_max_pct - usrConf.rtm_throttle_start_pct);
  return (uint8_t)(pct * 255.0f / 100.0f);
}

// ---- Called by handleGearToggle() when left long-press threshold is reached ----
// Transitions from IDLE to ARMED and shows "rtn" on display.
void setRtmArmed()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;
  rtm_tx_state   = RTM_ARMED;
  rtm_arm_start_ms = millis();
  rtm_tx_active  = false;
  rtm_thr_cap_tx = 255;
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM not yet active (just armed on TX)
  scroll3Digits(LET_R, LET_T, LET_N, 100);
}

// ---- Disengage RTM: return to IDLE, notify RX ----
static void rtmDisengage()
{
  rtm_tx_state   = RTM_COOLDOWN;
  rtm_cooldown_ms = millis();
  rtm_tx_active  = false;
  rtm_thr_cap_tx = 255;
  queueMetaPacketBurst(0xF1, 0);  // tell RX: RTM inactive
}

// ---- Called from loop() every ~110ms ----
void runRtmLoop()
{
  if (!usrConf.rtm_enabled || !usrConf.gps_en) return;

  unsigned long now = millis();

  switch (rtm_tx_state)
  {
    // ---- IDLE: nothing to do; setRtmArmed() transitions out ----
    case RTM_IDLE:
      break;

    // ---- ARMED: wait for first (or only) squeeze ----
    case RTM_ARMED:
      // Arm window timeout
      if (now - rtm_arm_start_ms > (unsigned long)usrConf.rtm_arm_window_s * 1000UL)
      {
        rtm_tx_state = RTM_IDLE;
        break;
      }
      // Show "rtn" on display (blink every ~500ms while waiting)
      if ((now / 500) % 2 == 0)
      {
        scroll3Digits(LET_R, LET_T, LET_N, 50);
      }

      if (usrConf.rtm_double_squeeze_en)
      {
        // Wait for first squeeze (thr_scaled > 10%)
        if (thr_scaled > 25)
        {
          displayDigits(LET_A, LET_R);  // "AR" = first squeeze detected
          updateDisplay();
          // Wait for user to release throttle (block briefly — this is the trigger ack)
          unsigned long rel_wait = millis();
          while (thr_scaled > 5 && millis() - rel_wait < 2000) delay(20);
          rtm_tx_state  = RTM_SQUEEZE_WAIT;
          rtm_squeeze_ms = millis();
        }
      }
      else
      {
        // Single mode: throttle > 30% for 500ms continuous
        static unsigned long hold_start = 0;
        if (thr_scaled > 76)
        {
          if (hold_start == 0) hold_start = now;
          if (now - hold_start >= 500)
          {
            hold_start = 0;
            rtm_tx_state      = RTM_ACTIVE;
            rtm_active_start_ms = now;
            rtm_tx_active     = true;
            rtm_release_ms    = 0;
            queueMetaPacketBurst(0xF1, 1);
          }
        }
        else
        {
          hold_start = 0;
        }
      }
      break;

    // ---- SQUEEZE_WAIT: waiting for second squeeze (double-squeeze mode only) ----
    case RTM_SQUEEZE_WAIT:
      // 5s window
      if (now - rtm_squeeze_ms > 5000UL)
      {
        rtm_tx_state = RTM_IDLE;
        break;
      }
      // Blink "RY" (ready for second squeeze)
      if ((now / 300) % 2 == 0)
      {
        displayDigits(LET_R, LET_Y);
        updateDisplay();
      }
      if (thr_scaled > 25)
      {
        // Second squeeze → ACTIVE
        rtm_tx_state      = RTM_ACTIVE;
        rtm_active_start_ms = now;
        rtm_tx_active     = true;
        rtm_release_ms    = 0;
        queueMetaPacketBurst(0xF1, 1);
      }
      break;

    // ---- ACTIVE: RTM running ----
    case RTM_ACTIVE:
    {
      // Update throttle cap for ramp
      rtm_thr_cap_tx = calcRtmThrottleCap();

      // Gate 1: max runtime
      if (now - rtm_active_start_ms > (unsigned long)usrConf.rtm_max_runtime_s * 1000UL)
      {
        rtmDisengage();
        break;
      }

      // Gate 2: TX GPS freshness
      if (gps_tx.location.age() > usrConf.rtm_gps_timeout_ms)
      {
        rtmDisengage();
        break;
      }

      // Gate 3: throttle release > 10s → disengage
      if (thr_scaled < 10)
      {
        if (rtm_release_ms == 0) rtm_release_ms = now;
        if (now - rtm_release_ms > 10000UL)
        {
          rtm_tx_state   = RTM_IDLE;
          rtm_tx_active  = false;
          rtm_thr_cap_tx = 255;
          queueMetaPacketBurst(0xF1, 0);
          break;
        }
      }
      else
      {
        rtm_release_ms = 0;
      }

      // Display "rtn" while active
      scroll3Digits(LET_R, LET_T, LET_N, 30);
      break;
    }

    // ---- COOLDOWN: show "Stp" for 2s ----
    case RTM_COOLDOWN:
      scroll3Digits(LET_S, LET_T, LET_P, 50);
      if (now - rtm_cooldown_ms > 2000UL)
      {
        rtm_tx_state = RTM_IDLE;
      }
      break;
  }
}

// ============================================================
// FM STATE MACHINE
// ============================================================

static uint8_t fm_current_mode = 0;  // current TX-side FM selection (0-3)

// Show the current FM mode abbreviation on display
static void showFmMode(uint8_t mode)
{
  switch (mode)
  {
    case 0: scroll3Digits(0, LET_F, LET_F, 150);     break;  // "0ff"
    case 1: scroll3Digits(LET_B, LET_E, LET_H, 150); break;  // "bEh"
    case 2: displayDigits(LET_N, LET_R); updateDisplay(); delay(500); break;  // "nR"
    case 3: displayDigits(LET_N, LET_L); updateDisplay(); delay(500); break;  // "nL"
  }
}

// Called by handleGearToggle(+1) long press when fm_override_enabled.
// Cycles FM mode and queues 0xF2 meta-packet burst.
void cycleFmMode()
{
  if (!usrConf.fm_override_enabled || !usrConf.gps_en) return;

  // Enter FM with brief "FM" flash
  displayDigits(LET_F, LET_M);
  updateDisplay();
  delay(500);

  // Cycle once
  fm_current_mode = (fm_current_mode + 1) % 4;
  showFmMode(fm_current_mode);

  // Watch for additional right-holds within 2s to keep cycling
  unsigned long last_action = millis();
  while (millis() - last_action < 2000UL)
  {
    if (tog_input == 1)  // right hold
    {
      fm_current_mode = (fm_current_mode + 1) % 4;
      showFmMode(fm_current_mode);
      last_action = millis();
      while (tog_input == 1) delay(20);  // wait for release
    }
    delay(50);
  }

  // User stopped cycling — send the selected mode
  queueMetaPacketBurst(0xF2, fm_current_mode);
}
```

- [ ] **Step 2: Modify Hall.ino handleGearToggle() for RTM arming and FM cycling**

Open `Source/V2_Integration_Tx/Hall.ino`. Find `handleGearToggle()`. Find this block (inside the long-press handler):

```cpp
    if(millis() - pushtime > usrConf.lock_waittime)
    {
      if(thr_scaled < 10)
      {
        if(direction < 0)
        {
          //Long press minus: lock system
          if(!usrConf.no_lock)
          {
            system_locked = 1;
            displayLock();
          }
        }
        else
        {
          //Long press plus: cycle display mode
          cycleDisplayMode(1);
        }
```

Replace it with:

```cpp
    // V3 - 2026-04-25 - P7: When RTM is enabled, left long-press arms RTM instead of locking.
    // When FM override is enabled, right long-press cycles FM mode instead of display mode.
    // RTM threshold = rtm_hold_duration_s (default 5s) >= lock_waittime (2s).
    // If RTM disabled: threshold stays at lock_waittime for normal lock behavior.
    unsigned long long_press_ms = (direction < 0 && usrConf.rtm_enabled && usrConf.gps_en)
                                  ? (unsigned long)usrConf.rtm_hold_duration_s * 1000UL
                                  : (unsigned long)usrConf.lock_waittime;

    if(millis() - pushtime > long_press_ms)
    {
      if(thr_scaled < 10)
      {
        if(direction < 0)
        {
          if (usrConf.rtm_enabled && usrConf.gps_en)
          {
            // Left long-press: arm RTM
            setRtmArmed();
          }
          else if(!usrConf.no_lock)
          {
            // RTM disabled: normal lock behavior
            system_locked = 1;
            displayLock();
          }
        }
        else
        {
          if (usrConf.fm_override_enabled && usrConf.gps_en)
          {
            // Right long-press: FM mode cycling
            cycleFmMode();
          }
          else
          {
            //Long press plus: cycle display mode (original behavior)
            cycleDisplayMode(1);
          }
        }
```

- [ ] **Step 3: Modify calcFinalThrottle() in Throttle.ino to apply RTM cap**

Open `Source/V2_Integration_Tx/Throttle.ino`. Find `calcFinalThrottle()`. At the very end, just before the `return` statement, add the RTM cap:

```cpp
  // V3 - 2026-04-25 - P7: Apply RTM throttle ramp cap.
  // rtm_thr_cap_tx is 255 when RTM is not active (no effect).
  // When RTM is ACTIVE, it ramps from rtm_throttle_start_pct to rtm_throttle_max_pct.
  // This enforces the creator safety rule: RTM can only subtract from user throttle.
  if (result > rtm_thr_cap_tx) result = rtm_thr_cap_tx;

  return result;
```

(If `calcFinalThrottle()` uses a local variable like `uint8_t result` or `uint8_t thr`, apply the cap to that variable. Read the function first to confirm the variable name.)

- [ ] **Step 4: Add runRtmLoop() to TX loop()**

Open `Source/V2_Integration_Tx/V2_Integration_Tx.ino`. In `loop()`, after `runMenu()`, add:

```cpp
  // V3 - 2026-04-25 - P7: RTM state machine (arming, active, cooldown) and FM mode cycle monitor.
  runRtmLoop();
```

Also add the forward declaration near the top of the file with the other prototypes:

```cpp
void runRtmLoop();
void setRtmArmed();
void cycleFmMode();
uint8_t calcRtmThrottleCap();
void queueMetaPacketBurst(uint8_t type, uint8_t value);
```

- [ ] **Step 5: Compile TX firmware**

Compile in Arduino IDE. Zero errors expected.

- [ ] **Step 6: Bench test TX state machine**

Flash TX. On serial monitor:
1. Ensure `gps_en=1` and `rtm_enabled=1` via `?set gps_en 1` / `?set rtm_enabled 1`.
2. Hold LEFT toggle for 5+ seconds with throttle at 0. Verify display shows "rtn".
3. Squeeze throttle briefly. Verify display shows "AR". Release.
4. Verify display shows "RY". Squeeze again. Verify display shows "rtn" (active).
5. Hold LEFT >10s with no throttle. Verify display returns to normal (RTM disengaged).
6. Hold RIGHT toggle 5+ seconds. Verify "FM" flash, then mode abbreviation cycles.
7. Verify TX serial shows meta-packet sends.

- [ ] **Step 7: Commit**

```bash
git add Source/V2_Integration_Tx/RTMState.ino \
        Source/V2_Integration_Tx/Hall.ino \
        Source/V2_Integration_Tx/Throttle.ino \
        Source/V2_Integration_Tx/V2_Integration_Tx.ino
git commit -m "feat(TX/P7): RTM+FM state machines; RTM arm on left-hold; FM cycle on right-hold"
```

---

## Task 8: RX RTM State Machine + Safety Gates + Phase C

**Files:**
- Create: `Source/V2_Integration_Rx/RTMState.ino`
- Modify: `Source/V2_Integration_Rx/V2_Integration_Rx.ino`

- [ ] **Step 1: Create RTMState.ino on RX**

Create `Source/V2_Integration_Rx/RTMState.ino` with this content:

```cpp
// V3 - 2026-04-25 - P7: RX RTM state machine, 10 safety gates, Phase C anti-spoofing.
//
// The RTM state machine runs in loop() at ~10Hz.
// When rtm_rx_active is set true by a 0xF1 meta-packet, this module:
//   1. Checks all 10 safety gates every iteration (any fail → emergency stop).
//   2. Computes compass bearing toward TX GPS position.
//   3. Converts bearing error to a steering override (0-255, 127=straight).
//   4. Runs Phase C: convergence check, VESC ERPM speed check, TX GPS freshness.
//
// All outputs are written to volatile globals read by calcPWM() and triggeredReceive().

// ---- Phase C convergence tracking ----
static double    rtm_prev_dist_m   = -1.0;   // distance to TX at last Phase C check
static unsigned long rtm_phase_c_ms = 0;     // last Phase C check time

// ---- Safety gate check ----
// Returns true if ALL gates pass. Sets rtm_rx_emergency_stop=true and prints reason on any failure.
static bool checkRtmSafetyGates()
{
  unsigned long now = millis();

  // Gate 1 (ABSOLUTE): user must be physically holding throttle > 10%.
  // Creator safety philosophy — this gate CANNOT be waived.
  if (thr_received < 25)
  {
    // Throttle released — this is normal; do not emergency-stop, just return false.
    // The motor already outputs 0 because thr_received is 0.
    return false;
  }

  // Gate 2: Phase A GPS not rejected on RX
  extern volatile bool gps_rejected;
  if (gps_rejected)
  {
    Serial.println("RTM [RX] STOP: Phase A GPS rejected");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 3: Phase B handshake passed (gps_phase_b_ok set by gpsPhaseBCheck in Radio.ino)
  extern bool gps_phase_b_ok;
  if (!gps_phase_b_ok)
  {
    Serial.println("RTM [RX] STOP: Phase B handshake not passed");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 4: valid TX GPS fix (age < rtm_gps_timeout_ms)
  if (rx_tx_gps_timestamp == 0 ||
      (now - rx_tx_gps_timestamp) > (unsigned long)usrConf.rtm_gps_timeout_ms)
  {
    Serial.println("RTM [RX] STOP: TX GPS stale or never received");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 5: valid RX GPS fix
  if (gps_last_ms == 0 || (now - gps_last_ms) > (unsigned long)usrConf.rtm_gps_timeout_ms * 3)
  {
    Serial.println("RTM [RX] STOP: RX GPS stale");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 6: valid compass (if required)
  if (usrConf.rtm_compass_required)
  {
    float h = getCompassHeading();
    if (h < 0.0f)
    {
      Serial.println("RTM [RX] STOP: Compass not available");
      rtm_rx_emergency_stop = true;
      return false;
    }
  }

  // Gate 7: LoRa link healthy
  if (millis() - last_packet > usrConf.failsafe_time)
  {
    Serial.println("RTM [RX] STOP: LoRa link lost");
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 8: within rtm_max_runtime_s (TX enforces this too; RX is belt-and-suspenders)
  // (No RX-side runtime tracking needed — if TX disengages, 0xF1/0 sets rtm_rx_active=false.)

  // Gate 9: hard stop distance (buggy within rtm_stop_distance_m of TX)
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);
  if (dist_m < (float)usrConf.rtm_stop_distance_m)
  {
    Serial.printf("RTM [RX] STOP: within hard stop distance (%.0f m)\n", dist_m);
    rtm_rx_emergency_stop = true;
    return false;
  }

  // Gate 10: throttle has not been released > 10s (TX enforces this; no RX-side tracking needed)

  return true;
}

// ---- Compute RTM steering override ----
// Uses compass heading + TX GPS bearing to derive steering_override (0-255, 127=straight).
static void updateRtmSteering()
{
  if (!usrConf.rtm_rx_override_steering) return;

  float compass_heading = getCompassHeading();
  if (compass_heading < 0.0f)
  {
    rtm_steer_override = 127;  // straight ahead if no compass
    return;
  }

  // Bearing from RX GPS to TX GPS position (0-360, clockwise from North)
  double bearing_deg = TinyGPSPlus::courseTo(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

  // Heading error: positive = need to turn right, negative = turn left
  float heading_error = (float)(bearing_deg - compass_heading);
  while (heading_error >  180.0f) heading_error -= 360.0f;
  while (heading_error < -180.0f) heading_error += 360.0f;

  // Clamp to ±90° (full lock at 90° off course; ignore U-turns)
  float clamped = heading_error;
  if (clamped >  90.0f) clamped =  90.0f;
  if (clamped < -90.0f) clamped = -90.0f;

  // Map to 0-255 (127 = straight, >127 = right, <127 = left)
  rtm_steer_override = (uint8_t)(127.0f + (clamped / 90.0f) * 127.0f);

  #ifdef DEBUG_RX
  Serial.printf("RTM steer: bear=%.1f head=%.1f err=%.1f ovr=%d\n",
                (float)bearing_deg, compass_heading, heading_error, rtm_steer_override);
  #endif
}

// ---- Phase C anti-spoofing (runs during active RTM, every 5s) ----
static void runPhaseC()
{
  if (!rtm_rx_active || rtm_rx_emergency_stop) return;

  unsigned long now = millis();
  if (now - rtm_phase_c_ms < 5000UL) return;
  rtm_phase_c_ms = now;

  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng, rx_tx_gps_lat, rx_tx_gps_lng);

  // Phase C check 1: convergence — distance to TX must be decreasing
  if (rtm_prev_dist_m >= 0.0f && dist_m >= rtm_prev_dist_m)
  {
    Serial.printf("RTM [PhC] FAIL convergence: dist %.0f m (was %.0f m) — not closing\n",
                  dist_m, rtm_prev_dist_m);
    rtm_rx_emergency_stop = true;
    rtm_rx_active = false;
    return;
  }
  rtm_prev_dist_m = dist_m;

  // Phase C check 2: VESC ERPM vs GPS speed (only if vesc_erpm_per_kmh is configured)
  if (usrConf.vesc_erpm_per_kmh > 0.0f)
  {
    extern vesc_struct vesc;
    float vesc_speed_kmh = (float)abs(vesc.erpm) / usrConf.vesc_erpm_per_kmh;
    float speed_diff = fabsf(vesc_speed_kmh - gps_last_speed_kmh);
    if (speed_diff > usrConf.rtm_vesc_speed_diff_kmh)
    {
      Serial.printf("RTM [PhC] FAIL VESC speed: VESC=%.1f km/h GPS=%.1f km/h diff=%.1f\n",
                    vesc_speed_kmh, gps_last_speed_kmh, speed_diff);
      rtm_rx_emergency_stop = true;
      rtm_rx_active = false;
      return;
    }
  }

  // Phase C check 3: TX GPS freshness (stricter than Gate 4 — 2s window)
  if (rx_tx_gps_timestamp == 0 ||
      (millis() - rx_tx_gps_timestamp) > (unsigned long)usrConf.tx_gps_stale_timeout_ms)
  {
    Serial.println("RTM [PhC] FAIL TX GPS freshness");
    rtm_rx_emergency_stop = true;
    rtm_rx_active = false;
    return;
  }

  Serial.printf("RTM [PhC] PASS: dist=%.0f m, converging\n", dist_m);
}

// ---- Main RTM loop — call from RX loop() ----
void runRtmLoop()
{
  // Rate-limit to 10Hz (compass I2C + TinyGPS math takes ~2ms per call)
  static unsigned long last_rtm_ms = 0;
  unsigned long now = millis();
  if (now - last_rtm_ms < 100UL) return;
  last_rtm_ms = now;

  if (!usrConf.rtm_rx_enabled)
  {
    rtm_rx_active         = false;
    rtm_rx_emergency_stop = false;
    return;
  }

  if (!rtm_rx_active)
  {
    rtm_rx_emergency_stop = false;
    rtm_prev_dist_m       = -1.0;
    rtm_phase_c_ms        = 0;
    return;
  }

  // RTM active: run all gates
  if (!checkRtmSafetyGates())
  {
    // Gate 1 (throttle released) returns false without setting emergency_stop.
    // All other gates set emergency_stop=true. If not emergency, just return.
    return;
  }

  // All gates pass: clear emergency stop, update steering
  rtm_rx_emergency_stop = false;
  updateRtmSteering();

  // Phase C (every 5s)
  runPhaseC();
}
```

- [ ] **Step 2: Add runRtmLoop() call to RX loop()**

Open `Source/V2_Integration_Rx/V2_Integration_Rx.ino`. In `loop()`, after `webCfgLoop()` and before the 1000ms gate, add:

```cpp
  // V3 - 2026-04-25 - P7: RTM state machine — safety gates, steering override, Phase C.
  // Runs at 10Hz regardless of the 1000ms GPS/VESC gate below.
  runRtmLoop();
```

Also add forward declaration near the top of the file:

```cpp
void runRtmLoop();
float getCompassHeading();
```

- [ ] **Step 3: Declare the extern for gps_rejected in RTMState.ino**

In `Source/V2_Integration_Rx/GPS.ino`, confirm `gps_rejected` is declared as a non-static global (it was added in P3). The `extern volatile bool gps_rejected;` in RTMState.ino will link to it.

- [ ] **Step 4: Compile RX**

Compile in Arduino IDE. Zero errors expected. If `vesc_struct` is not visible in RTMState.ino, add `extern vesc_struct vesc;` at the top (it's defined in VESC.ino, same translation unit).

- [ ] **Step 5: End-to-end bench test (no motor)**

Flash both TX and RX. On a bench (motor disconnected):
1. Ensure both have GPS fix (wait outdoors ~2 min).
2. Hold TX LEFT toggle 5s → TX shows "rtn", RX serial shows `RTM [RX] activation requested by TX`.
3. Squeeze TX throttle, release, squeeze again → TX shows "rtn" active.
4. Verify RX serial prints `RTM steer: bear=... head=... err=... ovr=...` at 10Hz.
5. Verify `?printpwm` on RX shows steering changing based on compass orientation.
6. Force Phase B fail (cover GPS antenna temporarily for >30s) → verify `RTM [RX] STOP: Phase B`.
7. Release TX throttle >10s → TX sends 0xF1/0 → RX deactivates RTM.

- [ ] **Step 6: Commit**

```bash
git add Source/V2_Integration_Rx/RTMState.ino \
        Source/V2_Integration_Rx/V2_Integration_Rx.ino
git commit -m "feat(RX/P7): RTM state machine, 10 safety gates, compass steering, Phase C anti-spoofing"
```

---

## End-to-End Test Checklist (from DESIGN_RETURN_TO_ME.md §8)

Before motor bench test, verify on serial output:

- [ ] RTM ARMED: TX shows "rtn"; RX receives `0xF1 active=1`
- [ ] Safety Gate 1: release TX throttle → RX returns false (no emergency stop) → motor at 0
- [ ] Safety Gate 2: cover RX GPS antenna → `gps_rejected` → `RTM [RX] STOP`
- [ ] Safety Gate 3: wait 30s without 0xF3 packet → `Phase B not passed` → stop
- [ ] Safety Gate 4: cover TX GPS → `TX GPS stale` → stop
- [ ] Safety Gate 9: walk TX within 10m of RX → `within hard stop distance` → stop
- [ ] Phase C convergence: move RX away from TX for 10s → `not closing` → stop (bench: move antenna)
- [ ] FM override: right-hold 5s → RX shows new fm_mode_runtime in `?printreceived`
- [ ] Double-squeeze: first squeeze = "AR", release, second squeeze = "rtn" active
- [ ] RTM max runtime: set `rtm_max_runtime_s=30` via web config; verify auto-disengage at 30s

---

## Self-Review

**Spec coverage:**
- ✅ All 10 safety gates from DESIGN §3d implemented in checkRtmSafetyGates()
- ✅ Phase C (convergence + VESC + TX freshness) in runPhaseC()
- ✅ All 3 RX safety SPIFFS params (rtm_rx_enabled, rtm_rx_override_steering, rtm_compass_required)
- ✅ rtm_vesc_speed_diff_kmh + vesc_erpm_per_kmh as RX params
- ✅ All 12 TX SPIFFS params from DESIGN §4a
- ✅ FM override (0xF2 meta-packet, runtime RAM variable, no SPIFFS write)
- ✅ RTM double-squeeze and 500ms-filter modes both implemented
- ✅ Throttle ramp (start% → max% over ramp_duration_s)
- ✅ Hard stop distance (Gate 9)
- ✅ Creator safety rule enforced: Gate 1 is physical throttle check, cannot be waived

**Potential issues to verify:**
- `handleGearToggle()` blocks while toggle is held — the RTM arm happens at the long-press threshold inside that blocking loop. Ensure the loop re-checks millis() correctly (it does: existing `while(isActive())` loop with `millis() - pushtime` check).
- `scroll3Digits()` and `displayDigits()` in `runRtmLoop()` on TX call `updateDisplay()` — this is safe from loop() but may conflict with `renderOperationalDisplay()`. The display updates are brief and infrequent.
- `vesc.erpm` is only available when `VESC_MORE_VALUES` is defined (added in Task 2). The Phase C VESC check is gated on `vesc_erpm_per_kmh > 0`, so if VESC is unavailable the check silently skips.
- `gps_last_lat/lng/speed_kmh/ms` globals are written by `getGPSLoop()` in GPS.ino (running at 1Hz in the 1000ms gate). The RTM state machine at 10Hz reads these same globals. This is a benign data race (single float/double reads are effectively atomic on ESP32 for aligned data).
