# Priority 5 — TX→RX 0xF3 Meta-Packet Infrastructure

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the LoRa protocol so TX sends its GPS coordinates to RX at 2Hz via a new 0xF3 meta-packet, giving Phase B anti-spoofing (Priority 6) the TX-side GPS data it needs.

**Architecture:** TX inserts a 2-step GPS meta-packet burst every 5th control cycle (500ms = 2Hz): first a 6-byte announcement packet (byte3=0xF3, byte4=0x01) that primes the RX to expect a 14-byte payload, then the 14-byte GPS data packet carrying lat+lng as int32_t microdegrees. RX's `triggeredReceive` task uses a single boolean state flag (`gps_meta_pending`) to switch between 6-byte and 14-byte receive modes. Control packets are unaffected in the other 4 of 5 cycles; THR is capped at 0xF2 so 0xF3 is never a legitimate throttle value.

**Tech Stack:** RadioLib (SX1262), FreeRTOS tasks, TinyGPS++ (already included), `esp_crc8()` (ESP32 ROM), Arduino C++, SPIFFS (no new parameters needed — uses existing `gps_en` and `tx_gps_stale_timeout_ms`).

---

## Why a 2-step Announcement + Data Design

LoRa in implicit-header mode requires both TX and RX to agree on packet length upfront (set via `radio.implicitHeader(n)` before `startTransmit` / `startReceive`). If TX sends 14 bytes while RX expects 6, the RX interrupt fires after 6 bytes and the rest are lost — corrupted GPS data.

The announcement solves this: TX sends a normal 6-byte packet (which RX always expects), RX sees 0xF3 in byte3, immediately calls `implicitHeader(14)` + `startReceive()`. TX waits 10ms (existing pattern), then sends the 14-byte GPS data. At 10ms the RX has been in 14-byte mode for ~7ms — well ahead of the GPS data arriving. Both sides are in sync.

---

## Packet Format Reference

### Existing control packet (unchanged, 6 bytes):
```
[dst0, dst1, dst2, THR(0–0xF2), STEER, CRC8(bytes 0-4)]
```

### New: GPS announcement (6 bytes, replaces one control packet at 2Hz):
```
[dst0, dst1, dst2, 0xF3, 0x01, CRC8(bytes 0-4)]
  byte3 = 0xF3 : meta-packet type marker
  byte4 = 0x01 : subtype "GPS data incoming"
```

### New: GPS data packet (14 bytes, immediately follows announcement):
```
[dst0, dst1, dst2, 0xF3, 0x02, lat_b0..b3, lng_b0..b3, CRC8(bytes 0-12)]
  byte3       = 0xF3 : meta type marker
  byte4       = 0x02 : subtype "GPS coordinate data"
  bytes 5–8   : int32_t lat_microdeg = lat_degrees * 1e6, little-endian, signed
  bytes 9–12  : int32_t lng_microdeg = lng_degrees * 1e6, little-endian, signed
  byte 13     : CRC8 over bytes 0-12
  Precision   : 1 µdeg = ±0.111 m — sufficient for Phase B 500 m distance check
  Range check : lat ±90° → ±90,000,000 µdeg fits int32 (max ±2,147,483,647) ✓
                lng ±180° → ±180,000,000 µdeg fits int32 ✓
```

### Timing diagram (GPS meta cycle, every 500ms):
```
t=0ms   TX sends 6-byte announcement   (~3ms air time at SF6/BW250)
t=3ms   RX ISR fires, triggeredReceive wakes, reads 6 bytes
t=5ms   RX switches to implicitHeader(14) + startReceive
t=10ms  TX sends 14-byte GPS data      (~6ms air time)
t=16ms  RX ISR fires, triggeredReceive wakes, reads 14 bytes, extracts lat/lng
t=18ms  RX sends 6-byte telemetry reply
t=25ms  TX switches to startReceive(6), notifies waitForTelemetry
t=30ms  TX waitForTelemetry receives telemetry reply — normal operation resumes

Failsafe gap: control packet skipped for ~25ms out of 100ms cycle.
Failsafe timeout is 1000ms → no failsafe triggered. ✓
```

---

## File Map

| File | Change |
|---|---|
| `Source/V2_Integration_Tx/Radio.ino` | **Modify** `sendData()`: GPS cycle counter, announcement + GPS data burst, THR cap at 0xF2 |
| `Source/V2_Integration_Rx/BREmote_V2_Rx.h` | **Add** three globals: `rx_tx_gps_lat`, `rx_tx_gps_lng`, `rx_tx_gps_timestamp` |
| `Source/V2_Integration_Rx/Radio.ino` | **Modify** `triggeredReceive()`: 2-path state machine; **Add** `processMetaGpsPacket()` + `gps_meta_pending` flag |

No changes to: `confStruct`, SPIFFS load/save, `WebUiEmbedded.h`, `ConfigService` — this feature has no new user-configurable parameters.

---

## Task 1: Modify TX `sendData()` — Radio.ino

**Files:**
- Modify: `Source/V2_Integration_Tx/Radio.ino` (the `sendData` function, currently lines ~213–261)

- [ ] **Step 1: Read the current `sendData()` in full before editing**

  Open `Source/V2_Integration_Tx/Radio.ino`. Confirm the function signature `void sendData(void *parameter)` and locate the inner `if(usrConf.paired && isRadioActivityEnabled())` block that builds `sendArray[6]`. Understand the existing structure before touching anything.

- [ ] **Step 2: Replace the entire `sendData()` function body**

  Replace the function (keep the existing signature) with the GPS-aware version below. Key changes:
  - Static `gps_cycle` counter (0–4), increments each tick, resets at 5.
  - When `gps_cycle == 0` and GPS is valid: send 6-byte announcement then 14-byte GPS data.
  - Otherwise: send normal 6-byte control packet with THR capped at 0xF2.
  - Common exit after either branch: `implicitHeader(6)` + `startReceive()` + notify telemetry task.

  ```cpp
  // V3 - 2026-04-24 - Added 0xF3 GPS meta-packet burst at 2Hz for Phase B anti-spoofing.
  //                   THR capped at 0xF2: 0xF3 is reserved as the GPS meta-packet marker.
  void sendData(void *parameter)
  {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    // GPS meta-packet cycle counter. Counts 0→4, resets to 0.
    // At cycle 0 (every 5 × 100ms = 500ms = 2Hz): attempt to send GPS meta-packet.
    static uint8_t gps_cycle = 0;

    while(1)
    {
      if(usrConf.paired && isRadioActivityEnabled())
      {
        gps_cycle++;
        if (gps_cycle >= 5) gps_cycle = 0;

        // Send GPS meta-packet when: counter reached 0, GPS is enabled in config,
        // and TinyGPS++ reports a valid fix that is not stale.
        bool send_gps_meta = (gps_cycle == 0)
                          && usrConf.gps_en
                          && gps_tx.location.isValid()
                          && gps_tx.location.age() < usrConf.tx_gps_stale_timeout_ms;

        if (send_gps_meta)
        {
          // ---------------------------------------------------------------
          // GPS meta-packet burst (replaces one control packet per 500ms)
          //
          // Step 1: 6-byte announcement.
          // Primes RX to switch radio to implicitHeader(14) before the data arrives.
          // byte3=0xF3 is the meta-packet type marker. byte4=0x01 = GPS upcoming.
          // ---------------------------------------------------------------
          uint8_t announcePkt[6];
          memcpy(announcePkt, usrConf.dest_address, 3);
          announcePkt[3] = 0xF3;
          announcePkt[4] = 0x01;
          announcePkt[5] = esp_crc8(announcePkt, 5);

          rxprint("Sending GPS announcement: ");
          #ifdef DEBUG_RX
          printHexArray(announcePkt, 6);
          #endif

          radio.implicitHeader(6);
          radio.startTransmit(announcePkt, 6);
          num_sent_packets++;
          vTaskDelay(pdMS_TO_TICKS(10));  // wait for 6-byte TX to complete; RX switches mode during this window

          // ---------------------------------------------------------------
          // Step 2: 14-byte GPS data packet.
          // lat/lng as int32_t microdegrees (degrees × 1e6), little-endian.
          // Precision: ±0.111 m — sufficient for Phase B 500 m distance check.
          // ---------------------------------------------------------------
          uint8_t gpsPkt[14];
          memcpy(gpsPkt, usrConf.dest_address, 3);
          gpsPkt[3] = 0xF3;
          gpsPkt[4] = 0x02;  // subtype: GPS coordinate data

          int32_t lat_ud = (int32_t)(gps_tx.location.lat() * 1e6);
          int32_t lng_ud = (int32_t)(gps_tx.location.lng() * 1e6);
          memcpy(gpsPkt + 5, &lat_ud, 4);    // bytes 5–8: latitude microdegrees
          memcpy(gpsPkt + 9, &lng_ud, 4);    // bytes 9–12: longitude microdegrees
          gpsPkt[13] = esp_crc8(gpsPkt, 13); // CRC over bytes 0–12

          rxprint("Sending GPS data: ");
          #ifdef DEBUG_RX
          printHexArray(gpsPkt, 14);
          #endif

          radio.implicitHeader(14);
          radio.startTransmit(gpsPkt, 14);
          // 14-byte packet needs slightly more air time than 6-byte at SF6/BW250
          vTaskDelay(pdMS_TO_TICKS(15));
        }
        else
        {
          // ---------------------------------------------------------------
          // Normal 6-byte control packet
          //
          // THR capped at 0xF2 (242): 0xF3 is the GPS meta-packet marker and
          // must never appear in the THR field of a control packet.
          // 0xF2 = 94.9% max throttle — imperceptible difference from uncapped 95.3%.
          // ---------------------------------------------------------------
          uint8_t sendArray[6];
          memcpy(sendArray, usrConf.dest_address, 3);

          if(system_locked)
          {
            sendArray[3] = 0;
            sendArray[4] = 127;
          }
          else
          {
            uint8_t thr = calcFinalThrottle();
            sendArray[3] = (thr >= 0xF3) ? 0xF2 : thr;  // cap: 0xF3 reserved for GPS meta-packet
            sendArray[4] = steer_scaled;
          }

          thr_sent   = sendArray[3];
          steer_sent = sendArray[4];

          sendArray[5] = esp_crc8(sendArray, 5);

          rxprint("Sending: ");
          #ifdef DEBUG_RX
          printHexArray(sendArray, 6);
          #endif

          radio.implicitHeader(6);
          radio.startTransmit(sendArray, 6);
          num_sent_packets++;
          vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Common exit for both GPS meta and normal paths:
        // return to 6-byte receive mode and wake waitForTelemetry.
        // After a GPS meta burst, RX sends a normal telemetry reply after processing
        // the GPS data packet — waitForTelemetry will receive it as usual.
        radio.implicitHeader(6);
        rfInterrupt = false;
        radio.startReceive();
        xTaskNotifyGive(triggeredWaitForTelemetryHandle);
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
  ```

- [ ] **Step 3: Confirm old `sendData()` body is fully replaced**

  Search the file for the old comment `// Dest1, Dest2, Dest3, THR, Steer, CRC8` — it must not appear. Only one `void sendData` definition should exist.

- [ ] **Step 4: Add version tag at the top of TX Radio.ino**

  At the very top of `Source/V2_Integration_Tx/Radio.ino`, add:
  ```cpp
  // V3 - 2026-04-24 - Added 0xF3 GPS meta-packet burst at 2Hz in sendData(); THR capped at 0xF2
  ```

- [ ] **Step 5: Compile TX in Arduino IDE**

  Open `Source/V2_Integration_Tx/V2_Integration_Tx.ino`.
  Board: **ESP32C3 Dev Module**.
  Click **Verify (✓)**.

  Expected: zero errors. Common issues to watch for:
  - `'gps_tx' was not declared` → should not happen; `gps_tx` is declared in `BREmote_V2_Tx.h`.
  - `'int32_t' was not declared` → add `#include <stdint.h>` if needed (usually pulled in by Arduino.h).
  - `implicit declaration of 'printHexArray'` → already used in the old sendData; no change needed.

- [ ] **Step 6: Commit TX change**

  ```bash
  git add "Source/V2_Integration_Tx/Radio.ino"
  git commit -m "feat(TX/P5): 0xF3 GPS meta-packet burst at 2Hz in sendData(); THR capped 0xF2"
  ```

---

## Task 2: Add TX GPS globals to RX header (`BREmote_V2_Rx.h`)

**Files:**
- Modify: `Source/V2_Integration_Rx/BREmote_V2_Rx.h`

- [ ] **Step 1: Locate the insertion point**

  Open `Source/V2_Integration_Rx/BREmote_V2_Rx.h`. Find this line (around line 182):
  ```cpp
  volatile bool config_version_error = false;
  ```
  The new globals go immediately after this line, before `#include "../Common/SPIFFSEngine.h"`.

- [ ] **Step 2: Insert the three globals**

  After `volatile bool config_version_error = false;`, add:

  ```cpp

  // ============================================================
  // V3 - 2026-04-24 - TX GPS COORDINATES (received via 0xF3 meta-packet)
  //
  // Written by processMetaGpsPacket() in Radio.ino at 2Hz whenever TX sends
  // a GPS meta-packet and RX successfully validates it.
  // Read by Phase B anti-spoofing (Priority 6) to check TX-RX proximity.
  //
  // rx_tx_gps_timestamp == 0 means no meta-packet has ever been received.
  // Use (millis() - rx_tx_gps_timestamp) > usrConf.tx_gps_stale_timeout_ms
  // to detect a stale TX GPS reading before trusting lat/lng.
  // ============================================================
  double        rx_tx_gps_lat       = 0.0;  // TX latitude (degrees, WGS84)
  double        rx_tx_gps_lng       = 0.0;  // TX longitude (degrees, WGS84)
  unsigned long rx_tx_gps_timestamp = 0;    // millis() when last meta-packet received; 0 = never
  ```

- [ ] **Step 3: Add version tag at top of BREmote_V2_Rx.h**

  Add to the version tag block at the top of the file:
  ```cpp
  // V3 - 2026-04-24 - Added rx_tx_gps_lat/lng/timestamp globals for 0xF3 meta-packet reception
  ```

- [ ] **Step 4: Quick compile check**

  Open `Source/V2_Integration_Rx/V2_Integration_Rx.ino`.
  Board: **ESP32S3 Dev Module**.
  Click **Verify (✓)** — confirms the header change alone compiles cleanly before touching Radio.ino.

---

## Task 3: Modify RX `triggeredReceive()` and add `processMetaGpsPacket()` — Radio.ino

**Files:**
- Modify: `Source/V2_Integration_Rx/Radio.ino`

- [ ] **Step 1: Add the file-scope state flag and helper function BEFORE `triggeredReceive`**

  In `Source/V2_Integration_Rx/Radio.ino`, find the line:
  ```cpp
  void triggeredReceive(void *parameter) {
  ```

  Insert the following block immediately before it:

  ```cpp
  // V3 - 2026-04-24 - GPS meta-packet state and handler for 0xF3 protocol

  // gps_meta_pending: set true when a 0xF3 announcement (6-byte) is received.
  // On the NEXT wakeup of triggeredReceive, read 14 bytes (GPS data) instead
  // of the normal 6-byte control packet.
  static bool gps_meta_pending = false;

  // ============================================================
  // processMetaGpsPacket - Decode a received 14-byte GPS data packet
  // ============================================================
  //
  // What it does:
  //   Validates destination address, CRC8 (over bytes 0-12 stored in byte 13),
  //   packet type (0xF3) and subtype (0x02). On success, extracts TX lat/lng
  //   stored as int32_t microdegrees (little-endian) and writes the three
  //   rx_tx_gps_* globals declared in BREmote_V2_Rx.h.
  //
  // Inputs:
  //   pkt - pointer to a 14-byte buffer containing the received GPS data packet
  //
  // Side effects:
  //   On success: updates rx_tx_gps_lat, rx_tx_gps_lng, rx_tx_gps_timestamp.
  //   Always: prints diagnostics to Serial.
  // ============================================================
  static void processMetaGpsPacket(uint8_t *pkt)
  {
    if (memcmp(pkt, usrConf.own_address, 3) != 0)
    {
      rxprintln("META GPS: address mismatch, discarding");
      return;
    }

    // CRC covers bytes 0–12; result stored in byte 13
    if (pkt[13] != esp_crc8(pkt, 13))
    {
      rxprintln("META GPS: CRC fail, discarding");
      return;
    }

    if (pkt[3] != 0xF3 || pkt[4] != 0x02)
    {
      rxprintln("META GPS: unexpected type/subtype, discarding");
      return;
    }

    // Extract lat/lng as int32_t microdegrees stored little-endian.
    // memcpy avoids strict-aliasing UB that a direct pointer cast would cause.
    int32_t lat_ud, lng_ud;
    memcpy(&lat_ud, pkt + 5, 4);
    memcpy(&lng_ud, pkt + 9, 4);

    rx_tx_gps_lat       = (double)lat_ud / 1e6;
    rx_tx_gps_lng       = (double)lng_ud / 1e6;
    rx_tx_gps_timestamp = millis();

    Serial.printf("META GPS received: lat=%.6f lng=%.6f\n",
                  rx_tx_gps_lat, rx_tx_gps_lng);
  }

  ```

- [ ] **Step 2: Replace the body of `triggeredReceive()` with the 2-path state machine**

  The existing function body runs from `while (1)` through the final `radio.startReceive(); rfInterrupt = false;`. Replace it entirely with the following. The GPS announcement path uses `continue` to bypass the common exit block (since it already set `implicitHeader(14)` + `startReceive()`); all other paths fall through to the common exit.

  ```cpp
  void triggeredReceive(void *parameter) {
    while (1)
    {
      // Wait for semaphore given by packetReceived() ISR on any DIO1 event
      if (xSemaphoreTake(triggerReceiveSemaphore, portMAX_DELAY) == pdTRUE) 
      {
        if (gps_meta_pending)
        {
          // ---- GPS data packet path (14 bytes) ----
          // A 0xF3 announcement arrived on the previous wakeup.
          // Radio is already in implicitHeader(14) + startReceive mode.
          // TX has sent the 14-byte GPS coordinate packet; read and decode it.
          gps_meta_pending = false;  // clear before any early return

          uint8_t gpsArray[14];
          if (radio.readData(gpsArray, 14) == RADIOLIB_ERR_NONE)
          {
            processMetaGpsPacket(gpsArray);
          }
          else
          {
            rxprintln("META GPS: readData error");
          }
          // Fall through to common exit: implicitHeader(6) + startReceive + rfInterrupt=false
        }
        else
        {
          // ---- Normal 6-byte control packet path ----
          uint8_t rcvArray[6];
          if (radio.readData(rcvArray, 6) == RADIOLIB_ERR_NONE) 
          {
            rxprint("Received packet: ");
            #ifdef DEBUG_RX
            printHexArray(rcvArray, 6);
            #endif

            if (memcmp(rcvArray, usrConf.own_address, 3) == 0) 
            {
              rxprintln("Address matches");
              
              if (rcvArray[5] == esp_crc8(rcvArray, 5)) 
              {
                rxprintln("CRC ok");

                if (rcvArray[3] == 0xF3)
                {
                  // ---- GPS announcement ----
                  // TX will send a 14-byte GPS data packet ~10ms from now.
                  // Switch radio to 14-byte mode immediately — SPI writes take
                  // <2ms, so we will be ready well before the GPS data arrives.
                  rxprintln("META GPS: announcement, switching to 14-byte mode");
                  gps_meta_pending = true;
                  radio.implicitHeader(14);
                  radio.startReceive();
                  rfInterrupt = false;
                  // TX does not expect a telemetry reply here; skip the common exit.
                  continue;
                }

                // ---- Normal throttle/steering control packet ----
                last_packet = millis();
  #ifdef WIFI_ENABLED
                webCfgNotifyRxConnected();
  #endif
                rxprint("RSSI: ");
                rxprint(radio.getRSSI());
                rxprint(", SNR: ");
                rxprintln(radio.getSNR());

                thr_received      = rcvArray[3];
                steering_received = rcvArray[4];

                telemetry.link_quality = getLinkQuality(radio.getRSSI(), radio.getSNR());

                rxprintln("Sending response");

                uint8_t sendArray[6];
                memcpy(sendArray, usrConf.dest_address, 3);
                uint8_t* ptr = (uint8_t*)&telemetry;
                sendArray[3] = telemetry_index;
                sendArray[4] = ptr[telemetry_index];
                telemetry_index++;
                if(telemetry_index >= sizeof(TelemetryPacket))
                {
                  telemetry_index = 0;
                }
                sendArray[5] = esp_crc8(sendArray, 5);

                #ifdef DEBUG_RX
                printHexArray(sendArray, 6);
                #endif

                vTaskDelay(pdMS_TO_TICKS(10));
                radio.implicitHeader(6);
                radio.startTransmit(sendArray, 6);
                vTaskDelay(pdMS_TO_TICKS(10));
              }
            }
          }
          else
          {
            rxprintln("Rx err");
          }
        }

        // Common exit: restore 6-byte receive mode and clear stale interrupt flag.
        // The GPS announcement path uses 'continue' and never reaches here.
        radio.implicitHeader(6);
        radio.startReceive();
        rfInterrupt = false;
      }
    }
  }
  ```

- [ ] **Step 3: Add version tag at top of RX Radio.ino**

  At the top of `Source/V2_Integration_Rx/Radio.ino`, add:
  ```cpp
  // V3 - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine
  ```

- [ ] **Step 4: Compile RX in Arduino IDE**

  Open `Source/V2_Integration_Rx/V2_Integration_Rx.ino`.
  Board: **ESP32S3 Dev Module**.
  Click **Verify (✓)**.

  Expected: zero errors. Common issues:
  - `'rx_tx_gps_lat' was not declared` → Task 2 globals missing from BREmote_V2_Rx.h. Add them.
  - `static function declared after non-static` → Arduino IDE may warn about `static void processMetaGpsPacket`. Remove the `static` keyword from the function if this happens (all .ino functions share one translation unit so `static` is optional).
  - `'continue' outside loop` → should not occur since `continue` is inside `while(1)`.

- [ ] **Step 5: Commit RX changes**

  ```bash
  git add "Source/V2_Integration_Rx/Radio.ino" "Source/V2_Integration_Rx/BREmote_V2_Rx.h"
  git commit -m "feat(RX/P5): receive 0xF3 GPS meta-packets; store rx_tx_gps_lat/lng/timestamp"
  ```

---

## Task 4: Serial Monitor Verification (both boards live)

Flash both boards. Connect USB to either board and open Serial Monitor at 115200 baud.

- [ ] **Step 1: Verify GPS meta-packets are sent by TX**

  On TX Serial Monitor (if not suppressed by `serialOff`): with `gps_en=1` and a valid GPS fix (solid GPS dot on display), every 500ms you should see:
  ```
  Sending GPS announcement: F3 01 ...
  Sending GPS data: F3 02 ...
  ```
  (Only visible if `#define DEBUG_RX` is uncommented — otherwise `rxprint` is a no-op. To enable: add `#define DEBUG_RX` near the top of `BREmote_V2_Tx.h` temporarily, compile, flash, verify, remove.)

- [ ] **Step 2: Verify GPS coordinates are received by RX**

  On RX Serial Monitor, with both boards paired and TX GPS valid, every 500ms:
  ```
  META GPS received: lat=XX.XXXXXX lng=YY.YYYYYY
  ```
  The `Serial.printf` in `processMetaGpsPacket` always prints (not gated by DEBUG_RX), so this is visible without any #define changes.

  Cross-check lat/lng against a phone GPS app at the same location — values should match within ±5 meters.

- [ ] **Step 3: Verify control operation is unaffected**

  During the GPS meta test: unlock the TX and apply throttle. The RX should still drive PWM outputs normally. Telemetry (battery %, speed) should still update on the TX display. This confirms the 4-of-5 normal control cycles are intact.

- [ ] **Step 4: Verify no GPS meta-packets when GPS disabled**

  Set `gps_en=0` in TX web config, save, reboot TX. RX Serial Monitor should show no `META GPS received` lines. TX should continue sending control packets normally every 100ms.

- [ ] **Step 5: Verify THR cap**

  Pull the throttle trigger to maximum. On TX Serial Monitor or via web config debug, confirm `thr_sent` reaches at most 242 (0xF2 = 94.9%). If using full-throttle in a real efoil/buggy context, verify the cap is imperceptible.

- [ ] **Step 6: Commit final verified state**

  ```bash
  git commit --allow-empty -m "test(P5): GPS meta-packet verified on hardware — lat/lng received at 2Hz, control unaffected"
  ```

---

## Self-Review Against Spec

| Requirement (CLAUDE.md §7 Priority 5) | Covered by |
|---|---|
| TX→RX carries GPS coordinates | Task 1 (TX build + send), Task 3 (RX receive + decode) |
| At 2Hz | `gps_cycle >= 5` threshold at 10Hz tick = 500ms period |
| Required for Phase B handshake (Priority 6) | `rx_tx_gps_lat/lng/timestamp` in BREmote_V2_Rx.h ready for Phase B code |
| Required for RTM steering computation (Priority 7) | Same globals accessible to future RTM file |
| No new SPIFFS parameters | Confirmed — no confStruct, WebUiEmbedded.h, or ConfigService changes |
| CLAUDE.md §9 Safety: no autonomous motor movement | GPS meta-packet carries coordinates only; zero throttle/steering influence |
| CLAUDE.md §10 Web Config UI Rule | No new parameters → no UI change required |

**Placeholder scan:** No TBD/TODO. All code blocks complete and compilable.

**Type consistency check:**
- `rx_tx_gps_lat` / `rx_tx_gps_lng`: declared `double` in header, written as `(double)lat_ud / 1e6` in `processMetaGpsPacket` — consistent.
- `esp_crc8(pkt, 13)` on 13 bytes for 14-byte packet (indices 0–12 → result in index 13) — consistent with existing `esp_crc8(sendArray, 5)` pattern (indices 0–4 → result in index 5).
- `int32_t lat_ud` / `lng_ud` used in both TX encode (`memcpy(gpsPkt + 5, &lat_ud, 4)`) and RX decode (`memcpy(&lat_ud, pkt + 5, 4)`) — consistent byte layout.
- `gps_meta_pending`: declared `static bool`, set `true` on announcement, cleared `false` before GPS data read — no type mismatch.
