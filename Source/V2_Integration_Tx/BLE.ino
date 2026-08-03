// V2.5-Evo - 2026-05-15 - feature/bluetooth Tier 2: VESC Tool compatible protocol
// Implements VESC binary protocol (COMM_GET_VALUES 0x04) over NUS so VESC Tool,
// Floaty, and other VESC-compatible apps display live gauges.
// Auto-detects app type: VESC binary requests → VESC protocol mode (request-driven).
// No VESC requests → CSV push every 500ms (Serial BT Terminal compat).
// Dependency: NimBLE-Arduino 2.x (install via Arduino Library Manager).
// v_in, duty, rpm now decoded from expanded 19-byte RX packet (foil_voltage, foil_duty, foil_erpm).
// V2.5-Evo - 2026-06-04 - Entire file guarded by BLE_ENABLED (BREmote_V2_Tx.h). With the guard
// undefined the whole NimBLE stack is excluded from the build (disabled for water testing).
// V2.5-Evo - 2026-07-20 - Added bleIsConnected() accessor so Display.ino can render the BT dot SOLID on a live connection.
// V2.5-Evo - 2026-07-20 - BLE re-enable deep-fix (Rex): onConnect relaxes the connection interval (§4.3) and
//   stops advertising for single-peripheral operation (§4.2); the three concurrent notify streams are
//   consolidated into ONE back-pressured stream (§4.4) pushed from a dedicated Core-0 task (§4.5), not loop().
// V2.5-Evo - 2026-07-21 - DIAG: promoted the bleServiceNotify() heap-suspend flag to a file-scope
//   volatile bool (ble_notify_heap_suspended) so the ?state command can report notify active/SUSPENDED.
// V2.5-Evo - 2026-07-20 - Rex re-audit: added a RUNTIME heap-floor net in bleServiceNotify that suspends
//   telemetry notifies below BLE_HEAP_RUNTIME_FLOOR_BYTES (M2), and made vescProtoMode std::atomic<bool> (L3).
#ifdef BLE_ENABLED

#include <NimBLEDevice.h>
#include <esp_mac.h>

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define COMM_FW_VERSION   0x00
#define COMM_GET_VALUES   0x04

static NimBLEServer*         bleServer     = nullptr;
static NimBLECharacteristic* nusTxChar     = nullptr;
static NimBLECharacteristic* nusRxChar     = nullptr;
static bool                  bleRunning    = false;
// V2.5-Evo - 2026-07-20 - Rex L3 (re-audit): vescProtoMode is written by the NimBLE host task
// (onWrite/onConnect/onDisconnect) and read by the Core-0 notify task (bleServiceNotify). As a plain
// bool that cross-task read/write is technically a data race (benign on the single-core C3 — only
// preemption, no true parallelism — but the owner wants it clean). Made std::atomic<bool> to match the
// established cross-task-flag pattern in this sketch (rfInterrupt, rtm_tx_active in BREmote_V2_Tx.h).
// <atomic> is already included via the header. No behavior change: bool store/load semantics are identical.
static std::atomic<bool>     vescProtoMode{false};  // true when VESC Tool-style app detected
// V2.5-Evo - 2026-07-21 - Promoted from a bleServiceNotify() function-static to file scope so the
// ?state command (serPrintStatus in System.ino) can read the runtime notify state. Written by the
// Core-0 notify task (bleServiceNotify) and read from the serial-command context, so it is volatile.
// The M2 heap-floor tripwire logic below is UNCHANGED — only the storage location moved.
volatile bool                ble_notify_heap_suspended = false;  // true while telemetry notifies are suspended by the runtime heap floor

// ===== CRC16/CCITT — VESC standard =====
static uint16_t crc16(const uint8_t* buf, uint16_t len) {
  uint16_t crc = 0;
  while (len--) {
    crc ^= (uint16_t)(*buf++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// ===== Big-endian fixed-point encoders — VESC wire format =====
static void appendFloat16(uint8_t* b, int& i, float v, float s) {
  int16_t x = (int16_t)(v * s);
  b[i++] = (uint8_t)(x >> 8);
  b[i++] = (uint8_t)(x & 0xFF);
}
static void appendFloat32(uint8_t* b, int& i, float v, float s) {
  int32_t x = (int32_t)(v * s);
  b[i++] = (uint8_t)(x >> 24);
  b[i++] = (uint8_t)(x >> 16);
  b[i++] = (uint8_t)(x >> 8);
  b[i++] = (uint8_t)(x & 0xFF);
}
static void appendInt32(uint8_t* b, int& i, int32_t v) {
  b[i++] = (uint8_t)(v >> 24);
  b[i++] = (uint8_t)(v >> 16);
  b[i++] = (uint8_t)(v >> 8);
  b[i++] = (uint8_t)(v & 0xFF);
}

// Wrap payload in VESC frame: 0x02 LEN [payload] CRC_H CRC_L 0x03
static int buildVescFrame(uint8_t* out, const uint8_t* payload, uint8_t plen) {
  int i = 0;
  out[i++] = 0x02;
  out[i++] = plen;
  memcpy(out + i, payload, plen);
  i += plen;
  uint16_t crc = crc16(payload, plen);
  out[i++] = (uint8_t)(crc >> 8);
  out[i++] = (uint8_t)(crc & 0xFF);
  out[i++] = 0x03;
  return i;
}

// Parse incoming VESC frame; returns command byte or -1 on invalid/short frame
static int parseVescCommand(const uint8_t* data, size_t len) {
  if (len < 5 || data[0] != 0x02) return -1;
  uint8_t plen = data[1];
  if (len < (size_t)(plen + 5)) return -1;
  uint16_t crc_recv = ((uint16_t)data[2 + plen] << 8) | data[3 + plen];
  if (crc16(data + 2, plen) != crc_recv) return -1;
  if (data[4 + plen] != 0x03) return -1;
  return data[2];  // command byte is first payload byte
}

// COMM_FW_VERSION (0x00) response — required handshake before VESC Tool shows gauges.
// Payload: cmd | major | minor | hw_name\0 | uuid[12] | pairing | fw_test | hw_type | custom_cfg
static void sendVescFwVersion() {
  if (!bleRunning || !nusTxChar) return;
  if (!bleServer->getConnectedCount()) return;

  uint8_t pl[64];
  int idx = 0;

  pl[idx++] = COMM_FW_VERSION;
  pl[idx++] = 6;                          // major
  pl[idx++] = 5;                          // minor — Floaty supports 6.02/6.05/6.06
  const char* hwName = "BREmote";
  memcpy(pl + idx, hwName, strlen(hwName) + 1);   // includes null terminator
  idx += (int)strlen(hwName) + 1;
  memset(pl + idx, 0, 12);               // STM32 UUID — zeroed
  idx += 12;
  pl[idx++] = 0;                          // pairing_done
  pl[idx++] = 0;                          // fw_test_version_number
  pl[idx++] = 0;                          // hw_type: 0 = VESC
  pl[idx++] = 0;                          // custom_config

  uint8_t frame[72];
  int flen = buildVescFrame(frame, pl, (uint8_t)idx);
  nusTxChar->setValue(frame, flen);
  nusTxChar->notify();
}

// Build and send COMM_GET_VALUES response (VESC Tool / Floaty format)
// Fields mapped from LoRa telemetry struct. Unknown fields sent as 0.
// VESC Tool gauges that work: Temp, Motor Amps, Power (W), Voltage, Duty, RPM.
// VESC Tool gauges that show 0: Ah, Wh (not tracked).
static void sendVescGetValues() {
  if (!bleRunning || !nusTxChar) return;
  if (!bleServer->getConnectedCount()) return;

  float temp      = (telemetry.foil_temp       != 0xFF) ? (float)telemetry.foil_temp       : 0.0f;
  float mAmps     = (telemetry.foil_motor_amps != 0xFF) ? (float)telemetry.foil_motor_amps : 0.0f;
  float v_in      = (telemetry.foil_voltage    != 0xFF) ? (float)telemetry.foil_voltage * 0.5f : 0.0f;
  float duty_frac = (telemetry.foil_duty       != 0xFF) ? (float)telemetry.foil_duty / 100.0f  : 0.0f;
  float rpm       = 0.0f;
  {
    uint16_t es = ((uint16_t)telemetry.foil_erpm_hi << 8) | telemetry.foil_erpm_lo;
    if (es != 0xFFFF) rpm = (float)es * 100.0f;
  }

  uint8_t pl[80];
  int idx = 0;

  pl[idx++] = COMM_GET_VALUES;
  appendFloat16(pl, idx, temp,      10.0f);    // temp_fet (°C × 10)
  appendFloat16(pl, idx, temp,      10.0f);    // temp_motor — same, only one sensor
  appendFloat32(pl, idx, mAmps,    100.0f);    // avg_motor_current (A × 100)
  appendFloat32(pl, idx, 0.0f,     100.0f);    // avg_input_current — not available
  appendFloat32(pl, idx, 0.0f,     100.0f);    // avg_id
  appendFloat32(pl, idx, 0.0f,     100.0f);    // avg_iq
  appendFloat16(pl, idx, duty_frac, 1000.0f);  // duty_cycle (fraction × 1000)
  appendFloat32(pl, idx, rpm,        1.0f);    // rpm (|ERPM|)
  appendFloat16(pl, idx, v_in,       10.0f);   // v_in (battery voltage)
  appendFloat32(pl, idx, 0.0f,  10000.0f);  // amp_hours
  appendFloat32(pl, idx, 0.0f,  10000.0f);  // amp_hours_charged
  appendFloat32(pl, idx, 0.0f,  10000.0f);  // watt_hours
  appendFloat32(pl, idx, 0.0f,  10000.0f);  // watt_hours_charged
  appendInt32  (pl, idx, 0);                // tachometer
  appendInt32  (pl, idx, 0);                // tachometer_abs
  pl[idx++] = telemetry.error_code;         // fault_code → VESC Tool fault indicator
  appendFloat32(pl, idx, 0.0f,  1e6f);      // pid_position
  pl[idx++] = 1;                            // controller_id
  appendFloat16(pl, idx, temp,  10.0f);     // temp_mos1
  appendFloat16(pl, idx, temp,  10.0f);     // temp_mos2
  appendFloat16(pl, idx, temp,  10.0f);     // temp_mos3
  appendFloat32(pl, idx, 0.0f,  1000.0f);   // avg_vd
  appendFloat32(pl, idx, 0.0f,  1000.0f);   // avg_vq

  uint8_t frame[90];
  int flen = buildVescFrame(frame, pl, (uint8_t)idx);
  nusTxChar->setValue(frame, flen);
  nusTxChar->notify();
}

// CSV push for non-VESC apps (Serial BT Terminal)
// V2.5-Evo - 2026-07-20 - Rex §4.4: returns the notify() result so the caller can apply backpressure.
static bool sendCSVTelemetry() {
  if (!bleRunning || !nusTxChar) return false;
  if (!bleServer->getConnectedCount()) return false;

  uint16_t watts = (telemetry.foil_power != 0xFF)
                   ? (uint16_t)telemetry.foil_power * 50u : 0xFFFF;
  char buf[64];
  snprintf(buf, sizeof(buf), "SPD:%u,BAT:%u,TMP:%u,A:%u,W:%u,SQ:%u\n",
    (unsigned)telemetry.foil_speed, (unsigned)telemetry.foil_bat,
    (unsigned)telemetry.foil_temp, (unsigned)telemetry.foil_motor_amps,
    (unsigned)watts, (unsigned)sq_graph);
  nusTxChar->setValue((uint8_t*)buf, strlen(buf));
  return nusTxChar->notify();
}

// NUS RX characteristic — receives commands from phone app
class NusRxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    auto val = pChar->getValue();
    int cmd = parseVescCommand((const uint8_t*)val.data(), val.length());
    if (cmd == COMM_FW_VERSION) {
      vescProtoMode = true;
      sendVescFwVersion();
    } else if (cmd == COMM_GET_VALUES) {
      vescProtoMode = true;
      sendVescGetValues();
    }
  }
};

class BLEServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    vescProtoMode = false;  // reset on each new connection

    // Rex §4.3 — request a RELAXED connection interval the instant a central connects. This is the
    // single highest-leverage single-core mitigation: a longer interval = the BT controller wakes far
    // less often = far less of the one C3 core stolen from the display render and the LoRa sendData
    // path, protecting BOTH the display and LoRa/FM timing. Units: 1.25 ms per interval step,
    // 10 ms per supervision-timeout step (see the BLE_CONN_* defines in BREmote_V2_Tx.h).
    pServer->updateConnParams(connInfo.getConnHandle(),
                              BLE_CONN_MIN_INTERVAL, BLE_CONN_MAX_INTERVAL,
                              BLE_CONN_LATENCY, BLE_CONN_TIMEOUT);

    // Rex §4.2 — peripheral, single connection. This firmware is server/peripheral-only by
    // construction (it never scans or acts as a central). Stop advertising while connected so a
    // second central cannot attach; advertising is restarted on disconnect below.
    NimBLEDevice::stopAdvertising();
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    vescProtoMode = false;
#ifdef EXT_TELEM_ENABLED
    extTelemClearSubscriber();
#endif
    NimBLEDevice::startAdvertising();
  }
};

void initBLE() {
  // Unique name: BRemote-TX-XX where XX = last byte of BT MAC (uppercase hex)
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char devName[20];
  snprintf(devName, sizeof(devName), "BRemote-TX-%02X", mac[5]);

  // Rex §4.1 — log free internal DRAM immediately before and after the NimBLE stack init so the
  // no-PSRAM C3's real cost is visible on the bench (the bleInitTask floor-guard already refused to
  // reach here if the pre-init heap was below BLE_HEAP_FLOOR_BYTES). Same instrumentation that caught
  // the sibling foilIQ S3 collapse (81776 → 11688 → init FAILED).
  uint32_t heap_pre = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  NimBLEDevice::init(devName);
  uint32_t heap_post = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  Serial.printf("BLE: internal heap NimBLEDevice::init() pre=%u post=%u (consumed %d)\n",
                (unsigned)heap_pre, (unsigned)heap_post, (int)heap_pre - (int)heap_post);
  NimBLEDevice::setPower(9);

  // Rex §4.2 — peripheral, single-connection footprint: this app creates ONLY a GATT server (no
  // scanner, no client), so it is peripheral-only by construction. Advertising is stopped on connect
  // (BLEServerCB::onConnect) so a second central can't attach.
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BLEServerCB());

  NimBLEService* nus = bleServer->createService(NUS_SERVICE_UUID);

  nusTxChar = nus->createCharacteristic(NUS_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  nusRxChar = nus->createCharacteristic(NUS_RX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  nusRxChar->setCallbacks(new NusRxCB());

  nus->start();
#ifdef EXT_TELEM_ENABLED
  initExtTelem(bleServer);
#endif

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

  // Main advert: flags (3 B) + NUS UUID 128-bit (18 B) = 21 B — fits in 31-byte BLE limit.
  // Putting UUID here ensures VESC Tool / Floaty UUID scan filters find this device.
  // Name + UUID together = 37 B and silently truncates the UUID, making the device invisible.
  NimBLEAdvertisementData advData;
  advData.setFlags(0x06);               // LE General Discoverable | BR/EDR Not Supported
  advData.addServiceUUID(NUS_SERVICE_UUID);
  adv->setAdvertisementData(advData);

  // Scan response: device name — sent on active SCAN_REQ; visible as device label in app list.
  NimBLEAdvertisementData scanRsp;
  scanRsp.setName(devName);
  adv->setScanResponseData(scanRsp);

  NimBLEDevice::startAdvertising();

  bleRunning = true;
  Serial.printf("BLE: advertising as %s (NUS + VESC protocol)\n", devName);
}

// V2.5-Evo - 2026-07-20 - True when a BLE central is actually connected (not merely advertising).
// Used by Display.ino to render the BT status dot SOLID on a live connection (e.g. Waveshare).
bool bleIsConnected() {
  return bleRunning && bleServer && bleServer->getConnectedCount() > 0;
}

// V2.5-Evo - 2026-07-20 - Rex §4.4 + §4.5 — consolidated, back-pressured telemetry push. ONE stream
// at a time, driven by bleNotifyTask() (below), NEVER from loop(). This replaces the old pair of
// concurrent pushers (bleTelemetryLoop CSV @500ms + ext-telemNotifyLoop @200ms):
//   - VESC-Tool mode → stay silent; the app drives its own request/response cycle (NusRxCB::onWrite).
//   - ext-telem client  → push the 28-byte ext-telem packet (that characteristic has a subscriber).
//   - otherwise      → push the CSV line for a generic Serial-BT-Terminal central.
// The notify() return is checked: on stack congestion (mbuf/ENOMEM) we skip and let the next tick
// retry instead of piling allocations onto the tight, no-PSRAM C3 heap (a direct H2/H3 mitigation).
void bleServiceNotify() {
  if (!bleRunning) return;
  if (!bleServer || bleServer->getConnectedCount() == 0) return;

  // Same active-gate the old bleTelemetryLoop() used: honor bt_enabled / boot gesture.
  bool active = (usrConf.bt_enabled == 2) ||
                (usrConf.bt_enabled == 1 && bt_dot_state != BT_DOT_OFF) ||
                bt_session_forced;
  if (!active) return;

  if (vescProtoMode) return;  // VESC Tool drives its own polling cycle via requests

  // V2.5-Evo - 2026-07-20 - Rex M2 (re-audit): RUNTIME heap floor. The init-time floor in bleInitTask
  // only guards NimBLEDevice::init() once at boot; it does nothing against a runtime H2 heap collapse
  // under a live connection. This is that missing net: before every push, read free INTERNAL DRAM and
  // SUSPEND telemetry notifies if it falls below BLE_HEAP_RUNTIME_FLOOR_BYTES — skipping the push keeps
  // BLE egress off the tight no-PSRAM heap while the connection + NimBLE stack stay fully up. Resume only
  // once heap recovers above floor + BLE_HEAP_RUNTIME_HYSTERESIS_BYTES so we don't flap at the threshold.
  // Fail-safe by construction: this only reduces BLE activity, never the motor / throttle / steer path.
  // Logged once per transition. Runs every notify tick (BLE_NOTIFY_TICK_MS) for prompt detection.
  // V2.5-Evo - 2026-07-21 - suspend flag promoted to file scope (ble_notify_heap_suspended, top of file)
  // so ?state can read it; logic identical to the former function-static.
  uint32_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  if (ble_notify_heap_suspended)
  {
    if (free_internal >= BLE_HEAP_RUNTIME_FLOOR_BYTES + BLE_HEAP_RUNTIME_HYSTERESIS_BYTES)
    {
      ble_notify_heap_suspended = false;
      Serial.printf("BLE: internal heap recovered to %u bytes — RESUMING telemetry notifies.\n",
                    (unsigned)free_internal);
    }
    else
    {
      return;  // still starved — keep pushes suspended; connection + stack stay up, motor path untouched
    }
  }
  else if (free_internal < BLE_HEAP_RUNTIME_FLOOR_BYTES)
  {
    ble_notify_heap_suspended = true;
    Serial.printf("BLE: internal heap %u < runtime floor %u — SUSPENDING telemetry notifies "
                  "(BLE stack stays up; motor/throttle/steer path unaffected).\n",
                  (unsigned)free_internal, (unsigned)BLE_HEAP_RUNTIME_FLOOR_BYTES);
    return;
  }

  static uint32_t last_ms = 0;
  if (millis() - last_ms < BLE_TELEM_INTERVAL_MS) return;
  last_ms = millis();

  // Exactly ONE stream per tick (Rex §4.4).
#ifdef EXT_TELEM_ENABLED
  bool ok = extTelemHasSubscriber() ? sendExtTelem() : sendCSVTelemetry();
#else
  // Module absent: CSV/NUS is the only stream. Same one-stream-per-tick contract.
  bool ok = sendCSVTelemetry();
#endif

  // Backpressure: on a failed notify (congestion / no free mbuf) don't hammer — hold off one extra
  // interval so the controller can drain before the next attempt.
  if (!ok) last_ms += BLE_TELEM_INTERVAL_MS;
}

// V2.5-Evo - 2026-07-20 - Rex §4.5 — dedicated BLE notify task. Core 0 (ESP32-C3 is single-core),
// priority 1 (below every app task, above idle) so BLE cadence can never couple to loop()'s display-
// render timing. It only reads telemetry/state and calls notify(); it NEVER touches displayBuffer or
// the Wire bus. It delays every tick via vTaskDelayUntil, so idle keeps running and the native task
// watchdog stays fed without this task subscribing to it.
void bleNotifyTask(void* param) {
  const TickType_t period = pdMS_TO_TICKS(BLE_NOTIFY_TICK_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    bleServiceNotify();
    vTaskDelayUntil(&xLastWakeTime, period);
  }
}

#endif // BLE_ENABLED
