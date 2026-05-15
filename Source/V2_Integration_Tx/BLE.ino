// V2.5-Evo - 2026-05-15 - feature/bluetooth Tier 2: VESC Tool compatible protocol
// Implements VESC binary protocol (COMM_GET_VALUES 0x04) over NUS so VESC Tool,
// Floaty, and other VESC-compatible apps display live gauges.
// Auto-detects app type: VESC binary requests → VESC protocol mode (request-driven).
// No VESC requests → CSV push every 500ms (Serial BT Terminal compat).
// Dependency: NimBLE-Arduino 2.x (install via Arduino Library Manager).
// v_in shows 0.0V until RX telemetry is extended to include foil_voltage (future).

#include <NimBLEDevice.h>
#include <esp_mac.h>

#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define COMM_GET_VALUES   0x04

static NimBLEServer*         bleServer     = nullptr;
static NimBLECharacteristic* nusTxChar     = nullptr;
static NimBLECharacteristic* nusRxChar     = nullptr;
static bool                  bleRunning    = false;
static bool                  vescProtoMode = false;  // true when VESC Tool-style app detected

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

// Build and send COMM_GET_VALUES response (VESC Tool / Floaty format)
// Fields mapped from LoRa telemetry struct. Unknown fields sent as 0.
// VESC Tool gauges that work: Temp, Motor Amps, Power (W).
// VESC Tool gauges that show 0: Voltage, RPM, Duty, Ah, Wh (no RX data yet).
static void sendVescGetValues() {
  if (!bleRunning || !nusTxChar) return;
  if (!bleServer->getConnectedCount()) return;

  float temp  = (telemetry.foil_temp       != 0xFF) ? (float)telemetry.foil_temp       : 0.0f;
  float mAmps = (telemetry.foil_motor_amps != 0xFF) ? (float)telemetry.foil_motor_amps : 0.0f;

  uint8_t pl[80];
  int idx = 0;

  pl[idx++] = COMM_GET_VALUES;
  appendFloat16(pl, idx, temp,  10.0f);     // temp_fet (°C × 10)
  appendFloat16(pl, idx, temp,  10.0f);     // temp_motor — same, only one sensor available
  appendFloat32(pl, idx, mAmps, 100.0f);    // avg_motor_current (A × 100)
  appendFloat32(pl, idx, 0.0f,  100.0f);    // avg_input_current — not available
  appendFloat32(pl, idx, 0.0f,  100.0f);    // avg_id
  appendFloat32(pl, idx, 0.0f,  100.0f);    // avg_iq
  appendFloat16(pl, idx, 0.0f,  1000.0f);   // duty_cycle — not available
  appendFloat32(pl, idx, 0.0f,  1.0f);      // rpm — not available
  appendFloat16(pl, idx, 0.0f,  10.0f);     // v_in — 0V until RX telemetry adds foil_voltage
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
static void sendCSVTelemetry() {
  if (!bleRunning || !nusTxChar) return;
  if (!bleServer->getConnectedCount()) return;

  uint16_t watts = (telemetry.foil_power != 0xFF)
                   ? (uint16_t)telemetry.foil_power * 50u : 0xFFFF;
  char buf[64];
  snprintf(buf, sizeof(buf), "SPD:%u,BAT:%u,TMP:%u,A:%u,W:%u,SQ:%u\n",
    (unsigned)telemetry.foil_speed, (unsigned)telemetry.foil_bat,
    (unsigned)telemetry.foil_temp, (unsigned)telemetry.foil_motor_amps,
    (unsigned)watts, (unsigned)sq_graph);
  nusTxChar->setValue((uint8_t*)buf, strlen(buf));
  nusTxChar->notify();
}

// NUS RX characteristic — receives commands from phone app
class NusRxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    auto val = pChar->getValue();
    int cmd = parseVescCommand((const uint8_t*)val.data(), val.length());
    if (cmd == COMM_GET_VALUES) {
      vescProtoMode = true;   // switch to request-driven VESC protocol
      sendVescGetValues();
    }
  }
};

class BLEServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    vescProtoMode = false;  // reset on each new connection
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    vescProtoMode = false;
    NimBLEDevice::startAdvertising();
  }
};

void initBLE() {
  // Unique name: BRemote-TX-XX where XX = last byte of BT MAC (uppercase hex)
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char devName[20];
  snprintf(devName, sizeof(devName), "BRemote-TX-%02X", mac[5]);

  NimBLEDevice::init(devName);
  NimBLEDevice::setPower(9);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BLEServerCB());

  NimBLEService* nus = bleServer->createService(NUS_SERVICE_UUID);

  nusTxChar = nus->createCharacteristic(NUS_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  nusRxChar = nus->createCharacteristic(NUS_RX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  nusRxChar->setCallbacks(new NusRxCB());

  nus->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setName(devName);
  advData.addServiceUUID(NUS_SERVICE_UUID);
  adv->setAdvertisementData(advData);
  NimBLEDevice::startAdvertising();

  bleRunning = true;
  Serial.printf("BLE: advertising as %s (NUS + VESC protocol)\n", devName);
}

// Called from loop() every 110ms tick.
// VESC protocol mode: app drives request-response, this function stays silent.
// CSV mode: pushes telemetry every 500ms when BLE is active.
void bleTelemetryLoop() {
  if (!bleRunning) return;

  bool active = (usrConf.bt_enabled == 2) ||
                (usrConf.bt_enabled == 1 && bt_dot_state != BT_DOT_OFF) ||
                bt_session_forced;
  if (!active) return;
  if (vescProtoMode) return;  // VESC Tool drives its own polling cycle via requests

  static uint32_t ble_last_ms = 0;
  if (millis() - ble_last_ms < 500) return;
  ble_last_ms = millis();

  sendCSVTelemetry();
}
