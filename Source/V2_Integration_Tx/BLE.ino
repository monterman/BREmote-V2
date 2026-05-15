// V2.5-Evo - 2026-05-15 - feature/bluetooth Tier 1: NUS skeleton
// Advertises Nordic UART Service UUIDs so any NUS-compatible app (VESC Tool,
// nRF Toolbox, Serial Bluetooth Terminal) can connect and receive CSV telemetry.
// Dependency: NimBLE-Arduino library (install via Arduino Library Manager).
// initBLE() is called from bleInitTask (Init.ino) — never call from setup() directly.

#include <NimBLEDevice.h>

#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static NimBLEServer*         bleServer  = nullptr;
static NimBLECharacteristic* nusTxChar  = nullptr;
static bool                  bleRunning = false;

// NimBLE-Arduino 2.x callback signatures — connInfo + reason added in 2.0
class BLEServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override { }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    NimBLEDevice::startAdvertising();  // auto-restart so phone can reconnect
  }
};

void initBLE()
{
  NimBLEDevice::init("BREmote-TX");
  NimBLEDevice::setPower(9);  // 9 dBm max; NimBLE 2.x takes dBm directly (not ESP_PWR_LVL enum)

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BLEServerCB());

  NimBLEService* nus = bleServer->createService(NUS_SERVICE_UUID);

  // TX char — ESP32 notifies phone with telemetry CSV
  nusTxChar = nus->createCharacteristic(NUS_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  // RX char — phone can write commands (future use; ignored for now)
  nus->createCharacteristic(NUS_RX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  nus->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  // setScanResponse removed — not available in NimBLE 2.x; advertising works without it
  NimBLEDevice::startAdvertising();

  bleRunning = true;
  Serial.println("BLE: advertising as BREmote-TX (NUS)");
}

// CSV format over NUS:
//   SPD:<km/h>,BAT:<%>,TMP:<°C>,A:<amps>,W:<watts>,SQ:<0-5>\n
//   0xFF fields (VESC offline) are sent as 255 — app should treat 255 as N/A.
//   W field is decoded: foil_power (VESC watts/50) × 50 = actual watts.
void sendBLETelemetry()
{
  if (!bleRunning || !nusTxChar) return;
  if (!bleServer->getConnectedCount()) return;

  uint16_t watts = (telemetry.foil_power != 0xFF)
                   ? (uint16_t)telemetry.foil_power * 50u
                   : 0xFFFF;

  char buf[64];
  snprintf(buf, sizeof(buf),
    "SPD:%u,BAT:%u,TMP:%u,A:%u,W:%u,SQ:%u\n",
    (unsigned)telemetry.foil_speed,
    (unsigned)telemetry.foil_bat,
    (unsigned)telemetry.foil_temp,
    (unsigned)telemetry.foil_motor_amps,
    (unsigned)watts,
    (unsigned)sq_graph
  );

  nusTxChar->setValue((uint8_t*)buf, strlen(buf));
  nusTxChar->notify();
}

// Called from loop() every 110ms tick.
// Sends when: bt_enabled==2 (always), bt_enabled==1 + dot active, or boot-gesture forced.
// Rate-limited to once per 500ms regardless of loop() cadence.
void bleTelemetryLoop()
{
  if (!bleRunning) return;

  bool active = (usrConf.bt_enabled == 2) ||
                (usrConf.bt_enabled == 1 && bt_dot_state != BT_DOT_OFF) ||
                bt_session_forced;
  if (!active) return;

  static uint32_t ble_last_ms = 0;
  if (millis() - ble_last_ms < 500) return;
  ble_last_ms = millis();

  sendBLETelemetry();
}
