#ifndef SPIFFS_ENGINE_H
#define SPIFFS_ENGINE_H

// Shared SPIFFS config persistence and WebUI embedding for BREmote V2 TX and RX.
// Requirements before #include:
//   - <SPIFFS.h>, "mbedtls/base64.h" included
//   - confStruct type defined, usrConf + defaultConf globals declared
//   - CONF_FILE_PATH global declared
//   - SW_VERSION constant defined
//   - config_version_error global declared
//   - esp_crc8() available (from SystemCommon.h or forward-declared)
//   - cfgValidateCrossField(), validateConfig() from ConfigServiceEngine.h
//
// Each side must define (in its own SPIFFS.ino):
//   void spiffsErrorHalt(int type);
//     type 1 = SPIFFS format failed
//     type 2 = default config write failed
//   void spiffsFormatNotify(bool starting);
//     called before (true) and after (false) SPIFFS format

// Forward declarations — defined per-side in SPIFFS.ino / System.ino.
void spiffsErrorHalt(int type);
void spiffsFormatNotify(bool starting);
uint8_t esp_crc8(uint8_t *data, uint8_t length);

// ===== WebUI Embedding =====

#include "WebUiEmbedded.h"

static const char* WEB_UI_INDEX_PATH = "/index.html";
static const char* WEB_UI_INDEX_TMP_PATH = "/index.new";
static const char* WEB_UI_VERSION_PATH = "/ui.version";
static const char* WEB_UI_VERSION = "2026-03-06.1";

static uint32_t webUiFnv1a(const uint8_t* data, size_t len)
{
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; i++)
  {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

static uint32_t webUiFileFnv1a(File &file)
{
  uint8_t buf[256];
  uint32_t hash = 2166136261UL;
  while (file.available())
  {
    const size_t rd = file.read(buf, sizeof(buf));
    for (size_t i = 0; i < rd; i++)
    {
      hash ^= buf[i];
      hash *= 16777619UL;
    }
  }
  return hash;
}

static bool webUiWriteVersionFile()
{
  File vf = SPIFFS.open(WEB_UI_VERSION_PATH, FILE_WRITE);
  if (!vf) return false;
  const size_t written = vf.print(WEB_UI_VERSION);
  vf.close();
  return written == String(WEB_UI_VERSION).length();
}

static bool webUiReadVersionFile(String &outVersion)
{
  if (!SPIFFS.exists(WEB_UI_VERSION_PATH)) return false;
  File vf = SPIFFS.open(WEB_UI_VERSION_PATH, FILE_READ);
  if (!vf) return false;
  outVersion = vf.readString();
  vf.close();
  outVersion.trim();
  return outVersion.length() > 0;
}

static bool webUiInstallEmbedded()
{
  SPIFFS.remove(WEB_UI_INDEX_TMP_PATH);
  File tmp = SPIFFS.open(WEB_UI_INDEX_TMP_PATH, FILE_WRITE);
  if (!tmp) return false;
  const size_t written = tmp.print(WEB_UI_INDEX_HTML);
  tmp.close();
  if (written != WEB_UI_INDEX_HTML_LEN)
  {
    SPIFFS.remove(WEB_UI_INDEX_TMP_PATH);
    return false;
  }

  File verify = SPIFFS.open(WEB_UI_INDEX_TMP_PATH, FILE_READ);
  if (!verify)
  {
    SPIFFS.remove(WEB_UI_INDEX_TMP_PATH);
    return false;
  }
  const size_t fileSize = verify.size();
  const uint32_t fileHash = webUiFileFnv1a(verify);
  verify.close();
  const uint32_t expectedHash = webUiFnv1a((const uint8_t*)WEB_UI_INDEX_HTML, WEB_UI_INDEX_HTML_LEN);
  if (fileSize != WEB_UI_INDEX_HTML_LEN || fileHash != expectedHash)
  {
    SPIFFS.remove(WEB_UI_INDEX_TMP_PATH);
    return false;
  }

  SPIFFS.remove(WEB_UI_INDEX_PATH);
  if (!SPIFFS.rename(WEB_UI_INDEX_TMP_PATH, WEB_UI_INDEX_PATH))
  {
    SPIFFS.remove(WEB_UI_INDEX_TMP_PATH);
    return false;
  }
  return webUiWriteVersionFile();
}

String getTargetWebUiVersion()
{
  return String(WEB_UI_VERSION);
}

String getInstalledWebUiVersion()
{
  String installed;
  if (!webUiReadVersionFile(installed)) return "none";
  return installed;
}

bool forceUpdateWebUiInSPIFFS()
{
  return webUiInstallEmbedded();
}

bool ensureWebUiInSPIFFS()
{
  bool needsInstall = !SPIFFS.exists(WEB_UI_INDEX_PATH);
  String installedVersion;
  if (!needsInstall)
  {
    if (!webUiReadVersionFile(installedVersion) || installedVersion != WEB_UI_VERSION)
    {
      needsInstall = true;
    }
  }
  if (!needsInstall) return true;
  return webUiInstallEmbedded();
}

// ===== Config Persistence =====

void saveConfToSPIFFS(const confStruct& data) {
    // Convert struct to byte array
    uint8_t rawData[sizeof(confStruct)];
    memcpy(rawData, &data, sizeof(confStruct));

    // Base64 encode
    size_t encodedLen = 0;
    mbedtls_base64_encode(NULL, 0, &encodedLen, rawData, sizeof(confStruct));
    uint8_t* encodedData = new uint8_t[encodedLen];
    if (mbedtls_base64_encode(encodedData, encodedLen, &encodedLen, rawData, sizeof(confStruct)) != 0) {
        Serial.println("Base64 encoding failed");
        delete[] encodedData;
        return;
    }

    // Save to SPIFFS via temp file to prevent corruption on power loss
    File file = SPIFFS.open("/data.tmp", FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open temp file for writing");
        delete[] encodedData;
        return;
    }
    file.write(encodedData, encodedLen);
    file.close();
    SPIFFS.remove(CONF_FILE_PATH);
    SPIFFS.rename("/data.tmp", CONF_FILE_PATH);
    Serial.println("Struct saved to SPIFFS as Base64");
    Serial.println("Encoded Data: " + String((char*)encodedData));
    delete[] encodedData;
}

bool readConfFromSPIFFS(confStruct& data) {
    // Recover from interrupted atomic write
    if (!SPIFFS.exists(CONF_FILE_PATH) && SPIFFS.exists("/data.tmp")) {
        SPIFFS.rename("/data.tmp", CONF_FILE_PATH);
    }
    if (!SPIFFS.exists(CONF_FILE_PATH)) {
        Serial.println("File does not exist");
        return false;
    }

    // Read file from SPIFFS
    File file = SPIFFS.open(CONF_FILE_PATH, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
    }

    String encodedString = file.readString();
    Serial.println("Encoded Data Read: " + encodedString);
    file.close();

    // Decode Base64
    size_t decodedLen = 0;
    mbedtls_base64_decode(NULL, 0, &decodedLen, (const uint8_t*)encodedString.c_str(), encodedString.length());
    uint8_t* decodedData = new uint8_t[decodedLen];
    if (mbedtls_base64_decode(decodedData, decodedLen, &decodedLen, (const uint8_t*)encodedString.c_str(), encodedString.length()) != 0) {
        Serial.println("Base64 decoding failed");
        delete[] decodedData;
        return false;
    }

    if (decodedLen < sizeof(confStruct)) {
        Serial.println("Config data too short, corrupted?");
        delete[] decodedData;
        return false;
    }

    memcpy(&data, decodedData, sizeof(confStruct));
    delete[] decodedData;

    // Clamp cross-dependent fields before range validation
    String crossErr;
    if (!cfgValidateCrossField(data, crossErr)) {
        Serial.println("Config cross-validation failed: " + crossErr);
        return false;
    }

    // Validate config values against their ranges
    String validationErr;
    if (!validateConfig(data, validationErr)) {
        Serial.println("Config validation failed: " + validationErr);
        return false;
    }

    Serial.println("Struct successfully read from SPIFFS");
    return true;
}

void deleteConfFromSPIFFS() {
    SPIFFS.remove("/data.tmp");
    if (SPIFFS.remove(CONF_FILE_PATH)) {
        Serial.println("File deleted successfully");
    } else {
        Serial.println("Failed to delete file");
    }
}

void initSPIFFS()
{
  if(!SPIFFS.begin(false))
  {
    Serial.println("SPIFFS Mount Failed, Formatting Flash");
    spiffsFormatNotify(true);
    if (!SPIFFS.format())
    {
      Serial.println("FORMAT ERROR!");
      spiffsErrorHalt(1);
    }
    spiffsFormatNotify(false);
    Serial.println("Rebooting..");
    delay(200);
    esp_restart();
  }
  Serial.println("Done");
}

void getConfFromSPIFFS()
{
  #ifdef DELETE_SPIFFS_CONF_AT_STARTUP
  deleteConfFromSPIFFS();
  #endif

  Serial.println("Getting usr conf from SPIFFS...");
  if (readConfFromSPIFFS(usrConf))
  {
    if(SW_VERSION != usrConf.version)
    {
      Serial.println("Config version mismatch!");
      Serial.print("Config version: "); Serial.print(usrConf.version);
      Serial.print(", firmware version: "); Serial.println(SW_VERSION);
      Serial.println("Run ?clearspiffs + ?reboot to load defaults.");
      config_version_error = true;
    }
  }
  else
  {
    Serial.println("No conf in SPIFFS, writing default...");
    //Generate Device Address

    uint64_t mac = ESP.getEfuseMac(); // Get MAC from ESP32 eFuse
    uint8_t mac_address[6];
    for (int i = 0; i < 6; i++) {
        mac_address[i] = (mac >> (8 * i)) & 0xFF;
    }

    defaultConf.own_address[0] = esp_crc8(mac_address, 4); // Compute CRC8 over the first 4 bytes
    defaultConf.own_address[1] = mac_address[4];
    defaultConf.own_address[2] = mac_address[5];

    saveConfToSPIFFS(defaultConf);
    if (!readConfFromSPIFFS(usrConf))
    {
      Serial.println("Error writing default conf!");
      spiffsErrorHalt(2);
    }
  }
  Serial.println("... Done");
}

#endif // SPIFFS_ENGINE_H
