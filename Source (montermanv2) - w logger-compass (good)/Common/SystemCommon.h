#ifndef SYSTEM_COMMON_H
#define SYSTEM_COMMON_H

// Shared system utilities and command handlers for BREmote V2 TX and RX.
// Requirements before #include:
//   - confStruct type defined, usrConf + defaultConf globals declared
//   - CONF_FILE_PATH, SW_VERSION defined
//   - ConfigServiceEngine.h, SPIFFSEngine.h, WebConfigEngine.h included
//   - web_cfg_service_enabled global declared
//
// Each side must define:
//   const char* SYS_DEVICE_LABEL = "TX" or "RX";

// Forward declarations — defined per-side in System.ino.
extern const char* SYS_DEVICE_LABEL;

// ===== Pure Utilities =====

// Non-static to match forward declaration in SPIFFSEngine.h
uint8_t esp_crc8(uint8_t *data, uint8_t length) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void printHexArray(const uint8_t* buffer, size_t size) {
  if (buffer == nullptr || size == 0) {
    return;
  }
  for (size_t i = 0; i < size; i++) {
    if (buffer[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(buffer[i], HEX);
    if (i < size - 1) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

static void printHexArray16(const volatile uint16_t* buffer, size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (buffer[i] < 0x1000) {
      Serial.print("0");
    }
    if (buffer[i] < 0x100) {
      Serial.print("0");
    }
    if (buffer[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(buffer[i], HEX);
    if (i < size - 1) {
      Serial.print(" ");
    }
  }
  Serial.println();
}

// ===== Setup Helpers =====

static void enterSetup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("");
  Serial.println("Entering Setup...");

  Serial.println("**************************************");
  Serial.printf("**          BREmote V2 %s           **\n", SYS_DEVICE_LABEL);
  Serial.printf("**        MAC: %012llX         **\n", ESP.getEfuseMac());
  Serial.printf("**          SW Version: %-10d  **\n", SW_VERSION);
  Serial.printf("**  Compiled: %s %s  **\n", __DATE__, __TIME__);
  Serial.println("**************************************");
}

static void exitSetup() {
  Serial.println("");
  Serial.println("...Leaving Setup");
  Serial.println("");
}

static bool checkSerialQuit() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equals("quit")) {
      Serial.println("Stopping print loop.");
      return true;
    }
  }
  return false;
}

// ===== Config Print =====

static void printConfStruct(const confStruct &data) {
    Serial.println("Configuration Struct Values:");
    for (size_t i = 0; i < kCfgFieldCount; i++) {
      String val;
      if (cfgReadFieldValue(data, kCfgFields[i], val)) {
        Serial.print(kCfgFields[i].key);
        Serial.print(": ");
        Serial.println(val);
      }
    }
    Serial.println("----------------------");
}

// ===== Shared Serial Helpers =====

static void serApplyConf() {
  Serial.print("Reading conf from SPIFFS and applying to usrConf");
  readConfFromSPIFFS(usrConf);
}

static void serClearConf() {
  Serial.println("Deleting conf from SPIFFS");
  deleteConfFromSPIFFS();
}

static void serSetConf(String data) {
  Serial.print("Setting configuration to: ");
  Serial.println(data);

  uint8_t* encodedData = new uint8_t[data.length()];
  for (size_t i = 0; i < data.length(); i++) {
    encodedData[i] = data[i];
  }

  // Save to SPIFFS via temp file to prevent corruption on power loss
  File file = SPIFFS.open("/data.tmp", FILE_WRITE);
  if (!file) {
      Serial.println("Failed to open temp file for writing");
      delete[] encodedData;
      return;
  }
  file.write(encodedData, data.length());
  file.close();
  SPIFFS.remove(CONF_FILE_PATH);
  SPIFFS.rename("/data.tmp", CONF_FILE_PATH);
  Serial.println("Struct saved to SPIFFS as Base64");
  delete[] encodedData;
}

static void serPrintConf() {
  Serial.println("**************************************");
  Serial.printf("**          BREmote V2 %s           **\n", SYS_DEVICE_LABEL);
  Serial.printf("**        MAC: %012llX         **\n", ESP.getEfuseMac());
  Serial.printf("**          SW Version: %-10d  **\n", SW_VERSION);
  Serial.printf("**  Compiled: %s %s  **\n", __DATE__, __TIME__);
  Serial.println("**************************************");

  File file = SPIFFS.open(CONF_FILE_PATH, FILE_READ);
  if (!file) {
      Serial.println("Failed to open file for reading");
  } else {
      String encodedString = file.readString();
      Serial.println("Encoded Data Read: " + encodedString);
      file.close();
  }

  printConfStruct(usrConf);
}

// ===== Shared Command Handlers =====
// These can be referenced directly from per-side dispatch tables.

static void cmdConf(const String &args) {
  if (args.equalsIgnoreCase("json")) {
    String json;
    if (cfgGetAllJson(json)) {
      Serial.println(json);
    } else {
      Serial.println("ERR: Failed to generate JSON");
    }
  } else {
    serPrintConf();
  }
}

static void cmdGet(const String &args) {
  if(args.length() == 0) { Serial.println("ERR: usage: ?get <key>"); return; }
  String key = args;
  String val, err;
  if(cfgGetValueByKey(key, val, err))
  {
    Serial.print(key); Serial.print("="); Serial.println(val);
  }
  else
  {
    Serial.print("ERR: "); Serial.println(err);
  }
}

static void cmdSet(const String &args) {
  // Support both "key value" and "key=value" formats
  int spacePos = args.indexOf(' ');
  int eqPos = args.indexOf('=');
  String key, value;
  if (spacePos > 0 && (eqPos < 0 || spacePos < eqPos)) {
    key = args.substring(0, spacePos);
    value = args.substring(spacePos + 1);
  } else if (eqPos > 0) {
    key = args.substring(0, eqPos);
    value = args.substring(eqPos + 1);
  }
  key.trim();
  value.trim();
  if (key.length() > 0 && value.length() > 0) {
    String err;
    bool radioReinit = false;
    if(cfgSetValueByKey(key, value, err, radioReinit))
    {
      String readback, rerr;
      cfgGetValueByKey(key, readback, rerr);
      Serial.print("OK "); Serial.print(key); Serial.print("="); Serial.println(readback);
      if(radioReinit) Serial.println("NOTE: radio reinit required (?reboot)");
    }
    else
    {
      Serial.print("ERR: "); Serial.println(err);
    }
  } else {
    Serial.println("ERR: usage: ?set <key> <value> or ?set <key>=<value>");
  }
}

static void cmdKeys(const String &args) {
  for(size_t i = 0; i < kCfgFieldCount; i++)
  {
    Serial.println(kCfgFields[i].key);
  }
}

static void cmdSave(const String &args) {
  String err;
  if (!cfgValidateCrossField(usrConf, err)) {
    Serial.println("ERR: cross-validation failed: " + err);
    return;
  }
  if (!validateConfig(usrConf, err)) {
    Serial.println("ERR: validation failed: " + err);
    return;
  }
  saveConfToSPIFFS(usrConf);
  Serial.println("OK config saved to SPIFFS");
}

static void cmdWifi(const String &args) {
#ifdef WIFI_ENABLED
  if(args == "on")       { webCfgEnableService(); Serial.println("WiFi/AP config service enabled."); }
  else if(args == "off") { webCfgDisableService(); Serial.println("WiFi/AP config service disabled."); }
  else if(args == "")    { Serial.print("wifi="); Serial.println(web_cfg_service_enabled ? "ON" : "OFF"); }
  else                   Serial.println("ERR: usage: ?wifi on|off");
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

static void cmdWifiDbg(const String &args) {
#ifdef WIFI_ENABLED
  if(args == "") {
    Serial.print("wifidbg=");
    Serial.println(webCfgGetDebugModeName());
  }
  else {
    if(webCfgSetDebugMode(args))
    {
      Serial.print("wifidbg=");
      Serial.println(webCfgGetDebugModeName());
    }
    else
    {
      Serial.println("ERR_WIFIDBG_MODE");
    }
  }
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

static void cmdWifiPs(const String &args) {
#ifdef WIFI_ENABLED
  if(args == "") {
    Serial.print("wifips_ms=");
    Serial.println(webCfgGetStartupTimeoutMs());
  }
  else if(args.equalsIgnoreCase("off")) {
    if(webCfgSetStartupTimeoutMs(0)) Serial.println("wifips_ms=0");
    else Serial.println("ERR_WIFIPS_VALUE");
  }
  else {
    long ms = args.toInt();
    bool isDigitsOnly = args.length() > 0;
    for(size_t i = 0; i < args.length(); i++)
    {
      if(!isDigit((unsigned char)args[i])) { isDigitsOnly = false; break; }
    }
    if(!isDigitsOnly || ms < 0 || !webCfgSetStartupTimeoutMs((uint32_t)ms)) Serial.println("ERR_WIFIPS_VALUE");
    else { Serial.print("wifips_ms="); Serial.println(ms); }
  }
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

static void cmdWifiVer(const String &args) {
#ifdef WIFI_ENABLED
  Serial.print("ui_target=");
  Serial.println(getTargetWebUiVersion());
  Serial.print("ui_installed=");
  Serial.println(getInstalledWebUiVersion());
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

static void cmdWifiUpd(const String &args) {
#ifdef WIFI_ENABLED
  if(forceUpdateWebUiInSPIFFS())
  {
    Serial.print("UI updated to ");
    Serial.println(getTargetWebUiVersion());
  }
  else
  {
    Serial.println("ERR_UI_UPDATE");
  }
#else
  Serial.println("ERR: WiFi disabled at compile time");
#endif
}

static void cmdWifiState(const String &args) {
#ifdef WIFI_ENABLED
  Serial.println(webCfgGetStateLine());
#else
  Serial.println("wifi=disabled (WIFI_ENABLED undefined)");
#endif
}

static void cmdWifiErr(const String &args) {
#ifdef WIFI_ENABLED
  Serial.println(webCfgGetLastError());
#else
  Serial.println("wifi_err=none (WiFi not compiled)");
#endif
}

static void cmdReboot(const String &args) {
  Serial.println("Rebooting now...");
  delay(1000);
  ESP.restart();
}

#endif // SYSTEM_COMMON_H
