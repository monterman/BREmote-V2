#ifndef SYSTEM_COMMON_H
#define SYSTEM_COMMON_H

// V2.5-Evo - 2026-08-16 - serApplyConf() now reports an unambiguous pass/fail; serSetConf() pre-checks the pasted blob (length, SW version, validation) before anything is written to flash.
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

// V2.5-Evo - 2026-08-16 - ?applyconf now checks the loader's return value and says plainly whether
// the config was applied or rejected. It previously discarded the return value, so a user pasting a
// bad backup got no clear indication that anything had gone wrong. (The matching half of this fix is
// in SPIFFSEngine.h: readConfFromSPIFFS() no longer touches usrConf unless validation passes, so a
// "rejected" message now genuinely means the live config is unchanged.)
static void serApplyConf() {
  Serial.println("Reading conf from SPIFFS and applying to usrConf");
  if (readConfFromSPIFFS(usrConf)) {
    Serial.println("OK config applied — usrConf now holds the config stored in SPIFFS.");
  } else {
    Serial.println("ERR: config REJECTED — nothing was applied.");
    Serial.println("ERR: usrConf is unchanged; the board is still running the config it had before this command.");
    Serial.println("ERR: the reason is printed above (file missing / Base64 decode failed / too short / validation failed).");
  }
}

static void serClearConf() {
  Serial.println("Deleting conf from SPIFFS");
  deleteConfFromSPIFFS();
}

// V2.5-Evo - 2026-08-16 - ?setconf pre-check before anything reaches flash.
//
// WHAT THE BUG WAS: this wrote whatever string it was handed straight into the config file with no
// checking at all. A backup taken from a different SW version, or a mangled paste, was accepted
// silently and only failed much later — at ?applyconf or at the next boot — as "Config data too
// short, corrupted?". At that point getConfFromSPIFFS() re-bakes defaults, so the owner loses their
// settings a SECOND time and the serial log never says why.
//
// WHAT THE FIX DOES: decode the pasted blob here and refuse to store it unless it is the right size
// for THIS firmware's confStruct, carries THIS firmware's SW_VERSION, and passes the same two checks
// the loader applies. Each rejection names the actual mismatch. Nothing is written on rejection —
// the config already stored on flash is left exactly as it was.
//
// STACK COST: one confStruct staging copy (RX 192 bytes, TX 136) plus the heap decode buffer.
// ?setconf only ever arrives through checkSerial() from loop(), i.e. the Arduino loop task and its
// 8192-byte stack, never from one of the 2048-4096 byte FreeRTOS tasks.
static void serSetConf(String data) {
  Serial.print("Setting configuration to: ");
  Serial.println(data);

  // ---- Pre-check step 1: does it decode as Base64 at all? ----
  size_t decodedLen = 0;
  mbedtls_base64_decode(NULL, 0, &decodedLen, (const uint8_t*)data.c_str(), data.length());
  uint8_t* decodedData = new uint8_t[decodedLen];
  if (mbedtls_base64_decode(decodedData, decodedLen, &decodedLen, (const uint8_t*)data.c_str(), data.length()) != 0) {
    Serial.println("ERR: that is not valid Base64 — it could not be decoded.");
    Serial.println("ERR: nothing was written; the config stored in SPIFFS is unchanged.");
    delete[] decodedData;
    return;
  }

  // The 'version' field is the first member of confStruct on both TX and RX, stored little-endian,
  // so the SW version a blob was taken from can be read straight off the front of the decoded bytes.
  const uint16_t blobVersion = (decodedLen >= sizeof(uint16_t))
                                 ? (uint16_t)(decodedData[0] | ((uint16_t)decodedData[1] << 8))
                                 : 0;

  // ---- Pre-check step 2: is it the right size for this firmware? ----
  // sizeof(confStruct) grows whenever a config field is added, so a backup from another SW version
  // decodes to a different number of bytes. Report both numbers so the mismatch is obvious.
  if (decodedLen != sizeof(confStruct)) {
    Serial.print("ERR: this backup decodes to ");
    Serial.print((unsigned)decodedLen);
    Serial.print(" bytes, but ");
    Serial.print(SYS_DEVICE_LABEL);
    Serial.print(" SW");
    Serial.print(SW_VERSION);
    Serial.print(" needs exactly ");
    Serial.print((unsigned)sizeof(confStruct));
    Serial.println(" bytes.");
    if (blobVersion != 0) {
      Serial.print("ERR: the backup says it was taken from SW");
      Serial.print(blobVersion);
      Serial.println(".");
    }
    Serial.println("ERR: config backups are NOT compatible across SW versions.");
    Serial.println("ERR: nothing was written; the config stored in SPIFFS is unchanged.");
    delete[] decodedData;
    return;
  }

  // ---- Pre-check step 3: is it from this SW version? ----
  // A same-size blob from another version would be accepted here and then re-baked to defaults by
  // getConfFromSPIFFS() at the next boot — i.e. a silent wipe. Refuse it now, with the reason.
  if (blobVersion != SW_VERSION) {
    Serial.print("ERR: this backup is from SW");
    Serial.print(blobVersion);
    Serial.print("; this ");
    Serial.print(SYS_DEVICE_LABEL);
    Serial.print(" is running SW");
    Serial.print(SW_VERSION);
    Serial.println(".");
    Serial.println("ERR: config backups are NOT compatible across SW versions — this one would be discarded and replaced by factory defaults at the next boot.");
    Serial.println("ERR: nothing was written; the config stored in SPIFFS is unchanged.");
    delete[] decodedData;
    return;
  }

  // ---- Pre-check step 4: the same two checks the loader runs ----
  // Done on a throwaway staging copy: this is a gate, not a transform. The blob is stored exactly as
  // pasted, and cfgValidateCrossField() clamps at load time as it always has.
  confStruct staged;
  memcpy(&staged, decodedData, sizeof(confStruct));
  delete[] decodedData;

  String err;
  if (!cfgValidateCrossField(staged, err)) {
    Serial.println("ERR: cross-validation failed: " + err);
    Serial.println("ERR: nothing was written; the config stored in SPIFFS is unchanged.");
    return;
  }
  if (!validateConfig(staged, err)) {
    Serial.println("ERR: validation failed: " + err);
    Serial.println("ERR: nothing was written; the config stored in SPIFFS is unchanged.");
    return;
  }

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
  Serial.println("OK backup accepted and stored — run ?applyconf (or ?reboot) to make it live.");
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
