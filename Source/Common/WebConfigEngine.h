#ifndef WEB_CONFIG_ENGINE_H
#define WEB_CONFIG_ENGINE_H

#ifdef WIFI_ENABLED

// Shared web config AP and HTTP API for BREmote V2 TX and RX.
// V2.5-Evo - 2026-08-17 - SAFETY: the web portal's Base64 config import now runs cfgValidateCrossField() as well as validateConfig(), which every other intake path (?setconf, the SPIFFS boot loader, ?set, cfgSetBatch, the legacy-migration branch) already did. It was the only path that did not, and validateConfig() checks each field against its own range in isolation - so a blob with PWM0_min = 2500 and PWM0_max = 500, both individually legal, was accepted and inverted the throttle map into a constant full-throttle output at every throttle position including zero. Called before validateConfig() and on the staging copy, matching every other path, so the validator's auto-corrections are what gets range-checked and written. Existing ERR_IMPORT_VALIDATION_FAILED reused with the reason in the existing detail field. No confStruct change, no new error code, both boards.
// V2.5-Evo - 2026-08-17 - the web portal's Base64 config import now accepts a legacy backup on the same terms ?setconf does: a blob that is the one older SW version whose confStruct is a byte-exact prefix of this one is migrated instead of refused with ERR_IMPORT_INVALID_SIZE. It calls the SAME cfgBlobIsMigratableLegacy() / cfgMigrateLegacyBlob() helpers the serial path calls, so the two importers cannot disagree about which bytes go where. Everything that is not that exact (size, version) pair takes the identical path, and gets the identical error, it does today. Inert on the TX (compile-time constant is false there). No confStruct change.
// V2.5-Evo - 2026-07-25 - STAGE 0 PART B (RX only, inside ENABLE_WEB_LOG_DOWNLOAD — TX never compiles this block): the WiFi log download now reads the 8-byte self-describing log header before sending anything, steps records by header.record_size instead of sizeof(VescLogData), emits the CSV column header matching the level the file was actually recorded at, and rejects a header-less/old/corrupt file with a JSON error rather than a half-sent body. Row formatting moved to the single shared logFormatCsvRow() in BREmote_V2_Rx.h that the serial ?download path also calls, so the two CSV outputs can no longer drift apart. No TX impact, no confStruct change.
// V2.5-Evo - 2026-07-25 - F-WEBCSV: WiFi log download resynced to the serial ?download CSV (26->31 columns); ERPM x10 scaling and duty_cycle cast now match Logger.ino; row buffer 400->512
// V2.5-Evo - 2026-05-03 - Content-Disposition header on export (iPhone filename fix)
// V2.5-Evo - 2026-05-08 - Bundle 1: HTTP log download updated for 26-column CSV (+heading_error_dx10, d_error_dx10)
// V2.5-Evo - 2026-05-06 - FIX-LOGDL-1: log download CSV updated for LOG-EXT-1 fields (24 columns); WDT reset + FreeRTOS yield added inside read loop to support files >40KB without AP reboot

// Forward declarations — defined per-side in WebConfig.ino.
extern const char* WEB_CFG_AP_SSID;
extern const char* WEB_CFG_SHUTDOWN_REASON;
void webCfgResetCalibration(confStruct& conf);

// Forward declarations for functions defined later in this header.
String webCfgGetStateLine();
String webCfgGetLastError();

// ===== Internal State =====

static WebServer webCfgServer(80);
static const uint8_t WEB_CFG_DBG_OFF = 0;
static const uint8_t WEB_CFG_DBG_SOME = 1;
static const uint8_t WEB_CFG_DBG_FULL = 2;
static bool web_cfg_ap_started = false;
static bool web_cfg_ap_had_client = false;
static bool web_cfg_should_shutdown = false;
static uint8_t web_cfg_last_station_count = 0;
static uint32_t web_cfg_ap_started_at_ms = 0;
static String web_cfg_last_shutdown_reason = "";

// ===== Helpers =====

static void webCfgStopService(const char* reason)
{
  if(!web_cfg_ap_started) return;
  web_cfg_last_shutdown_reason = reason ? String(reason) : String("unknown");
  webCfgServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  web_cfg_ap_started = false;
  web_cfg_service_enabled = false;
  Serial.print("Web config AP stopped. Reason: ");
  Serial.println(web_cfg_last_shutdown_reason);
}

static const char* webCfgMethodName(HTTPMethod method)
{
  switch(method)
  {
    case HTTP_GET: return "GET";
    case HTTP_POST: return "POST";
    case HTTP_PUT: return "PUT";
    case HTTP_PATCH: return "PATCH";
    case HTTP_DELETE: return "DELETE";
    case HTTP_OPTIONS: return "OPTIONS";
    default: return "OTHER";
  }
}

static bool webCfgDebugEnabledFor(bool importantOnly)
{
  if (web_cfg_debug_mode == WEB_CFG_DBG_OFF) return false;
  if (web_cfg_debug_mode == WEB_CFG_DBG_FULL) return true;
  return importantOnly;
}

String webCfgGetDebugModeName()
{
  if (web_cfg_debug_mode == WEB_CFG_DBG_FULL) return "full";
  if (web_cfg_debug_mode == WEB_CFG_DBG_SOME) return "some";
  return "off";
}

bool webCfgSetDebugMode(const String& modeName)
{
  String m = modeName;
  m.trim();
  m.toLowerCase();
  if (m == "off" || m == "0")
  {
    web_cfg_debug_mode = WEB_CFG_DBG_OFF;
    return true;
  }
  if (m == "some" || m == "1")
  {
    web_cfg_debug_mode = WEB_CFG_DBG_SOME;
    return true;
  }
  if (m == "full" || m == "2")
  {
    web_cfg_debug_mode = WEB_CFG_DBG_FULL;
    return true;
  }
  return false;
}

static void webCfgLogReq(const char* action, const String& detail, bool importantOnly = false)
{
  if (!webCfgDebugEnabledFor(importantOnly)) return;
  Serial.print("[WEB] ");
  Serial.print(webCfgMethodName(webCfgServer.method()));
  Serial.print(" ");
  Serial.print(webCfgServer.uri());
  Serial.print(" | ");
  Serial.print(action);
  if(detail.length() > 0)
  {
    Serial.print(" | ");
    Serial.print(detail);
  }
  Serial.println();
}

static void webCfgMarkOk()
{
  web_cfg_req_total++;
  web_cfg_req_ok++;
}

static void webCfgMarkErr(const String& err)
{
  web_cfg_req_total++;
  web_cfg_req_err++;
  web_cfg_last_err = err;
}

static void webCfgSendJson(int code, const String& payload)
{
  webCfgServer.send(code, "application/json", payload);
}

// ===== HTTP Handlers =====

static void webCfgHandleRoot()
{
  webCfgLogReq("root", "", true);
  const char* webPath = "/index.html";
  if(!SPIFFS.exists(webPath))
  {
    webCfgMarkErr("ERR_UI_NOT_FOUND");
    webCfgSendJson(500, "{\"ok\":0,\"err\":\"ERR_UI_NOT_FOUND\"}");
    return;
  }

  File file = SPIFFS.open(webPath, FILE_READ);
  if(!file)
  {
    webCfgMarkErr("ERR_UI_OPEN");
    webCfgSendJson(500, "{\"ok\":0,\"err\":\"ERR_UI_OPEN\"}");
    return;
  }

  webCfgServer.streamFile(file, "text/html; charset=utf-8");
  file.close();
}

static void webCfgHandleState()
{
  webCfgLogReq("state", "");
  String json = "{\"ok\":1,\"state\":\"" + webCfgGetStateLine() + "\",\"last_err\":\"" + webCfgGetLastError() + "\"}";
  webCfgSendJson(200, json);
}

static void webCfgHandleGetAll()
{
  webCfgLogReq("config_get_all", "");
  String out;
  if(!cfgGetAllJson(out))
  {
    webCfgLogReq("config_get_all_err", "ERR_GET_ALL_FAILED");
    webCfgMarkErr("ERR_GET_ALL_FAILED");
    webCfgSendJson(500, "{\"ok\":0,\"err\":\"ERR_GET_ALL_FAILED\"}");
    return;
  }
  webCfgLogReq("config_get_all_ok", "");
  webCfgMarkOk();
  webCfgSendJson(200, "{\"ok\":1,\"data\":" + out + "}");
}

static void webCfgHandleGet()
{
  String key = webCfgServer.arg("key");
  webCfgLogReq("config_get", "key=" + key);
  String out;
  String err;
  if(!cfgGetValueByKey(key, out, err))
  {
    if(err.length() == 0) err = "ERR_GET_FAILED";
    webCfgLogReq("config_get_err", err);
    webCfgMarkErr(err);
    webCfgSendJson(400, "{\"ok\":0,\"err\":\"" + err + "\"}");
    return;
  }
  webCfgLogReq("config_get_ok", "key=" + key + ",value=" + out);
  webCfgMarkOk();
  webCfgSendJson(200, "{\"ok\":1,\"key\":\"" + key + "\",\"value\":\"" + out + "\"}");
}

static void webCfgHandleSet()
{
  String key = webCfgServer.arg("key");
  String value = webCfgServer.arg("value");
  webCfgLogReq("config_set", "key=" + key + ",value=" + value);
  String err;
  bool radioReinit = false;
  if(!cfgSetValueByKey(key, value, err, radioReinit))
  {
    if(err.length() == 0) err = "ERR_SET_FAILED";
    webCfgLogReq("config_set_err", err);
    webCfgMarkErr(err);
    webCfgSendJson(400, "{\"ok\":0,\"err\":\"" + err + "\"}");
    return;
  }

  web_cfg_pending_save = true;
  if(radioReinit) web_cfg_radio_reinit_required = true;

  webCfgLogReq("config_set_ok", radioReinit ? "radio_reinit_required=1" : "");
  webCfgMarkOk();
  String data = "{\"ok\":1,\"pending_save\":1";
  if(radioReinit) data += ",\"radio_reinit_required\":1";
  data += "}";
  webCfgSendJson(200, data);
}

static void webCfgHandleSetBatch()
{
  String payload = webCfgServer.arg("payload");
  String payloadInfo = "len=" + String(payload.length());
  if(payload.length() <= 120) payloadInfo += ",payload=" + payload;
  webCfgLogReq("config_set_batch", payloadInfo);
  String err;
  bool radioReinit = false;
  if(!cfgSetBatch(payload, err, radioReinit))
  {
    if(err.length() == 0) err = "ERR_SET_BATCH_FAILED";
    webCfgLogReq("config_set_batch_err", err);
    webCfgMarkErr(err);
    webCfgSendJson(400, "{\"ok\":0,\"err\":\"" + err + "\"}");
    return;
  }

  web_cfg_pending_save = true;
  if(radioReinit) web_cfg_radio_reinit_required = true;

  webCfgLogReq("config_set_batch_ok", radioReinit ? "radio_reinit_required=1" : "");
  webCfgMarkOk();
  String data = "{\"ok\":1,\"pending_save\":1";
  if(radioReinit) data += ",\"radio_reinit_required\":1";
  data += "}";
  webCfgSendJson(200, data);
}

static void webCfgHandleSave()
{
  webCfgLogReq("config_save", "", true);
  saveConfToSPIFFS(usrConf);
  web_cfg_pending_save = false;
  webCfgLogReq("config_save_ok", "", true);
  webCfgMarkOk();
  webCfgSendJson(200, "{\"ok\":1,\"saved\":1}");
}

static void webCfgHandleLoad()
{
  webCfgLogReq("config_load", "");
  if(!readConfFromSPIFFS(usrConf))
  {
    webCfgLogReq("config_load_err", "ERR_LOAD_FAILED");
    webCfgMarkErr("ERR_LOAD_FAILED");
    webCfgSendJson(500, "{\"ok\":0,\"err\":\"ERR_LOAD_FAILED\"}");
    return;
  }
  web_cfg_pending_save = false;
  webCfgLogReq("config_load_ok", "");
  webCfgMarkOk();
  webCfgSendJson(200, "{\"ok\":1,\"loaded\":1}");
}

static void webCfgHandleReboot()
{
  webCfgLogReq("device_reboot", "", true);
  webCfgMarkOk();
  webCfgSendJson(200, "{\"ok\":1,\"rebooting\":1}");
  
  // Forcefully close the web client so the phone receives the OK message
  webCfgServer.client().stop(); 
  
  // Give the Wi-Fi stack enough time to finish transmitting before killing the processor
  delay(500); 
  
  ESP.restart();
}

static void webCfgHandleExport()
{
  webCfgLogReq("config_export", "");

  String format = webCfgServer.arg("format");
  format.toLowerCase();

  if (format == "json")
  {
    String jsonOut;
    if (!cfgGetAllJson(jsonOut))
    {
      webCfgMarkErr("ERR_EXPORT_JSON_FAILED");
      webCfgSendJson(500, "{\"ok\":0,\"err\":\"ERR_EXPORT_JSON_FAILED\"}");
      return;
    }
    webCfgMarkOk();
    // Content-Disposition forces iOS Safari to download the file instead of
    // rendering it inline. filename includes SW_VERSION for easy identification.
    webCfgServer.sendHeader("Content-Disposition",
        "attachment; filename=\"bremote-config-sw" + String(SW_VERSION) + ".json\"");
    webCfgSendJson(200, "{\"ok\":1,\"format\":\"json\",\"data\":" + jsonOut + "}");
  }
  else
  {
    uint8_t rawData[sizeof(confStruct)];
    memcpy(rawData, &usrConf, sizeof(confStruct));

    size_t encodedLen = 0;
    mbedtls_base64_encode(NULL, 0, &encodedLen, rawData, sizeof(confStruct));
    uint8_t* encodedData = new uint8_t[encodedLen];
    mbedtls_base64_encode(encodedData, encodedLen, &encodedLen, rawData, sizeof(confStruct));

    String b64 = String((char*)encodedData);
    delete[] encodedData;

    webCfgMarkOk();
    webCfgSendJson(200, "{\"ok\":1,\"format\":\"base64\",\"data\":\"" + b64 + "\"}");
  }
}

static void webCfgHandleImport()
{
  webCfgLogReq("config_import", "");

  String format = webCfgServer.arg("format");
  format.toLowerCase();

  String data = webCfgServer.arg("data");
  String resetCal = webCfgServer.arg("reset_cal");
  String resetBind = webCfgServer.arg("reset_bind");

  if (data.length() == 0)
  {
    webCfgMarkErr("ERR_IMPORT_NO_DATA");
    webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_NO_DATA\"}");
    return;
  }

  bool doResetCal = (resetCal == "1");
  bool doResetBind = (resetBind == "1");
  // V2.5-Evo - 2026-08-17 - set when the pasted backup turned out to be the one older SW version
  // this firmware can still read and had to be migrated. It lives up here with the other two
  // response flags because the success response at the bottom of this handler is shared with the
  // JSON branch, which is outside the block that sets it.
  bool migratedFromLegacy = false;

  if (format == "json")
  {
    String err;
    bool radioReinit = false;
    if (!cfgSetBatch(data, err, radioReinit))
    {
      webCfgMarkErr("ERR_IMPORT_JSON_PARSE");
      webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_JSON_PARSE\",\"detail\":\"" + err + "\"}");
      return;
    }
    if (radioReinit) web_cfg_radio_reinit_required = true;
  }
  else
  {
    size_t decodedLen = 0;
    mbedtls_base64_decode(NULL, 0, &decodedLen, (const uint8_t*)data.c_str(), data.length());

    // Declared ahead of the legacy branch below so the migrated path and the ordinary path can both
    // fill it and then share every check that follows. Value-initialised for the same reason the
    // ?setconf path value-initialises its staging copy: what ends up here is written into usrConf,
    // PWM0_min lives in it, and it must not start life as whatever was on the stack. The ordinary
    // path below overwrites all of it anyway.
    confStruct newConf = {};

    // ============================================================
    // V2.5-Evo - 2026-08-17 - LEGACY BACKUP IMPORT (the web half of the ?setconf migration)
    //
    // WHAT THE BUG WAS: ?setconf learned to accept a backup from the one older SW version whose
    // confStruct is a byte-exact prefix of this one, but this handler did not. The same rider
    // restoring the same backup therefore got two different answers depending on whether they pasted
    // it into the serial console or into the board's own WiFi config page — here it was refused with
    // ERR_IMPORT_INVALID_SIZE, at exactly the moment the backup was needed: the upgrade flash that
    // re-bakes defaults and wipes throttle calibration, pairing and compass calibration.
    //
    // WHAT THE FIX DOES: nothing except reuse. Both the decision and the byte mapping stay in
    // cfgBlobIsMigratableLegacy() / cfgMigrateLegacyBlob() in SPIFFSEngine.h — the same two functions
    // ?setconf calls — so the two importers cannot drift into disagreeing about which bytes go where.
    // The argument for why this one (size, version) pair and ONLY this one may be reinterpreted is in
    // the LEGACY CONFIG BLOB MIGRATION block in that file, and is not repeated here.
    //
    // WHY IT SITS AHEAD OF THE SIZE CHECK: a legacy backup decodes to a different number of bytes by
    // definition, so the size check would refuse it first and the rider would never get this far.
    //
    // WHY THERE IS A LENGTH PRE-TEST: the authoritative gate is cfgBlobIsMigratableLegacy() below,
    // which needs the version stamped in the blob and therefore needs the blob decoded first.
    // Decoding early for EVERY import would change which error a non-legacy import gets — an
    // unparseable paste leaves decodedLen at 0 and is answered ERR_IMPORT_INVALID_SIZE today, not
    // ERR_IMPORT_BASE64_FAILED. Entering this block only at the legacy length keeps every other blob
    // on exactly the path, and exactly the error, it has today.
    //
    // INERT ON THE TX: kCfgLegacyMigrationSupported is a compile-time false there (136-byte struct,
    // SW27), so the whole block folds away and a TX owner sees today's behaviour byte for byte.
    // ============================================================
    if (kCfgLegacyMigrationSupported && decodedLen == (size_t)CFG_LEGACY_BLOB_BYTES)
    {
      // Its own length variable: mbedtls_base64_decode() overwrites the one it is handed, and the
      // unchanged checks further down still need the probed length if we fall through from here.
      size_t legacyLen = decodedLen;
      uint8_t* legacyData = new uint8_t[legacyLen];
      const bool decodeOk = (mbedtls_base64_decode(legacyData, legacyLen, &legacyLen,
                                                   (const uint8_t*)data.c_str(), data.length()) == 0);

      // 'version' is the first member of confStruct on both boards, stored little-endian, so the SW
      // version a backup was taken from reads straight off the front of the decoded bytes.
      const uint16_t blobVersion = (decodeOk && legacyLen >= sizeof(uint16_t))
                                     ? (uint16_t)(legacyData[0] | ((uint16_t)legacyData[1] << 8))
                                     : 0;

      if (decodeOk && cfgBlobIsMigratableLegacy(legacyLen, blobVersion))
      {
        String migErr;
        const bool migOk = cfgMigrateLegacyBlob(legacyData, legacyLen, blobVersion, newConf, migErr);
        delete[] legacyData;

        // A migrated backup earns no shortcut when it fails. cfgMigrateLegacyBlob() runs the same
        // cross-field and range validation every other config on this board goes through, and leaves
        // newConf untouched when it fails — so nothing reaches usrConf, nothing is queued for saving,
        // and the live config is exactly what it was before the request arrived.
        if (!migOk)
        {
          webCfgMarkErr("ERR_IMPORT_VALIDATION_FAILED");
          webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_VALIDATION_FAILED\",\"detail\":\"SW"
                              + String(CFG_LEGACY_BLOB_SW) + " backup could not be migrated: " + migErr + "\"}");
          return;
        }

        migratedFromLegacy = true;
        webCfgLogReq("config_import", "migrated from SW" + String(CFG_LEGACY_BLOB_SW));
      }
      else
      {
        // Legacy LENGTH but not the legacy pair — some other version stamped in it — or it does not
        // decode at all. Not migratable, so nothing here touched it: fall through and let the
        // unchanged checks below refuse it with exactly the error they give today.
        delete[] legacyData;
      }
    }

    if (!migratedFromLegacy)
    {
      if (decodedLen != sizeof(confStruct))
      {
        webCfgMarkErr("ERR_IMPORT_INVALID_SIZE");
        webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_INVALID_SIZE\"}");
        return;
      }

      uint8_t* decodedData = new uint8_t[decodedLen];
      if (mbedtls_base64_decode(decodedData, decodedLen, &decodedLen, (const uint8_t*)data.c_str(), data.length()) != 0)
      {
        delete[] decodedData;
        webCfgMarkErr("ERR_IMPORT_BASE64_FAILED");
        webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_BASE64_FAILED\"}");
        return;
      }

      memcpy(&newConf, decodedData, sizeof(confStruct));
      delete[] decodedData;
    }

    if (doResetCal)
    {
      webCfgResetCalibration(newConf);
      webCfgLogReq("config_import", "calibration reset");
    }

    if (doResetBind)
    {
      newConf.paired = 0;
      newConf.own_address[0] = 0;
      newConf.own_address[1] = 0;
      newConf.own_address[2] = 0;
      newConf.dest_address[0] = 0;
      newConf.dest_address[1] = 0;
      newConf.dest_address[2] = 0;
      webCfgLogReq("config_import", "binding reset");
    }

    if (newConf.version != SW_VERSION)
    {
      webCfgMarkErr("ERR_IMPORT_VERSION_MISMATCH");
      webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_VERSION_MISMATCH\",\"expected\":" + String(SW_VERSION) + ",\"got\":" + String(newConf.version) + "}");
      return;
    }

    // ============================================================
    // V2.5-Evo - 2026-08-17 - CROSS-FIELD VALIDATION ON THE BASE64 IMPORT PATH
    //
    // WHAT THE BUG WAS: this handler ran validateConfig() and nothing else. validateConfig() is a
    // table walk over kCfgFields that checks each field against its OWN min/max in isolation; it has
    // no way to see a relationship between two fields. The PWM0_max > PWM0_min rule lives only in
    // cfgValidateCrossField(), and this was the one intake path that never called it.
    //
    // WHY THAT WAS DANGEROUS: PWM0_min = 2500 with PWM0_max = 500 passes validateConfig(), because
    // 500 and 2500 are both inside the legal 500-2500 range - it is their ORDER that is illegal.
    // calcPWM() then evaluates constrain(map(thr, 0, 255, 2500, 500) + trim, 2500, 500), and Arduino's
    // constrain() with low > high returns low, so PWM0_time comes out 2500 us at EVERY throttle value
    // including zero. The neutral floor does not catch it either, because that floor assigns
    // PWM0_min - which IS 2500 here. motor_ramp_s only rate-limits the rise, so the output ramps to
    // full over about 0.75 s and stays there, and generatePWM() emits full throttle continuously as
    // soon as any TX packet satisfies the failsafe window. On a buggy that tows a person.
    //
    // WHAT THE FIX DOES: runs the same cross-field validator every other intake path already runs.
    // ?setconf, the SPIFFS boot loader, ?set, cfgSetBatch and the legacy-migration branch directly
    // above all call it; this handler was the outlier. Nothing new is being invented here - an
    // inconsistency is being removed.
    //
    // WHY IT RUNS BEFORE validateConfig(): identical ordering to every other path (readConfFromSPIFFS,
    // cfgMigrateLegacyBlob, the ?setconf pre-check). cfgValidateCrossField() takes a NON-CONST
    // reference and auto-corrects some fields rather than rejecting them - it raises a too-small
    // fm_engage_dist_m to the tow-rope floor and clears rtm_compass_required in COG-only mode, both
    // announced on Serial. Calling it first means the CORRECTED value is what validateConfig() then
    // range-checks and, below, what is copied into usrConf. Calling it second would have written a
    // corrected value that nothing had range-checked.
    //
    // It is called on newConf, the staging copy, exactly like the two checks around it - so a blob
    // that fails leaves the live usrConf untouched and nothing is queued for saving.
    //
    // The migrated path falls through here too and is validated a second time. That is deliberate and
    // harmless: cfgValidateCrossField() is idempotent, so an already-corrected value produces no
    // change and no repeated Serial notice - and the reset_cal / reset_bind edits above happen AFTER
    // the migration branch ran its own checks, so re-validating is the honest thing to do.
    //
    // Reuses the existing ERR_IMPORT_VALIDATION_FAILED code with the reason in the same "detail"
    // field this handler already uses, so no client needs to learn a new error. Like every other
    // string interpolated into these responses, cross-field messages must contain no double quote or
    // backslash - the JSON here is built by concatenation with no escaping.
    // ============================================================
    String crossErr;
    if (!cfgValidateCrossField(newConf, crossErr))
    {
      webCfgMarkErr("ERR_IMPORT_VALIDATION_FAILED");
      webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_VALIDATION_FAILED\",\"detail\":\"" + crossErr + "\"}");
      return;
    }

    String validationErr;
    if (!validateConfig(newConf, validationErr))
    {
      webCfgMarkErr("ERR_IMPORT_VALIDATION_FAILED");
      webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_IMPORT_VALIDATION_FAILED\",\"detail\":\"" + validationErr + "\"}");
      return;
    }

    memcpy(&usrConf, &newConf, sizeof(confStruct));
    web_cfg_pending_save = true;
  }

  webCfgMarkOk();
  String response = "{\"ok\":1,\"imported\":1";
  if (doResetCal) response += ",\"cal_reset\":1";
  if (doResetBind) response += ",\"bind_reset\":1";
  // V2.5-Evo - 2026-08-17 - say plainly that the backup was migrated, and name the one setting the
  // old version did not have. Same substance the ?setconf path prints, in this handler's own shape:
  // flags and numbers as sibling fields (as the version-mismatch error already does with
  // expected/got), and the human-readable sentence in "detail" (as ERR_IMPORT_VALIDATION_FAILED and
  // ERR_LOG_FORMAT already do). This handler builds its JSON by concatenation, so the text
  // deliberately contains no quotes or backslashes that would need escaping.
  if (migratedFromLegacy)
  {
    response += ",\"migrated\":1";
    response += ",\"migrated_from_sw\":" + String(CFG_LEGACY_BLOB_SW);
    response += ",\"migrated_to_sw\":" + String(SW_VERSION);
    response += ",\"detail\":\"This backup was taken from SW" + String(CFG_LEGACY_BLOB_SW);
    response += " and has been MIGRATED to SW" + String(SW_VERSION);
    response += ". Your settings came across: throttle calibration, pairing, compass calibration and all the tuning values.";
    response += " Exactly one setting could not come from the backup, because SW" + String(CFG_LEGACY_BLOB_SW);
    response += " did not have it: the compass mounting orientation (mag_orientation) is set to 0, meaning no rotation.";
    response += " 0 is exactly how SW" + String(CFG_LEGACY_BLOB_SW);
    response += " behaved, so nothing changes unless your compass module is physically mounted turned.";
    response += " If it IS mounted turned, run ?compasscal afterwards (or ?magalign, which sets just this one setting).\"";
  }
  response += "}";
  webCfgSendJson(200, response);
}

// --- Log Management Endpoints ---
#ifdef ENABLE_WEB_LOG_DOWNLOAD
static void webCfgHandleListLogs()
{
  webCfgLogReq("logs_list", "");
  File root = SPIFFS.open("/");
  String json = "{\"ok\":1,\"logs\":[";
  bool first = true;
  File file = root.openNextFile();
  while(file)
  {
    String fname = String(file.name());
    if(fname.endsWith(".log"))
    {
      if(!first) json += ",";
      json += "{\"name\":\"" + fname + "\",\"size\":" + String(file.size()) + "}";
      first = false;
    }
    file = root.openNextFile();
  }
  json += "]}";
  webCfgSendJson(200, json);
}

static void webCfgHandleDownloadLog()
{
  String fname = webCfgServer.arg("file");
  if(!fname.startsWith("/")) fname = "/" + fname;
  webCfgLogReq("logs_download", fname);

  if(!SPIFFS.exists(fname))
  {
    webCfgSendJson(404, "{\"ok\":0,\"err\":\"Not Found\"}");
    return;
  }
  File file = SPIFFS.open(fname, FILE_READ);
  if(!file)
  {
    webCfgSendJson(500, "{\"ok\":0,\"err\":\"Failed to open\"}");
    return;
  }

  // ============================================================
  // V2.5-Evo - 2026-07-25 - STAGE 0 PART B: read the self-describing file header FIRST.
  //
  // Records are no longer a fixed size — a level-4 file writes 65-byte records where a level-3
  // file writes 59 — so the reader must take the size from the FILE rather than assume
  // sizeof(VescLogData). This runs BEFORE the chunked 200 response is opened, so an unreadable
  // file can still be answered with a proper JSON error instead of a half-sent CSV body.
  //
  // Files written before this change have no header (their first 4 bytes are a millis()
  // timestamp), so the magic test rejects them cleanly. Those files were already undecodable
  // after the 53 -> 59 byte record change (F9, 2026-07-24).
  // ============================================================
  LogFileHeader hdr;
  bool hdrOk = (file.size() >= sizeof(LogFileHeader)) &&
               (file.read((uint8_t*)&hdr, sizeof(hdr)) == sizeof(hdr)) &&
               (hdr.magic == LOG_FILE_MAGIC) &&
               (hdr.format_ver == LOG_FILE_FORMAT_VER) &&
               (hdr.record_size >= (uint16_t)sizeof(VescLogData)) &&
               (hdr.record_size <= (uint16_t)sizeof(VescLogDataL4));
  if(!hdrOk)
  {
    file.close();
    webCfgMarkErr("ERR_LOG_FORMAT");
    webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_LOG_FORMAT\",\"detail\":\"missing or unsupported BRLG log header - this file predates the self-describing log format or is corrupt\"}");
    return;
  }

  // Use Chunked transfer so we don't run out of memory converting the binary to text!
  webCfgServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webCfgServer.sendHeader("Content-Disposition", "attachment; filename=\"" + fname.substring(1) + ".csv\"");
  webCfgServer.send(200, "text/csv", "");

  // V2.5-Evo - 2026-07-25 - F-WEBCSV: header resynced to the serial ?download header.
  // The bug: this WiFi path was left on the 2026-05-08 Bundle-1 26-column format. It never picked up
  // the 2026-07-19 FM-triage columns (remote_error, effective_steer) nor the 2026-07-24 F9 columns
  // (tx_distance_m, rssi_dbm, snr_db), so every CSV pulled over WiFi silently dropped 5 columns even
  // though those fields ARE written into the binary records on SPIFFS. Anyone who downloads logs over
  // WiFi (the normal workflow) lost the distance and LoRa link-quality data entirely.
  // V2.5-Evo - 2026-07-25 - STAGE 0: that resync was a hand-copied duplicate of the serial string,
  // which is exactly how the drift happened in the first place. The column list now lives ONCE, as
  // LOG_CSV_HEADER_L3 / _L4 in BREmote_V2_Rx.h, and both readers emit the same macro — so they
  // cannot diverge again. The only difference is the trailing "\n" this chunked path must add
  // because Serial.println() supplies its own line ending.
  // The header emitted matches the level the file was RECORDED at (from its own header), not the
  // level the config happens to be set to now.
  String header = String((hdr.log_level >= 4) ? LOG_CSV_HEADER_L4 : LOG_CSV_HEADER_L3) + "\n";
  webCfgServer.sendContent(header);

  // V2.5-Evo - 2026-07-25 - STAGE 0: raw record buffer, sized for the largest record this
  // firmware understands (level 4). The file header guarantees hdr.record_size fits in it.
  uint8_t rec_buf[sizeof(VescLogDataL4)];
  // V2.5-Evo - 2026-07-25 - F-WEBCSV: row buffer resized 400 -> 512 for the 31-column CSV.
  // V2.5-Evo - 2026-07-25 - STAGE 0: the buffer size and the sizing arithmetic behind it now live
  // once, as LOG_CSV_ROW_BUF in BREmote_V2_Rx.h (640 B: the ~282 B pathological level-3 row plus
  // the ~20 B level-4 block, with margin). It is a stack local in the Arduino loop task (8 KB).
  char row[LOG_CSV_ROW_BUF];
  uint16_t recordCount = 0;
  while (file.available())
  {
    // V2.5-Evo - 2026-05-06 - FIX-LOGDL-1: feed WDT inside loop and yield to FreeRTOS.
    // Without these, files >~40KB cause WDT (3s timeout) to fire mid-download because the
    // WiFi-bound sendContent() per-record latency accumulates past the timeout.
    // Andres confirmed crash on 58.6KB log without these fixes.
    esp_task_wdt_reset();

    // Step by the size THIS file declares. A short read means the tail is truncated (power cut
    // mid-write): stop cleanly rather than formatting a partial record.
    size_t bytesRead = file.read(rec_buf, (size_t)hdr.record_size);
    if (bytesRead == (size_t)hdr.record_size)
    {
      // V2.5-Evo - 2026-07-25 - STAGE 0: one shared formatter, called by the serial ?download
      // path too (Logger.ino). Same fields, same order, same specifiers, same scaling, same N/A
      // sentinels — by construction rather than by careful copying. The two earlier silent
      // divergences this path had (ERPM emitted 10x too low because VescLogData.ERPM stores
      // ERPM/10, and an inconsistent duty_cycle cast) are gone with the duplicate code.
      logFormatCsvRow(row, sizeof(row), rec_buf, hdr.record_size, hdr.log_level);
      webCfgServer.sendContent(row);

      // Yield to FreeRTOS every 50 records to keep the WiFi stack and other tasks responsive.
      if ((++recordCount % 50) == 0) {
        delay(1);
      }
    }
    else
    {
      break;
    }
  }
  file.close();
  webCfgServer.sendContent(""); // Empty chunk closes the connection
}

static void webCfgHandleDeleteLog()
{
  String fname = webCfgServer.arg("file");
  if(!fname.startsWith("/")) fname = "/" + fname;
  webCfgLogReq("logs_delete", fname);

  if(SPIFFS.remove(fname))
  {
    webCfgSendJson(200, "{\"ok\":1}");
  }
  else
  {
    webCfgSendJson(500, "{\"ok\":0,\"err\":\"Delete failed\"}");
  }
}

// Forward declaration — defined in Logger.ino; visible after Arduino links all .ino files.
void deleteAllLogFiles();

static void webCfgHandleDeleteAllLogs()
{
  webCfgLogReq("logs_delete_all", "");
  deleteAllLogFiles();
  webCfgSendJson(200, "{\"ok\":1}");
}
#endif


// ==============================================================================================
// WEB SERIAL CONSOLE - Phase 2 routes
// ==============================================================================================
// V2.5-Evo - 2026-08-24 - WEB-SERIAL-2. Two routes onto the ring buffer that Common/SerialTee.h
// already fills. Guarded on SERIAL_TEE_RING_SIZE so the TX, which does not include the tee, is
// completely unaffected - the routes do not exist there rather than existing and failing.
//
// Polling, not WebSocket: this is the synchronous WebServer, one client at a time, and a poll
// matches it. No second server, no async rewrite, no third-party library.
#ifdef SERIAL_TEE_RING_SIZE

// Defined in the RX's System.ino. Arduino compiles the sketch as ONE translation unit and
// concatenates the .ino files after this header, so the declaration is what makes the call legal
// - the same pattern headingDisagreeLatched() uses.
void executeSerialCommand(const String &line);

// Also forward-declared: SystemCommon.h is included AFTER this header (BREmote_V2_Rx.h line
// ~1325 vs ~1158), so requestSerialQuit() is not visible here yet. Same one-translation-unit
// reasoning as above.
static inline void requestSerialQuit();

// JSON-escape a block of captured serial text. Control characters MUST be escaped or the browser
// silently fails to parse the whole response - the console would look dead with no error anywhere.
static void webCfgAppendJsonEscaped(String &dst, const uint8_t *src, size_t n)
{
  for (size_t i = 0; i < n; i++) {
    const char c = (char)src[i];
    switch (c) {
      case '"':  dst += "\\\""; break;
      case '\\': dst += "\\\\"; break;
      case '\n': dst += "\\n";  break;
      case '\r': dst += "\\r";  break;
      case '\t': dst += "\\t";  break;
      default:
        if ((uint8_t)c < 0x20 || (uint8_t)c == 0x7F) {
          char esc[7];
          snprintf(esc, sizeof(esc), "\\u%04X", (unsigned)(uint8_t)c);
          dst += esc;
        } else {
          dst += c;
        }
    }
  }
}

// GET /api/serial/read?since=<cursor>
//
// ⚠️ DELIBERATELY DOES NOT CALL webCfgLogReq(). Every other handler logs itself to Serial - but
// Serial now feeds the ring this route is reading. Logging here would append a line per poll, the
// next poll would return it, and at two polls a second the console would fill with a transcript
// of itself and evict the boot log. The one route that must stay silent.
static void webCfgHandleSerialRead()
{
  static uint8_t buf[1536];          // static: 1.5 KB does not belong on the loop task's stack

  uint32_t cursor;
  if (webCfgServer.hasArg("since")) {
    cursor = (uint32_t)strtoul(webCfgServer.arg("since").c_str(), NULL, 10);
  } else {
    cursor = serialTeeTail();        // no cursor = start from the oldest byte still held
  }

  bool   gap = false;
  uint32_t newCursor = cursor;
  const size_t n = serialTeeRead(cursor, buf, sizeof(buf), newCursor, gap);

  String out;
  out.reserve(n * 2 + 96);           // worst case is escaping; one allocation beats many
  out  = "{\"ok\":1,\"cursor\":";
  out += String((unsigned long)newCursor);
  out += ",\"head\":";
  out += String((unsigned long)serialTeeHead());
  out += ",\"gap\":";
  out += gap ? "1" : "0";
  out += ",\"data\":\"";
  webCfgAppendJsonEscaped(out, buf, n);
  out += "\"}";

  webCfgSendJson(200, out);
}

// POST /api/serial/write   body/arg: cmd=<the command line>
static void webCfgHandleSerialWrite()
{
  String cmd = webCfgServer.arg("cmd");
  cmd.trim();

  if (cmd.length() == 0) {
    webCfgSendJson(400, "{\"ok\":0,\"err\":\"ERR_EMPTY\"}");
    return;
  }

  // "quit" is NOT a command in the table - it is an interrupt aimed at a handler that is already
  // running and holding the loop task. Dispatching it would do nothing; the flag is what the
  // dozen streaming commands actually poll. See requestSerialQuit() in SystemCommon.h.
  if (cmd == "quit" || cmd == "?quit") {
    requestSerialQuit();
    webCfgLogReq("serial_quit", "");
    webCfgSendJson(200, "{\"ok\":1,\"quit\":1}");
    return;
  }

  // ⚠️ FORBIDDEN BY NAME, not merely discouraged. ?download streams a whole log file - megabytes -
  // through Serial, which now feeds an 8 KB ring. It would evict the entire boot log and every
  // command result with it, destroying the one feature this console exists to provide. The web UI
  // already has /api/logs/download, which streams straight to the browser and never touches the
  // ring.
  String bare = cmd;
  if (bare.startsWith("?")) bare = bare.substring(1);
  int sp = bare.indexOf(' ');
  if (sp > 0) bare = bare.substring(0, sp);
  bare.toLowerCase();
  if (bare == "download") {
    webCfgSendJson(403,
      "{\"ok\":0,\"err\":\"ERR_FORBIDDEN\",\"msg\":\"?download would flood the console buffer and "
      "evict the boot log. Use the log download button instead.\"}");
    return;
  }

  webCfgLogReq("serial_cmd", cmd);
  executeSerialCommand(cmd);         // the SAME dispatcher the USB path uses, guards and all
  webCfgSendJson(200, "{\"ok\":1}");
}

#endif // SERIAL_TEE_RING_SIZE

static void webCfgHandleNotFound()
{
  webCfgLogReq("not_found", "");
  webCfgMarkErr("ERR_NOT_FOUND");
  webCfgSendJson(404, "{\"ok\":0,\"err\":\"ERR_NOT_FOUND\"}");
}

// ===== Public API =====

void webCfgInit()
{
  if(web_cfg_service_enabled) return;

  WiFi.mode(WIFI_AP);
  char ap_pass[9];
  memcpy(ap_pass, usrConf.wifi_password, 8);
  ap_pass[8] = '\0';
  if(!WiFi.softAP(WEB_CFG_AP_SSID, ap_pass))
  {
    web_cfg_last_err = "ERR_AP_START";
    Serial.println("Web config AP start failed.");
    return;
  }

  Serial.print("Web config AP started. SSID: ");
  Serial.println(WEB_CFG_AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Web debug mode: ");
  Serial.println(webCfgGetDebugModeName());

  webCfgServer.on("/", HTTP_GET, webCfgHandleRoot);
  webCfgServer.on("/api/state", HTTP_GET, webCfgHandleState);
  webCfgServer.on("/api/config", HTTP_GET, webCfgHandleGetAll);
  webCfgServer.on("/api/config/export", HTTP_GET, webCfgHandleExport);
  webCfgServer.on("/api/config/import", HTTP_POST, webCfgHandleImport);
  webCfgServer.on("/api/get", HTTP_GET, webCfgHandleGet);
  webCfgServer.on("/api/set", HTTP_POST, webCfgHandleSet);
  webCfgServer.on("/api/set_batch", HTTP_POST, webCfgHandleSetBatch);
  webCfgServer.on("/api/save", HTTP_POST, webCfgHandleSave);
  webCfgServer.on("/api/load", HTTP_POST, webCfgHandleLoad);
  webCfgServer.on("/api/reboot", HTTP_POST, webCfgHandleReboot);
  
#ifdef ENABLE_WEB_LOG_DOWNLOAD
#ifdef SERIAL_TEE_RING_SIZE
  webCfgServer.on("/api/serial/read",  HTTP_GET,  webCfgHandleSerialRead);
  webCfgServer.on("/api/serial/write", HTTP_POST, webCfgHandleSerialWrite);
#endif
  webCfgServer.on("/api/logs/list", HTTP_GET, webCfgHandleListLogs);
  webCfgServer.on("/api/logs/download", HTTP_GET, webCfgHandleDownloadLog);
  webCfgServer.on("/api/logs/delete", HTTP_POST, webCfgHandleDeleteLog);
  webCfgServer.on("/api/logs/delete_all", HTTP_POST, webCfgHandleDeleteAllLogs);
#endif

  webCfgServer.onNotFound(webCfgHandleNotFound);
  webCfgServer.begin();

  web_cfg_ap_started = true;
  web_cfg_ap_had_client = false;
  web_cfg_should_shutdown = false;
  web_cfg_last_station_count = 0;
  web_cfg_ap_started_at_ms = millis();
  web_cfg_last_shutdown_reason = "";
  web_cfg_service_enabled = true;
}

void webCfgLoop()
{
  if(!web_cfg_ap_started) return;
  webCfgServer.handleClient();

  const uint8_t stationCount = WiFi.softAPgetStationNum();
  if(stationCount > 0)
  {
    web_cfg_ap_had_client = true;
  }
  if(stationCount != web_cfg_last_station_count)
  {
    Serial.print("Web config client count: ");
    Serial.println(stationCount);
    web_cfg_last_station_count = stationCount;
  }

  if(web_cfg_should_shutdown)
  {
    webCfgStopService(WEB_CFG_SHUTDOWN_REASON);
    return;
  }

  if(!web_cfg_ap_had_client && web_cfg_ap_startup_timeout_ms > 0)
  {
    const uint32_t elapsedMs = millis() - web_cfg_ap_started_at_ms;
    if(elapsedMs >= web_cfg_ap_startup_timeout_ms)
    {
      webCfgStopService("startup_no_client_timeout");
      return;
    }
  }
}

String webCfgGetStateLine()
{
  String line = "enabled=";
  line += web_cfg_service_enabled ? "1" : "0";
  line += ",pending_save=";
  line += web_cfg_pending_save ? "1" : "0";
  line += ",radio_reinit_required=";
  line += web_cfg_radio_reinit_required ? "1" : "0";
  line += ",req_total=" + String(web_cfg_req_total);
  line += ",req_ok=" + String(web_cfg_req_ok);
  line += ",req_err=" + String(web_cfg_req_err);
  if(web_cfg_ap_started)
  {
    line += ",ap_clients=" + String(WiFi.softAPgetStationNum());
    line += ",ap_had_client=" + String(web_cfg_ap_had_client ? 1 : 0);
    line += ",startup_timeout_ms=" + String(web_cfg_ap_startup_timeout_ms);
    line += ",ap_uptime_ms=" + String(millis() - web_cfg_ap_started_at_ms);
  }
  if(web_cfg_last_shutdown_reason.length() > 0)
  {
    line += ",last_shutdown=" + web_cfg_last_shutdown_reason;
  }
  return line;
}

String webCfgGetLastError()
{
  return web_cfg_last_err;
}

uint32_t webCfgGetStartupTimeoutMs()
{
  return web_cfg_ap_startup_timeout_ms;
}

bool webCfgSetStartupTimeoutMs(uint32_t timeoutMs)
{
  if(timeoutMs > 3600000UL) return false;
  web_cfg_ap_startup_timeout_ms = timeoutMs;
  return true;
}

void webCfgEnableService()
{
  web_cfg_should_shutdown = false;
  webCfgInit();
}

void webCfgDisableService()
{
  web_cfg_should_shutdown = true;
  if(web_cfg_ap_started)
  {
    webCfgStopService("serial_wifioff");
    return;
  }
  WiFi.mode(WIFI_OFF);
  web_cfg_service_enabled = false;
}

#endif // WIFI_ENABLED

#endif // WEB_CONFIG_ENGINE_H