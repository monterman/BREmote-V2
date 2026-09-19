#ifndef SPIFFS_ENGINE_H
#define SPIFFS_ENGINE_H

// V2.5-Evo - 2026-09-19 - The legacy pair moves one step: an SW35 (192-byte) RX config is migrated onto the SW36 (200-byte) struct, because SW35 is a byte-exact prefix of SW36 (three u16 fields appended, plus tail padding). The byte mapping is now the pure cfgPrefixMigrateImage() in Common/ConfigMigrate.h (host-tested on the owner's real SW35 image in Tools/tests/config_migrate_test.cpp): prefix from the blob, tail from defaultConf, so the appended fields take their real defaults (1 / 13 / 80) rather than zero. And the migration now ALSO runs on the BOOT PATH: readConfFromSPIFFS() migrates a stored legacy blob, saves the migrated struct and continues with it, so the SW35 -> SW36 flash does NOT reset the config. An SW34 (184-byte) blob is no longer migratable (the path is exactly one version back, as before). Inert on the TX.
// V2.5-Evo - 2026-08-16 - Legacy config-blob migration: an SW34 (184-byte) RX backup is now accepted and migrated onto the SW35 (192-byte) struct, because SW34 is a byte-exact prefix of SW35. Gated on that exact size/version pair only; inert on the TX.
// V2.5-Evo - 2026-08-16 - readConfFromSPIFFS() stages the decoded blob in a local copy and only writes it into the caller's struct AFTER validation passes; a rejected config no longer runs the board.
// V2.5-Evo - 2026-07-21 - Stale-config trap fix (shared TX/RX): getConfFromSPIFFS() now re-bakes defaults on a SAME-SIZE SW_VERSION mismatch instead of running on stale config bytes. Dormant when versions match — no wipe on a same-version reflash.
// V2.5-Evo - 2026-04-30 - WebUI auto-reinstall via FNV1a content hash; removed WEB_UI_VERSION date string
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

// V2.5-Evo - 2026-09-19 - the pure byte mapping of the legacy-config migration (host-tested).
#include "ConfigMigrate.h"

// V2.5-Evo - 2026-09-19 - defined further down in this file (the LEGACY CONFIG BLOB MIGRATION
// block); declared here because readConfFromSPIFFS() now calls them on the boot path.
bool cfgBlobIsMigratableLegacy(size_t decodedLen, uint16_t blobVersion);
bool cfgMigrateLegacyBlob(const uint8_t* blob, size_t decodedLen, uint16_t blobVersion,
                          confStruct& out, String& err);

// ===== WebUI Embedding =====

#include "WebUiEmbedded.h"

static const char* WEB_UI_INDEX_PATH = "/index.html";
static const char* WEB_UI_INDEX_TMP_PATH = "/index.new";
static const char* WEB_UI_VERSION_PATH = "/ui.version";
// V2.5-Evo - 2026-04-30 - WebUI version tracking replaced with automatic FNV1a content hash.
// WEB_UI_VERSION date string removed. webUiWriteVersionFile() now writes the hash of
// WEB_UI_INDEX_HTML to /ui.version. ensureWebUiInSPIFFS() compares that hash against
// a freshly computed hash of the embedded HTML on every boot. Any change to
// WebUiEmbedded.h changes the hash, triggers automatic reinstall. No manual bump needed.

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

// Writes the FNV1a hash of the embedded HTML to /ui.version as an 8-char hex string.
// Called after every successful WebUI install. The stored hash is compared on next boot
// to detect whether WebUiEmbedded.h has changed since the last install.
static bool webUiWriteVersionFile()
{
  const uint32_t hash = webUiFnv1a((const uint8_t*)WEB_UI_INDEX_HTML, WEB_UI_INDEX_HTML_LEN);
  char hashStr[9];  // 8 hex chars + null terminator
  snprintf(hashStr, sizeof(hashStr), "%08X", hash);
  File vf = SPIFFS.open(WEB_UI_VERSION_PATH, FILE_WRITE);
  if (!vf) return false;
  const size_t written = vf.print(hashStr);
  vf.close();
  return written == 8;
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

// Returns the FNV1a hash of the currently embedded HTML as an 8-char hex string.
// Used for serial diagnostics — shows what version would be installed on next mismatch.
String getTargetWebUiVersion()
{
  const uint32_t hash = webUiFnv1a((const uint8_t*)WEB_UI_INDEX_HTML, WEB_UI_INDEX_HTML_LEN);
  char hashStr[9];
  snprintf(hashStr, sizeof(hashStr), "%08X", hash);
  return String(hashStr);
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

// Checks whether the WebUI stored in SPIFFS matches the embedded HTML in WebUiEmbedded.h.
// Comparison is done via FNV1a hash — any change to WebUiEmbedded.h changes the hash,
// which triggers automatic reinstall on next boot. No manual version bump ever needed.
bool ensureWebUiInSPIFFS()
{
  // Compute hash of the currently embedded HTML (runs once at boot, ~1-2ms)
  const uint32_t currentHash = webUiFnv1a((const uint8_t*)WEB_UI_INDEX_HTML, WEB_UI_INDEX_HTML_LEN);
  char currentHashStr[9];
  snprintf(currentHashStr, sizeof(currentHashStr), "%08X", currentHash);

  // Reinstall if the page file is missing entirely
  bool needsInstall = !SPIFFS.exists(WEB_UI_INDEX_PATH);
  if (!needsInstall)
  {
    // Reinstall if stored hash doesn't match current HTML hash
    String storedHash;
    if (!webUiReadVersionFile(storedHash) || storedHash != String(currentHashStr))
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
    #if defined(DEBUG_RX) || defined(DEBUG_TX)
    Serial.println("Encoded Data: " + String((char*)encodedData));
    #endif
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

    // ============================================================
    // V2.5-Evo - 2026-09-19 - BOOT-PATH MIGRATION (SW35 192 B -> SW36 200 B, and only that pair).
    //
    // WHAT WAS WRONG: the legacy migration below existed only for ?setconf and the web import. On
    // the boot path a stored blob one version old fell into the "too short" refusal, the caller
    // re-baked defaults, and the rider lost pairing, compass calibration and every tuning value on
    // the first boot after a struct-growing flash - exactly the 2026-08-29 RX1 config loss. The
    // remedy was "restore the backup with ?setconf afterwards", which only works if a backup exists.
    //
    // WHAT THIS DOES: when the stored blob is EXACTLY the one legacy (size, version) pair this build
    // migrates from, migrate it here - the same cfgMigrateLegacyBlob() the two importers call, with
    // the same full validation - save the migrated struct back to SPIFFS in this firmware's shape
    // (so the next boot reads it directly), hand it to the caller, and print ONE NOTE line. Anything
    // that is not that exact pair takes the identical path, and gets the identical refusal, it did
    // before. A migrated blob that fails validation is refused like any other bad blob and the
    // caller falls back as today. Inert on the TX (kCfgLegacyMigrationSupported is false there).
    // ============================================================
    if (decodedLen >= sizeof(uint16_t) && decodedLen < sizeof(confStruct)) {
        // The version field is the first member of confStruct on both boards, stored little-endian.
        const uint16_t blobVersion = (uint16_t)(decodedData[0] | ((uint16_t)decodedData[1] << 8));
        if (cfgBlobIsMigratableLegacy(decodedLen, blobVersion)) {
            confStruct migrated = {};
            String migErr;
            const bool migOk = cfgMigrateLegacyBlob(decodedData, decodedLen, blobVersion, migrated, migErr);
            delete[] decodedData;
            if (!migOk) {
                Serial.println("Config migration failed: " + migErr);
                Serial.println("Config REJECTED — live config left untouched.");
                return false;
            }
            saveConfToSPIFFS(migrated);
            data = migrated;
            Serial.printf("NOTE: config migrated SW%u (%u B) -> SW%u (%u B) by prefix - every stored value kept; the fields SW%u added take their factory defaults (fm_return_mode 1, fm_align_cap 13, fm_align_influence 80) and the result is saved.\n",
                          (unsigned)blobVersion, (unsigned)decodedLen, (unsigned)SW_VERSION,
                          (unsigned)sizeof(confStruct), (unsigned)SW_VERSION);
            return true;
        }
    }

    if (decodedLen < sizeof(confStruct)) {
        Serial.println("Config data too short, corrupted?");
        delete[] decodedData;
        return false;
    }

    // V2.5-Evo - 2026-08-16 - Stage-then-commit (shared TX/RX code — same behaviour on both boards).
    //
    // WHAT THE BUG WAS: the decoded bytes were copied straight into the caller's struct — and for
    // every caller that struct is the LIVE usrConf — and only validated afterwards. So when
    // validation failed, the rejection was a report, not a rejection: the bad bytes were already
    // running the board. A corrupt blob that still decodes to the right number of bytes (a mangled
    // Base64 paste, a truncated paste that happens to land on the right length, a blob from another
    // build) put garbage PWM0_min / PWM0_max / failsafe_time / steering_type into the live control
    // path until the next reboot. PWM0_min is the one that matters most: at zero throttle the RX
    // output IS PWM0_min, so a garbage minimum can hold an ESC above idle on a craft sitting in
    // the water.
    //
    // WHAT THE FIX DOES: decode and validate into a local staging copy, and copy into the caller's
    // struct only once BOTH checks have passed. On any failure the caller's struct is left exactly
    // as it was — the board keeps running the config it was already running, and the caller's
    // false return is now the whole truth.
    //
    // STACK COST: one confStruct (RX 200 bytes since SW36, TX 136); the boot-path migration branch
    // above peaks at three (its own 'migrated' plus the helper's staging copy) and returns before
    // this one exists. Every call site on both boards runs on the
    // Arduino loop task and its 8192-byte stack — setup() via getConfFromSPIFFS(), ?applyconf via
    // checkSerial(), and the WebUI /config/load handler via webCfgLoop(). None of the 2048-4096
    // byte xTaskCreatePinnedToCore() tasks reach this function. It is also the same shape
    // ConfigServiceEngine.h already uses on that same stack in cfgSetValueByKey() and cfgSetBatch()
    // ("confStruct staged = usrConf; ... usrConf = staged;").
    confStruct staged;
    memcpy(&staged, decodedData, sizeof(confStruct));
    delete[] decodedData;

    // Clamp cross-dependent fields before range validation
    String crossErr;
    if (!cfgValidateCrossField(staged, crossErr)) {
        Serial.println("Config cross-validation failed: " + crossErr);
        Serial.println("Config REJECTED — live config left untouched.");
        return false;
    }

    // Validate config values against their ranges
    String validationErr;
    if (!validateConfig(staged, validationErr)) {
        Serial.println("Config validation failed: " + validationErr);
        Serial.println("Config REJECTED — live config left untouched.");
        return false;
    }

    // Both checks passed — only now does the caller's struct (the live usrConf) change.
    data = staged;

    Serial.println("Struct successfully read from SPIFFS");
    return true;
}

// ============================================================
// V2.5-Evo - 2026-08-16 - LEGACY CONFIG BLOB MIGRATION (exactly one version back, prefix-only)
//
// WHAT PROBLEM THIS SOLVES
//   Every SW_VERSION bump that changes sizeof(confStruct) re-bakes factory defaults on the first
//   boot after the flash, so the owner loses throttle calibration, pairing, compass calibration and
//   every tuning value. The documented remedy is to take a backup with ?conf first and restore it
//   with ?setconf afterwards. That remedy did not actually work: ?setconf length-checks the pasted
//   blob against THIS firmware's confStruct, so the backup was refused at exactly the moment it was
//   needed. This block makes the backup work for the one upgrade step where it provably can.
//
// WHY A MIGRATION IS SAFE FOR THIS ONE PAIR, AND ONLY THIS ONE PAIR
//   V2.5-Evo - 2026-09-19 - the pair is now SW35 -> SW36. The RX SW36 change APPENDED its three
//   fields at the very END of confStruct:
//     SW35 = 192 bytes, ending with rsvd_f32_1 (float) at offset 188.
//     SW36 = those same 192 bytes, then fm_return_mode (uint16_t, 2) + fm_align_cap (uint16_t, 2)
//            + fm_align_influence (uint16_t, 2) = 198, padded to 200 for the 4-byte alignment.
//   Nothing was inserted, moved, resized or reordered inside the first 192 bytes, so an SW35 blob
//   is a BYTE-EXACT PREFIX of an SW36 struct: copying it into the front of an SW36 struct puts every
//   value back at the offset it already belonged to. The three appended fields are given their
//   factory defaults (1 / 13 / 80) by copying the TAIL of defaultConf, not by zeroing - an
//   fm_return_mode of 0 would have silently turned auto-return OFF on every migrated board.
//   Verified on the owner's real SW35 image by Tools/tests/config_migrate_test.cpp.
//   (History: the previous pair was SW34 184 B -> SW35 192 B - mag_orientation + two reserved slots
//   appended, all correctly 0. An SW34 blob is no longer migratable by this build.)
//
// WHY IT FAILS CLOSED, AND WHEN IT STOPS BEING SAFE
//   The prefix argument holds ONLY for these four numbers. It is not a general rule, and it is NOT
//   true of any change that inserts, reorders or resizes a field anywhere in the first 192 bytes.
//   So the migration is gated on all four at once: the blob must be exactly the legacy size AND
//   carry exactly the legacy version, and this firmware must be exactly the struct that blob is a
//   prefix OF. Any future struct change bumps both sizeof(confStruct) and SW_VERSION, both target
//   terms go false, and the whole path switches itself off - an old blob then gets the ordinary
//   "not compatible across SW versions" refusal instead of being silently mis-mapped.
//   That conservatism is the point. A wrong layout assumption here writes misaligned bytes into
//   PWM0_min, PWM0_max, failsafe_time and steering_type, and at zero throttle the RX output IS
//   PWM0_min - so a garbage minimum can hold an ESC above idle on a craft sitting in the water.
//
//   NEXT PERSON ADDING A FIELD: do not simply bump these numbers. Re-check field by field that the
//   old struct is still a byte-exact prefix of the new one. If it is not, DELETE the pair rather
//   than adjusting it; losing the migration is a bad day, mis-mapping the control fields is worse.
//
// SHARED TX/RX CODE, AND INERT ON THE TX. The TX confStruct is 136 bytes at SW27, so
// kCfgLegacyMigrationSupported is false at compile time there, every branch below folds away, and
// a TX owner sees byte-for-byte the behaviour they see today.
// ============================================================
#define CFG_LEGACY_BLOB_BYTES    192   // sizeof(confStruct) on the RX at SW35 - the stored/backup blob's decoded length
#define CFG_LEGACY_BLOB_SW        35   // the SW_VERSION stamped in that blob's first two bytes
#define CFG_LEGACY_TARGET_BYTES  200   // sizeof(confStruct) on the RX at SW36 - the struct it is a prefix OF
#define CFG_LEGACY_TARGET_SW      36   // the SW_VERSION this pairing was verified against

// True only when THIS build is the exact firmware the legacy blob is a prefix of. Both terms are
// compile-time constants, so on the TX (136 bytes, SW27) this is false before the optimiser even
// runs and the migration code is removed from the binary entirely.
static constexpr bool kCfgLegacyMigrationSupported =
    (sizeof(confStruct) == CFG_LEGACY_TARGET_BYTES) && (SW_VERSION == CFG_LEGACY_TARGET_SW);

// Reports whether a decoded blob is the ONE (size, version) pair this firmware knows how to
// migrate. Inputs: the decoded byte count, and the version read from the blob's first two bytes.
// Returns true only for an exact match on both. Anything else - any other size, any other version,
// a 184-byte blob stamped SW33 - returns false, and the caller must refuse it with its normal
// incompatible-backup message. A blob of unknown provenance is never reinterpreted. No side effects.
bool cfgBlobIsMigratableLegacy(size_t decodedLen, uint16_t blobVersion)
{
    // V2.5-Evo - 2026-09-19 - the decision itself is the pure, host-tested cfgLegacyPairMatches().
    return cfgLegacyPairMatches(decodedLen, blobVersion,
                                (size_t)CFG_LEGACY_BLOB_BYTES, (uint16_t)CFG_LEGACY_BLOB_SW,
                                kCfgLegacyMigrationSupported);
}

// Migrates one legacy config blob onto this firmware's confStruct.
// Inputs:  blob / decodedLen / blobVersion - the already-Base64-decoded bytes, their length, and
//          the version read from their first two bytes.
// Outputs: 'out' receives the migrated config ONLY on success and is left untouched on failure.
//          'err' receives the validator's reason when validation is what failed.
// Returns: true only if the blob is the known legacy pair AND the migrated result passes the SAME
//          two checks every other config on this board goes through.
// Side effects: none. Nothing is written to flash, and the live usrConf is not touched - the caller
//          decides what to do with the result.
bool cfgMigrateLegacyBlob(const uint8_t* blob, size_t decodedLen, uint16_t blobVersion,
                          confStruct& out, String& err)
{
    if (blob == nullptr || !cfgBlobIsMigratableLegacy(decodedLen, blobVersion)) {
        err = "not a migratable legacy config";
        return false;
    }

    // Staged, never the live usrConf - the same stage-then-commit rule readConfFromSPIFFS() above
    // follows, and for the same reason: a config that fails validation must never have run the board.
    confStruct staged;

    // V2.5-Evo - 2026-09-19 - prefix from the blob, TAIL FROM defaultConf. Until today this zeroed
    // the struct and overlaid the prefix, which was right only because every field SW35 appended
    // happened to default to 0. SW36's fm_return_mode defaults to 1, so zeroing would have turned
    // auto-return OFF on every migrated board and disagreed with a fresh board. Copying the tail of
    // defaultConf gives every appended field its real default (and a future appended field cannot
    // be forgotten here either). The mapping is the pure cfgPrefixMigrateImage() (ConfigMigrate.h),
    // host-tested on the owner's real SW35 image; it refuses - and leaves 'staged' untouched - if the
    // sizes do not say the blob is a prefix, so the copy is provably in bounds on any board.
    memset(&staged, 0, sizeof(staged));
    if (!cfgPrefixMigrateImage(blob, decodedLen, (size_t)CFG_LEGACY_BLOB_BYTES,
                               (const uint8_t*)&defaultConf, sizeof(confStruct),
                               (uint8_t*)&staged)) {
        err = "legacy config is not a prefix of this struct";
        return false;
    }

    // Stamp it as this firmware's config. Without this the loader would see a version mismatch and
    // re-bake defaults at the next boot - which is the wipe this whole path exists to prevent.
    staged.version = SW_VERSION;

    // The FULL existing validation, unchanged and unskipped. A migrated blob earns no shortcut:
    // cfgValidateCrossField() clamps the cross-dependent fields exactly as it does on every other
    // save path, and validateConfig() range-checks every field, including the three new ones.
    if (!cfgValidateCrossField(staged, err)) return false;
    if (!validateConfig(staged, err)) return false;

    out = staged;
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

  // needDefaults = we must (re)bake defaultConf into SPIFFS. Set true when there is NO stored
  // config at all, OR when the stored config is a different SW_VERSION than this firmware.
  bool needDefaults = false;

  if (readConfFromSPIFFS(usrConf))
  {
    if(SW_VERSION != usrConf.version)
    {
      // V2.5-Evo - 2026-07-21 - Stale-config trap fix (Rex MEDIUM). On a SAME-SIZE version bump the
      // size guard in readConfFromSPIFFS() passes, so usrConf now holds STALE bytes from the previous
      // firmware — a field repurposed at the same offset would be silently misread. Previously we only
      // set config_version_error and RAN ON the stale config. Instead, take the same default re-bake
      // path a struct-size change already takes, so we never operate on mismatched config bytes.
      // Shared TX/RX code — symmetric for both boards. Dormant when versions match (no wipe on a
      // same-version reflash), which is the case for the current SW34->SW34 RX flash.
      Serial.println("Config version mismatch! Re-baking defaults.");
      Serial.print("Config version: "); Serial.print(usrConf.version);
      Serial.print(", firmware version: "); Serial.println(SW_VERSION);
      config_version_error = true;
      needDefaults = true;
    }
  }
  else
  {
    Serial.println("No conf in SPIFFS, writing default...");
    needDefaults = true;
  }

  if (needDefaults)
  {
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
