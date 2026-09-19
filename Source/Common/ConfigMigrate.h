// V2.5-Evo - 2026-09-19 - Config-blob prefix migration, pure and host-testable.
//   When a SW_VERSION bump APPENDS fields to confStruct, the previous version's stored blob is a
//   byte-exact PREFIX of the new struct. This header holds the two decisions that turn such a blob
//   into a current config image - is this blob the one legacy pair we migrate, and which bytes go
//   where - with no Arduino dependency, so Tools/tests/config_migrate_test.cpp runs the exact code
//   the RX runs on a real 192-byte SW35 image. The firmware wrapper (cfgMigrateLegacyBlob() in
//   Common/SPIFFSEngine.h) adds the version stamp and the full validation on top.
#ifndef BREMOTE_CONFIG_MIGRATE_H
#define BREMOTE_CONFIG_MIGRATE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// cfgLegacyPairMatches - is a decoded blob EXACTLY the one (size, version) pair this firmware
// knows how to migrate?
// Inputs:  decoded_len / blob_version - the blob's decoded byte count and the SW version read from
//          its first two bytes; legacy_len / legacy_version - the pair this build migrates from;
//          supported - false when THIS build is not the struct the legacy blob is a prefix of
//          (kCfgLegacyMigrationSupported on the boards), which switches the whole path off.
// Returns: true only for an exact match on all of it. A 184-byte SW34 blob, a 192-byte blob stamped
//          SW34, a 200-byte blob - anything that is not the pair - is false and the caller must
//          refuse it with its ordinary incompatible-config path. Side effects: none.
static inline bool cfgLegacyPairMatches(size_t decoded_len, uint16_t blob_version,
                                        size_t legacy_len, uint16_t legacy_version, bool supported)
{
  return supported && decoded_len == legacy_len && blob_version == legacy_version;
}

// cfgPrefixMigrateImage - build a current-layout config image from a legacy blob.
// What it does: the first legacy_len bytes of the image come from the blob (every legacy field
//   keeps its offset and its value); every byte after that - the appended fields AND any tail
//   padding - comes from the factory-default image at the same offset. Copying the tail from the
//   defaults, rather than zeroing it, is what gives an appended field a non-zero default (SW36's
//   fm_return_mode 1 / fm_align_cap 13 / fm_align_influence 100) and means a future appended field
//   cannot be forgotten here.
// Inputs:  blob / blob_len - the decoded legacy bytes; legacy_len - the size the blob must be;
//          defaults / target_len - the factory-default image and the current struct size;
//          out - target_len bytes to fill.
// Returns: true and fills out on success. False - and out untouched - on a null pointer, a blob
//          that is not exactly legacy_len, an empty legacy_len, or a legacy_len larger than the
//          target (then the blob is not a prefix of anything and must not be reinterpreted).
// Side effects: none. Does NOT stamp the version: the caller sets out's version field to its own
//          SW_VERSION, because only the firmware knows it.
static inline bool cfgPrefixMigrateImage(const uint8_t* blob, size_t blob_len, size_t legacy_len,
                                         const uint8_t* defaults, size_t target_len, uint8_t* out)
{
  if (blob == NULL || defaults == NULL || out == NULL) return false;
  if (legacy_len == 0 || blob_len != legacy_len || legacy_len > target_len) return false;
  memcpy(out, blob, legacy_len);
  memcpy(out + legacy_len, defaults + legacy_len, target_len - legacy_len);
  return true;
}

#endif
