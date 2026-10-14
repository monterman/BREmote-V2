// V2.5-Evo - 2026-10-14 - RESET REASON + BOOT COUNTER (owner-approved for RX1): host test for Source/Common/ResetReason.h.
//   1. Every ESP-IDF code 0-15 has its own name; every other byte value reads "?" / "other / unknown".
//   2. resetReasonIsCrash() is true exactly for the panic / watchdog / lock-up codes, and exactly those classes say CRASH.
//   3. No name or class contains a comma (they go into a CSV line), for all 256 byte values.
//   4. The boot line the RX writes after a log's CSV header (logBootMarker() in BREmote_V2_Rx.h) fits its 160-byte
//      buffer for the largest boot number and the longest class, and the format string here is the firmware's own
//      (checked against the header text, so the two cannot drift).
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in SOP-012's host-test note):
//   g++ -std=c++17 -Wall -Wextra -o reset_reason_test reset_reason_test.cpp && ./reset_reason_test

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>

#include "../../Source/Common/ResetReason.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// logBootMarker()'s format string, verbatim (BREmote_V2_Rx.h). testMarker() checks the header still carries it.
static const char *kBootMarkerFmt = "=== RX boot #%lu / reset %s (%u): %s ===";

static void testNames() {
  for (int a = 0; a <= 15; a++) {
    CHECK(std::strcmp(resetReasonName((uint8_t)a), "?") != 0);
    for (int b = 0; b < a; b++) CHECK(std::strcmp(resetReasonName((uint8_t)a), resetReasonName((uint8_t)b)) != 0);
  }
  for (int c = 16; c <= 255; c++) {
    CHECK(std::strcmp(resetReasonName((uint8_t)c), "?") == 0);
    CHECK(std::strcmp(resetReasonClass((uint8_t)c), "other / unknown") == 0);
    CHECK(!resetReasonIsCrash((uint8_t)c));
  }
  CHECK(std::strcmp(resetReasonName(RR_POWERON), "POWERON") == 0);
  CHECK(std::strcmp(resetReasonName(RR_TASK_WDT), "TASK_WDT") == 0);
  CHECK(std::strcmp(resetReasonName(RR_BROWNOUT), "BROWNOUT") == 0);
  CHECK(std::strcmp(resetReasonName(RR_CPU_LOCKUP), "CPU_LOCKUP") == 0);
  // ESP-IDF's numbering (esp_system.h); Init.ino also static_asserts these against the installed core.
  CHECK(RR_UNKNOWN == 0 && RR_POWERON == 1 && RR_EXT == 2 && RR_SW == 3 && RR_PANIC == 4 && RR_INT_WDT == 5);
  CHECK(RR_TASK_WDT == 6 && RR_WDT == 7 && RR_DEEPSLEEP == 8 && RR_BROWNOUT == 9 && RR_SDIO == 10 && RR_USB == 11);
  CHECK(RR_JTAG == 12 && RR_EFUSE == 13 && RR_PWR_GLITCH == 14 && RR_CPU_LOCKUP == 15);
}

static void testClasses() {
  for (int c = 0; c <= 255; c++) {
    const uint8_t code = (uint8_t)c;
    const bool crash_code = code == RR_PANIC || code == RR_INT_WDT || code == RR_TASK_WDT || code == RR_WDT ||
                            code == RR_CPU_LOCKUP;
    CHECK(resetReasonIsCrash(code) == crash_code);
    CHECK((std::strncmp(resetReasonClass(code), "CRASH", 5) == 0) == crash_code);
    CHECK(std::strchr(resetReasonName(code), ',') == nullptr);
    CHECK(std::strchr(resetReasonClass(code), ',') == nullptr);
    CHECK(std::strchr(resetReasonClass(code), '\n') == nullptr);
  }
  // the power side, which is what an on-water restart is first checked against
  CHECK(std::strcmp(resetReasonClass(RR_POWERON), "power-up or power loss") == 0);
  CHECK(std::strcmp(resetReasonClass(RR_BROWNOUT), "supply sagged (brownout)") == 0);
  CHECK(!resetReasonIsCrash(RR_POWERON) && !resetReasonIsCrash(RR_BROWNOUT) && !resetReasonIsCrash(RR_SW));
}

static void testMarker() {
  // the format string is the firmware's
  std::string text;
  if (FILE *f = std::fopen("../../Source/V2_Integration_Rx/BREmote_V2_Rx.h", "rb")) {
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
    std::fclose(f);
  }
  CHECK(!text.empty());
  CHECK(text.find(std::string("snprintf(buf, len, \"") + kBootMarkerFmt + "\"") != std::string::npos);
  CHECK(text.find("char boot_line[160]") == std::string::npos);   // the buffers are in Logger.ino / WebConfigEngine.h

  // worst case: the largest boot number, every code; must fit the 160-byte buffers, never truncate, no comma
  size_t longest = 0;
  for (int c = 0; c <= 255; c++) {
    char line[160];
    const int n = std::snprintf(line, sizeof line, kBootMarkerFmt, 4294967295UL, resetReasonName((uint8_t)c),
                                (unsigned)c, resetReasonClass((uint8_t)c));
    CHECK(n > 0 && n < (int)sizeof line);
    CHECK(std::strchr(line, ',') == nullptr);
    CHECK(std::strncmp(line, "=== RX boot #", 13) == 0);
    if ((size_t)n > longest) longest = (size_t)n;
  }
  std::printf("reset_reason_test: longest boot line %zu of 159 characters\n", longest);
}

static void testBufferSites() {
  // both download paths size the line at 160 bytes
  const char *paths[] = {"../../Source/V2_Integration_Rx/Logger.ino", "../../Source/Common/WebConfigEngine.h"};
  for (const char *p : paths) {
    std::string text;
    if (FILE *f = std::fopen(p, "rb")) {
      char buf[4096];
      size_t n;
      while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
      std::fclose(f);
    }
    CHECK(!text.empty());
    CHECK(text.find("char boot_line[160];") != std::string::npos);
    CHECK(text.find("logBootMarker(boot_line, sizeof(boot_line), boot);") != std::string::npos);
  }
}

int main() {
  testNames();
  testClasses();
  testMarker();
  testBufferSites();
  if (fails) { std::printf("reset_reason_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("reset_reason_test: all %d checks passed\n", checks);
  return 0;
}
