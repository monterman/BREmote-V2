// V2.5-Evo - 2026-10-14 - RESET REASON + BOOT COUNTER (owner-approved for RX1; log check 2026-10-14 "RX1 unexplained
//   reboots", recommendation 1): names and a plain-English class for the ESP32 reset-reason codes, so the boot banner,
//   ?diag and every log file can say whether the RX last started from a power-up (or a power loss), a crash or watchdog,
//   or a deliberate restart. Pure C++ (no Arduino), host-tested in Tools/tests/reset_reason_test.cpp. The numeric codes
//   are ESP-IDF's esp_reset_reason_t; the RX checks them against the installed core with static_asserts (Init.ino), and
//   Tools/logreader/bremote_log.py carries the same table for log files.
#pragma once
#include <stdint.h>

// ESP-IDF esp_reset_reason_t values (esp_system.h). Kept as plain numbers here so this header needs no ESP-IDF.
enum : uint8_t {
  RR_UNKNOWN = 0, RR_POWERON = 1, RR_EXT = 2, RR_SW = 3, RR_PANIC = 4, RR_INT_WDT = 5, RR_TASK_WDT = 6, RR_WDT = 7,
  RR_DEEPSLEEP = 8, RR_BROWNOUT = 9, RR_SDIO = 10, RR_USB = 11, RR_JTAG = 12, RR_EFUSE = 13, RR_PWR_GLITCH = 14,
  RR_CPU_LOCKUP = 15
};

// resetReasonName - the ESP-IDF name without its ESP_RST_ prefix. Input: code. Returns: a constant string, "?" for a
// code this table does not know. Side effects: none.
static inline const char *resetReasonName(uint8_t code)
{
  switch (code) {
    case RR_UNKNOWN:    return "UNKNOWN";
    case RR_POWERON:    return "POWERON";
    case RR_EXT:        return "EXT";
    case RR_SW:         return "SW";
    case RR_PANIC:      return "PANIC";
    case RR_INT_WDT:    return "INT_WDT";
    case RR_TASK_WDT:   return "TASK_WDT";
    case RR_WDT:        return "WDT";
    case RR_DEEPSLEEP:  return "DEEPSLEEP";
    case RR_BROWNOUT:   return "BROWNOUT";
    case RR_SDIO:       return "SDIO";
    case RR_USB:        return "USB";
    case RR_JTAG:       return "JTAG";
    case RR_EFUSE:      return "EFUSE";
    case RR_PWR_GLITCH: return "PWR_GLITCH";
    case RR_CPU_LOCKUP: return "CPU_LOCKUP";
    default:            return "?";
  }
}

// resetReasonClass - what the code means for a ride, in words a rider can act on. Input: code. Returns: a constant
// string. The split that matters on the water: POWER (the supply went away or sagged: a pack, a cable, a BMS, the
// switch) vs CRASH (the firmware restarted itself: a panic or a watchdog) vs DELIBERATE (?reboot, a flash, a reset
// button). Side effects: none.
static inline const char *resetReasonClass(uint8_t code)
{
  switch (code) {
    case RR_POWERON:    return "power-up or power loss";
    case RR_BROWNOUT:   return "supply sagged (brownout)";
    case RR_PWR_GLITCH: return "power glitch";
    case RR_PANIC:      return "CRASH (firmware panic)";
    case RR_INT_WDT:    return "CRASH (interrupt watchdog)";
    case RR_TASK_WDT:   return "CRASH (task watchdog: a loop stalled)";
    case RR_WDT:        return "CRASH (watchdog)";
    case RR_CPU_LOCKUP: return "CRASH (CPU lock-up)";
    case RR_SW:         return "deliberate restart (?reboot / a flash / a config reset)";
    case RR_EXT:        return "reset pin";
    case RR_USB:        return "USB reset (flashing tool or serial monitor)";
    case RR_JTAG:       return "JTAG reset";
    case RR_DEEPSLEEP:  return "wake from deep sleep";
    default:            return "other / unknown";
  }
}

// resetReasonIsCrash - true for the codes where the firmware restarted itself. Input: code. Side effects: none.
static inline bool resetReasonIsCrash(uint8_t code)
{
  return code == RR_PANIC || code == RR_INT_WDT || code == RR_TASK_WDT || code == RR_WDT || code == RR_CPU_LOCKUP;
}
