#ifndef SERIAL_TEE_H
#define SERIAL_TEE_H

// ============================================================================================
// SerialTee - a T-junction in the serial pipe
// ============================================================================================
// V2.5-Evo - 2026-08-18 - WEB-SERIAL-1, Phase 0. Everything this firmware prints goes to the USB
// port and nowhere else. If nobody is plugged in when the board boots, the boot log - compass
// detection, GPS init, dynModel, the mounting-angle warning - is gone forever. That is the one
// thing a beta tester most needs and cannot get without a laptop attached BEFORE power-on.
//
// This header puts a T-junction in that pipe. Output still reaches USB byte-for-byte as it does
// today, and a copy also lands in an 8 KB ring buffer that can be read back later - over USB via
// ?dump today, and over WiFi in a later phase.
//
// HOW IT ATTACHES, AND WHY IT IS A #define. There are ~741 Serial.print call sites. None of them
// change. Instead this header redefines what the word `Serial` MEANS, once:
//
//     #undef Serial
//     #define Serial gSerialTee
//
// The #undef is mandatory and is not defensive coding: `Serial` is ALREADY a macro, defined by
// the core's HardwareSerial.h. A -D on the compiler command line would lose to that header.
//
// WHY NOT esp_log_set_vprintf(). It captures a DISJOINT set: it is the sink for ESP_LOGx, and
// Serial.print never passes through it. For this firmware, which prints with Serial.print
// everywhere and ESP_LOGx essentially nowhere, it would capture almost nothing. The tee is not
// the preferred option, it is the only one.
//
// INCLUDE ORDER IS LOAD-BEARING. Include this immediately after <Arduino.h> and BEFORE any of our
// own headers. Anything included ahead of it binds to the real Serial and is silently missed -
// the failure mode is a capture that looks like it works but has holes in it.
//
// STATIC-INITIALISATION SAFETY. Because the #define is total, a print during static construction
// also lands here, at a moment when a normally-constructed object might not exist yet. So the
// RING STATE below is deliberately plain old data at file scope: it lives in .bss, zeroed by the
// loader before any constructor runs. The tee owns no String, no vector, nothing heap-backed,
//
// and the mutex handle is NULL-checked so a pre-setup() print still captures without locking.
// On a motor controller, heap corruption at boot is not an acceptable failure mode.
//
// V2.5-Evo - 2026-08-24 - CORRECTION (Dexter audit): the paragraph above is true of the four ring
// variables and FALSE of gSerialTee itself. That object is NOT pure POD - it has a vtable and a
// Stream base that sets _timeout, so its vptr is written at RUNTIME by this TU's _GLOBAL__sub_I,
// not by the loader. Safe today only because this header is included FIRST in the one TU that
// uses it, so its constructor runs ahead of every other dynamic initialiser there. Add a printing
// global ABOVE that include and the failure is a jump through a null vptr inside
// __libc_init_array: no output, no core dump, a board that simply will not boot. Keep it first.
//
// NEVER CAPTURED: the ROM and second-stage bootloader banner, which print before any of our code
// runs at all. Document it so a tester does not report it as a bug.
// ============================================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// The real port, captured BEFORE the macro below hides it. `auto&` rather than HardwareSerial&
// so this keeps compiling if a board is ever built with USB CDC on boot, where Serial is a
// different class entirely.
static auto &gRealSerial = Serial;

// --- Ring buffer -----------------------------------------------------------------------------
// 8192 B: Tasmota's ESP32 console ships 6096 and a boot log is a few KB. Sized in .bss, so the
// cost is paid whether or not capture is enabled - see the valve note below.
#define SERIAL_TEE_RING_SIZE 8192

static uint8_t  gTeeRing[SERIAL_TEE_RING_SIZE];

// Monotonic count of bytes EVER written, not an index. A reader holds one of these as its
// cursor and asks "what is new since N?". Deliberately not Tasmota's 8-bit wrapping line
// counter, which becomes ambiguous past 255 entries. Wraps at 2^32 bytes; unsigned arithmetic
// makes the difference correct across that wrap.
static volatile uint32_t gTeeHead;

// The valve. Capture is ON from boot, because the boot log is the whole point and WiFi does not
// exist yet when it is printed. Closing it does NOT reclaim the 8 KB - the buffer is a fixed
// array - it only skips the copy, which is a memcpy far cheaper than the USB write it rides
// alongside. Provided so capture can be shut off entirely while armed, or by an owner who wants
// it gone.
static volatile bool gTeeCaptureEnabled = true;

// Recursive, because a print can legitimately happen INSIDE code that already holds this - an
// HTTP handler that prints while assembling a response, for instance. A plain mutex would
// deadlock there. Not a FreeRTOS stream buffer: those are single-writer by contract and there
// are two writers here: the loop task and the triggeredReceive task. (There is no logger
// task - initTasks() creates three, and logging runs on the loop task via loggerLoop().)
static SemaphoreHandle_t gTeeMutex;

static inline void serialTeeLock()
{
  // NULL until serialTeeInit() runs. A print before that is single-threaded by definition, so
  // capturing without the lock is correct rather than merely tolerable.
  if (gTeeMutex) xSemaphoreTakeRecursive(gTeeMutex, portMAX_DELAY);
}

static inline void serialTeeUnlock()
{
  if (gTeeMutex) xSemaphoreGiveRecursive(gTeeMutex);
}

static void serialTeeInit()
{
  if (!gTeeMutex) gTeeMutex = xSemaphoreCreateRecursiveMutex();
}

static void serialTeeAppend(const uint8_t *data, size_t len)
{
  if (!gTeeCaptureEnabled || len == 0) return;

  // A write longer than the ring can only leave its tail. Skip the part that would be
  // immediately overwritten rather than looping over it.
  if (len > SERIAL_TEE_RING_SIZE) {
    data += (len - SERIAL_TEE_RING_SIZE);
    len   = SERIAL_TEE_RING_SIZE;
  }

  serialTeeLock();
  uint32_t head = gTeeHead;
  for (size_t i = 0; i < len; i++) {
    gTeeRing[(head + i) % SERIAL_TEE_RING_SIZE] = data[i];
  }
  gTeeHead = head + len;
  serialTeeUnlock();
}

// --- Reader ----------------------------------------------------------------------------------
// Copies out everything after `cursor`, up to maxLen. Returns bytes copied.
//   newCursor - where the caller should resume next time
//   gap       - true if the ring wrapped past `cursor` and bytes were lost. Overflow drops the
//               OLDEST data and never stops capturing (unanimous across Tasmota and ESPEasy).
//               ESPEasy's "disable capture when nobody is fetching" is explicitly NOT adopted:
//               it would destroy the boot-log feature, which is the reason this exists.
// Oldest byte still retrievable, given a head the caller has already read under the lock.
// Never underflows: before the ring has filled once, the oldest byte is 0.
static inline uint32_t serialTeeTailLocked(uint32_t head)
{
  return (head > SERIAL_TEE_RING_SIZE) ? (head - SERIAL_TEE_RING_SIZE) : 0;
}

static size_t serialTeeRead(uint32_t cursor, uint8_t *out, size_t maxLen,
                            uint32_t &newCursor, bool &gap)
{
  serialTeeLock();
  const uint32_t head = gTeeHead;

  gap = false;

  // V2.5-Evo - 2026-08-24 - WEB-SERIAL-4. A cursor AHEAD of head means the board rebooted while a
  // client held a cursor from the previous boot - head restarts at 0, the browser still has a big
  // number. Found by a Dexter audit, and it is one click away: the console's own ?reboot button.
  //
  // Without this the unsigned subtraction below underflowed to ~2^32, tripped the overflow guard,
  // and underflowed `cursor` too. Because 8192 divides 2^32 exactly, the modulo then landed
  // precisely on head and returned a bufferful of UNWRITTEN ring - zeros on a fresh boot - while
  // newCursor stayed near 4.29e9. It repeated forever: the console showed a wrap warning twice a
  // second and no output, permanently, until the page was reloaded.
  if (cursor > head) {
    gap    = true;                          // tell the client its position is meaningless now
    cursor = serialTeeTailLocked(head);     // restart from the oldest byte we still hold
  }

  uint32_t behind = head - cursor;          // unsigned: correct across the 2^32 wrap

  if (behind > SERIAL_TEE_RING_SIZE) {      // caller fell further behind than the ring holds
    gap    = true;
    cursor = serialTeeTailLocked(head);
    behind = head - cursor;
  }

  size_t n = (behind > maxLen) ? maxLen : (size_t)behind;
  for (size_t i = 0; i < n; i++) {
    out[i] = gTeeRing[(cursor + i) % SERIAL_TEE_RING_SIZE];
  }

  newCursor = cursor + n;
  serialTeeUnlock();
  return n;
}

static inline uint32_t serialTeeHead() { return gTeeHead; }

// Oldest byte still retrievable. A reader starting from scratch should begin here, not at 0.
static inline uint32_t serialTeeTail()
{
  const uint32_t head = gTeeHead;
  return (head > SERIAL_TEE_RING_SIZE) ? (head - SERIAL_TEE_RING_SIZE) : 0;
}

// --- The tee itself ---------------------------------------------------------------------------
// Derives from Stream, so printf, readStringUntil, readBytes, find and setTimeout all come free
// and keep working exactly as they do today. begin(), end() and operator bool() are
// HardwareSerial-only and are forwarded by hand; a method missed here is a COMPILE error, which
// is the loud kind of failure rather than the silent kind.
class SerialTee : public Stream
{
public:
  // Both write forms are overridden. Without the block form, Print::write(buf,size) would fall
  // back to calling write(uint8_t) per character - taking and releasing the mutex once per byte
  // of every line this firmware prints.
  size_t write(uint8_t c) override
  {
    serialTeeAppend(&c, 1);
    return gRealSerial.write(c);
  }

  size_t write(const uint8_t *buf, size_t size) override
  {
    serialTeeAppend(buf, size);
    return gRealSerial.write(buf, size);
  }

  // Input is pass-through only. Nothing typed is captured - the ring is an output record, and
  // echoing input into it would corrupt the transcript with half-typed commands.
  int    available() override            { return gRealSerial.available(); }
  int    read() override                 { return gRealSerial.read(); }
  int    peek() override                 { return gRealSerial.peek(); }
  void   flush() override                { gRealSerial.flush(); }
  int    availableForWrite() override    { return gRealSerial.availableForWrite(); }

  // HardwareSerial surface used by this firmware.
  void   begin(unsigned long baud)       { gRealSerial.begin(baud); }
  void   end()                           { gRealSerial.end(); }
  void   setDebugOutput(bool en)         { gRealSerial.setDebugOutput(en); }
  operator bool() const                  { return (bool)gRealSerial; }
};

static SerialTee gSerialTee;

// ============================================================================================
// The redefinition. Everything below this line, in every file that includes this header after
// it, prints through the tee. Nothing above it does.
// ============================================================================================
#undef Serial
#define Serial gSerialTee

#endif // SERIAL_TEE_H
