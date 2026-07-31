// V2.5-Evo - 2026-07-29 - GPS-ACK-1: every UBX config write is ACK-verified, and a NAK auto-switches the module to the modern CFG-VALSET interface (u-blox M9/M10 removed the legacy CFG messages). Adds ?gpscfg readback.
// V2.5-Evo - 2026-06-05 - L-2: GSV/GLL/VTG NMEA filter RE-ENABLED in BOTH paths (BN-220 + M10) after the 05-06 fix-acquisition diagnostics; txGpsColdReset() retained. Audit #5 (GPS chatter choking the link) = RESOLVED.
// V2.5-Evo - 2026-05-06 - FIX-GPS-1: dual-baud init in initTxGPS() to prevent UART RX lockout from retained-config baud mismatch
// V2.5-Evo - 2026-04-21 - New TX GPS module: UBX init (115200/5Hz) and non-blocking speed polling for speed_src 2/3/5
// V2.5-Evo - 2026-04-22 - Added speed_src guard to initTxGPS(); 512-byte RX buffer; NMEA sentence filtering (GPGSV/GPGLL/GPVTG disabled); HDOP gate in getTxGPSLoop()
// V2.5-Evo - 2026-04-22 - Added gps_chip_type branch: type 0=BN-220 (existing path), type 2=M10 (115200 direct, 10Hz, all constellations)
// V2.5-Evo - 2026-05-03 - Decouple TX GPS init from speed_src (M1 audit fix).
//                   GPS now inits whenever gps_en=1 — Phase B anti-spoofing
//                   no longer silently broken for RX-speed users.

// ============================================================
// TX GPS - Reads GPS data over Serial1 on the TX (handheld
//          remote). Feeds the SP (Speed) display mode when the
//          user selects a TX-GPS option via the web config:
//              speed_src == 2  -> km/h
//              speed_src == 3  -> knots
//              speed_src == 5  -> mph
//          All other speed_src values (0, 1, 4) continue to use
//          the speed that arrives in the LoRa telemetry packet
//          from the RX (handled in Radio.ino, untouched here).
//
//          Supported GPS chips (selected via usrConf.gps_chip_type):
//              0 = BN-220 (factory 9600 baud → 115200, 5Hz)
//              2 = M10    (native 115200, 10Hz, multi-constellation)
//          Types 1 and 3 (compass variants) are rejected on TX
//          because the TX hardware has no compass connector.
//
// Design goals:
//   - Never block the main loop or the 10Hz LoRa TX cycle
//   - Never stress the watchdog
//   - Fail safely: if the GPS is missing or loses fix, publish
//     the sentinel 0xFF so the display helper renders "--"
//   - Nothing hardcoded that a user might want to change:
//     enable flag, chip type, stale timeout, and display unit
//     all come from usrConf (SPIFFS), per the project's standing rule
// ============================================================

// Has initTxGPS() actually configured Serial1 yet? Used by
// getTxGPSLoop() to avoid poking an uninitialized UART (e.g.
// if the user enables gps_en via web config without rebooting).
static bool tx_gps_initialized = false;

// ============================================================
// UBX configuration helpers — ACK-verified and generation-agnostic
//
// V2.5-Evo - 2026-07-29 - GPS-ACK-1. TX port of the RX change of 2026-07-28 (f7fbcbd),
// extended to cover module generations other than the one on this bench.
//
// WHAT WAS WRONG: every UBX write on the TX was fire-and-forget. Nothing read the ACK, so
// a frame that was dropped, rejected, or NOT SUPPORTED BY THE MODULE AT ALL was
// indistinguishable from one that worked. On 2026-07-28 the RX's newly-added ?gpscfg
// readback caught exactly that: the module was still in dynModel 0 (Portable) after a
// flash that had "sent" dynModel 5 (Sea). The TX has the identical blind-write pattern and
// had never been checked.
//
// WHY IT MATTERS: Portable lets the receiver's own navigation filter accept solutions up
// to 310 m/s horizontal and 50 m/s VERTICAL. The foilIQ logger — same u-blox family, same
// buggy — emitted 254 km/h and 4800 m as HIGH-CONFIDENCE fixes (5-7 satellites, HDOP < 3)
// on 2026-07-24. On the TX that same GPS feeds the speed display and Phase B
// anti-spoofing, so a module inventing 254 km/h corrupts both.
//
// --- WHY THE ACK IS THE RIGHT MECHANISM, AND IS SAFE ON ANY MODULE ---
// u-blox guarantees that every well-formed UBX-CFG message is answered by either
// UBX-ACK-ACK (0x05 0x01) or UBX-ACK-NAK (0x05 0x00), and the 2-byte payload carries the
// class and id of the message being answered. That contract is identical on u-blox 6, 7,
// 8, M9 and M10 — it is the standard handshake, not a trick specific to one chip. Reading
// it costs nothing and cannot upset a module that a user fits in place of ours.
//
// --- WHY A NAK IS NOT A FAILURE: THE GENERATION PROBLEM ---
// u-blox M10 (protocol 34.x) REMOVED the legacy configuration messages. Its entire UBX-CFG
// class is five messages — CFG-CFG, CFG-RST, CFG-VALDEL, CFG-VALGET, CFG-VALSET (M10 SPG
// 5.10 interface description, section 3.10). CFG-PRT, CFG-MSG, CFG-RATE, CFG-NAV5 and
// CFG-GNSS do not exist there and are answered with ACK-NAK.
//
// Four of those five are sent by this file. So on a genuine M10 the firmware as previously
// written applied NO measurement rate, NO NMEA filter and — the one that matters — NO
// dynModel. The module silently stayed in Portable. That is not hypothetical: it is the
// current state of the gps_chip_type=2 path.
//
// A NAK therefore means "wrong dialect", not "bad hardware", and is used here to
// AUTO-DETECT the module generation and re-send the same setting through the modern
// CFG-VALSET key/value interface. The user never has to declare which chip they fitted: a
// BN-220, BN-880, NEO-M8N, NEO-M9N or MAX-M10S all end up correctly configured from one
// firmware image.
//
// --- LENIENCY IS DELIBERATE ---
// Nothing in here can block boot, disable the GPS, or refuse to run. A module that never
// answers at all — a clone that does not implement ACK, or a wiring/baud problem — is
// written to blind, exactly as this firmware behaved before, and the outcome is REPORTED
// rather than enforced. Being strict here would break working setups for other users for
// no safety gain, since an unconfigured module still navigates; it just navigates with
// factory defaults.
// ============================================================

// Outcome of a UBX config write. NAK is distinguished from silence on purpose: NAK is an
// answer (the module understood the frame and refused it), silence is not.
//
// Plain uint8_t rather than an enum on purpose: the Arduino builder hoists auto-generated
// function prototypes to the TOP of the concatenated .ino, above any type declared in the
// file, so a custom enum in a function signature fails to compile with "does not name a
// type". Keeping these as constants avoids leaking GPS-only types into the shared header.
static const uint8_t UBX_ACK     = 0;
static const uint8_t UBX_NAK     = 1;
static const uint8_t UBX_NOREPLY = 2;

// Which configuration dialect this module speaks. Established once per init by probing
// with a legacy frame and reading the answer.
static const uint8_t UBX_DIALECT_LEGACY = 0;
static const uint8_t UBX_DIALECT_VALSET = 1;
static const uint8_t UBX_DIALECT_MUTE   = 2;

// Configuration-interface key IDs (u-blox M9/M10). Verified against the u-blox M10 SPG
// 5.10 interface description, section 4.9 — key ID, storage type and constants all cited
// from the published tables rather than inferred.
static const uint32_t KEY_NAVSPG_DYNMODEL   = 0x20110021UL;  // E1, Table 23: SEA = 5
static const uint32_t KEY_RATE_MEAS         = 0x30210001UL;  // U2, milliseconds
static const uint32_t KEY_MSGOUT_NMEA_GLL_U1 = 0x209100caUL; // U1, rate on UART1
static const uint32_t KEY_MSGOUT_NMEA_GSV_U1 = 0x209100c5UL; // U1, rate on UART1
static const uint32_t KEY_MSGOUT_NMEA_VTG_U1 = 0x209100b1UL; // U1, rate on UART1
static const uint32_t KEY_UART1_BAUDRATE     = 0x40520001UL; // U4, bits/s. Table 58 maps
                                                             // UBX-CFG-PRT.baudRate here.

// Baud rates probed when hunting for a module, most likely first. The scan cost is
// ~250 ms per entry, so order matters more than length.
static const uint32_t kGpsBauds[] = { 115200, 38400, 9600, 57600, 19200 };
static const uint8_t  kGpsBaudCount = sizeof(kGpsBauds) / sizeof(kGpsBauds[0]);

// The TX needs >= 38400 to carry 10 Hz. GGA+RMC is roughly 1400 bytes/s at 10 Hz, and at
// 8N1 every byte costs 10 bits on the wire, so ~14000 bits/s of payload — 9600 cannot do
// it no matter how the firmware is written, and 19200 leaves no headroom.
static const uint32_t GPS_BAUD_PREFERRED = 115200;

// Whatever baud Serial1 is currently open at. Maintained by gpsOpenAt() and by the BN-220
// dual-baud dance. Used to size the ACK window — see gpsAckWindowMs().
static uint32_t gps_current_baud = 115200;

// ------------------------------------------------------------
// gpsAckWindowMs - how long to wait for a UBX ACK, scaled to the link speed.
//
// ⚠️ V2.5-Evo - 2026-07-30 - GPS-ACK-2. This replaces a FIXED 300 ms, which was the real
// defect: 300 ms silently encoded an assumption that the module runs at 115200. It does not.
// The owner's BN-220 runs at 9600, and there the assumption became a coin flip — two
// consecutive boots of identical firmware gave `rate=200ms no-ACK` then `rate=200ms OK`.
//
// The ACK is never lost, it is LATE. The module queues its reply behind whatever NMEA is
// already in its output buffer, so the wait is dominated by that backlog, not by the 10-byte
// ACK. Allow ~500 B of backlog; at 8N1 each byte costs 10 bits, so that is 5,000,000/baud ms.
//
//   9600 -> ~671 ms    19200 -> ~410 ms    38400 and up -> the 300 ms floor
//
// Floor at 300 ms so nothing regresses at high baud; cap at 1200 ms so a wrong-baud probe
// cannot stall boot. Fixing the constant fixes the whole CLASS — any slow module inherits it,
// not just this one.
// ------------------------------------------------------------
static uint16_t gpsAckWindowMs()
{
  uint32_t baud = gps_current_baud ? gps_current_baud : 115200UL;
  uint32_t w    = 150UL + (5000000UL / baud);
  if (w < 300)  w = 300;
  if (w > 1200) w = 1200;
  return (uint16_t)w;
}

// ------------------------------------------------------------
// ubxSendAcked - send a UBX frame and wait for the module's answer.
//
// Returns UBX_ACK / UBX_NAK / UBX_NOREPLY. Retries only on SILENCE, never on NAK: a NAK is
// a definite answer, so re-sending the identical frame can only produce the identical
// refusal. (The RX's first version of this retried on NAK too; on an M10, where five
// separate messages are all refused, that wasted several seconds of boot for nothing.)
//
// Bounded: tries x 350 ms worst case.
// ------------------------------------------------------------
static uint8_t ubxSendAcked(const byte *msg, size_t len, uint8_t tries)
{
  const byte wantCls = msg[2];   // class of the message we are sending
  const byte wantId  = msg[3];   // id    of the message we are sending

  for (uint8_t t = 0; t < tries; t++)
  {
    while (Serial1.available()) Serial1.read();   // clear stale NMEA so the parser starts clean
    Serial1.write(msg, len);
    Serial1.flush();

    uint32_t deadline = millis() + gpsAckWindowMs();   // scaled to baud — see GPS-ACK-2
    uint8_t  state = 0;
    byte     cls = 0, id = 0, ackCls = 0;
    uint16_t plen = 0, idx = 0;

    while ((int32_t)(millis() - deadline) < 0)
    {
      if (!Serial1.available()) { delay(1); continue; }
      byte c = Serial1.read();

      switch (state)
      {
        case 0: state = (c == 0xB5) ? 1 : 0; break;
        case 1: state = (c == 0x62) ? 2 : 0; break;
        case 2: cls = c; state = 3; break;
        case 3: id  = c; state = 4; break;
        case 4: plen = c; state = 5; break;
        case 5:
          plen |= ((uint16_t)c << 8);
          idx = 0;
          // ACK-ACK = 0x05/0x01, ACK-NAK = 0x05/0x00. Payload is always 2 bytes.
          state = (cls == 0x05 && plen == 2) ? 6 : 0;
          break;
        case 6:
          if (idx == 0) { ackCls = c; idx = 1; }
          else
          {
            // Only trust an ACK that names the message we actually sent — the module may
            // be ACKing something else that was queued ahead of us.
            if (ackCls == wantCls && c == wantId)
              return (id == 0x01) ? UBX_ACK : UBX_NAK;
            state = 0;                  // an ACK for a different message; keep looking
          }
          break;
      }
    }
    delay(50);   // brief settle before re-sending
  }
  return UBX_NOREPLY;
}

// ------------------------------------------------------------
// ubxAppendChecksum - Fletcher-8 over class..payload, written into the last two bytes.
// Needed because CFG-VALSET frames are built at runtime and cannot be pre-computed the way
// the fixed legacy frames in this file are.
// ------------------------------------------------------------
static void ubxAppendChecksum(byte *frame, size_t frameLen)
{
  byte ckA = 0, ckB = 0;
  for (size_t i = 2; i < frameLen - 2; i++) { ckA += frame[i]; ckB += ckA; }
  frame[frameLen - 2] = ckA;
  frame[frameLen - 1] = ckB;
}

// ------------------------------------------------------------
// ubxValset - set ONE configuration item through UBX-CFG-VALSET (0x06 0x8A).
//
// Layers RAM|BBR: the setting takes effect now and survives a hot restart, but flash is
// never written. Deliberate — flash has a finite erase budget and this runs on every boot.
//
// Payload: version(0) | layers | reserved[2] | key(4, little-endian) | value(valLen, LE).
// ------------------------------------------------------------
static uint8_t ubxValset(uint32_t key, const byte *val, uint8_t valLen, uint8_t tries,
                         bool persist = false)
{
  if (valLen == 0 || valLen > 8) return UBX_NOREPLY;   // defensive; all our keys are 1-2 bytes

  byte f[6 + 4 + 4 + 8 + 2];
  const uint16_t payload = 4 + 4 + valLen;
  size_t n = 0;

  f[n++] = 0xB5; f[n++] = 0x62;
  f[n++] = 0x06; f[n++] = 0x8A;                      // CFG-VALSET
  f[n++] = (byte)(payload & 0xFF);
  f[n++] = (byte)(payload >> 8);
  f[n++] = 0x00;                                     // version 0
  // Layers. Boot-time writes use RAM|BBR only — flash has a finite erase budget and boot
  // runs constantly. ?gpssetup passes persist=true to add the Flash layer, which is the
  // whole point of a one-time setup: it must outlive a power cycle.
  f[n++] = persist ? (0x01 | 0x02 | 0x04) : (0x01 | 0x02);
  f[n++] = 0x00; f[n++] = 0x00;                      // reserved
  f[n++] = (byte)( key        & 0xFF);               // key, little-endian
  f[n++] = (byte)((key >>  8) & 0xFF);
  f[n++] = (byte)((key >> 16) & 0xFF);
  f[n++] = (byte)((key >> 24) & 0xFF);
  for (uint8_t i = 0; i < valLen; i++) f[n++] = val[i];
  n += 2;                                            // room for the checksum

  ubxAppendChecksum(f, n);
  return ubxSendAcked(f, n, tries);
}

// ------------------------------------------------------------
// gpsApplyCfg - apply one setting using whichever dialect this module speaks.
//
// Returns a short status string for the boot report. Never throws, never blocks boot.
//   "OK"        - the module confirmed the write
//   "OK/valset" - legacy refused, the modern interface accepted it
//   "REJECTED"  - both refused; the module does not support this setting at all
//   "no-ACK"    - the module never answered; write was sent blind (pre-2026-07-29 behavior)
// ------------------------------------------------------------
static const char *gpsApplyCfg(uint8_t dialect,
                               const byte *legacy, size_t legacyLen,
                               uint32_t key, const byte *val, uint8_t valLen,
                               bool persist = false)
{
  if (dialect == UBX_DIALECT_VALSET)
    return (ubxValset(key, val, valLen, 2, persist) == UBX_ACK) ? "OK/valset" : "REJECTED";

  if (dialect == UBX_DIALECT_MUTE) {
    // Module does not answer. Send it anyway — this is exactly what the firmware did
    // before this change, and an unconfigured module still navigates on defaults.
    Serial1.write(legacy, legacyLen);
    Serial1.flush();
    delay(20);
    return "no-ACK";
  }

  uint8_t r = ubxSendAcked(legacy, legacyLen, 2);
  if (r == UBX_ACK) return "OK";
  if (r == UBX_NAK) {
    // Legacy message unsupported on this generation even though the probe suggested
    // otherwise. Fall through to the modern interface rather than giving up.
    return (ubxValset(key, val, valLen, 2, persist) == UBX_ACK) ? "OK/valset" : "REJECTED";
  }
  return "no-ACK";
}

// ------------------------------------------------------------
// gpsSaveConfig - commit the module's CURRENT settings to its own non-volatile memory.
//
// UBX-CFG-CFG (0x06 0x09), save-all. This is one of the FIVE messages M10 kept, so the same
// frame works on u-blox 6/7/8 AND on M9/M10 — no special-casing. On M9/M10 the VALSET Flash
// layer has usually already done the job, but sending this is harmless and covers a setting
// that went out RAM-only.
//
// Payload (13 B): clearMask(4) | saveMask(4) | loadMask(4) | deviceMask(1).
// saveMask 0x0000FFFF = every config section. deviceMask 0x17 = BBR + Flash + EEPROM + SPI.
//
// ⚠️ This is the ONLY place the firmware writes the module's flash, and it is reachable only
// from ?gpssetup — never from boot. Flash has a finite erase budget: a one-time setup spends
// one cycle, a boot-time write would spend one per power-on.
// ------------------------------------------------------------
static bool gpsSaveConfig()
{
  byte f[21] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,       // clearMask — clear nothing
    0xFF, 0xFF, 0x00, 0x00,       // saveMask  — save every section
    0x00, 0x00, 0x00, 0x00,       // loadMask  — load nothing
    0x17,                         // deviceMask: BBR + Flash + EEPROM + SPI flash
    0x00, 0x00                    // checksum, computed below
  };
  ubxAppendChecksum(f, sizeof(f));
  return ubxSendAcked(f, sizeof(f), 3) == UBX_ACK;
}

// ============================================================
// Baud discovery
//
// V2.5-Evo - 2026-07-29 - GPS-BAUD-1.
//
// WHY THIS EXISTS: the TX changes a module's baud with UBX-CFG-PRT. That message does not
// exist on u-blox M9/M10 (Table 58 of the M10 interface description maps it to the
// configuration key CFG-UART1-BAUDRATE instead), so on an M10 the dual-baud dance in
// initTxGPS() is answered with ACK-NAK and the baud NEVER CHANGES. The firmware then
// reopens at 115200 and talks past a module still sitting at 9600 or 38400 — no NMEA, no
// fix, no error, and the troubleshooting doc's advice (set gps_chip_type) does not help
// because the chip type was never the problem.
//
// Rather than hard-code a baud per module — which only works for modules we have met — we
// ask the module. Probing is read-only and cannot misconfigure anything, so it is safe to
// do while we are still guessing.
// ============================================================
static const uint8_t GPS_SAW_UBX  = 0x01;
static const uint8_t GPS_SAW_NMEA = 0x02;

// ------------------------------------------------------------
// gpsOpenAt - reopen Serial1 at a known baud, no probing, buffer clean.
//
// V2.5-Evo - 2026-07-30 - GPS-BAUD-2. Extracted so that "put the UART somewhere known" is a
// single named operation rather than an end/begin/flush sequence copied around. Every path
// that can END a scan without finding anything must call this — see gpsDetectBaud().
// ------------------------------------------------------------
static void gpsOpenAt(uint32_t baud)
{
  Serial1.end();
  delay(10);
  Serial1.begin(baud, SERIAL_8N1, P_U1_RX, P_U1_TX);
  gps_current_baud = baud;                       // keeps gpsAckWindowMs() honest
  delay(60);                                     // let the UART settle before trusting a byte
  while (Serial1.available()) Serial1.read();
}

// ------------------------------------------------------------
// gpsProbeAt - LISTEN at this baud. Transmits NOTHING. Leaves Serial1 open at it either way.
//
// 🚨 V2.5-Evo - 2026-07-30 - GPS-BAUD-4. THIS FUNCTION MUST NEVER TRANSMIT. It used to send a
// UBX-MON-VER poll at each candidate baud, and that BRICKED THE GPS on hardware the same day.
//
// WHAT HAPPENED: u-blox counts UART framing errors, and past roughly 100 it DISABLES its
// receiver until the module is power-cycled. Sending even 8 bytes at a wrong baud produces
// framing errors. Scanning five bauds means four wrong ones, on every boot, plus four more
// every time ?gpsbaud ran. Those accumulated and the module latched its receiver off: NMEA
// still streamed out, but every UBX write and poll was ignored. Boot then reported
// `dynModel=Sea no-ACK | rate no-ACK | GLL no-ACK | GSV no-ACK | VTG no-ACK` and ?gpsbaud
// showed exactly the signature — `9600: UBX - , NMEA yes`.
//
// This is the SAME hazard FIX-GPS-1 exists to prevent. That fix was careful never to retry
// setBaud at the wrong baud; the scan then reintroduced the identical problem from a
// different direction.
//
// WHY LISTENING IS SUFFICIENT: a u-blox module streams NMEA continuously from power-on,
// before it has a fix and regardless of configuration. Seeing "$G.." at a baud is unambiguous
// proof of that baud. There is no need to speak to something whose language is not yet
// established — and every reason not to.
//
// Transmission is safe ONLY after the baud is confirmed. The dialect probe in initTxGPS() and
// gpsUbxAliveHere() below both run after detection, at a known-good baud.
//
// The window must outlast one sentence interval: a module at its 1 Hz default emits roughly
// every second, so a 300 ms window would miss it and report the baud dead. Callers pass ~1100 ms.
// Returns early the moment NMEA is seen, so a healthy 5 Hz module costs ~200 ms.
// ------------------------------------------------------------
static uint8_t gpsProbeAt(uint32_t baud, uint16_t window_ms)
{
  gpsOpenAt(baud);

  uint8_t  seen = 0;
  byte     prev = 0;
  uint32_t deadline = millis() + window_ms;

  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!Serial1.available()) { delay(1); continue; }
    byte c = Serial1.read();

    // Any "$G.." talker id is proof of life at this speed. Nothing is sent to obtain it.
    if (prev == '$' && c == 'G') { seen |= GPS_SAW_NMEA; break; }
    prev = c;
  }
  return seen;
}

// NOTE - V2.5-Evo - 2026-07-30 - GPS-BAUD-5. A gpsUbxAliveHere() helper used to live here and
// probed with UBX-MON-VER. It was REMOVED because it produced a FALSE NEGATIVE on hardware:
// it reported "UBX input DEAD" on a module that ?gpscfg was polling successfully seconds
// either side of it. The difference is that it polled immediately after gpsOpenAt()'s
// end()/begin(), where 60 ms of settle was not enough, while ?gpscfg polls a port that has
// been open and quiet. A diagnostic that cries wolf is worse than none — it would have sent
// the owner off to power-cycle a perfectly healthy module.
//
// The UBX-alive check now reuses gpsDetectDialect() (CFG-NAV5 poll, then CFG-VALGET), which
// is the same path ?gpscfg uses and is proven on hardware. It also returns MORE information:
// alive-and-which-dialect rather than a bare boolean.

// ------------------------------------------------------------
// gpsDetectBaud - hunt the candidate list.
//
// Returns the baud that answered, or 0. Serial1 is ALWAYS left open at a known speed: the
// one that answered on success, or fallback_baud on failure.
//
// ⚠️ V2.5-Evo - 2026-07-30 - GPS-BAUD-2, defect found in the 2026-07-30 audit of 2a33fe9.
// The first version of this function just returned 0, which left Serial1 parked wherever
// gpsProbeAt() had last looked — the FINAL entry of kGpsBauds, i.e. 19200. Nothing any
// module was using.
//
// That is worse than it sounds. It does not merely fail to find the GPS; it converts a
// TRANSIENT fault into a PERMANENT one. A module slow to wake, or a connector momentarily
// loose at boot, would leave the UART at 19200 for the entire session — getTxGPSLoop()
// feeding garbage into TinyGPS++ with no recovery short of a reboot, even after the module
// came good. Three of the four call sites did not restore the baud themselves.
//
// The restore belongs HERE, not at the call sites: a helper that can strand the UART is a
// trap for every future caller, and the audit found the trap had already been stepped in
// three times out of four.
// ------------------------------------------------------------
static uint32_t gpsDetectBaud(uint16_t window_ms, uint32_t fallback_baud)
{
  for (uint8_t i = 0; i < kGpsBaudCount; i++)
    if (gpsProbeAt(kGpsBauds[i], window_ms)) return kGpsBauds[i];

  gpsOpenAt(fallback_baud);   // never leave the port on the last baud we happened to try
  return 0;
}

// ------------------------------------------------------------
// gpsBuildCfgPrt - legacy UBX-CFG-PRT for an arbitrary baud, checksum computed at runtime.
// Payload (20 B): portID | res | txReady(2) | mode(4) | baudRate(4) | inProto(2) |
//                 outProto(2) | flags(2) | res(2)
// ------------------------------------------------------------
static void gpsBuildCfgPrt(byte *f, uint32_t baud)
{
  static const byte tmpl[28] = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x01,0x00,0x00,0x00,
    0xD0,0x08,0x00,0x00,          // mode: 8N1
    0x00,0x00,0x00,0x00,          // baudRate — patched below
    0x07,0x00,                    // inProtoMask : UBX + NMEA + RTCM
    0x03,0x00,                    // outProtoMask: UBX + NMEA
    0x00,0x00, 0x00,0x00,
    0x00,0x00                     // checksum
  };
  memcpy(f, tmpl, sizeof(tmpl));
  f[14] = (byte)( baud        & 0xFF);
  f[15] = (byte)((baud >>  8) & 0xFF);
  f[16] = (byte)((baud >> 16) & 0xFF);
  f[17] = (byte)((baud >> 24) & 0xFF);
  ubxAppendChecksum(f, 28);
}

// ------------------------------------------------------------
// gpsSetModuleBaud - move the MODULE to a new baud and follow it there.
//
// The ACK is deliberately NOT read: the module answers and then switches, so the reply can
// land at the old speed, the new one, or be torn in half. Instead we send, reopen at the
// target, and PROVE the move by probing. If the proof fails we reopen where we came from,
// so a wrong guess can never strand the GPS at a baud nothing is listening on.
//
// persist=true also writes the module's FLASH layer, so the setting outlives a power cycle
// — the whole point of `?gpsbaud set`. Boot-time changes use RAM|BBR only, because flash
// has a finite erase budget and boot runs far more often than a manual reconfigure.
// ------------------------------------------------------------
static bool gpsSetModuleBaud(uint8_t dialect, uint32_t from_baud, uint32_t to_baud, bool persist)
{
  if (dialect == UBX_DIALECT_VALSET) {
    byte f[6 + 4 + 4 + 4 + 2];
    const uint16_t payload = 4 + 4 + 4;
    size_t n = 0;
    f[n++] = 0xB5; f[n++] = 0x62; f[n++] = 0x06; f[n++] = 0x8A;   // CFG-VALSET
    f[n++] = (byte)(payload & 0xFF); f[n++] = (byte)(payload >> 8);
    f[n++] = 0x00;                                                 // version 0
    f[n++] = persist ? (0x01 | 0x02 | 0x04) : (0x01 | 0x02);       // RAM|BBR[|Flash]
    f[n++] = 0x00; f[n++] = 0x00;
    f[n++] = (byte)( KEY_UART1_BAUDRATE        & 0xFF);
    f[n++] = (byte)((KEY_UART1_BAUDRATE >>  8) & 0xFF);
    f[n++] = (byte)((KEY_UART1_BAUDRATE >> 16) & 0xFF);
    f[n++] = (byte)((KEY_UART1_BAUDRATE >> 24) & 0xFF);
    f[n++] = (byte)( to_baud        & 0xFF);                       // U4, little-endian
    f[n++] = (byte)((to_baud >>  8) & 0xFF);
    f[n++] = (byte)((to_baud >> 16) & 0xFF);
    f[n++] = (byte)((to_baud >> 24) & 0xFF);
    n += 2;
    ubxAppendChecksum(f, n);
    Serial1.write(f, n);
  } else {
    byte f[28];
    gpsBuildCfgPrt(f, to_baud);
    Serial1.write(f, sizeof(f));
  }
  Serial1.flush();
  delay(100);                       // let the frame drain and the module switch

  if (gpsProbeAt(to_baud, 1100)) return true;   // listen-only; must outlast a 1 Hz interval

  gpsProbeAt(from_baud, 300);       // follow it back; the caller is no worse off than before
  return false;
}

// ============================================================
// initTxGPS - Initialize Serial1 GPS based on usrConf.gps_chip_type
// ============================================================
//
// What it does:
//   Branches on usrConf.gps_chip_type and sends the appropriate
//   UBX binary commands to the GPS module attached to Serial1:
//     Type 0 (BN-220):
//       1) UBX-CFG-PRT : switch from factory 9600 baud to 115200
//       2) UBX-CFG-RATE: set measurement rate to 5Hz (200ms period)
//       3) UBX-CFG-MSG : disable GPGSV, GPGLL, GPVTG sentences
//     Type 2 (M10):
//       1) UBX-CFG-RATE: set measurement rate to 10Hz (100ms period)
//       2) UBX-CFG-GNSS: enable GPS+Galileo+BDS+GLONASS constellations
//       3) UBX-CFG-MSG : disable GPGSV, GPGLL, GPVTG sentences
//     Types 1/3 (compass variants): rejected by cfgValidateCrossField()
//       before this is reached — TX has no compass hardware.
//     Unknown types: skipped with a Serial error message.
//
// Inputs:
//   - usrConf.gps_en        : master enable; returns early if 0.
//   - usrConf.gps_chip_type : selects the init sequence (0 or 2 on TX).
//   (speed_src no longer gates init — GPS runs whenever gps_en=1.)
//
// Outputs:
//   None (no return value).
//
// Side effects:
//   - Calls Serial1.setRxBufferSize(512) before any begin().
//   - For type 0: dual-baud: begin(115200)/end()/begin(9600)/end()/begin(115200).
//     Blocks ~750ms across five delays.
//   - For type 2: same dual-baud sequence. Blocks ~750ms.
//   - Sets tx_gps_initialized = true on success. No UBX ACK is
//     verified — the flag means "init attempted".
//
// Reboot-required behavior (documented for the user):
//   Changing gps_en or gps_chip_type via the web config at runtime
//   does NOT reinitialize the GPS. A reboot is required.
// ============================================================
void initTxGPS()
{
  // Master switch: if GPS is disabled in config, do nothing.
  // This keeps the UART free and avoids boot delay on units not using GPS.
  if (!usrConf.gps_en)
  {
    Serial.println("TX GPS: disabled (gps_en=0), skipping init");
    return;
  }

  // TX GPS initializes whenever gps_en=1, regardless of speed_src.
  // Anti-spoofing (Phase B) always needs TX GPS — it compares RX vs TX position,
  // independent of which speed source the user has selected for display.

  // V2.5-Evo - 2026-04-22 - Increase RX buffer to 512 bytes before any begin().
  // At 115200 baud / 5-10Hz the GPS emits several NMEA sentences per cycle;
  // 256 bytes can overflow between loop ticks and cause sentence fragments
  // that confuse TinyGPS++. setRxBufferSize() MUST be called before begin().
  Serial1.setRxBufferSize(512);

  // V2.5-Evo - 2026-07-29 - GPS-ACK-1: the desired measurement period, in milliseconds, is
  // now chosen by the chip-type branch but APPLIED after the switch, through the
  // ACK-verified path. 200 ms = 5 Hz (BN-220), 100 ms = 10 Hz (M10). Applying it after the
  // switch means it goes out in whichever dialect the module turns out to speak, instead of
  // being written blind as CFG-RATE — a message that does not exist on M9/M10.
  uint16_t meas_ms = 0;

  // V2.5-Evo - 2026-07-30 - M-1. True once NMEA has actually been HEARD, which is the only
  // thing that proves the baud. Everything that transmits is gated on this: at an unconfirmed
  // baud, every byte we send is a framing error, and ~100 of those make the module disable
  // its receiver until it loses power. Silence is the one state in which we must not talk.
  bool baud_confirmed = false;

  // V2.5-Evo - 2026-04-22 - Branch on GPS chip type. Each chip type requires a
  // different init sequence (factory baud, rate, constellation config).
  // TX hardware has no compass, so types 1/3 are blocked by ConfigService.
  switch (usrConf.gps_chip_type)
  {
    // --------------------------------------------------------
    // Type 0: BN-220 — factory 9600 baud, switch to 115200, 5Hz
    // --------------------------------------------------------
    case 0:
    {
      // UBX-CFG-PRT: configure the GPS UART for 115200 baud, 8N1,
      // with both UBX and NMEA protocols enabled in each direction.
      // The final two bytes are a pre-calculated Fletcher-8 checksum.
      //
      // ⚠️ V2.5-Evo - 2026-07-29 - GPS-ACK-1: setBaud is the ONE write in this file that is
      // deliberately NOT ACK-verified, and it must stay that way. The dual-baud sequence
      // below sends it twice on purpose, once at the WRONG baud, where the module cannot
      // parse it and will never ACK — that is the whole design of FIX-GPS-1. Routing it
      // through ubxSendAcked() would retry a write that is guaranteed to go unanswered, and
      // the retries multiply the wrong-baud byte count: at 2-4 tries this exceeds the
      // ~100-frame-error threshold at which u-blox firmware DISABLES its UART receiver, so
      // "verifying" it would cause the exact lockout FIX-GPS-1 exists to prevent.
      byte setBaud[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
      };

      // V2.5-Evo - 2026-07-29 - GPS-ACK-1: 5 Hz. The rate frame itself is built and sent
      // after the switch, in the dialect the module actually speaks.
      meas_ms = 200;

      // ============================================================
      // V2.5-Evo - 2026-07-30 - GPS-BAUD-6: LISTEN BEFORE DANCING.
      //
      // The dual-baud dance below is the fallback now, not the first move. It is effective
      // but it always transmits ~28 bytes at a baud the module is NOT using, and u-blox
      // counts framing errors: past roughly 100 it DISABLES its receiver until the module is
      // power-cycled. 28 per boot is safe in isolation — but the counter does NOT reset when
      // the ESP32 reboots, only when the GPS loses power. A bench session of reflashes and
      // ?reboot cycles therefore ACCUMULATES, and on 2026-07-30 that is exactly what bricked
      // the module's receiver mid-session: NMEA still streaming, every UBX write ignored.
      //
      // Listening first costs nothing and skips the dance entirely whenever the module is
      // already talking — which is every boot after the first. If it is found below the
      // preferred baud, CFG-PRT is sent AT THE MODULE'S OWN BAUD, so it is parsed correctly
      // and produces ZERO framing errors, instead of being sprayed blind at a guess.
      //
      // Net effect for a NEW module: found by listening at its factory 9600, moved cleanly to
      // 115200, configured, verified. No wrong-baud bytes at any point.
      // ============================================================
      uint32_t seen_baud = gpsDetectBaud(1100, 115200);
      if (seen_baud) {
        Serial.printf("TX GPS [BN-220]: heard the module at %lu — skipping the dual-baud dance\n",
                      (unsigned long)seen_baud);
        if (seen_baud != GPS_BAUD_PREFERRED) {
          Serial.printf("TX GPS [BN-220]: moving %lu -> %lu ... ",
                        (unsigned long)seen_baud, (unsigned long)GPS_BAUD_PREFERRED);
          // Legacy CFG-PRT, sent at the module's ACTUAL baud so it is understood.
          Serial.println(gpsSetModuleBaud(UBX_DIALECT_LEGACY, seen_baud, GPS_BAUD_PREFERRED, false)
                         ? "OK" : "failed, staying put");
        }
        baud_confirmed = true;         // we HEARD it; transmitting is now safe
        tx_gps_initialized = true;
        break;   // configuration happens after the switch, as usual
      }
      Serial.println("TX GPS [BN-220]: nothing heard — falling back to the dual-baud dance");

      // V2.5-Evo - 2026-05-06 - FIX-GPS-1: dual-baud init.
      // Standard u-blox best practice. Handles both possible GPS startup states:
      //   (a) Factory default 9600 (first ever power-on)
      //   (b) Retained 115200 in battery-backed memory (after a prior successful init)
      // Without dual-baud, scenario (b) floods the GPS at wrong baud → 100+ frame errors
      // → u-blox firmware disables UART RX → GPS becomes unresponsive to all commands.
      // Confirmed bug 2026-05-06 via "$GNTXT ... UART RX was disabled" diagnostic message.
      //
      // Total wrong-baud bytes sent in either path: ~28. Well under the 100-frame-error
      // threshold that triggers the UART RX disable.

      // Step 1: open at 115200. If GPS was already at 115200 from a previous boot,
      // this command is delivered cleanly and confirms the config.
      Serial.println("TX GPS [BN-220]: dual-baud init, attempt 115200 first...");
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      gps_current_baud = 115200;   // no ACK is read during the dance, but keep this honest
                                   // so a future edit that DOES read one is not silently wrong
      delay(200);

      // Send UBX-CFG-PRT targeting 115200. If GPS is at 115200 → accepted.
      // If GPS is at 9600 → looks like garbage (~28 bytes), well under threshold.
      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(100);

      // Step 2: close and reopen at 9600 to handle the factory-default case.
      Serial1.end();
      delay(100);
      Serial.println("TX GPS [BN-220]: dual-baud init, attempt 9600 fallback...");
      Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
      gps_current_baud = 9600;
      delay(200);

      // Send same UBX-CFG-PRT at 9600. If GPS was at factory default 9600 → accepted,
      // GPS switches to 115200. If GPS is already at 115200 (from step 1) → garbage at
      // 115200 receiver, ~28 bytes, under threshold.
      Serial1.write(setBaud, sizeof(setBaud));
      Serial1.flush();
      delay(100);

      // Step 3: GPS should now be at 115200 regardless of starting state.
      // Reopen our side at 115200 to match.
      Serial1.end();
      delay(100);
      Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
      gps_current_baud = 115200;
      delay(100);
      // ⚠️ This says where OUR uart now is, NOT where the module is. On the owner's BN-220
      // the CFG-PRT writes above are ignored and the module stays at 9600 — the message used
      // to read "now at 115200" and was simply false. The dialect probe below catches the
      // mismatch and rescans; that rescue is what makes this path work at all.
      Serial.println("TX GPS [BN-220]: our UART now at 115200 (module baud not yet confirmed)");

      // V2.5-Evo - 2026-07-29 - GPS-ACK-1: the rate and the GSV/GLL/VTG NMEA filter used to
      // be written here, blind. Both now happen after the switch through the ACK-verified,
      // dialect-aware path, so the two chip branches no longer carry duplicate copies of
      // the same three filter frames.
      tx_gps_initialized = true;
      break;
    }

    // --------------------------------------------------------
    // Type 2: M10 — auto-detect the baud, then raise it if it is too slow for 10 Hz
    // --------------------------------------------------------
    case 2:
    {
      // ============================================================
      // V2.5-Evo - 2026-07-29 - GPS-BAUD-1: the dual-baud dance that used to live here has
      // been REPLACED, not tuned. It relied on UBX-CFG-PRT to move the module, and CFG-PRT
      // does not exist on M9/M10 — the module NAKs it and stays where it was, after which
      // the old code reopened at 115200 and talked past a module still on 9600 or 38400.
      // Symptom: a completely silent GPS that looks like a wiring fault.
      //
      // Asking the module where it is costs one scan and works on hardware we have never
      // seen, which hard-coding a per-module baud cannot.
      // ============================================================
      Serial.println("TX GPS [M10]: probing for the module...");
      // gpsDetectBaud() guarantees the port is left at GPS_BAUD_PREFERRED if nothing answers,
      // so the config block below still runs and reports honestly instead of failing silently.
      uint32_t found = gpsDetectBaud(1100, GPS_BAUD_PREFERRED);

      if (found == 0) {
        Serial.println("TX GPS [M10]: !! no reply at any baud. Check 3.3V, GND, and that "
                       "TX/RX are not swapped. Run ?gpsbaud to scan, ?gpsraw to see bytes.");
      } else {
        Serial.printf("TX GPS [M10]: module answers at %lu baud\n", (unsigned long)found);
        baud_confirmed = true;         // heard it; transmitting is now safe

        // 9600 physically cannot carry 10 Hz: GGA+RMC is ~1400 B/s, and at 8N1 that is
        // ~14000 bits/s of payload. Raise anything below 38400 before configuring the rate,
        // or the rate write succeeds and the UART quietly drops sentences.
        if (found < 38400) {
          Serial.printf("TX GPS [M10]: %lu is too slow for 10 Hz — raising to %lu\n",
                        (unsigned long)found, (unsigned long)GPS_BAUD_PREFERRED);
          // Dialect is not known yet, so try the modern interface first (this branch is the
          // M10 path) and fall back to legacy for an M8 wired to chip_type 2 by mistake.
          if (gpsSetModuleBaud(UBX_DIALECT_VALSET, found, GPS_BAUD_PREFERRED, false) ||
              gpsSetModuleBaud(UBX_DIALECT_LEGACY, found, GPS_BAUD_PREFERRED, false)) {
            Serial.printf("TX GPS [M10]: now at %lu\n", (unsigned long)GPS_BAUD_PREFERRED);
            Serial.println("TX GPS [M10]: this is RAM/BBR only — it reverts on a full power "
                           "cycle. Run '?gpsbaud set 115200' once to write it to the module.");
          } else {
            Serial.printf("TX GPS [M10]: !! could not raise the baud; staying at %lu. "
                          "10 Hz will drop sentences — run ?gpsbaud.\n", (unsigned long)found);
          }
        }
      }
      Serial.println("TX GPS [M10]: sending GNSS config...");

      // V2.5-Evo - 2026-07-29 - GPS-ACK-1: 10 Hz. Applied after the switch, in the dialect
      // the module speaks. On a genuine M10 the legacy CFG-RATE frame that used to be sent
      // here does not exist and was being NAKed, so this path was silently running at the
      // module default of 1 Hz — not the 10 Hz the boot message claimed.
      meas_ms = 100;

      // UBX-CFG-GNSS: enable GPS, Galileo, BDS (BeiDou), and GLONASS.
      // This gives the M10 all available constellations for best fix accuracy.
      // Payload: 4-byte header (msgver=0, numTrkChHw=0, numTrkChUse=0xFF, numConfigBlocks=4)
      // + 4 blocks of 8 bytes each (gnssId, resTrkCh, maxTrkCh, reserved, flags).
      // Fletcher-8 checksum: CK_A=0xCE, CK_B=0xC0 (pre-calculated over class→payload).
      byte setGNSS[] = {
        0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00,
        0x00, 0x00, 0xFF, 0x04,
        0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // GPS (id=0)
        0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00,  // Galileo (id=2)
        0x03, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // BDS (id=3)
        0x06, 0x08, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // GLONASS (id=6)
        0xCE, 0xC0
      };
      // V2.5-Evo - 2026-07-29 - GPS-ACK-1: constellation selection is left on the legacy
      // frame ONLY, and is allowed to fail quietly. UBX-CFG-GNSS does not exist on M9/M10,
      // where the equivalent lives in the CFG-SIGNAL key group with a separate enable key
      // per constellation. That mapping is not worth carrying here because M10 modules
      // already enable multiple constellations by default — unlike dynModel, rate and the
      // NMEA filter, a NAK here costs accuracy at worst, not safety. Reported at boot so it
      // is visible rather than assumed.
      // V2.5-Evo - 2026-07-30 - M-1: gated on `found`. If nothing was heard, the baud is a
      // GUESS, and pushing 44 more bytes at a guessed baud is exactly the framing-error spray
      // that disabled the module's receiver on 2026-07-30. Silence is the one state in which
      // we must not talk.
      if (found) {
        uint8_t gnss_r = ubxSendAcked(setGNSS, sizeof(setGNSS), 2);
        Serial.printf("TX GPS [M10]: constellations %s\n",
                      gnss_r == UBX_ACK ? "GPS+Galileo+BDS+GLONASS OK"
                    : gnss_r == UBX_NAK ? "left at module defaults (CFG-GNSS not supported — normal on M10)"
                                        : "no ACK (unverified)");
      }

      tx_gps_initialized = true;
      break;
    }

    default:
      // gps_chip_type values 1 and 3 (compass variants) are rejected by
      // cfgValidateCrossField() on TX. Any other value is an unknown chip type.
      Serial.print("TX GPS: unknown gps_chip_type=");
      Serial.println(usrConf.gps_chip_type);
      Serial.println("TX GPS: init skipped — check web config");
      // tx_gps_initialized stays false; getTxGPSLoop() will safely do nothing.
      break;
  }

  // ============================================================
  // V2.5-Evo - 2026-07-27 - GPS-CFG-1: DYNAMIC PLATFORM MODEL = 5 (Sea)
  //
  // Applies to BOTH supported TX chip types. Placed after the switch so neither path can
  // miss it, and gated on tx_gps_initialized so the unknown-chip-type branch — which
  // deliberately leaves Serial1 unconfigured — is skipped rather than written into.
  //
  // WHY: these modules ship in dynModel 0 (Portable), which permits the receiver's own
  // navigation filter to accept solutions up to 310 m/s horizontal and 50 m/s VERTICAL,
  // on the assumption the device might be aboard an aircraft. This TX never leaves the
  // surface of a lake. That permissiveness has a measured cost — the foilIQ logger, same
  // u-blox family on the same buggy, emitted bogus 254 km/h speeds and 4800 m altitudes as
  // HIGH-CONFIDENCE fixes (5-7 satellites, HDOP < 3) on 2026-07-24.
  //
  // On TX the GPS feeds the speed display and Phase B anti-spoofing, so a module inventing
  // 254 km/h corrupts both. dynModel 5 (Sea) constrains the filter to ~25 m/s horizontal
  // and ~0 m/s vertical and pins altitude near the surface, killing those solutions inside
  // the receiver instead of downstream of it.
  //
  // ⚠️ ALTITUDE CEILING: dynModel 5 is valid to 500 m. The Great Lakes sit at 75-183 m
  // (Michigan 176 m) — roughly 3x margin. On a mountain lake above 500 m this MUST become
  // dynModel 4 (Automotive). Owner confirmed 2026-07-27 that will not happen, so this is
  // hard-coded on purpose and is NOT a config field: no confStruct change, no size change,
  // SW_VERSION unchanged, and no SPIFFS settings wipe on this flash.
  //
  // Payload carries the u-blox DEFAULT field set with only dynModel altered — copied from
  // the foilIQ WaveShare firmware where it is field-proven. mask=0x0001 applies dynModel
  // alone, but shipping real defaults rather than zeros is the safer form. Checksum
  // 0x86/0x51 independently recomputed and verified 2026-07-27.
  // ============================================================
  if (tx_gps_initialized) {
    static const byte setNav5Sea[] = {
      0xB5,0x62,0x06,0x24,0x24,0x00,0x01,0x00,0x05,0x03,
      0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,
      0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
      0x00,0x00,0x86,0x51
    };

    // ============================================================
    // V2.5-Evo - 2026-07-29 - GPS-ACK-1: DIALECT PROBE
    //
    // dynModel is both the safety-critical setting and the ideal probe, so it doubles as
    // one. Its answer tells us which configuration dialect this module speaks, and every
    // remaining write follows that verdict — one probe, not one guess per setting.
    //
    //   ACK     -> u-blox 6/7/8 (BN-220, BN-880, NEO-M8N ...). Legacy dialect. Done here.
    //   NAK     -> M9/M10. Legacy CFG messages were REMOVED, so re-send via CFG-VALSET.
    //   silence -> module does not answer at all. Fall back to blind writes, i.e. exactly
    //              how this firmware behaved before today. Never a hard failure.
    // ============================================================
    uint8_t dialect;
    const char *nav5_status;

    // ============================================================
    // V2.5-Evo - 2026-07-30 - M-1: LISTEN BEFORE THE PROBE, NOT AFTER.
    //
    // This block used to fire the 44-byte NAV5 probe (x3 tries = 132 bytes) FIRST and only
    // scan if it went unanswered. On the not-heard path that meant the probe, and then the
    // MUTE blind writes, all went out at a GUESSED baud — roughly 279 wrong-baud bytes per
    // boot, against the ~28 this file's own comments claimed. That is the framing-error
    // spray that disabled the module's receiver on 2026-07-30, arriving by a different door
    // than the one GPS-BAUD-4 closed.
    //
    // Now: if the chip branch did not already hear the module, listen (silently) first. Only
    // a confirmed baud earns the right to transmit.
    // ============================================================
    if (!baud_confirmed) {
      Serial.println("TX GPS: baud not confirmed yet — listening before sending anything...");
      uint32_t found = gpsDetectBaud(1100, GPS_BAUD_PREFERRED);
      if (found) {
        Serial.printf("TX GPS: heard the module at %lu baud\n", (unsigned long)found);
        baud_confirmed = true;
      }
    }

    if (!baud_confirmed) {
      // Nothing anywhere. Say so and send NOTHING. Blind writes here would be aimed at a
      // baud we have no evidence for, and the only thing they can reliably achieve is
      // framing errors on a module that may be perfectly healthy but slow to wake.
      Serial.println("TX GPS: !! no NMEA at any baud — GPS not detected.");
      Serial.println("TX GPS: !! NOTHING was sent. Transmitting at an unconfirmed baud is what");
      Serial.println("TX GPS: !! disables a u-blox receiver, so silence is the safe response.");
      Serial.println("TX GPS: !! Check 3.3V, GND and that TX/RX are not swapped, then ?gpsbaud.");
      return;
    }

    uint8_t probe = ubxSendAcked(setNav5Sea, sizeof(setNav5Sea), 3);

    if (probe == UBX_ACK) {
      dialect     = UBX_DIALECT_LEGACY;
      nav5_status = "OK";
    } else if (probe == UBX_NAK) {
      dialect = UBX_DIALECT_VALSET;
      const byte sea = 5;   // M10 SPG 5.10, Table 23: CFG-NAVSPG-DYNMODEL constant SEA = 5
      nav5_status = (ubxValset(KEY_NAVSPG_DYNMODEL, &sea, 1, 3) == UBX_ACK)
                    ? "OK/valset" : "REJECTED";
    } else {
      dialect = UBX_DIALECT_MUTE;
      Serial1.write(setNav5Sea, sizeof(setNav5Sea));   // blind, as before
      Serial1.flush();
      delay(20);
      nav5_status = "no-ACK";
    }

    // ============================================================
    // V2.5-Evo - 2026-07-30 - GPS-ACK-2: SILENCE THE MODULE BEFORE CONFIGURING IT.
    //
    // The NMEA filter now runs BEFORE the rate write. It used to be the other way round, and
    // on 2026-07-30 that produced an INTERMITTENT failure on real hardware: two consecutive
    // boots of the same firmware on the same BN-220 gave `rate=200ms no-ACK` and then
    // `rate=200ms OK`.
    //
    // The ACK was never lost — it was LATE. The module queues its reply BEHIND whatever NMEA
    // is already in its output buffer, and at the 9600 baud this module actually runs at, a
    // single GSV burst (4+ sentences x ~70 B = ~280 B) costs ~292 ms of wire time against a
    // 300 ms deadline. A coin flip.
    //
    // Turning GSV/GLL/VTG off FIRST removes the backlog instead of waiting it out, so every
    // write after this point gets a prompt answer. dynModel stays ahead of all of it because
    // it doubles as the dialect probe and is the one setting that must not be skipped.
    // ============================================================
    // UBX-CFG-MSG (legacy): B5 62 06 01 03 00 [class F0] [msg id] [rate 0] CK_A CK_B
    static const byte disableGLL[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11};
    static const byte disableGSV[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15};
    static const byte disableVTG[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19};
    static const byte off = 0;   // U1 output rate 0 = disabled, for the VALSET path

    // GSV first — it is by far the largest burst and therefore the main source of the delay.
    const char *gsv_status = gpsApplyCfg(dialect, disableGSV, sizeof(disableGSV),
                                         KEY_MSGOUT_NMEA_GSV_U1, &off, 1);
    const char *gll_status = gpsApplyCfg(dialect, disableGLL, sizeof(disableGLL),
                                         KEY_MSGOUT_NMEA_GLL_U1, &off, 1);
    const char *vtg_status = gpsApplyCfg(dialect, disableVTG, sizeof(disableVTG),
                                         KEY_MSGOUT_NMEA_VTG_U1, &off, 1);

    // --- Measurement rate, LAST, when the link is quietest. 200 ms = 5 Hz, 100 ms = 10 Hz. ---
    const char *rate_status = "skipped";
    if (meas_ms > 0) {
      // UBX-CFG-RATE: measRate(U2, ms) | navRate(U2, cycles) | timeRef(U2, 1 = GPS).
      // Built at runtime so the period is stated once, in meas_ms, instead of hiding in a
      // pre-computed frame whose checksum nobody can check by eye.
      byte setRate[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        (byte)(meas_ms & 0xFF), (byte)(meas_ms >> 8),
        0x01, 0x00,
        0x01, 0x00,
        0x00, 0x00
      };
      ubxAppendChecksum(setRate, sizeof(setRate));

      const byte rate_val[2] = { (byte)(meas_ms & 0xFF), (byte)(meas_ms >> 8) };   // U2, LE
      rate_status = gpsApplyCfg(dialect, setRate, sizeof(setRate),
                                KEY_RATE_MEAS, rate_val, 2);
    }

    // ⚠️ Link-budget check. At 9600 baud, GGA+RMC at 5 Hz is ~710 B/s against 960 B/s
    // available — 74% utilisation with no headroom, so the link drops sentences under any
    // extra load. Warn rather than silently run saturated; ?gpssetup raises the baud.
    if (meas_ms > 0 && gps_current_baud > 0) {
      uint32_t need = (uint32_t)((1000 / meas_ms) * 142UL * 10UL);   // GGA+RMC, 8N1
      if (need * 100UL / gps_current_baud > 70UL)
        Serial.printf("TX GPS: !! %lu baud is only %lu%% clear of what %u Hz needs — sentences "
                      "WILL drop. Run '?gpssetup' to move the module to 115200 permanently.\n",
                      (unsigned long)gps_current_baud,
                      (unsigned long)(100UL - (need * 100UL / gps_current_baud)),
                      (unsigned)(1000 / meas_ms));
    }

    // --- Report every write, so a rejected config is visible at boot rather than shipped. ---
    Serial.printf("TX GPS config [%s]: dynModel=Sea %s | rate=%ums %s | GLL %s | GSV %s | VTG %s\n",
                  dialect == UBX_DIALECT_LEGACY ? "legacy CFG (u-blox 6/7/8)"
                : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                                : "UNVERIFIED - module sends no ACK",
                  nav5_status, meas_ms, rate_status,
                  gll_status, gsv_status, vtg_status);

    if (dialect == UBX_DIALECT_MUTE)
      Serial.println("TX GPS: !! module never answered a config write. It may still be on the "
                     "wrong baud, or be a clone that does not implement UBX-ACK. Settings were "
                     "sent blind and CANNOT be confirmed — run ?gpscfg.");

    // dynModel is the one worth shouting about: Portable permits 310 m/s horizontal and
    // 50 m/s vertical solutions, which is how the foilIQ logger produced 254 km/h and
    // 4800 m as high-confidence fixes on 2026-07-24.
    if (strcmp(nav5_status, "OK") != 0 && strcmp(nav5_status, "OK/valset") != 0)
      Serial.println("TX GPS: !! dynModel NOT confirmed — module may still be in Portable, "
                     "which permits 310 m/s / 50 m/s solutions. Run ?gpscfg.");
  }
}

// ============================================================
// cmdGpsCfg (?gpscfg) - READ BACK what the GPS module actually has
//
// V2.5-Evo - 2026-07-29. The ACK in initTxGPS() confirms the module ACCEPTED a write.
// This confirms what it is actually RUNNING. Two independent checks, because they fail
// differently: an ACK cannot catch a setting that a later write or a warm restart undid,
// and a readback cannot tell you which write was responsible.
//
// This is the command that caught the original defect on the RX on 2026-07-28 — minutes
// after it was first flashed it reported dynModel 0 (Portable) following a flash that had
// "sent" dynModel 5 (Sea).
//
// Polls in whichever dialect answers:
//   legacy  UBX-CFG-NAV5  (0x06 0x24) -> dynModel is byte 2 of the payload
//   modern  UBX-CFG-VALGET(0x06 0x8B) -> CFG-NAVSPG-DYNMODEL, value after the 4-byte key
//
// ⚠️ WARNING: blocks the main loop for up to ~3 s. Bench use only. Type 'quit' to abort.
//
// V2.5-Evo - 2026-07-30 - the earlier version of this warning said the block "stalls the
// 10 Hz LoRa TX cycle, which will trip the RX failsafe". That is WRONG and was corrected in
// the 2026-07-30 audit. sendData runs at FreeRTOS priority 5 on core 0 while loop() runs at
// priority 1, and every wait in here yields, so a blocked loop() cannot delay the LoRa task.
// The RX failsafe is NOT tripped.
//
// What actually freezes: getTxGPSLoop(), runRtmLoop(), runFmLoop(), the display render, the
// menu, mag gestures and the auto-sleep check. That is still a real reason to keep it off
// the water — but it fails SAFE, because the GPS fix simply ages past
// tx_gps_stale_timeout_ms and the 0xF3 GPS meta-packet stops being sent, which BLOCKS
// RTM/FM rather than feeding them stale position.
//
// Keeping the wrong mechanism in a warning is its own hazard: the next person to read it
// would mis-scope the risk in both directions.
// ============================================================
static const char *dynModelName(uint8_t m)
{
  switch (m) {
    case 0:  return "Portable  <-- FACTORY DEFAULT, not what we want";
    case 2:  return "Stationary";
    case 3:  return "Pedestrian";
    case 4:  return "Automotive";
    case 5:  return "Sea  <-- CORRECT for this buggy";
    case 6:  return "Airborne <1g";
    case 7:  return "Airborne <2g";
    case 8:  return "Airborne <4g";
    case 9:  return "Wrist";
    case 10: return "Bike";
    case 11: return "Lawnmower";
    case 12: return "E-scooter";
    default: return "unknown";
  }
}

// gpsBuildValget - UBX-CFG-VALGET request for ONE key, 16-byte frame.
// Payload: version 0 (= request) | layer 0 (RAM) | position u2 | key u4 (little-endian).
static void gpsBuildValget(byte *f, uint32_t key)
{
  f[0] = 0xB5; f[1] = 0x62; f[2] = 0x06; f[3] = 0x8B;
  f[4] = 0x08; f[5] = 0x00;
  f[6] = 0x00; f[7] = 0x00;
  f[8] = 0x00; f[9] = 0x00;
  f[10] = (byte)( key        & 0xFF);
  f[11] = (byte)((key >>  8) & 0xFF);
  f[12] = (byte)((key >> 16) & 0xFF);
  f[13] = (byte)((key >> 24) & 0xFF);
  ubxAppendChecksum(f, 16);
}

// Send a UBX frame, then wait for a reply of the given class/id. Returns payload length,
// or 0 on timeout. Payload is copied into out[] up to outMax.
static uint16_t ubxPoll(const byte *req, size_t reqLen,
                        byte wantCls, byte wantId,
                        byte *out, uint16_t outMax)
{
  while (Serial1.available()) Serial1.read();   // discard stale NMEA
  Serial1.write(req, reqLen);
  Serial1.flush();

  uint32_t deadline = millis() + 1500;
  uint8_t  state = 0;
  byte     cls = 0, id = 0;
  uint16_t len = 0, idx = 0;

  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!Serial1.available()) { delay(1); continue; }
    byte c = Serial1.read();

    switch (state) {
      case 0: state = (c == 0xB5) ? 1 : 0; break;
      case 1: state = (c == 0x62) ? 2 : 0; break;
      case 2: cls = c; state = 3; break;
      case 3: id  = c; state = 4; break;
      case 4: len = c;                   state = 5; break;
      case 5: len |= ((uint16_t)c << 8); idx = 0;
              // Not the frame we asked for (an ACK, or unrelated) — resync.
              state = (cls == wantCls && id == wantId && len <= outMax) ? 6 : 0;
              if (state == 6 && len == 0) return 0;
              break;
      case 6:
        out[idx++] = c;
        if (idx >= len) return len;   // checksum bytes follow; we do not need them
        break;
    }
  }
  return 0;
}

void cmdGpsCfg(const String &args)
{
  (void)args;   // no arguments — signature matches the command-table dispatcher

  Serial.println("----- TX GPS live config (polled from the module) -----");
  if (!tx_gps_initialized) {
    Serial.println("  tx_gps_initialized=false — Serial1 may not be configured.");
    Serial.println("  Run ?gpsreinit first, then ?gpscfg again.");
    Serial.println("-------------------------------------------------------");
    return;
  }

  byte pl[64];

  // --- Try the legacy poll first (u-blox 6/7/8). ---
  static const byte pollNav5[] = { 0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84 };
  uint16_t n = ubxPoll(pollNav5, sizeof(pollNav5), 0x06, 0x24, pl, sizeof(pl));

  if (n >= 3) {
    Serial.println("  dialect  : legacy UBX-CFG (u-blox 6/7/8)");
    Serial.printf("  dynModel : %u  (%s)\n", pl[2], dynModelName(pl[2]));
    Serial.printf("  fixMode  : %u  (1=2D 2=3D 3=auto)\n", pl[3]);
  } else if (checkSerialQuit()) {
    // The legacy poll already cost 1.5 s; let the operator skip the second one.
    Serial.println("  aborted after the legacy poll (no reply).");
  } else {
    // --- No legacy answer. Try the modern configuration interface (M9/M10). ---
    byte pollDyn[16];
    gpsBuildValget(pollDyn, KEY_NAVSPG_DYNMODEL);
    n = ubxPoll(pollDyn, sizeof(pollDyn), 0x06, 0x8B, pl, sizeof(pl));

    if (n >= 9) {
      // Response payload: version | layer | position(2) | key(4) | value(...)
      Serial.println("  dialect  : CFG-VALSET/VALGET (u-blox M9/M10)");
      Serial.printf("  dynModel : %u  (%s)\n", pl[8], dynModelName(pl[8]));
    } else {
      Serial.println("  dynModel : NO REPLY (neither CFG-NAV5 nor CFG-VALGET answered)");
      Serial.println("             The module is silent, on a different baud, or is a clone");
      Serial.println("             that implements neither. If ?gpsraw shows NMEA flowing,");
      Serial.println("             the config writes are being ignored rather than the module");
      Serial.println("             being dead.");
    }
  }

  Serial.printf("  chip_type: %u (0=BN-220 2=M10, from usrConf)\n", usrConf.gps_chip_type);
  Serial.println("  Expect dynModel=5 (Sea). Anything else means initTxGPS() did not stick;");
  Serial.println("  Portable (0) permits 310 m/s / 50 m/s solutions and must not go afloat.");
  Serial.println("-------------------------------------------------------");
}

// ============================================================
// cmdGpsBaud (?gpsbaud) - find, change, and PERSIST the GPS module's baud rate
//
// V2.5-Evo - 2026-07-29 - GPS-BAUD-1. The recovery path for when boot-time detection is not
// enough: an exotic baud outside the scan list, a module that needs a cold reset first, or
// simply wanting to see what is actually out there.
//
//   ?gpsbaud             scan every candidate baud and report what answered. READ-ONLY.
//   ?gpsbaud <rate>      move OUR uart only, module untouched. Reverts on reboot.
//                        Pair with ?gpsraw to eyeball the output before committing.
//   ?gpsbaud set <rate>  move the MODULE and ask it to save the setting to its own
//                        survives a power cycle.
//
// WHY THE BAUD IS STORED IN THE MODULE AND NOT IN usrConf: adding a field to confStruct
// would push sizeof past its pinned 136 bytes and force a SW_VERSION bump, which RESETS the
// TX SPIFFS config to defaults — pairing, calibration and every setting gone. The module
// already has non-volatile memory of its own, so the setting lives there instead and costs
// nothing. It also means the module keeps its baud if it is moved to another remote.
//
// ⚠️ Blocks the main loop for up to ~2 s scanning (~6 s for `set` in the worst case).
// Bench use only. Type 'quit' between probes to abort.
//
// It does NOT stall the LoRa TX cycle or trip the RX failsafe — an earlier version of this
// comment claimed it did, and the 2026-07-30 audit disproved it: sendData is priority 5 on
// core 0, loop() is priority 1, and every wait here yields. What freezes is the main loop
// (GPS parsing, RTM/FM, display), and that fails safe by ageing the fix out and stopping the
// 0xF3 packet, which blocks RTM/FM rather than corrupting them.
//
// Note also that both this and ?gpscfg are unreachable on a normal battery boot: checkCharger()
// sets serialOff, after which setup() calls Serial.end(). They exist only over USB.
// ============================================================
// Which dialect does the module speak? Decided by which POLL it answers — read-only, so it
// is safe to call before we have earned the right to change anything.
static uint8_t gpsDetectDialect()
{
  byte pl[64];

  static const byte pollNav5[] = { 0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84 };
  if (ubxPoll(pollNav5, sizeof(pollNav5), 0x06, 0x24, pl, sizeof(pl)) >= 3)
    return UBX_DIALECT_LEGACY;

  byte pollDyn[16];
  gpsBuildValget(pollDyn, KEY_NAVSPG_DYNMODEL);
  if (ubxPoll(pollDyn, sizeof(pollDyn), 0x06, 0x8B, pl, sizeof(pl)) >= 9)
    return UBX_DIALECT_VALSET;

  return UBX_DIALECT_MUTE;
}

void cmdGpsBaud(const String &args)
{
  String a = args;
  a.trim();

  // ---------- ?gpsbaud : scan ----------
  if (a.length() == 0)
  {
    // Listen-only across the candidates — see GPS-BAUD-4. UBX is checked ONCE afterwards, at
    // the baud that answered, because transmitting at a wrong baud is what disabled the
    // module's receiver on 2026-07-30.
    Serial.println("----- GPS baud scan (listen-only) -----");
    Serial.println("  baud     NMEA");
    uint32_t found = 0;
    for (uint8_t i = 0; i < kGpsBaudCount; i++) {
      uint8_t s = gpsProbeAt(kGpsBauds[i], 1100);
      Serial.printf("  %-7lu  %s\n", (unsigned long)kGpsBauds[i],
                    (s & GPS_SAW_NMEA) ? "yes" : "-");
      if (s && !found) { found = kGpsBauds[i]; break; }   // stop; no need to poke the rest

      // Abort between probes, matching ?gpsraw / ?printrssi. Checked HERE rather than inside
      // gpsProbeAt() because that helper also runs during boot, where reading Serial would
      // swallow input nobody is there to type.
      if (checkSerialQuit()) {
        gpsOpenAt(found ? found : GPS_BAUD_PREFERRED);
        Serial.printf("  aborted — Serial1 left at %lu.\n",
                      (unsigned long)(found ? found : GPS_BAUD_PREFERRED));
        return;
      }
    }

    if (found) {
      gpsOpenAt(found);                    // park the UART where the module actually is
      Serial.printf("  -> NMEA at %lu; Serial1 left open there.\n", (unsigned long)found);

      // Now — and only now, at a CONFIRMED baud — it is safe to transmit. This is what
      // distinguishes a healthy module from one whose receiver u-blox has disabled after too
      // many framing errors: NMEA flowing but UBX dead.
      //
      // Settle before polling. The port was just reopened, and 60 ms proved insufficient —
      // a MON-VER probe here returned a false "DEAD" on a module that was answering fine.
      delay(250);
      uint8_t d = gpsDetectDialect();
      bool ubx = (d != UBX_DIALECT_MUTE);
      Serial.printf("  -> UBX input: %s\n",
                    d == UBX_DIALECT_LEGACY ? "alive — legacy UBX-CFG (u-blox 6/7/8)"
                  : d == UBX_DIALECT_VALSET ? "alive — CFG-VALSET (u-blox M9/M10)"
                                            : "DEAD — module is NOT accepting UBX");
      if (!ubx) {
        Serial.println("     u-blox disables its UART receiver after ~100 framing errors and");
        Serial.println("     stays that way until POWER-CYCLED. Switch the TX off and on —");
        Serial.println("     a reboot alone will not clear it, the module needs its power removed.");
      } else if (found != GPS_BAUD_PREFERRED) {
        Serial.printf("     Run '?gpssetup' to move it to %lu and make it permanent.\n",
                      (unsigned long)GPS_BAUD_PREFERRED);
      }
    } else {
      // The scan above walked the whole list, so the port is sitting on the LAST baud tried.
      // Put it back somewhere sane before returning — otherwise a scan that finds nothing
      // would leave the GPS unusable for the rest of the session as a side effect of merely
      // having LOOKED at it. A read-only diagnostic must not change the machine.
      gpsOpenAt(GPS_BAUD_PREFERRED);
      Serial.println("  -> NOTHING answered at any baud.");
      Serial.println("     That points at wiring or power, not configuration: check 3.3V,");
      Serial.println("     GND, and that TX/RX are not swapped. ?gpsraw shows raw bytes.");
      Serial.printf("     Serial1 restored to %lu.\n", (unsigned long)GPS_BAUD_PREFERRED);
    }
    Serial.println("-------------------------");
    return;
  }

  // ---------- ?gpsbaud set <rate> ----------
  bool persist = a.startsWith("set");
  if (persist) { a = a.substring(3); a.trim(); }

  uint32_t rate = (uint32_t)a.toInt();
  if (rate < 4800 || rate > 921600) {
    Serial.println("Usage: ?gpsbaud            scan every baud (read-only)");
    Serial.println("       ?gpsbaud <rate>     move OUR uart only (reverts on reboot)");
    Serial.println("       ?gpsbaud set <rate> move the MODULE and ask it to save that permanently");
    return;
  }

  if (!persist) {
    // V2.5-Evo - 2026-07-30 - M-4: this used to report "UBX yes/no" from the probe bitmask.
    // gpsProbeAt() has been listen-only since GPS-BAUD-4, so GPS_SAW_UBX is never set and the
    // line printed "UBX no" on every module, healthy or not — a diagnostic that always lies is
    // worse than no diagnostic, because it invites a pointless power-cycle hunt. UBX liveness
    // is now asked properly, with a poll, once the baud is confirmed.
    uint8_t s = gpsProbeAt(rate, 1100);
    Serial.printf("Serial1 reopened at %lu — NMEA %s.\n", (unsigned long)rate,
                  (s & GPS_SAW_NMEA) ? "yes" : "no");
    if (s & GPS_SAW_NMEA) {
      delay(250);                       // settle after the reopen before polling
      uint8_t d = gpsDetectDialect();
      Serial.printf("  UBX input: %s\n",
                    d == UBX_DIALECT_LEGACY ? "alive — legacy UBX-CFG (u-blox 6/7/8)"
                  : d == UBX_DIALECT_VALSET ? "alive — CFG-VALSET (u-blox M9/M10)"
                                            : "DEAD — module is NOT accepting UBX");
    }
    Serial.println("Module NOT changed. This reverts on reboot. ?gpsraw to see the bytes.");
    return;
  }

  // V2.5-Evo - 2026-07-30 - M-2: refuse a baud the scanner cannot rediscover. Moving the
  // module somewhere kGpsBauds does not list would strand it permanently: every future boot
  // would fail to hear it, fall back to the blind dance, and there would be no automatic way
  // home. A command that can put the GPS beyond the firmware's own reach is a foot-gun.
  {
    bool known = false;
    for (uint8_t i = 0; i < kGpsBaudCount; i++) if (kGpsBauds[i] == rate) known = true;
    if (!known) {
      Serial.printf("Refusing: %lu is not in the scan list, so boot could never find the\n",
                    (unsigned long)rate);
      Serial.print("module again. Supported:");
      for (uint8_t i = 0; i < kGpsBaudCount; i++) Serial.printf(" %lu", (unsigned long)kGpsBauds[i]);
      Serial.println();
      return;
    }
  }

  Serial.printf("Locating the module before moving it to %lu...\n", (unsigned long)rate);
  uint32_t cur = gpsDetectBaud(1100, GPS_BAUD_PREFERRED);
  if (!cur) {
    Serial.println("  No module found at any baud — aborting rather than writing blind.");
    Serial.println("  Run ?gpsbaud to scan, and check power/wiring first.");
    Serial.printf("  Serial1 restored to %lu.\n", (unsigned long)GPS_BAUD_PREFERRED);
    return;
  }
  Serial.printf("  found at %lu\n", (unsigned long)cur);

  if (cur == rate) {
    Serial.println("  Already there. Nothing to do.");
    return;
  }

  // Last chance to bail before anything is WRITTEN to the module. Everything above this
  // line was read-only.
  if (checkSerialQuit()) {
    Serial.printf("  aborted before any write — module untouched, Serial1 left at %lu.\n",
                  (unsigned long)cur);
    return;
  }

  uint8_t dialect = gpsDetectDialect();
  Serial.printf("  dialect: %s\n",
                dialect == UBX_DIALECT_LEGACY ? "legacy UBX-CFG (u-blox 6/7/8)"
              : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                              : "unknown - module answers no config poll");

  // An unknown dialect is still worth attempting: try the modern form, then the legacy one.
  bool ok = (dialect == UBX_DIALECT_LEGACY)
              ? gpsSetModuleBaud(UBX_DIALECT_LEGACY, cur, rate, true)
              : gpsSetModuleBaud(UBX_DIALECT_VALSET, cur, rate, true);
  if (!ok && dialect == UBX_DIALECT_MUTE)
    ok = gpsSetModuleBaud(UBX_DIALECT_LEGACY, cur, rate, true);

  if (ok) {
    Serial.printf("  OK — module is now at %lu and it was written to the module's own\n",
                  (unsigned long)rate);
    Serial.println("  flash, so it survives a power cycle and a battery pull.");
    Serial.println("  Reboot the TX so initTxGPS() runs cleanly at the new speed, then ?gpscfg.");
  } else {
    Serial.printf("  FAILED — nothing answered at %lu afterwards; reverted to %lu.\n",
                  (unsigned long)rate, (unsigned long)cur);
    Serial.println("  The module kept its old baud, so the GPS is no worse off than before.");
  }
}

// ============================================================
// cmdGpsSetup (?gpssetup) - configure the GPS ONCE, properly, and make it PERMANENT
//
// V2.5-Evo - 2026-07-30 - GPS-SETUP-1.
//
// WHY THIS EXISTS: boot writes go to RAM|BBR, which do not survive a full power cycle, so
// every boot has to redo the whole configuration. Worse, on the owner's BN-220 the legacy
// CFG-PRT baud change has NEVER worked — the module sits at 9600, where GGA+RMC at 5 Hz is
// ~710 of 960 bytes/s (74% utilisation, no headroom) and sentences drop under load. Verified
// on hardware 2026-07-30: two consecutive boots reported `rate=200ms no-ACK` then
// `rate=200ms OK` — a marginal link producing intermittent failures, which hide.
//
// Run this ONCE on the bench when a remote is assembled, or after changing GPS module.
// It finds the module wherever it is, moves it to 115200, applies every setting with the ACK
// checked, writes the lot to the MODULE's own non-volatile memory, then reads it back to
// prove it. After that, boot only has to VERIFY rather than reconfigure.
//
// Deliberately NOT stored in usrConf: adding a field would push sizeof(confStruct) past its
// pinned 136 bytes, forcing a SW_VERSION bump, which RESETS the TX SPIFFS config — pairing,
// calibration, everything. The module has its own non-volatile memory; the setting lives
// there, costs nothing, and follows the module if it is moved to another remote.
//
// ⚠️ Blocks the main loop for up to ~15 s. Bench only, USB only. 'quit' aborts between steps.
// ============================================================
void cmdGpsSetup(const String &args)
{
  (void)args;
  Serial.println("===== GPS one-time setup =====");

  // --- 1. Find the module ---
  Serial.println("[1/6] locating module...");
  uint32_t cur = gpsDetectBaud(1100, GPS_BAUD_PREFERRED);
  if (!cur) {
    Serial.println("  FAILED: nothing answered at any baud.");
    Serial.println("  That is wiring or power, not configuration. Check 3.3V, GND, and that");
    Serial.printf("  TX/RX are not swapped. Serial1 restored to %lu.\n",
                  (unsigned long)GPS_BAUD_PREFERRED);
    return;
  }
  Serial.printf("  found at %lu baud\n", (unsigned long)cur);

  // --- 2. Which dialect? ---
  uint8_t dialect = gpsDetectDialect();
  Serial.printf("[2/6] dialect: %s\n",
                dialect == UBX_DIALECT_LEGACY ? "legacy UBX-CFG (u-blox 6/7/8)"
              : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                              : "UNKNOWN - answers no config poll");
  if (dialect == UBX_DIALECT_MUTE) {
    Serial.println("  The module replies to neither CFG-NAV5 nor CFG-VALGET, so nothing can");
    Serial.println("  be verified. Refusing to write blind — that is what got us here.");
    return;
  }
  if (checkSerialQuit()) { Serial.println("  aborted before any write."); return; }

  // --- 3. Raise the baud, if needed ---
  Serial.printf("[3/6] baud: ");
  if (cur == GPS_BAUD_PREFERRED) {
    Serial.printf("already %lu\n", (unsigned long)GPS_BAUD_PREFERRED);
  } else {
    Serial.printf("%lu -> %lu ... ", (unsigned long)cur, (unsigned long)GPS_BAUD_PREFERRED);
    if (gpsSetModuleBaud(dialect, cur, GPS_BAUD_PREFERRED, true)) {
      Serial.println("OK");
      cur = GPS_BAUD_PREFERRED;
      // The dialect probe is cheap and the link just changed underneath us; re-confirm.
      dialect = gpsDetectDialect();
    } else {
      Serial.printf("FAILED (still at %lu)\n", (unsigned long)cur);
      Serial.println("  Continuing at the old baud — a slow link is better than none, but");
      Serial.println("  10 Hz will drop sentences. See the warning at the end.");
    }
  }

  // --- 4. Apply everything, persisted ---
  Serial.println("[4/6] applying config (persisted)...");
  static const byte setNav5Sea[] = {
    0xB5,0x62,0x06,0x24,0x24,0x00,0x01,0x00,0x05,0x03,
    0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,
    0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x86,0x51
  };
  static const byte disableGSV[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15};
  static const byte disableGLL[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11};
  static const byte disableVTG[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19};
  static const byte off = 0;
  const byte sea = 5;

  // Same ordering rule as boot: dynModel first (it is the setting that must not be missed),
  // then silence the NMEA chatter, then the rate once the link is quiet.
  const char *s_nav5 = gpsApplyCfg(dialect, setNav5Sea, sizeof(setNav5Sea),
                                   KEY_NAVSPG_DYNMODEL, &sea, 1, true);
  const char *s_gsv  = gpsApplyCfg(dialect, disableGSV, sizeof(disableGSV),
                                   KEY_MSGOUT_NMEA_GSV_U1, &off, 1, true);
  const char *s_gll  = gpsApplyCfg(dialect, disableGLL, sizeof(disableGLL),
                                   KEY_MSGOUT_NMEA_GLL_U1, &off, 1, true);
  const char *s_vtg  = gpsApplyCfg(dialect, disableVTG, sizeof(disableVTG),
                                   KEY_MSGOUT_NMEA_VTG_U1, &off, 1, true);

  const uint16_t meas_ms = (usrConf.gps_chip_type == 2) ? 100 : 200;
  byte setRate[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    (byte)(meas_ms & 0xFF), (byte)(meas_ms >> 8),
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00
  };
  ubxAppendChecksum(setRate, sizeof(setRate));
  const byte rate_val[2] = { (byte)(meas_ms & 0xFF), (byte)(meas_ms >> 8) };
  const char *s_rate = gpsApplyCfg(dialect, setRate, sizeof(setRate),
                                   KEY_RATE_MEAS, rate_val, 2, true);

  Serial.printf("  dynModel=Sea %s | GSV %s | GLL %s | VTG %s | rate=%ums %s\n",
                s_nav5, s_gsv, s_gll, s_vtg, meas_ms, s_rate);

  // --- 5. Commit to the module's own non-volatile memory ---
  // V2.5-Evo - 2026-07-30 - M-3: say what actually happens. CFG-CFG asks the module to save to
  // whatever non-volatile memory it HAS. Many BN-220 boards — including the owner's — carry
  // only battery-backed RAM, kept alive by a small backup cell, with no config flash at all.
  // An ACK means "saved to what I have", not "saved to flash". Claiming flash outright was
  // false on this hardware, and a false persistence promise is how a setting silently reverts
  // after the remote sits in a drawer.
  Serial.print("[5/6] asking the module to save (BBR, plus flash if fitted)... ");
  bool saved = gpsSaveConfig();
  Serial.println(saved ? "OK" : "NOT CONFIRMED");
  if (!saved) {
    Serial.println("  The module did not ACK the save. Settings are live now but may not");
    Serial.println("  survive a power cycle — some modules have no config flash, only BBR.");
  }

  // --- 6. Prove it by reading it back ---
  Serial.println("[6/6] verifying by readback:");
  cmdGpsCfg("");

  Serial.println("Reboot the TX so initTxGPS() runs against the saved config.");
  if (cur < 38400)
    Serial.printf("!! STILL AT %lu BAUD. GGA+RMC at %u Hz needs ~%lu bytes/s of the %lu\n"
                  "   available — the link is saturated and WILL drop sentences.\n",
                  (unsigned long)cur, (unsigned)(1000 / meas_ms),
                  (unsigned long)((1000 / meas_ms) * 142UL), (unsigned long)(cur / 10));
  Serial.println("==============================");
}

// ============================================================
// txGpsColdReset - Send UBX-CFG-RST cold-restart to GPS module
// ============================================================
//
// V2.5-Evo - 2026-05-06 - DIAG: send UBX-CFG-RST cold-restart clear-all to GPS.
// Used by ?gpscoldreset serial command. Forces the GPS module to discard all
// ephemeris/almanac/clock data and acquire from scratch. Useful when the chip
// appears stuck (e.g. PPS LED firing but NMEA empty due to stale cache).
// bbr_mask=0xFFFF (clear all BBR), reset_mode=0x02 (controlled GPS-subsystem
// software reset). Checksum CK_A=0x0E, CK_B=0x61 verified by Fletcher-8.
// (Note: prompt specified mode=0x01 but checksum 0x0E/0x61 is correct for 0x02.)
void txGpsColdReset() {
  if (!tx_gps_initialized) {
    Serial.println("TX GPS: not initialized; run ?gpsreinit first");
    return;
  }
  byte coldReset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
    0xFF, 0xFF, 0x02, 0x00,
    0x0E, 0x61
  };
  Serial1.write(coldReset, sizeof(coldReset));
  Serial1.flush();
  Serial.println("TX GPS: cold-restart command sent. Wait 30-60s for fresh acquisition.");
}

// ============================================================
// txGpsGoodFix - Canonical "trustworthy fix" gate (single source of truth)
// ============================================================
// V2.5-Evo - 2026-07-20 - GPS dot: solid only on FM-grade fix (adds HDOP + speed-valid to match the publish gate).
// One definition of "good fix" shared by getTxGPSLoop() (the publish decision below)
// and the Display.ino GPS status dot (solid branch). A fix is good when location and
// speed are both valid, the fix is fresher than usrConf.tx_gps_stale_timeout_ms, and it
// passes the HDOP quality gate. gps_max_hdop == 0 disables the HDOP check; both
// usrConf.gps_max_hdop and gps_tx.hdop.value() are stored as HDOP*100 (uint16 compare).
bool txGpsGoodFix()
{
  return gps_tx.location.isValid()
      && gps_tx.speed.isValid()
      && gps_tx.location.age() < usrConf.tx_gps_stale_timeout_ms
      && (usrConf.gps_max_hdop == 0 ||
          (gps_tx.hdop.isValid() && gps_tx.hdop.value() <= usrConf.gps_max_hdop));
}

// ============================================================
// getTxGPSLoop - Drain pending GPS bytes and update tx_gps_speed
// ============================================================
//
// What it does:
//   Reads every byte currently sitting in the Serial1 RX buffer,
//   feeds each one to the TinyGPS++ parser (gps_tx), and — if
//   we have a valid and fresh fix — publishes the current speed
//   to the global tx_gps_speed in the unit chosen by
//   usrConf.speed_src.
//
//   If there is no fix, or the fix is older than the user's
//   configured stale-timeout, tx_gps_speed is set to the
//   sentinel 0xFF so the display helpers render "--" (matching
//   the existing convention used for telemetry.foil_speed).
//
// Inputs (all from usrConf, SPIFFS-backed):
//   - usrConf.speed_src              : 2=km/h, 3=knots, 5=mph
//   - usrConf.tx_gps_stale_timeout_ms: max allowed fix age (ms)
//
// Outputs:
//   Writes the global volatile uint8_t tx_gps_speed.
//
// Side effects:
//   - Consumes bytes from Serial1 (the RX buffer).
//   - Does NOT call delay(), does NOT block, does NOT yield.
//     It simply drains whatever is already there and returns.
//     Intended to be called once per main-loop iteration.
//
// Safety:
//   - Returns immediately if initTxGPS() never ran, so we never
//     touch an uninitialized UART.
//   - The caller (loop() in V2_Integration_Tx.ino) additionally
//     gates this call on usrConf.gps_en and on speed_src being
//     one of the TX-GPS options, so we don't waste cycles when
//     a TX-GPS display unit isn't selected.
// ============================================================
void getTxGPSLoop()
{
  // Guard: do nothing until initTxGPS() has set up Serial1.
  if (!tx_gps_initialized) return;

  // Pull every available byte into the parser. Non-blocking:
  // Serial1.available() only reports bytes already in the
  // buffer; we never wait for new ones.
  while (Serial1.available())
  {
    gps_tx.encode((char)Serial1.read());
  }

  // Decide whether the data is trustworthy enough to publish.
  // V2.5-Evo - 2026-07-20 - Publish gate now delegates to txGpsGoodFix() (the canonical
  // "trustworthy fix" definition above), so the publish path and the Display.ino GPS
  // status dot share ONE gate. This folds in the former have_current_fix test
  // (location valid + speed valid + fresher than tx_gps_stale_timeout_ms) AND the HDOP
  // quality gate (N-3 fix): reject fixes with poor satellite geometry. Both
  // usrConf.gps_max_hdop and gps_tx.hdop.value() are stored as HDOP*100 (uint16 compare;
  // a value of 0 disables the HDOP check).
  if (!txGpsGoodFix())
  {
    tx_gps_speed = 0xFF;   // no fix / stale / poor HDOP → display renders "--"
    return;
  }

  // Convert the current speed into the unit the user selected
  // in the web config. TinyGPS++ provides the conversion helpers
  // natively so we don't carry any magic numbers here.
  float speed_val = 0.0f;
  switch (usrConf.speed_src)
  {
    case 2: speed_val = gps_tx.speed.kmph();  break;   // km/h
    case 3: speed_val = gps_tx.speed.knots(); break;   // knots
    case 5: speed_val = gps_tx.speed.mph();   break;   // mph
    default:
      // speed_src was flipped to a non-TX-GPS option between
      // the caller's gate and this line. Defensive: publish
      // the sentinel rather than a stale value.
      tx_gps_speed = 0xFF;
      return;
  }

  // Clamp into a byte. We reserve 0xFF (255) as the sentinel,
  // so the max displayable value is 254. No realistic foil,
  // buggy, or hand-held speed approaches 254 in any unit.
  if (speed_val < 0.0f)   speed_val = 0.0f;
  if (speed_val > 254.0f) speed_val = 254.0f;

  tx_gps_speed = (uint8_t)speed_val;
}
