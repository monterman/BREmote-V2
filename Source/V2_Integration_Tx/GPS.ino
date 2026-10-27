// V2.5-Evo - 2026-10-27 - TX GPS PASS, the remote's start-up brought in line with the receiver's 2026-10-27 GPS work (audits 2026-10-27 §4.3/§4.4 and delta §8 item 3): (1) NO CONSTELLATION REQUEST: the old-form four-constellation CFG-GNSS write in the type 2 branch is gone and no CFG-SIGNAL key is added, so the module keeps its own set (HGLRC ships GPS+Galileo, which u-blox rates at 10 Hz; all four would cap an M10 at 4 Hz). (2) THREE-RESULT BAUD MOVE, AFTER THE DIALECT PROBE: gpsSetModuleBaud() returns GPS_MOVE_OK / GPS_MOVE_STAYED (heard again at the old baud, listening 1100 ms; it listened 300 ms and ignored the answer) / GPS_MOVE_LOST. initTxGPS() moves a module heard below 115200 once, in the form it answers, never one that answers nothing; an old-form move that STAYED is tried once in CFG-VALSET; LOST rescans with gpsDetectBaud() and, if nothing answers, sends nothing more and returns. The type 0 branch no longer moves the baud before the probe, and the type 2 branch no longer sends the move in both forms before the dialect is known. ?gpssetup and ?gpsbaud set use the same three results. (3) NMEA REPAIR KEYED TO THE SYMPTOM: gpsProbeAt() also recognises a complete, checksum-valid UBX frame (it still transmits nothing), so a module left UBX-only is heard; when the listen heard UBX but no NMEA and the module answers a config message, gpsEnableNmeaOut(dialect) runs at boot and in ?gpssetup step 4 (old form: CFG-PRT at the proven baud, ACK-checked; GGA/RMC through gpsApplyCfg(); then the CFG-VALSET NMEA-out writes). A module that answers no config message is skipped and the log says so; a module whose NMEA was heard is not touched. (4) NO FLASH LAYER on any CFG-VALSET write (ubxValset(), gpsSetModuleBaud(), so ?gpssetup and ?gpsbaud set too): RAM|BBR only; ?gpssetup's CFG-CFG save is unchanged. (5) the link-budget warning prints the share of the line USED, computed on the baud after the move (it printed 100 - used in unsigned arithmetic, a huge number at 10 Hz on 9600). (6) ?gpscfg reads the rate back (CFG-RATE poll, or CFG-VALGET CFG-RATE-MEAS with the key echo checked) and polls UBX-MON-VER (silicon / sw / ext lines), with a quit check before each new poll; the dialect labels name the format, not the chip. Pin auto-detect, dynModel, the GSV/GLL/VTG filter and everything outside GPS unchanged. No confStruct change, sizeof stays 136, SW_VERSION stays 27, SPIFFS config NOT reset by this flash.
// V2.5-Evo - 2026-10-23 - audit L-1/L-2/L-6 on GPS-PIN-1, text and comments only: the pin-check comment now says the GPS's receive line reads LOW during the check on boards with 10 k pull-downs; the "no check yet" hint says to turn the GPS on first (gps_en=0 skips the check); the no-NMEA hint prints once, "GPS not detected" only when no pin had data, and it says ?gpssetup cannot fix a UBX-only module. Serial text only: nothing changes in what is sent to the GPS, on which pins, or when.
// V2.5-Evo - 2026-10-23 - GPS-PIN-1: initTxGPS() first finds which pin the GPS transmits on (G18 = DIY wiring, G19 = factory wiring) by counting edges on both pins with the UART closed, then opens Serial1 on the pins it found (gps_rx_pin / gps_tx_pin). Factory-wired remotes heard nothing before. The four "check that TX/RX are not swapped" hints now report what the pin check saw (gpsPrintNoNmeaHint()). RAM only: no confStruct change, sizeof stays 136, SW_VERSION stays 27, SPIFFS config NOT reset by this flash.
// V2.5-Evo - 2026-08-15 - ubxPoll() now verifies CK_A/CK_B before accepting a frame. Was returning as soon as the payload was complete, never reading the checksum, so any B5 62 <cls> <id> <len> lookalike in the NMEA/UBX stream was believed. Same defect the RX had (fixed 1f2ba8c, after it reported an M8 as M9/M10 three times running) - it survived here because the parser exists twice instead of in Common/. Code only: no confStruct change, sizeof unchanged, SW_VERSION stays 27, SPIFFS config NOT reset by this flash.
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
// V2.5-Evo - 2026-10-23 - GPS-PIN-1: THE PINS THE GPS UART ACTUALLY USES
//
// THE BUG: every Serial1.begin() for the GPS used the fixed defines P_U1_RX 18 / P_U1_TX 19. Factory-made remotes
// (Ludwig / OpenFoil) have the GPS transmitting into G19, so on those remotes the firmware listened on the wrong pin
// and heard nothing at all - no baud found, no fix, and our UART transmitter on G19 pushing against the GPS's own
// output. Ludwig's GPS test firmware, which uses RX=G19, got a fix on the same potted remote.
//
// THE FIX: gpsDetectDataPin() (below, called at the top of initTxGPS()) watches both pins before Serial1 is opened and
// sets these two. Every Serial1.begin() for the GPS uses them. They start at the defaults, so anything that opens the
// UART before a check has run (?gpsbaud with gps_en=0, for example) behaves exactly as before.
// RAM only - never written to SPIFFS, no confStruct change. Written and read by the loop task only.
// ============================================================
static uint8_t  gps_rx_pin        = P_U1_RX;               // pin Serial1 RECEIVES the GPS on (G18 or G19)
static uint8_t  gps_tx_pin        = P_U1_TX;               // pin Serial1 TRANSMITS to the GPS on (the other one)
static bool     gps_pin_checked   = false;                 // has gpsDetectDataPin() run since power-up?
static uint8_t  gps_pin_verdict   = GPS_PIN_VERDICT_NONE;  // result of the last check (Common/GpsPinDetect.h)
static uint32_t gps_pin_edges_rx  = 0;                     // edges seen on P_U1_RX (G18) in the last check
static uint32_t gps_pin_edges_tx  = 0;                     // edges seen on P_U1_TX (G19) in the last check

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
// V2.5-Evo - 2026-10-27 - TOO STRONG, corrected: the M10 interface description lists only those
// five in its UBX-CFG class, but it also says M10 firmware keeps some backwards compatibility with
// the old UBX-CFG messages (SPG 5.10 §1.3 and §4.10). The owner's HGLRC M100 Mini and M100-5883
// accept them. So the form a module answers in does not say which chip it is; UBX-MON-VER
// (?gpscfg's silicon line) does. Everything below still holds for a module that refuses them.
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
// V2.5-Evo - 2026-10-27 - the NMEA-output keys, for gpsEnableNmeaOut() (the repair for a module left
// UBX-only). The same four keys the receiver has used since 2026-08-16 (V2_Integration_Rx/GPS.ino,
// GPS-NMEA-1), where each was checked two ways: bits 30-28 carry the storage size (0x1 = L, one
// bit; 0x2 = U1, one byte), then the group byte, then the item id; and in the CFG-MSGOUT NMEA block
// the ports run I2C, UART1, UART2, USB, SPI, so UART1 is the I2C id + 1 (GGA 0xba -> 0xbb, RMC 0xab
// -> 0xac), the same rule the GLL/GSV/VTG keys above follow. CFG-UART1OUTPROT is group 0x74,
// item 1 = UBX, item 2 = NMEA, type L.
static const uint32_t KEY_UART1OUTPROT_UBX   = 0x10740001UL; // L, output UBX on UART1
static const uint32_t KEY_UART1OUTPROT_NMEA  = 0x10740002UL; // L, output NMEA on UART1
static const uint32_t KEY_MSGOUT_NMEA_GGA_U1 = 0x209100bbUL; // U1, rate on UART1
static const uint32_t KEY_MSGOUT_NMEA_RMC_U1 = 0x209100acUL; // U1, rate on UART1

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

// ============================================================
// gpsSelectedDynModel - the ONE place that turns usrConf.gps_dyn_model into a NAV5 value
// ============================================================
// V2.5-Evo - 2026-08-18 - GPS-DYN-TX. The TX GPS was hard-coded to dynModel 5 (Sea) in four
// separate places: two CFG-NAV5 literals, two CFG-VALSET writes, and two status prints that
// said "dynModel=Sea" whatever the module actually held. The RX gained a selectable model in
// SW35; the TX did not, and beta tester heiguga hit it on 2026-08-18 with
// `?set gps_dyn_model 4` -> ERR_UNKNOWN_KEY. The TX carries its own GPS and feeds tx_gps_speed,
// so the 500 m Sea ceiling bites here exactly as it does on the RX.
//
// CONSTRAINT (identical to the RX): anything other than an explicit 4 must resolve to Sea (5),
// never to Portable (0). 0 is what every TX already in the field holds after the in-place rename
// of the reserved slot, and it must keep meaning the pre-existing hard-coded behaviour, so a
// corrupt or out-of-range value fails toward the conservative model.
// ============================================================
static inline uint8_t gpsSelectedDynModel()
{
  return (usrConf.gps_dyn_model == 4) ? 4 : 5;
}

static inline const char* gpsDynModelName(uint8_t dyn) {
  return (dyn == 4) ? "Automotive" : "Sea";
}

// ============================================================
// gpsSelectedMeasMs - the measurement period asked of the module, resolved in ONE place
// ============================================================
// V2.5-Evo - 2026-10-27 - 100 ms (10 Hz) for gps_chip_type 2 (M10), 200 ms (5 Hz) otherwise
// (0 = BN-220; the TX accepts 0 and 2 only). Read by initTxGPS() and ?gpssetup when they write the
// rate and by ?gpscfg when it judges the readback, so the three cannot disagree. ?gpssetup used to
// carry its own copy of this rule, and ?gpscfg had none.
// Input: usrConf.gps_chip_type. Output: the period in ms. No side effects.
// ============================================================
static inline uint16_t gpsSelectedMeasMs()
{
  return (usrConf.gps_chip_type == 2) ? 100 : 200;
}

// ============================================================
// gpsBuildNav5 - CFG-NAV5 frame carrying the CONFIGURED dynamic model
// ============================================================
// ONE builder, called from both initTxGPS() and cmdGpsSetup(). The payload used to be a
// pre-checksummed literal duplicated in both — the same shape that let the UBX checksum bug
// live on this board for thirteen days after the RX was fixed. Build it once.
//
// The checksum is COMPUTED, never a pre-calculated pair: the dynModel byte is now variable, so
// the old hard-coded 0x86/0x51 would be wrong for Automotive — and a bad checksum fails
// SILENTLY. The module ignores the write and stays in whatever model it had, which on a fresh
// module is dynModel 0 (Portable), the exact setting this mechanism exists to escape.
//
// Layout: [0..1] sync · [2..3] class/id 0x06/0x24 CFG-NAV5 · [4..5] len 0x0024
//         [6..7] mask 0x0001 (apply dynModel only) · [8] dynModel · [9] fixMode
//         ...u-blox default field values... · [42..43] checksum, filled here.
#define GPS_NAV5_FRAME_LEN 44

static uint8_t gpsBuildNav5(byte out[GPS_NAV5_FRAME_LEN])
{
  static const byte kNav5Template[GPS_NAV5_FRAME_LEN] = {
    0xB5,0x62,0x06,0x24,0x24,0x00,0x01,0x00,0x05,0x03,
    0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,
    0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00
  };
  memcpy(out, kNav5Template, GPS_NAV5_FRAME_LEN);

  // The selection, and the "anything but an explicit 4 means Sea" fail-safe behind it, live in
  // gpsSelectedDynModel(). This builder must not carry a second copy of that rule.
  const uint8_t dyn = gpsSelectedDynModel();
  out[8] = dyn;
  ubxAppendChecksum(out, GPS_NAV5_FRAME_LEN);
  return dyn;
}

// ------------------------------------------------------------
// ubxValset - set ONE configuration item through UBX-CFG-VALSET (0x06 0x8A).
//
// Layers RAM|BBR: the setting takes effect now and survives a hot restart, but flash is
// never written. Deliberate — flash has a finite erase budget and this runs on every boot.
//
// V2.5-Evo - 2026-10-27 - RAM|BBR ALWAYS. The optional Flash layer (persist=true, asked for only by
// ?gpssetup) is gone, as on the receiver (audit 2026-10-27 §4.4). The Flash layer exists only on a
// module with external flash, the HGLRC M100s have none, and what a CFG-VALSET asking for a missing
// layer does is not documented: if the module refuses the whole message, every ?gpssetup write on
// such a module read REJECTED. ?gpssetup's step 5 CFG-CFG save makes the result permanent where the
// module can keep it.
//
// Payload: version(0) | layers | reserved[2] | key(4, little-endian) | value(valLen, LE).
// ------------------------------------------------------------
static uint8_t ubxValset(uint32_t key, const byte *val, uint8_t valLen, uint8_t tries)
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
  f[n++] = 0x01 | 0x02;                              // layers: RAM | BBR (never Flash, see above)
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
// V2.5-Evo - 2026-10-27 - the persist argument is gone with the Flash layer (see ubxValset()).
// ------------------------------------------------------------
static const char *gpsApplyCfg(uint8_t dialect,
                               const byte *legacy, size_t legacyLen,
                               uint32_t key, const byte *val, uint8_t valLen)
{
  if (dialect == UBX_DIALECT_VALSET)
    return (ubxValset(key, val, valLen, 2) == UBX_ACK) ? "OK/valset" : "REJECTED";

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
    return (ubxValset(key, val, valLen, 2) == UBX_ACK) ? "OK/valset" : "REJECTED";
  }
  return "no-ACK";
}

// ------------------------------------------------------------
// gpsSaveConfig - commit the module's CURRENT settings to its own non-volatile memory.
//
// UBX-CFG-CFG (0x06 0x09), save-all. This is one of the FIVE messages M10 kept, so the same
// frame works on u-blox 6/7/8 AND on M9/M10 — no special-casing.
// V2.5-Evo - 2026-10-27 - this said "On M9/M10 the VALSET Flash layer has usually already done the
// job". No write asks for the Flash layer any more (see ubxValset()), so this save is now the one
// step that makes ?gpssetup's settings permanent, in either form, wherever the module has flash.
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
  Serial1.begin(baud, SERIAL_8N1, gps_rx_pin, gps_tx_pin);   // V2.5-Evo - 2026-10-23 - GPS-PIN-1: detected pins, was P_U1_RX/P_U1_TX
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
//
// ============================================================
// V2.5-Evo - 2026-10-27 - ALSO RECOGNISE UBX, STILL WITHOUT TRANSMITTING (the receiver's GPS-NMEA-2
// of 2026-08-16, ported).
//
// WHAT WAS WRONG: this window looked for NMEA and nothing else, so "no NMEA" was read as "no module".
// A u-blox left UBX-only by a flight controller (Betaflight turns NMEA output off and saves it) is
// not silent: it streams UBX frames continuously. Every entry point bailed out with "no NMEA at any
// baud", so the NMEA repair (gpsEnableNmeaOut()) could never be reached.
//
// The NMEA test and its early break are UNCHANGED and still run FIRST on every byte, so a module
// that sends NMEA exits at the same byte, after the same time, with the same return value as before.
// UBX evidence never breaks early: the window runs its full length, so a module sending BOTH is never
// reported as UBX-only, and no window gets longer. A candidate must be a COMPLETE, CHECKSUM-VALID
// frame (sync B5 62, class, id, little-endian length, payload, Fletcher-8 pair over class..payload),
// the same parse ubxPoll() uses: a false positive would mark a WRONG baud as proven and let the
// caller transmit into it, the framing-error hazard this listen-only design exists to avoid.
// GPS_SAW_UBX is set here by RECOGNISING a frame; GPS-BAUD-4 retired the old bit, which came from
// transmitting a poll.
// ============================================================
static uint8_t gpsProbeAt(uint32_t baud, uint16_t window_ms)
{
  gpsOpenAt(baud);

  uint8_t  seen = 0;
  byte     prev = 0;
  uint32_t deadline = millis() + window_ms;

  // UBX recogniser state (V2.5-Evo - 2026-10-27). Shares only the byte with the NMEA test.
  uint8_t  ubx_state = 0;
  uint16_t ubx_len   = 0;
  uint16_t ubx_idx   = 0;
  byte     ubx_ckA   = 0;
  byte     ubx_ckB   = 0;

  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!Serial1.available()) { delay(1); continue; }
    byte c = Serial1.read();

    // Any "$G.." talker id is proof of life at this speed. Nothing is sent to obtain it.
    if (prev == '$' && c == 'G') { seen |= GPS_SAW_NMEA; break; }
    prev = c;

    // ---- parallel UBX frame recogniser (V2.5-Evo - 2026-10-27, see the note above) ----
    if (seen & GPS_SAW_UBX) continue;      // already proven; only NMEA is still worth watching
    switch (ubx_state)
    {
      case 0: ubx_state = (c == 0xB5) ? 1 : 0; break;
      case 1: ubx_state = (c == 0x62) ? 2 : 0; break;
      case 2: ubx_ckA = c; ubx_ckB = ubx_ckA; ubx_state = 3; break;            // class
      case 3: ubx_ckA += c; ubx_ckB += ubx_ckA; ubx_state = 4; break;          // id
      case 4: ubx_len  = c; ubx_ckA += c; ubx_ckB += ubx_ckA; ubx_state = 5; break;
      case 5:
        ubx_len |= ((uint16_t)c << 8);
        ubx_ckA += c; ubx_ckB += ubx_ckA;
        ubx_idx  = 0;
        // A garbage length must not park this machine in the payload state for thousands of bytes
        // and swallow the real frames behind it. The largest frame expected here is UBX-NAV-PVT
        // (92 bytes of payload).
        if (ubx_len > 512)      { ubx_state = 0; break; }
        ubx_state = (ubx_len == 0) ? 7 : 6;
        break;
      case 6:
        ubx_ckA += c; ubx_ckB += ubx_ckA;
        if (++ubx_idx >= ubx_len) ubx_state = 7;
        break;
      case 7: ubx_state = (c == ubx_ckA) ? 8 : 0; break;   // bad CK_A: not a frame, resync
      case 8:
        if (c == ubx_ckB) seen |= GPS_SAW_UBX;             // verified: the module speaks UBX
        ubx_state = 0;
        break;
    }
  }
  return seen;
}

// V2.5-Evo - 2026-10-27 - plain-English name for what a listen window heard, for the log lines.
static const char *gpsProtoName(uint8_t seen)
{
  if ((seen & GPS_SAW_NMEA) && (seen & GPS_SAW_UBX)) return "NMEA+UBX";
  if (seen & GPS_SAW_NMEA)                           return "NMEA";
  if (seen & GPS_SAW_UBX)                            return "UBX only";
  return "nothing";
}

// V2.5-Evo - 2026-10-27 - the diagnosis for a module heard in UBX only, said in one place because boot,
// ?gpssetup and ?gpsbaud can all meet it. States the diagnosis only: what is done about it differs
// per caller, and ?gpsbaud writes nothing. Side effect: five Serial lines.
static void gpsReportUbxOnly(uint32_t baud)
{
  Serial.printf("TX GPS: !! module is ALIVE at %lu baud but speaking UBX ONLY — no NMEA at all.\n",
                (unsigned long)baud);
  Serial.println("TX GPS: !! That is the signature of a flight-controller setup: Betaflight puts");
  Serial.println("TX GPS: !! u-blox in UBX-only mode and saves it to battery-backed RAM and flash,");
  Serial.println("TX GPS: !! so it survives every power cycle. This firmware parses NMEA, so until");
  Serial.println("TX GPS: !! NMEA output is switched back on it sees an empty wire.");
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
//
// V2.5-Evo - 2026-10-27 - seen_out optionally reports WHICH protocol answered (gpsProbeAt() now
// hears UBX too), so a caller can tell a healthy module from one that is alive but UBX-only. The
// candidate list, its order, the window and the return value are unchanged; callers that pass
// nothing behave exactly as before.
// ------------------------------------------------------------
static uint32_t gpsDetectBaud(uint16_t window_ms, uint32_t fallback_baud,
                              uint8_t *seen_out = nullptr)
{
  if (seen_out) *seen_out = 0;

  for (uint8_t i = 0; i < kGpsBaudCount; i++) {
    const uint8_t s = gpsProbeAt(kGpsBauds[i], window_ms);
    if (s) {
      if (seen_out) *seen_out = s;
      return kGpsBauds[i];
    }
  }

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
// V2.5-Evo - 2026-10-27 - RAM|BBR ONLY, from every caller. persist=true used to add the FLASH layer
// for ?gpsbaud set and ?gpssetup; it is gone (see ubxValset()). ?gpssetup's step 5 CFG-CFG save
// makes a move permanent where the module has flash.
//
// V2.5-Evo - 2026-10-27 - THREE RESULTS, NOT A BOOL (the receiver's audit of 2026-10-27, LOW row
// "failed move"). A failed move used to listen at the old baud for 300 ms and ignore the answer -
// shorter than one 1 Hz epoch, so it proved nothing - and return false either way, so the caller went
// on writing at a baud nobody had proven. Now the listen back is as long as the proof (1100 ms) and
// its answer is returned:
//   GPS_MOVE_OK     heard at to_baud; the port is open there.
//   GPS_MOVE_STAYED not heard at to_baud, heard again at from_baud; the port is open there, and
//                   from_baud is proven, so the caller may keep writing at it.
//   GPS_MOVE_LOST   heard at neither. The port is left at from_baud, but nothing proves the module
//                   is there: the caller must find it again (gpsDetectBaud) or send nothing more.
// Inputs: dialect (the form to send the move in), from_baud (where the module was proven),
// to_baud. Side effects: one frame at from_baud, then up to two 1100 ms listens; ~2.3 s worst case.
// ------------------------------------------------------------
static const uint8_t GPS_MOVE_OK     = 0;
static const uint8_t GPS_MOVE_STAYED = 1;
static const uint8_t GPS_MOVE_LOST   = 2;

static uint8_t gpsSetModuleBaud(uint8_t dialect, uint32_t from_baud, uint32_t to_baud)
{
  if (dialect == UBX_DIALECT_VALSET) {
    byte f[6 + 4 + 4 + 4 + 2];
    const uint16_t payload = 4 + 4 + 4;
    size_t n = 0;
    f[n++] = 0xB5; f[n++] = 0x62; f[n++] = 0x06; f[n++] = 0x8A;   // CFG-VALSET
    f[n++] = (byte)(payload & 0xFF); f[n++] = (byte)(payload >> 8);
    f[n++] = 0x00;                                                 // version 0
    f[n++] = 0x01 | 0x02;                                          // RAM|BBR (never Flash)
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

  if (gpsProbeAt(to_baud, 1100)) return GPS_MOVE_OK;     // listen-only; must outlast a 1 Hz interval

  // V2.5-Evo - 2026-10-27 - follow it back and PROVE it is there, with the same 1100 ms as the proof
  // above (was gpsProbeAt(from_baud, 300) with the answer ignored).
  if (gpsProbeAt(from_baud, 1100)) return GPS_MOVE_STAYED;
  return GPS_MOVE_LOST;
}

// ============================================================
// gpsEnableNmeaOut - put NMEA output back on UART1, in whichever form the module answers
// ============================================================
// V2.5-Evo - 2026-10-27 - ported from V2_Integration_Rx/GPS.ino (GPS-NMEA-1 of 2026-08-16, keyed to
// the symptom by the receiver's audit of 2026-10-27, GGA/RMC in either form by its delta audit).
//
// WHY: Betaflight's GPS auto-config switches u-blox modules to UBX-only and saves it to the module's
// battery-backed RAM and flash. A module that has been on a flight controller - or was set up for
// the drone market, which is most of what is sold as an "M10 GPS" - arrives with NMEA output off and
// stays that way through every power cycle. TinyGPS++ reads NMEA and nothing else, so the remote had
// no GPS, and nothing in this file ever wrote the output-protocol configuration.
//
// WHAT IT WRITES, in this order:
//   0. On a module that answers the OLD form (dialect LEGACY): UBX-CFG-PRT from gpsBuildCfgPrt() at
//      gps_current_baud, the baud the module has just been proven on by listening. So it is NOT a
//      baud move, and the ACK comes back readable. Its output mask is UBX+NMEA, which is the old-form
//      way to switch NMEA output back on. ACK-checked through ubxSendAcked(). A module whose firmware
//      keeps some old messages but not CFG-PRT (some M10) refuses it, and writes 1-3 do the job.
//   1. GGA and RMC message rates, through gpsApplyCfg(): old-form UBX-CFG-MSG in its 3-byte "current
//      port" form (class F0, id 00 = GGA / 04 = RMC, rate 1 on the port the message arrives on, i.e.
//      UART1), ACK-checked and re-sent as CFG-VALSET on a NAK; a CFG-VALSET module goes straight to
//      the key. These are the two sentences TinyGPS++ reads, and Betaflight may have zeroed them one
//      by one, in which case turning the protocol back on by itself changes nothing.
//   2. CFG-UART1OUTPROT-UBX, kept ON on purpose: our own ACK checking and ?gpscfg are UBX.
//   3. CFG-UART1OUTPROT-NMEA, LAST, so the sentence stream only restarts once every write above has
//      been answered. Writes 2-3 are CFG-VALSET whichever form answered; an M8 has no CFG-VALSET and
//      refuses or ignores them, which at a proven baud costs time, not harm (CFG-PRT has done their
//      job there).
// RAM|BBR, at boot and in ?gpssetup alike (no Flash layer, see ubxValset()); ?gpssetup's step 5
// CFG-CFG save makes it permanent where the module can keep it.
//
// WHO CALLS IT: initTxGPS() and ?gpssetup step 4, only when the listen heard UBX but no NMEA (the
// symptom) and the module answers a config message (never UBX_DIALECT_MUTE). A module whose NMEA was
// heard is not touched, so the BN-220 and M100 Mini boots are unchanged. Because MUTE never reaches
// here, gpsApplyCfg()'s blind-write branch is never used: every write below is ACK-checked.
//
// NEVER FATAL: the outcome is REPORTED; a module that refuses these still navigates on its defaults.
// Input: dialect (LEGACY or VALSET). Output: none. Side effects: up to five ACK-checked writes and two
// or three Serial lines. Cost: a few milliseconds when the module answers; if it stops answering, up
// to ~3.5 s at 115200 and ~7 s at 9600. Loop task (boot or a serial command) only.
// ============================================================
static void gpsEnableNmeaOut(uint8_t dialect)
{
  static const byte on = 1;    // U1 rate 1 = one sentence per nav epoch; type L 1 = enabled

  // 0. The old form first, on a module that answers it. Same baud as the one the module was heard
  //    on, so this changes the output mask and nothing else.
  const char *prt = nullptr;
  if (dialect == UBX_DIALECT_LEGACY) {
    byte f[28];
    gpsBuildCfgPrt(f, gps_current_baud);
    const uint8_t r = ubxSendAcked(f, sizeof(f), 2);
    prt = (r == UBX_ACK) ? "OK" : (r == UBX_NAK) ? "REJECTED" : "no-ACK";
  }

  // 1. GGA and RMC in the form the module answers. UBX-CFG-MSG, 3-byte payload: msgClass F0
  //    (standard NMEA), msgID 00 = GGA / 04 = RMC, rate 1. The checksum is computed here, never typed
  //    in (it comes out FB 10 for GGA and FF 18 for RMC).
  byte enableGGA[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0x00, 0x00 };
  byte enableRMC[] = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x01, 0x00, 0x00 };
  ubxAppendChecksum(enableGGA, sizeof(enableGGA));
  ubxAppendChecksum(enableRMC, sizeof(enableRMC));
  const char *s_gga  = gpsApplyCfg(dialect, enableGGA, sizeof(enableGGA), KEY_MSGOUT_NMEA_GGA_U1, &on, 1);
  const char *s_rmc  = gpsApplyCfg(dialect, enableRMC, sizeof(enableRMC), KEY_MSGOUT_NMEA_RMC_U1, &on, 1);
  const bool ok_gga  = (strcmp(s_gga, "OK") == 0 || strcmp(s_gga, "OK/valset") == 0);
  const bool ok_rmc  = (strcmp(s_rmc, "OK") == 0 || strcmp(s_rmc, "OK/valset") == 0);
  // 2-3. Output protocols, NMEA last.
  const bool ok_ubx  = (ubxValset(KEY_UART1OUTPROT_UBX,  &on, 1, 2) == UBX_ACK);
  const bool ok_nmea = (ubxValset(KEY_UART1OUTPROT_NMEA, &on, 1, 2) == UBX_ACK);

  Serial.printf("TX GPS NMEA output on UART1: %s%s%sGGA %s | RMC %s | UBX-out %s | NMEA-out %s\n",
                prt ? "CFG-PRT " : "", prt ? prt : "", prt ? " | " : "",
                s_gga, s_rmc,
                ok_ubx  ? "OK/valset" : "REJECTED",
                ok_nmea ? "OK/valset" : "REJECTED");

  // The verdict says only what is known: NMEA output is back on if CFG-PRT or the NMEA-out write was
  // accepted, and GGA and RMC are known to be on only if their writes were accepted.
  const bool prt_ok   = (prt != nullptr && strcmp(prt, "OK") == 0);
  const bool proto_on = prt_ok || ok_nmea;
  if (proto_on && ok_gga && ok_rmc)
    Serial.println("TX GPS: NMEA output re-enabled on UART1 — GGA and RMC are the two sentences "
                   "this firmware parses.");
  else if (proto_on)
    Serial.printf("TX GPS: NMEA output switched back on (%s), but the GGA and RMC rates were not "
                  "confirmed (see their fields above). If ?gpsraw shows binary but no $G lines, "
                  "they are still off and need u-center.\n",
                  prt_ok ? "CFG-PRT, UBX+NMEA" : "CFG-VALSET NMEA-out");
  else
    Serial.println("TX GPS: !! NMEA output could NOT be fully re-enabled. If ?gpsraw shows binary "
                   "but no $G lines, this module is still UBX-only — a flight controller leaves "
                   "them that way — and needs u-center to recover.");
}

// ============================================================
// GPS-PIN-1 - FIND WHICH PIN THE GPS TRANSMITS ON (G18 or G19)
// V2.5-Evo - 2026-10-23. See the gps_rx_pin block at the top of this file for the bug, and Common/GpsPinDetect.h for
// the verdict rules.
//
// HOW: with the UART closed, both pins are plain inputs with the internal pull-up on, so the remote drives NEITHER pin
// and can never fight a GPS output. A CHANGE interrupt on each pin counts edges for kGpsPinWindowMs. Counting edges
// works for NMEA and for binary UBX, at any baud: it never needs to understand a byte.
//
// PULL-DOWN BOARDS - V2.5-Evo - 2026-10-23 - audit L-1 (this comment used to say the pins rest at the UART idle
// level, high; on these boards that is not true). Boards with 10 k pull-downs on these two lines (R23/R24) beat the
// internal pull-up (~45 k, the strongest the C3 has), so the line that goes INTO the GPS's receive pin reads LOW
// (~0.6 V) for the whole ~1.1 s window. To the GPS that is one UART break per init, not a stream of framing errors.
// On the DIY wiring that line is G19, whose USB D+ pull-up most likely keeps it high anyway; on the factory wiring it
// is G18, which reads LOW. Plain INPUT would be worse: both lines would float, and on G19 it also switches the USB D+
// pull-up off.
//
// WINDOW, 1100 ms: the same reasoning as gpsProbeAt() - a module at its 1 Hz default sends one burst per second, so
// the window must outlast one second.
//
// LOAD: at 115200 baud a busy GPS line can toggle ~50,000 times a second. Once a pin has kGpsPinEdgeCap edges that is
// proof enough, so its interrupt is detached at the next 10 ms check and stops costing anything. Worst case is roughly
// 1,000 interrupt calls per pin, a few ms of CPU in total.
//
// SINGLE CORE: the waits are vTaskDelay(10), so the loop task yields the whole time. The LoRa task (priority 5), the
// ADC task (6), the display/bargraph tasks and the idle task all run normally; nothing busy-waits and the watchdog is
// fed as usual.
//
// SHARED GPIO INTERRUPT SERVICE (magnet on GPIO 9, LoRa DIO1 on GPIO 3): attachInterrupt()/detachInterrupt() in the
// ESP32 Arduino core 3.3.8 add and remove the handler of ONE pin (gpio_isr_handler_add/_remove) on the single shared
// GPIO interrupt service. Nothing here uninstalls that service or touches any pin other than G18 and G19, so the
// magnet interrupt (Hall.ino magEdgeIsr, attached from loop() after boot) and the radio interrupt keep running and
// keep their exact edge times. At boot this check runs in setup(), before loop() has ever attached the magnet
// interrupt. On ?gpsreinit it runs while the magnet interrupt may be attached; the only effect there is ~1.1 s more of
// the loop() pause that command already causes.
//
// PERIPHERAL MANAGER (core 3.3.8): Serial1.end() detaches both UART pins; pinMode() then claims them as GPIO.
// detachInterrupt() removes the handler and sets the interrupt type back to DISABLE. The next Serial1.begin() sees the
// pins owned by the GPIO bus, clears that (gpioDetachBus, nothing to undo) and attaches its own RX/TX - the same path
// every first begin() at boot already takes. The pull-up is left on until then so the GPS's receive line is not left
// floating; on pull-down boards it reads LOW until begin() drives it (see PULL-DOWN BOARDS above).
//
// Inputs: none (pins P_U1_RX / P_U1_TX). Outputs: gps_rx_pin, gps_tx_pin, gps_pin_verdict, gps_pin_edges_rx/_tx,
// gps_pin_checked. Side effects: closes Serial1; prints one line; blocks the calling (loop) task ~1.1 s.
// ============================================================
static const uint32_t kGpsPinWindowMs = 1100u;   // how long both pins are watched, ms
static const uint32_t kGpsPinEdgeCap  = 512u;    // edges after which a pin is proven and its interrupt is detached

// Edge counters, written only by the two ISRs below, read by the loop task. One aligned 32-bit word each, so a read
// cannot tear on this core. Zeroed only while no interrupt is attached.
static volatile uint32_t gps_isr_edges_rx = 0;   // edges on P_U1_RX (G18)
static volatile uint32_t gps_isr_edges_tx = 0;   // edges on P_U1_TX (G19)

// Counting ISRs. One increment each: never block, never call anything. Same attribute as magEdgeIsr() in Hall.ino.
static void ARDUINO_ISR_ATTR gpsPinEdgeIsrRx() { gps_isr_edges_rx = gps_isr_edges_rx + 1; }
static void ARDUINO_ISR_ATTR gpsPinEdgeIsrTx() { gps_isr_edges_tx = gps_isr_edges_tx + 1; }

static void gpsDetectDataPin()
{
  // 1. Release the UART so the remote drives neither pin. Safe when Serial1 was never opened (boot).
  Serial1.end();

  // 2. Both pins as inputs with the internal pull-up (on pull-down boards the line into the GPS still reads LOW - see
  //    PULL-DOWN BOARDS above). Counters zeroed BEFORE any interrupt is attached.
  pinMode(P_U1_RX, INPUT_PULLUP);
  pinMode(P_U1_TX, INPUT_PULLUP);
  vTaskDelay(pdMS_TO_TICKS(2));          // let the pull-ups settle so the first edge counted is a real one
  gps_isr_edges_rx = 0;
  gps_isr_edges_tx = 0;
  attachInterrupt(digitalPinToInterrupt(P_U1_RX), gpsPinEdgeIsrRx, CHANGE);
  attachInterrupt(digitalPinToInterrupt(P_U1_TX), gpsPinEdgeIsrTx, CHANGE);
  bool rx_attached = true;
  bool tx_attached = true;

  // 3. Watch. Yield every 10 ms; detach a pin as soon as it has proven itself, so a fast GPS stops costing CPU.
  const uint32_t start = millis();
  while ((uint32_t)(millis() - start) < kGpsPinWindowMs)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (rx_attached && gps_isr_edges_rx >= kGpsPinEdgeCap) { detachInterrupt(digitalPinToInterrupt(P_U1_RX)); rx_attached = false; }
    if (tx_attached && gps_isr_edges_tx >= kGpsPinEdgeCap) { detachInterrupt(digitalPinToInterrupt(P_U1_TX)); tx_attached = false; }
    if (!rx_attached && !tx_attached) break;   // both proven: the verdict (BOTH) cannot change any more
  }

  // 4. Detach whatever is still attached. From here no ISR touches the counters, so the snapshot below is final.
  if (rx_attached) detachInterrupt(digitalPinToInterrupt(P_U1_RX));
  if (tx_attached) detachInterrupt(digitalPinToInterrupt(P_U1_TX));
  gps_pin_edges_rx = gps_isr_edges_rx;
  gps_pin_edges_tx = gps_isr_edges_tx;

  // 5. Decide and apply. Only a clean "signal on G19 and not on G18" moves the UART; everything else keeps the
  //    default, which is exactly what every remote did before this change.
  gps_pin_verdict = gpsPinVerdict(gps_pin_edges_rx, gps_pin_edges_tx, kGpsPinMinEdges);
  gps_rx_pin      = gpsPinRxFor(gps_pin_verdict, P_U1_RX, P_U1_TX);
  gps_tx_pin      = gpsPinTxFor(gps_pin_verdict, P_U1_RX, P_U1_TX);
  gps_pin_checked = true;

  const unsigned d_rx = P_U1_RX, d_tx = P_U1_TX, rx = gps_rx_pin, tx = gps_tx_pin;
  const unsigned long e_rx = gps_pin_edges_rx, e_tx = gps_pin_edges_tx;
  if (gps_pin_verdict == GPS_PIN_VERDICT_SWAPPED || gps_pin_verdict == GPS_PIN_VERDICT_DEFAULT)
    Serial.printf("TX GPS: data on G%u, using RX=G%u TX=G%u (%s) [edges G%u=%lu G%u=%lu]\n",
                  rx, rx, tx, gpsPinVerdictLabel(gps_pin_verdict), d_rx, e_rx, d_tx, e_tx);
  else if (gps_pin_verdict == GPS_PIN_VERDICT_BOTH)
    Serial.printf("TX GPS: data on BOTH G%u and G%u - cannot tell which is the GPS, using default RX=G%u TX=G%u "
                  "[edges G%u=%lu G%u=%lu]\n", d_rx, d_tx, rx, tx, d_rx, e_rx, d_tx, e_tx);
  else
    Serial.printf("TX GPS: no data on G%u or G%u, using default RX=G%u TX=G%u [edges G%u=%lu G%u=%lu]\n",
                  d_rx, d_tx, rx, tx, d_rx, e_rx, d_tx, e_tx);
}

// ------------------------------------------------------------
// gpsPrintNoNmeaHint - what to tell the user when no NMEA was heard at any baud.
// V2.5-Evo - 2026-10-23 - GPS-PIN-1. Replaces four copies of "check that TX/RX are not swapped". The firmware now finds
// the GPS on G18 or G19 by itself, so that advice can only send someone to cut open a remote for nothing - and on a
// potted factory remote there are no wires to swap. What is left depends on what the last pin check saw:
//   no check yet  -> it has not looked: the remote booted with gps_en=0, and initTxGPS() (boot and ?gpsreinit) skips
//                    the check while gps_en=0. The GPS has to be turned on first.
//                    V2.5-Evo - 2026-10-23 - audit L-2: this used to say "run ?gpsreinit", which does nothing while
//                    gps_en=0.
//   no signal     -> nothing is transmitting on either pin: power, GND, or a dead / missing module.
//   signal seen   -> the module IS talking, just not NMEA at a listed baud (UBX-only output, or an unlisted baud).
//                    V2.5-Evo - 2026-10-27 - UBX-only output is now heard by the scan, so only the
//                    unlisted baud (or a line that is not a u-blox stream) is left here.
//                    V2.5-Evo - 2026-10-23 - audit L-6: says "power and the GPS's transmit wire are fine" (edges prove
//                    those, not the wire into the GPS), and that ?gpssetup / ?gpsbaud set cannot repair it: both
//                    start with the same baud scan (gpsDetectBaud(); NMEA or UBX since 2026-10-27) and stop when
//                    it finds nothing.
// Input: prefix printed before each line (matches the caller's indentation). Side effects: two or three Serial lines.
// ------------------------------------------------------------
static void gpsPrintNoNmeaHint(const char *prefix)
{
  const unsigned d_rx = P_U1_RX, d_tx = P_U1_TX;
  if (!gps_pin_checked) {
    Serial.printf("%sThe GPS was off at boot (gps_en=0), so G%u/G%u were never checked. Turn it on first:\n",
                  prefix, d_rx, d_tx);
    Serial.printf("%s?set gps_en 1, ?save, wait about 15 s, then power-cycle or ?gpsreinit.\n", prefix);
  } else if (gps_pin_verdict == GPS_PIN_VERDICT_NONE) {
    Serial.printf("%sNo signal on G%u or G%u. Both pins were checked, so swapped data wires are not the cause.\n",
                  prefix, d_rx, d_tx);
    Serial.printf("%sCheck 3.3V, GND and that the module is alive (blue LED blinking), then ?gpsreinit.\n", prefix);
  } else if (gps_pin_verdict == GPS_PIN_VERDICT_BOTH) {
    Serial.printf("%sSignal on BOTH G%u and G%u (edges %lu / %lu), so the default pins were used. Something other\n",
                  prefix, d_rx, d_tx, (unsigned long)gps_pin_edges_rx, (unsigned long)gps_pin_edges_tx);
    Serial.printf("%sthan the GPS is toggling one of them, or the two lines are linked. Run ?gpsreinit to check again.\n",
                  prefix);
  } else {
    // V2.5-Evo - 2026-10-27 - the scan now hears a UBX-only module too (gpsProbeAt()), so that is no
    // longer a reason to land here: what is left is a baud outside the scan list, or a line that is
    // not a u-blox stream. The three lines used to name UBX-only output first.
    Serial.printf("%sData seen on G%u (edges G%u=%lu G%u=%lu), but no NMEA or UBX at any listed baud:\n",
                  prefix, (unsigned)gps_rx_pin, d_rx, (unsigned long)gps_pin_edges_rx,
                  d_tx, (unsigned long)gps_pin_edges_tx);
    Serial.printf("%sthe module is likely on a baud outside the scan list. Power and the GPS's transmit wire are fine.\n",
                  prefix);
    Serial.printf("%s?gpssetup and ?gpsbaud set cannot fix this: they also need to hear the module at a listed baud.\n", prefix);
  }
}

// ============================================================
// initTxGPS - Initialize Serial1 GPS based on usrConf.gps_chip_type
// ============================================================
//
// What it does:
//   V2.5-Evo - 2026-10-27 - rewritten for the TX GPS pass (the list said type 2 sends a
//   four-constellation CFG-GNSS and type 0 moves the baud first; neither is true any more).
//   1) finds which pin the GPS sends on (G18 or G19), then LISTENS for the module (NMEA or
//      UBX, nothing sent). Type 0 (BN-220) falls back to the blind dual-baud dance only when
//      nothing is heard; type 2 (M10) sends nothing until the module is heard.
//   2) dynModel (CFG-NAV5), which is also the dialect probe: old UBX-CFG form or CFG-VALSET.
//   3) a module heard below 115200 is moved there in the form it answers (three results:
//      OK / STAYED / LOST; see gpsSetModuleBaud()).
//   4) GSV, GLL and VTG off, then the rate (5 Hz type 0, 10 Hz type 2, gpsSelectedMeasMs()),
//      every write ACK-checked.
//   5) a module heard in UBX only gets its NMEA output switched back on (gpsEnableNmeaOut()).
//   No constellation request, in either form: the module keeps its own set.
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
//   V2.5-Evo - 2026-07-31 - this block was REWRITTEN. Every one of its five claims had become
//   false across the 2026-07-29/30 GPS work, and two consecutive audits flagged it. A stale
//   header on a safety-relevant function is not a cosmetic problem: the next person reasons
//   from it, and here it described a firmware that no longer exists.
//
//   - Calls Serial1.setRxBufferSize(512) before the first begin(). Note this is a no-op on any
//     later call — the core refuses a resize once the driver is installed — but the value
//     persists across end()/begin(), so 512 holds for the session.
//   - LISTENS FIRST. Both chip types now probe for NMEA (listen-only, no transmission) before
//     sending anything. Typical cost ~200 ms when the module answers on the first baud tried.
//   - The dual-baud dance is now a FALLBACK, reached only when listening hears nothing at any
//     baud. It is the sole remaining path that transmits at an unconfirmed baud.
//   - Blocking time is therefore variable, not the old fixed ~750 ms: roughly 300 ms in the
//     normal case, and several seconds in the module-absent case, which is bounded and
//     reported rather than silent.
//   - EVERY UBX config write is ACK-VERIFIED, and a NAK triggers an automatic retry through
//     the CFG-VALSET interface for u-blox M9/M10. tx_gps_initialized = true means "the UART is
//     configured", NOT "the module accepted the settings" — that question is answered by the
//     per-write boot report and, independently, by ?gpscfg.
//   - V2.5-Evo - 2026-10-23 - GPS-PIN-1: FIRST of all, gpsDetectDataPin() finds which pin the GPS
//     transmits on (G18 or G19) with the UART closed, and sets gps_rx_pin / gps_tx_pin for every
//     Serial1.begin() that follows. Adds ~1.1 s (fixed) to every init, at boot and on ?gpsreinit.
//   - V2.5-Evo - 2026-10-27 - BOOT TIME, with the 1.1 s pin check included: a healthy module heard
//     at 115200 takes ~1.3-1.5 s. A module heard below 115200 adds the baud move: ~1.3 s when it
//     lands, ~2.5 s when it stays put (~5 s with the one CFG-VALSET retry after an old-form move).
//     A move that loses the module adds a ~6 s rescan (~12-15 s in all, up to ~18 s when the retry
//     ran first). No module at all: ~13 s on type 2, ~14 s on type 0 (two full scans; type 0 also
//     runs the dual-baud dance between them). The NMEA repair, only on a module heard in UBX only,
//     adds a few ms, or up to ~3.5 s at 115200 / ~7 s at 9600 if the module stops answering.
//     WATCHDOG: initWatchdog() runs on the line after this call in setup() and arms nothing (the
//     Arduino core's task watchdog is initialised but watches neither the idle task nor the loop
//     task on this build), and every wait here yields (delay(1), vTaskDelay), so no length of this
//     function can trip it. initTasks() runs BEFORE this call, so the radio and ADC tasks are
//     already running while it blocks the loop task.
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

  // V2.5-Evo - 2026-10-23 - GPS-PIN-1: find the GPS on G18 or G19 BEFORE anything opens Serial1 or sends a byte.
  // Factory-made remotes have the GPS transmitting into G19, the DIY wiring into G18; the old code assumed G18 and
  // heard nothing on a factory remote. Every init path comes through here (boot from setup(), and ?gpsreinit), so
  // the check is redone each time. See gpsDetectDataPin() above.
  gpsDetectDataPin();

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

  // V2.5-Evo - 2026-10-27 - WHERE the module was heard and IN WHAT (GPS_SAW_NMEA / GPS_SAW_UBX), from
  // whichever listen proved the baud: the chip branch's, or the listen after the switch. The baud
  // move after the dialect probe reads `heard`; the NMEA repair reads `heard_proto` (UBX heard, no
  // NMEA = the symptom). Both are updated by the rescan if a baud move loses the module.
  uint32_t heard       = 0;
  uint8_t  heard_proto = 0;

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
      // V2.5-Evo - 2026-10-27 - from gpsSelectedMeasMs() (200 ms for type 0), the one place the
      // period is decided, so ?gpssetup and ?gpscfg's readback cannot disagree with boot.
      meas_ms = gpsSelectedMeasMs();

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
      //
      // V2.5-Evo - 2026-10-27 - THE MOVE IS NO LONGER MADE HERE. It used to go out right after this
      // listen, as old-form CFG-PRT, before anything had shown the module answers config messages,
      // and its failure was ignored ("failed, staying put"). It now runs once, after the dialect
      // probe, in the form the module answers, with three results (OK / STAYED / LOST) - see the
      // BAUD block after the probe. This branch only listens, and sends nothing when it hears.
      // ============================================================
      uint32_t seen_baud = gpsDetectBaud(1100, 115200, &heard_proto);
      if (seen_baud) {
        Serial.printf("TX GPS [BN-220]: heard the module at %lu (%s) — skipping the dual-baud dance\n",
                      (unsigned long)seen_baud, gpsProtoName(heard_proto));
        heard = seen_baud;
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
      Serial1.begin(115200, SERIAL_8N1, gps_rx_pin, gps_tx_pin);   // V2.5-Evo - 2026-10-23 - GPS-PIN-1: detected pins
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
      Serial1.begin(9600, SERIAL_8N1, gps_rx_pin, gps_tx_pin);   // V2.5-Evo - 2026-10-23 - GPS-PIN-1: detected pins
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
      Serial1.begin(115200, SERIAL_8N1, gps_rx_pin, gps_tx_pin);   // V2.5-Evo - 2026-10-23 - GPS-PIN-1: detected pins
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
    // Type 2: M10 — auto-detect the baud (V2.5-Evo - 2026-10-27 - listen only; a module below
    // 115200 is moved after the dialect probe, and no constellations are requested)
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
      Serial.println("TX GPS [M10]: listening first; nothing is sent until the module is heard");
      // gpsDetectBaud() guarantees the port is left at GPS_BAUD_PREFERRED if nothing answers,
      // so the config block below still runs and reports honestly instead of failing silently.
      uint32_t found = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &heard_proto);

      if (found == 0) {
        // V2.5-Evo - 2026-10-23 - GPS-PIN-1: no more "check that TX/RX are not swapped" - the pin check found the
        // orientation already.
        // V2.5-Evo - 2026-10-23 - audit L-6: the hint is NOT printed here any more. This path always reaches the
        // "baud not confirmed" block after the switch, which listens once more and prints the hint there, so it
        // appeared twice. It is printed once, by that block, and only if the second listen also hears nothing.
        // V2.5-Evo - 2026-10-27 - "in NMEA or UBX": the listen hears a UBX-only module too.
        Serial.println("TX GPS [M10]: !! nothing heard at any baud, in NMEA or UBX. Run ?gpsbaud to scan, ?gpsraw to see bytes.");
      } else {
        Serial.printf("TX GPS [M10]: module answers at %lu baud (%s)\n", (unsigned long)found,
                      gpsProtoName(heard_proto));
        heard = found;
        baud_confirmed = true;         // heard it; transmitting is now safe
      }

      // ============================================================
      // V2.5-Evo - 2026-10-27 - NOTHING IS SENT FROM THIS BRANCH ANY MORE.
      //
      // BAUD: a module heard below 38400 used to be moved here, before the dialect was known, by
      // sending the move in BOTH forms one after the other (CFG-VALSET, then old-form CFG-PRT) and
      // keeping whichever proof landed; a failed proof was ignored. The move now runs once, after the
      // dialect probe, in the form the module answers, with three results (OK / STAYED / LOST) - see
      // the BAUD block after the probe - and for anything below 115200, not only below 38400.
      //
      // CONSTELLATIONS: the four-constellation UBX-CFG-GNSS (GPS + Galileo + BeiDou + GLONASS) that
      // was sent here is GONE, and no CFG-SIGNAL key replaces it: the module keeps its own set. On an
      // M10 that accepts the old form (the owner's HGLRC M100 Mini does) that request could land, and
      // u-blox rates an M10 with all four on at 4 Hz, below the 10 Hz asked for below; with BeiDou B1I
      // and GLONASS together it is an invalid set. HGLRC ships GPS + Galileo, which u-blox rates at
      // 10 Hz. Same ruling as the receiver (audit 2026-10-27 §4.3).
      // ============================================================

      // V2.5-Evo - 2026-07-29 - GPS-ACK-1: 10 Hz. Applied after the switch, in the dialect
      // the module speaks. On a genuine M10 the legacy CFG-RATE frame that used to be sent
      // here does not exist and was being NAKed, so this path was silently running at the
      // module default of 1 Hz — not the 10 Hz the boot message claimed.
      // V2.5-Evo - 2026-10-27 - from gpsSelectedMeasMs() (100 ms for type 2).
      meas_ms = gpsSelectedMeasMs();

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
    // V2.5-Evo - 2026-08-18 - GPS-DYN-TX. Was a pre-checksummed literal with dynModel pinned to
    // 0x05 (Sea) and checksum 0x86/0x51. Both are now built from usrConf.gps_dyn_model, because
    // a variable dynModel byte invalidates a fixed checksum and a bad checksum fails silently.
    byte setNav5Sea[GPS_NAV5_FRAME_LEN];
    const uint8_t dyn_sel = gpsBuildNav5(setNav5Sea);

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
    // V2.5-Evo - 2026-10-27 - the ACK line names the FORM, not the chip: the owner's HGLRC M100 Mini
    // (an M10) ACKs the old form too, and "REMOVED" is too strong for M10 (see the UBX helpers note).
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
      uint32_t found = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &heard_proto);
      if (found) {
        Serial.printf("TX GPS: heard the module at %lu baud (%s)\n", (unsigned long)found,
                      gpsProtoName(heard_proto));
        heard = found;
        baud_confirmed = true;
      }
    }

    if (!baud_confirmed) {
      // Nothing anywhere. Say so and send NOTHING. Blind writes here would be aimed at a
      // baud we have no evidence for, and the only thing they can reliably achieve is
      // framing errors on a module that may be perfectly healthy but slow to wake.
      // V2.5-Evo - 2026-10-23 - audit L-6: "GPS not detected" only when the pin check also saw nothing. When it saw
      // data on a pin, the GPS WAS detected - it just does not speak NMEA at a listed baud - and the hint below says so.
      // V2.5-Evo - 2026-10-27 - "no NMEA or UBX": the listen hears a UBX-only module too.
      if (gps_pin_verdict == GPS_PIN_VERDICT_DEFAULT || gps_pin_verdict == GPS_PIN_VERDICT_SWAPPED)
        Serial.println("TX GPS: !! no NMEA or UBX at any baud.");
      else
        Serial.println("TX GPS: !! no NMEA or UBX at any baud — GPS not detected.");
      Serial.println("TX GPS: !! NOTHING was sent. Transmitting at an unconfirmed baud is what");
      Serial.println("TX GPS: !! disables a u-blox receiver, so silence is the safe response.");
      // V2.5-Evo - 2026-10-23 - GPS-PIN-1: was "Check 3.3V, GND and that TX/RX are not swapped, then ?gpsbaud".
      gpsPrintNoNmeaHint("TX GPS: !! ");
      return;
    }

    // V2.5-Evo - 2026-10-27 - a module heard in UBX only is confirmed like any other (gpsProbeAt() now
    // recognises a checksum-valid UBX frame), which is what lets the NMEA repair at the end of this
    // block run at a baud PROVEN by listening. Whether it runs depends on the module answering a
    // config message, which is not known yet here - so this says so, and the "NMEA output" line after
    // the config report (or a "NOT switched back on" line) says what was done.
    if (!(heard_proto & GPS_SAW_NMEA)) {
      gpsReportUbxOnly(heard);
      Serial.println("TX GPS: !! NMEA output is switched back on below if the module answers a config");
      Serial.println("TX GPS: !! message - see the NMEA line after the config report.");
    }

    uint8_t probe = ubxSendAcked(setNav5Sea, sizeof(setNav5Sea), 3);

    if (probe == UBX_ACK) {
      dialect     = UBX_DIALECT_LEGACY;
      nav5_status = "OK";
    } else if (probe == UBX_NAK) {
      dialect = UBX_DIALECT_VALSET;
      // V2.5-Evo - 2026-08-18 - GPS-DYN-TX. Was a hard-coded `const byte sea = 5`, which meant
      // the M9/M10 path silently ignored gps_dyn_model — on exactly the modern hardware the
      // setting exists for. Same defect the RX carried until GPS-DYN-2. M10 SPG 5.10, Table 23:
      // CFG-NAVSPG-DYNMODEL constants SEA = 5, AUTOMOT = 4.
      const byte dyn_valset = dyn_sel;
      nav5_status = (ubxValset(KEY_NAVSPG_DYNMODEL, &dyn_valset, 1, 3) == UBX_ACK)
                    ? "OK/valset" : "REJECTED";
    } else {
      dialect = UBX_DIALECT_MUTE;
      Serial1.write(setNav5Sea, sizeof(setNav5Sea));   // blind, as before
      Serial1.flush();
      delay(20);
      nav5_status = "no-ACK";
    }

    // ============================================================
    // V2.5-Evo - 2026-10-27 - BAUD, IN THE FORM THE MODULE ANSWERS, AFTER THE PROBE.
    //
    // WHAT WAS WRONG: the type 0 branch moved a module heard below 115200 with old-form CFG-PRT
    // before anything had shown it answers config messages; the type 2 branch moved one heard below
    // 38400 by sending CFG-VALSET and then old-form CFG-PRT, one after the other, before the form was
    // known. Both ignored a failed move: gpsSetModuleBaud() listened back at the old baud for 300 ms
    // (shorter than one 1 Hz epoch, so it proved nothing) and threw the answer away, so every write
    // after it went out at a baud nobody had proven.
    //
    // NOW: one move, here, only when the module was heard below 115200 and answered the probe above
    // (a MUTE module answers nothing, so a move could not be checked and is not tried), sent in the
    // form it answered. The move is proven by LISTENING at the new baud, never by the ACK, which can
    // be lost across the switch. Three results (see gpsSetModuleBaud()):
    //   GPS_MOVE_STAYED after an old-form CFG-PRT: an M10 that keeps old CFG-NAV5 may still refuse
    //     old CFG-PRT, so the move is tried ONCE more in CFG-VALSET, at the baud just proven. An M8
    //     refuses or ignores that frame; at a proven baud it adds no framing errors.
    //   GPS_MOVE_STAYED otherwise: carry on at the old baud; the link-budget line below says whether
    //     the rate fits there.
    //   GPS_MOVE_LOST: rescan with gpsDetectBaud() (listen only) before anything else is sent. If
    //     nothing answers, send NOTHING more and return: the port is left at 115200 by the rescan.
    // Cost when it runs: one frame plus a 1.1 s listen; +1.1 s when the move does not land; +2.3 s
    // for the retry; +~6 s for the rescan. RAM|BBR only - ?gpssetup's save makes it permanent.
    // ============================================================
    if (heard < GPS_BAUD_PREFERRED && dialect != UBX_DIALECT_MUTE) {
      Serial.printf("TX GPS: heard at %lu - moving the module to %lu (%s) ... ",
                    (unsigned long)heard, (unsigned long)GPS_BAUD_PREFERRED,
                    dialect == UBX_DIALECT_VALSET ? "CFG-VALSET" : "CFG-PRT");
      uint8_t mv = gpsSetModuleBaud(dialect, heard, GPS_BAUD_PREFERRED);
      if (mv == GPS_MOVE_STAYED && dialect == UBX_DIALECT_LEGACY) {
        Serial.printf("not taken, heard again at %lu; trying once in CFG-VALSET ... ",
                      (unsigned long)heard);
        mv = gpsSetModuleBaud(UBX_DIALECT_VALSET, heard, GPS_BAUD_PREFERRED);
      }
      if (mv == GPS_MOVE_OK) {
        Serial.println("OK, heard at the new baud (RAM/BBR; ?gpssetup saves it)");
      } else if (mv == GPS_MOVE_STAYED) {
        Serial.printf("failed, heard again at %lu and staying there\n", (unsigned long)heard);
      } else if (mv == GPS_MOVE_LOST) {
        Serial.println("LOST - heard at neither baud. Rescanning; nothing is sent until it is heard.");
        heard = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &heard_proto);
        if (!heard) {
          Serial.println("TX GPS: !! the module was lost during the baud move and is not heard at any baud.");
          Serial.println("TX GPS: !! NOTHING more was sent. Power-cycle the remote and its GPS, then run ?gpsbaud.");
          return;
        }
        Serial.printf("TX GPS: found again at %lu baud (%s)\n", (unsigned long)heard,
                      gpsProtoName(heard_proto));
      }
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
    //
    // V2.5-Evo - 2026-10-27 - THE BUG: this printed "only (100 - used)% clear" with both sides
    // unsigned, so on a line that is over-full (10 Hz at 9600 needs ~148% of it) the subtraction
    // wrapped and printed a number near 4.3 billion. It now prints the share USED, as the receiver
    // does (no subtraction, so nothing can wrap; the largest product is 14200 x 100 = 1.42e6). It
    // reads gps_current_baud, which every listen sets (gpsOpenAt()), so this is the baud AFTER the
    // move above: it speaks up for a move that stayed put, a module that answers no config message
    // (no move tried), or one a rescan found elsewhere. GSA (one per constellation) is not counted.
    if (meas_ms > 0 && gps_current_baud > 0) {
      const uint32_t need = (1000UL / meas_ms) * 142UL * 10UL;       // bit/s for GGA+RMC, 8N1
      const uint32_t used = need * 100UL / gps_current_baud;         // % of the line
      if (used > 70UL)
        Serial.printf("TX GPS: !! at %lu baud, GGA+RMC alone at %u Hz need ~%lu%% of the line - "
                      "sentences WILL drop. Run ?gpssetup to move the module to 115200.\n",
                      (unsigned long)gps_current_baud, (unsigned)(1000 / meas_ms),
                      (unsigned long)used);
    }

    // --- Report every write, so a rejected config is visible at boot rather than shipped. ---
    // V2.5-Evo - 2026-08-18 - GPS-DYN-TX. Was the literal "dynModel=Sea", which printed Sea even
    // on a board configured for Automotive — reassurance in the exact spot a failure shows up.
    // V2.5-Evo - 2026-10-27 - "legacy CFG form" (was "legacy CFG (u-blox 6/7/8)"): the label names the
    // FORMAT the module answered in, not the chip - the M100 Mini, an M10, answers the old form.
    // ?gpscfg's silicon line (UBX-MON-VER) names the chip.
    Serial.printf("TX GPS config [%s]: dynModel=%s %s | rate=%ums %s | GLL %s | GSV %s | VTG %s\n",
                  dialect == UBX_DIALECT_LEGACY ? "legacy CFG form"
                : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                                : "UNVERIFIED - module sends no ACK",
                  gpsDynModelName(dyn_sel),
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

    // ============================================================
    // V2.5-Evo - 2026-10-27 - NMEA OUTPUT, KEYED TO THE SYMPTOM (ported from the receiver, whose
    // audit of 2026-10-27 graded the format-keyed version MEDIUM).
    //
    // The remote never wrote the output-protocol configuration, so a module left UBX-only by a
    // flight controller gave it no GPS on every boot. Now, LAST, after every write above has been
    // answered:
    //   - the listen heard UBX but no NMEA (the symptom) and the module answers config messages:
    //     gpsEnableNmeaOut() in whichever form it answers (old form: CFG-PRT at the proven baud
    //     first, ACK-checked; then GGA/RMC; then the CFG-VALSET NMEA-out writes);
    //   - a module whose NMEA was heard is NOT touched, whatever form it answers, so the BN-220 and
    //     M100 Mini boots send nothing new (the receiver also writes these to every CFG-VALSET module
    //     on every boot; the remote never did, and does not start now);
    //   - a MUTE module with the symptom is skipped: nothing sent to it could be checked. The log
    //     says so instead of promising a repair.
    // heard_proto is the last listen's answer, the rescan's if a lost baud move needed one.
    // ============================================================
    const bool ubx_only = !(heard_proto & GPS_SAW_NMEA);
    if (dialect != UBX_DIALECT_MUTE && ubx_only)
      gpsEnableNmeaOut(dialect);        // RAM|BBR
    else if (ubx_only)
      Serial.println("TX GPS: !! NMEA output NOT switched back on: the module answered no config "
                     "message, so nothing sent to it could be checked. Run ?gpsbaud; u-center can "
                     "switch NMEA output back on.");
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
// V2.5-Evo - 2026-10-27 - and two more, as on the receiver:
//   UBX-CFG-RATE (0x06 0x08, empty payload) -> measRate in ms, on a module that answered the
//                old-form poll; CFG-VALGET CFG-RATE-MEAS (key echo checked) on one that answered
//                VALGET. Not polled when neither dynModel poll was answered.
//   UBX-MON-VER  (0x0A 0x04) -> swVersion, hwVersion and the extension strings, so the remote says
//                which silicon is fitted. The dialect does NOT say that: the M100 Mini, an M10,
//                answers the old form.
// ABORT: 'quit' is checked before each new poll begins, never between a request and its reply
// (ubxPoll() writes and reads inside one call), so nothing is left half-asked. Nothing is
// transmitted on the way out, and nothing here changes the baud or the module's settings.
//
// ⚠️ WARNING: blocks the main loop for up to ~6 s (V2.5-Evo - 2026-10-27 - was ~3 s: four 1.5 s
// polls can go unanswered - legacy NAV5, VALGET dynModel, VALGET rate, MON-VER; a healthy module
// answers each in milliseconds). Bench use only. Type 'quit' to abort.
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
// V2.5-Evo - 2026-08-18 - GPS-DYN-TX. This is now a PURE NAME TABLE. It used to carry the
// verdict inline — "Sea <-- CORRECT for this buggy" — which was true only while Sea was
// hard-coded. On a remote configured for Automotive whose write silently failed, it printed
// reassurance in the exact spot the failure appears. The verdict moved to dynModelVerdict(),
// which compares the readback against what gps_dyn_model actually asked for.
static const char *dynModelName(uint8_t m)
{
  switch (m) {
    case 0:  return "Portable";
    case 2:  return "Stationary";
    case 3:  return "Pedestrian";
    case 4:  return "Automotive";
    case 5:  return "Sea";
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

// V2.5-Evo - 2026-08-18 - GPS-DYN-TX. The verdict, separated from the name table above so it
// can compare the READBACK against what this remote was actually configured to want. Portable
// is called out specifically: it is the factory default and the one value that must never go
// afloat, whatever gps_dyn_model says.
static const char *dynModelVerdict(uint8_t m)
{
  if (m == gpsSelectedDynModel()) return "  <-- matches gps_dyn_model";
  if (m == 0) return "  <-- FACTORY DEFAULT (Portable) - the write did NOT stick";
  return "  <-- does NOT match gps_dyn_model";
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

  // Running Fletcher-8 over class..payload, so the frame can be CHECKSUM-VERIFIED
  // before it is believed. See the note at case 7.
  byte     ckA = 0, ckB = 0;

  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!Serial1.available()) { delay(1); continue; }
    byte c = Serial1.read();

    switch (state) {
      case 0: state = (c == 0xB5) ? 1 : 0; break;
      case 1: state = (c == 0x62) ? 2 : 0; break;
      case 2: cls = c; ckA = c;  ckB = ckA;          state = 3; break;
      case 3: id  = c; ckA += c; ckB += ckA;         state = 4; break;
      case 4: len = c;                   ckA += c; ckB += ckA; state = 5; break;
      case 5: len |= ((uint16_t)c << 8); ckA += c; ckB += ckA; idx = 0;
              // Not the frame we asked for (an ACK, or unrelated) — resync.
              state = (cls == wantCls && id == wantId && len <= outMax) ? 6 : 0;
              if (state == 6 && len == 0) { state = 7; }
              break;
      case 6:
        out[idx++] = c;
        ckA += c; ckB += ckA;
        if (idx >= len) state = 7;    // payload complete — now PROVE it with the checksum
        break;

      // ============================================================
      // V2.5-Evo - 2026-08-15 - Verify CK_A/CK_B before trusting the frame.
      //
      // This function used to `return len` the instant it had collected len payload
      // bytes, never reading the two checksum bytes at all. So ANY byte sequence in
      // the stream that happened to look like B5 62 <wantCls> <wantId> <plausible len>
      // was accepted as a real reply.
      //
      // That is not hypothetical, and it was not caught here first: the RX carried the
      // identical defect and reported a BN-880 (u-blox M8, which has no CFG-VALGET at
      // all) as "CFG-VALSET (u-blox M9/M10)" on three consecutive runs. Fixed on the RX
      // in 1f2ba8c (2026-08-02); the TX was left with the same bug because the parser
      // exists twice instead of once — the cost of Common/GpsUbx.h never being extracted.
      //
      // A false verdict is not cosmetic on either board: gpsDetectDialect() feeds
      // ?gpssetup, which uses it to choose which dialect to WRITE in, so an M8 would be
      // sent CFG-VALSET frames it cannot understand and the configuration silently
      // fails.
      //
      // The stream this parser runs on is full of NMEA and UBX at 5 Hz, so a chance
      // match on four header bytes is entirely plausible. Two bytes of Fletcher-8 make
      // it ~1 in 65536 instead, and cost nothing.
      //
      // No watchdog feed here, unlike the RX: the TX never arms esp_task_wdt, so there
      // is nothing to reset. Bounded by the same 1500 ms deadline as before.
      // ============================================================
      case 7:
        if (c != ckA) { state = 0; break; }          // bad CK_A — not our frame, resync
        state = 8;
        break;
      case 8:
        if (c == ckB) return len;                    // verified
        state = 0;                                   // bad CK_B — resync and keep looking
        break;
    }
  }
  return 0;
}

// ------------------------------------------------------------
// gpsValgetOk - is this a GENUINE CFG-VALGET response for the key we asked for?
// V2.5-Evo - 2026-10-27 - ported from the receiver (RX-POLL-1) for ?gpscfg's rate readback. The
// checksum in ubxPoll() proves the frame is intact; this proves it is the answer to OUR question: a
// CFG-VALGET response carries version | layer | position(2) | key(4) | value(...), so the key we
// sent must come back at bytes 4-7. Inputs: payload, its length, the key asked for. Output: true =
// the key echoes. No side effects.
// ------------------------------------------------------------
static bool gpsValgetOk(const byte *pl, uint16_t n, uint32_t key)
{
  if (n < 9) return false;
  return pl[4] == (byte)( key        & 0xFF)
      && pl[5] == (byte)((key >>  8) & 0xFF)
      && pl[6] == (byte)((key >> 16) & 0xFF)
      && pl[7] == (byte)((key >> 24) & 0xFF);
}

// ------------------------------------------------------------
// gpsHwGenName - UBX-MON-VER hwVersion (8 ASCII characters) -> u-blox generation, for ?gpscfg's
// silicon line. V2.5-Evo - 2026-10-27 - ported from the receiver. Only the published generation codes
// are named (00080000 = M8, 00190000 = M9 and F9, 000A0000 = M10; audit 2026-10-27 §4.4); anything
// else (a clone, or a part not listed) says so, and the version strings printed under it are the
// evidence. Input: pointer to the 8 hwVersion characters. Output: a name. No side effects.
// ------------------------------------------------------------
static const char *gpsHwGenName(const byte *hw)
{
  if (memcmp(hw, "00040007", 8) == 0) return "u-blox 6";
  if (memcmp(hw, "00070000", 8) == 0) return "u-blox 7";
  if (memcmp(hw, "00080000", 8) == 0) return "u-blox M8";
  if (memcmp(hw, "00190000", 8) == 0) return "u-blox M9 (the F9 parts report the same)";
  if (memcmp(hw, "000A0000", 8) == 0) return "u-blox M10";
  return "not a generation this firmware knows - read the strings below";
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
  // V2.5-Evo - 2026-10-27 - which poll answered, so the rate readback below asks in the same form,
  // and whether the operator typed 'quit' (checked before each new poll, never mid-poll).
  uint8_t cfg_dialect = UBX_DIALECT_MUTE;
  bool    aborted     = false;

  // --- Try the legacy poll first (the old UBX-CFG form). ---
  static const byte pollNav5[] = { 0xB5,0x62,0x06,0x24,0x00,0x00,0x2A,0x84 };
  uint16_t n = ubxPoll(pollNav5, sizeof(pollNav5), 0x06, 0x24, pl, sizeof(pl));

  if (n >= 3) {
    cfg_dialect = UBX_DIALECT_LEGACY;
    // V2.5-Evo - 2026-10-27 - names the format, not the chip (was "(u-blox 6/7/8)"): the M100 Mini,
    // an M10, answers the old form. The silicon line further down names the chip.
    Serial.println("  dialect  : legacy UBX-CFG form (the format it answers; see silicon below)");
    Serial.printf("  dynModel : %u  (%s)%s\n", pl[2], dynModelName(pl[2]), dynModelVerdict(pl[2]));
    Serial.printf("  fixMode  : %u  (1=2D 2=3D 3=auto)\n", pl[3]);
  } else if (checkSerialQuit()) {
    // The legacy poll already cost 1.5 s; let the operator skip the second one.
    Serial.println("  aborted after the legacy poll (no reply).");
    aborted = true;
  } else {
    // --- No legacy answer. Try the modern configuration interface (M9/M10). ---
    byte pollDyn[16];
    gpsBuildValget(pollDyn, KEY_NAVSPG_DYNMODEL);
    n = ubxPoll(pollDyn, sizeof(pollDyn), 0x06, 0x8B, pl, sizeof(pl));

    if (n >= 9) {
      cfg_dialect = UBX_DIALECT_VALSET;
      // Response payload: version | layer | position(2) | key(4) | value(...)
      Serial.println("  dialect  : CFG-VALSET/VALGET (u-blox M9/M10)");
      Serial.printf("  dynModel : %u  (%s)%s\n", pl[8], dynModelName(pl[8]), dynModelVerdict(pl[8]));
    } else {
      Serial.println("  dynModel : NO REPLY (neither CFG-NAV5 nor CFG-VALGET answered)");
      Serial.println("             The module is silent, on a different baud, or is a clone");
      Serial.println("             that implements neither. If ?gpsraw shows NMEA flowing,");
      Serial.println("             the config writes are being ignored rather than the module");
      Serial.println("             being dead.");
    }
  }

  // ============================================================
  // V2.5-Evo - 2026-10-27 - RATE READBACK (ported from the receiver).
  //
  // The rate is written ACK-checked at boot, but an ACK cannot catch a setting a later write or a
  // warm restart undid, so it is read back like dynModel: the old-form CFG-RATE poll (empty payload)
  // on a module that answered the old-form NAV5 poll, CFG-VALGET CFG-RATE-MEAS (key echo checked)
  // on one that answered VALGET. A module that answered neither is not asked: that would only add a
  // 1.5 s wait to a report that has already said the module answers no config poll.
  // "setting matches": this is the module's SETTING, not the rate it achieves. An M10 accepts a rate
  // its constellations cannot keep up with (u-blox rates GPS+Galileo+BeiDou B1I at 3 Hz). On the
  // remote, ?gpsraw for a few seconds shows how many GGA lines arrive per second.
  // ============================================================
  const uint16_t rate_want = gpsSelectedMeasMs();
  if (!aborted && cfg_dialect != UBX_DIALECT_MUTE && checkSerialQuit()) {
    Serial.println("  aborted before the rate poll.");
    aborted = true;
  }
  if (!aborted) {
    uint16_t rate_got  = 0;
    bool     rate_read = false;
    if (cfg_dialect == UBX_DIALECT_LEGACY) {
      byte pollRate[8] = { 0xB5,0x62,0x06,0x08,0x00,0x00,0x00,0x00 };   // UBX-CFG-RATE poll
      ubxAppendChecksum(pollRate, sizeof(pollRate));
      n = ubxPoll(pollRate, sizeof(pollRate), 0x06, 0x08, pl, sizeof(pl));
      if (n >= 6) { rate_got = (uint16_t)(pl[0] | (pl[1] << 8)); rate_read = true; }   // measRate U2
    } else if (cfg_dialect == UBX_DIALECT_VALSET) {
      byte pollRateV[16];
      gpsBuildValget(pollRateV, KEY_RATE_MEAS);
      n = ubxPoll(pollRateV, sizeof(pollRateV), 0x06, 0x8B, pl, sizeof(pl));
      // version | layer | position(2) | key(4) | value U2
      if (n >= 10 && gpsValgetOk(pl, n, KEY_RATE_MEAS)) {
        rate_got  = (uint16_t)(pl[8] | (pl[9] << 8));
        rate_read = true;
      }
    }
    if (rate_read)
      Serial.printf("  rate     : %u ms (%.1f Hz)%s %u ms asked for (achieved rate: ?gpsraw)\n",
                    (unsigned)rate_got,
                    rate_got ? 1000.0f / (float)rate_got : 0.0f,
                    rate_got == rate_want ? "  <-- setting matches the" : "  <-- setting is NOT the",
                    (unsigned)rate_want);
    else if (cfg_dialect == UBX_DIALECT_MUTE)
      Serial.println("  rate     : not read (the module answered no config poll above)");
    else
      Serial.println("  rate     : NO REPLY");
  }

  // ============================================================
  // V2.5-Evo - 2026-10-27 - UBX-MON-VER: WHICH SILICON (ported from the receiver).
  //
  // The dialect lines above cannot say it. MON-VER does: payload is swVersion char[30] | hwVersion
  // char[10] | then extension strings of char[30] each (firmware, protocol version, module name,
  // GNSS list). Polled here, on a port that has been open and quiet, never right after a reopen
  // (GPS-BAUD-5: a MON-VER probe straight after Serial1.end()/begin() gave false "dead" verdicts).
  // The buffer is static (BSS, not the loop task's stack) and sized for 12 extensions; a longer reply
  // is not accepted by ubxPoll() and reads NO REPLY rather than being cut. The %.10s / %.30s limits
  // keep the prints inside the fields, which carry no terminating zero when full.
  // ============================================================
  if (!aborted && checkSerialQuit()) {
    Serial.println("  aborted before the MON-VER poll.");
    aborted = true;
  }
  if (!aborted) {
    static byte ver[40 + 30 * 12];
    byte pollVer[8] = { 0xB5,0x62,0x0A,0x04,0x00,0x00,0x00,0x00 };   // UBX-MON-VER poll
    ubxAppendChecksum(pollVer, sizeof(pollVer));
    const uint16_t vn = ubxPoll(pollVer, sizeof(pollVer), 0x0A, 0x04, ver, sizeof(ver));
    if (vn >= 40) {
      Serial.printf("  silicon  : %s  (MON-VER hwVersion %.10s)\n", gpsHwGenName(&ver[30]),
                    (const char *)&ver[30]);
      Serial.printf("  sw       : %.30s\n", (const char *)ver);
      for (uint16_t i = 40; i + 30 <= vn; i += 30)
        Serial.printf("  ext      : %.30s\n", (const char *)&ver[i]);
    } else {
      Serial.println("  silicon  : NO REPLY to UBX-MON-VER (silent module, its UART receiver");
      Serial.println("             disabled, or a clone that does not implement MON-VER)");
    }
  }

  Serial.printf("  chip_type: %u (0=BN-220 2=M10, from usrConf)\n", usrConf.gps_chip_type);
  // V2.5-Evo - 2026-08-18 - GPS-DYN-TX. Was hard-coded "Expect dynModel=5 (Sea)", which told an
  // Automotive-configured owner to expect the one value the firmware was no longer sending.
  // V2.5-Evo - 2026-10-27 - and the rate, per gps_chip_type (gpsSelectedMeasMs()).
  Serial.printf("  Expect dynModel=%u (%s) and rate=%ums, per gps_dyn_model and gps_chip_type.\n",
                gpsSelectedDynModel(), gpsDynModelName(gpsSelectedDynModel()), (unsigned)rate_want);
  Serial.println("  Anything else means initTxGPS() did not stick; Portable (0) permits");
  Serial.println("  310 m/s / 50 m/s solutions and must not go afloat.");
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
//                        V2.5-Evo - 2026-10-27 - NO LONGER SAVED HERE: the move is RAM|BBR, with
//                        no Flash layer (see ubxValset()), and three results (OK / STAYED / LOST,
//                        see gpsSetModuleBaud()). ?gpssetup's step 5 asks the module to save.
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
    // V2.5-Evo - 2026-10-27 - the UBX column is back, backed by a bit that is really set now:
    // gpsProbeAt() recognises an unsolicited, checksum-valid UBX frame (nothing is transmitted to earn
    // it). Without it a UBX-only module would be "found" with "-" under NMEA and nothing saying why.
    Serial.println("  baud     NMEA  UBX");
    uint32_t found = 0;
    uint8_t  found_proto = 0;
    for (uint8_t i = 0; i < kGpsBaudCount; i++) {
      uint8_t s = gpsProbeAt(kGpsBauds[i], 1100);
      Serial.printf("  %-7lu  %-4s  %s\n", (unsigned long)kGpsBauds[i],
                    (s & GPS_SAW_NMEA) ? "yes" : "-",
                    (s & GPS_SAW_UBX)  ? "yes" : "-");
      if (s && !found) { found = kGpsBauds[i]; found_proto = s; break; }   // stop; no need to poke the rest

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
      // V2.5-Evo - 2026-10-27 - names what was heard (was "-> NMEA at"): a UBX-only module is found too.
      Serial.printf("  -> heard %s at %lu; Serial1 left open there.\n", gpsProtoName(found_proto),
                    (unsigned long)found);

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
                    d == UBX_DIALECT_LEGACY ? "alive — legacy UBX-CFG form"   // the format, not the chip (V2.5-Evo - 2026-10-27)
                  : d == UBX_DIALECT_VALSET ? "alive — CFG-VALSET (u-blox M9/M10)"
                                            : "DEAD — module is NOT accepting UBX");
      if (!ubx) {
        // V2.5-Evo - 2026-07-31 - wording corrected after the 2026-07-31 audit. This used to
        // assert that the receiver "stays that way until POWER-CYCLED" as though it were
        // vendor-documented. It is not: u-blox documents an automatic re-enable after about a
        // second. What we actually OBSERVED on 2026-07-30 is that the module stayed
        // unresponsive until it lost power. Both may be true — a module still being sprayed
        // with wrong-baud traffic would re-disable as fast as it recovers — but the honest
        // statement separates what was measured from what is claimed. The recommended action
        // is unchanged and correct under either model.
        Serial.println("     u-blox disables its UART receiver after ~100 framing errors.");
        Serial.println("     OBSERVED here: it stayed dead until power was removed. Switch off —");
        Serial.println("     a reboot alone will not clear it, the module needs its power removed.");
      } else if (found != GPS_BAUD_PREFERRED) {
        Serial.printf("     Run '?gpssetup' to move it to %lu and make it permanent.\n",
                      (unsigned long)GPS_BAUD_PREFERRED);
      }
      // V2.5-Evo - 2026-10-27 - a UBX-only module: this command only LOOKS, so it names the repair
      // rather than performing one. Boot and ?gpssetup switch NMEA output back on, but only on a
      // module that answers config messages.
      if (!(found_proto & GPS_SAW_NMEA)) {
        Serial.println("     UBX frames but no NMEA: a flight controller left this module in");
        if (!ubx) {
          Serial.println("     UBX-only mode, and it answers no config message, so neither boot");
          Serial.println("     nor ?gpssetup can switch NMEA output back on. Power-cycle it (above)");
          Serial.println("     and reboot; if it still answers nothing, it needs u-center.");
        } else {
          Serial.println("     UBX-only mode. Reboot (initTxGPS() switches NMEA output back on)");
          Serial.println("     or run ?gpssetup, which does the same and asks the module to save it.");
        }
      }
    } else {
      // The scan above walked the whole list, so the port is sitting on the LAST baud tried.
      // Put it back somewhere sane before returning — otherwise a scan that finds nothing
      // would leave the GPS unusable for the rest of the session as a side effect of merely
      // having LOOKED at it. A read-only diagnostic must not change the machine.
      gpsOpenAt(GPS_BAUD_PREFERRED);
      // V2.5-Evo - 2026-10-23 - GPS-PIN-1: the "TX/RX are not swapped" advice is gone - the pin check handles the
      // orientation. The hint now reports what that check saw.
      Serial.println("  -> NOTHING answered at any baud, in NMEA or UBX. ?gpsraw shows raw bytes.");   // V2.5-Evo - 2026-10-27 - "in NMEA or UBX"
      gpsPrintNoNmeaHint("     ");
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
    Serial.println("       ?gpsbaud set <rate> move the MODULE (RAM + battery-backed; ?gpssetup saves)");   // V2.5-Evo - 2026-10-27 - no Flash layer
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
                    d == UBX_DIALECT_LEGACY ? "alive — legacy UBX-CFG form"   // the format, not the chip (V2.5-Evo - 2026-10-27)
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
                dialect == UBX_DIALECT_LEGACY ? "legacy UBX-CFG form"   // the format, not the chip (V2.5-Evo - 2026-10-27)
              : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                              : "unknown - module answers no config poll");

  // An unknown dialect is still worth attempting: try the modern form, then the legacy one.
  // V2.5-Evo - 2026-10-27 - THREE RESULTS, NO FLASH LAYER (see gpsSetModuleBaud()). The second form is
  // tried only after STAYED, i.e. only when the module has just been heard again at cur, so it goes
  // out at a proven baud: after an old-form move that is the one CFG-VALSET retry boot also makes,
  // on an unknown dialect it is the old form after CFG-VALSET (as before). Never after LOST: nothing
  // is proven there, so nothing more is sent. This used to retry on any failure and to say the baud
  // had been written to the module's flash.
  uint8_t mv = (dialect == UBX_DIALECT_LEGACY)
                 ? gpsSetModuleBaud(UBX_DIALECT_LEGACY, cur, rate)
                 : gpsSetModuleBaud(UBX_DIALECT_VALSET, cur, rate);
  if (mv == GPS_MOVE_STAYED && dialect != UBX_DIALECT_VALSET)
    mv = gpsSetModuleBaud(dialect == UBX_DIALECT_LEGACY ? UBX_DIALECT_VALSET : UBX_DIALECT_LEGACY, cur, rate);

  if (mv == GPS_MOVE_OK) {
    Serial.printf("  OK — module is now at %lu (its live settings, plus battery-backed memory\n",
                  (unsigned long)rate);
    Serial.println("  on a CFG-VALSET module; the Flash layer is never asked for). To keep it");
    Serial.println("  across a power cycle run ?gpssetup, which asks the module to save its settings");
    Serial.println("  (permanent only on a module with its own flash; the HGLRC M100s have none).");
    Serial.println("  Reboot the TX so initTxGPS() runs cleanly at the new speed, then ?gpscfg.");
  } else if (mv == GPS_MOVE_STAYED) {
    Serial.printf("  FAILED — not heard at %lu afterwards; heard again at %lu, so the module\n",
                  (unsigned long)rate, (unsigned long)cur);
    Serial.printf("  kept its old baud and the GPS is no worse off. Serial1 is left at %lu.\n",
                  (unsigned long)cur);
  } else if (mv == GPS_MOVE_LOST) {
    Serial.printf("  LOST — heard at neither %lu nor %lu afterwards. Nothing more was sent.\n",
                  (unsigned long)rate, (unsigned long)cur);
    Serial.println("  Run ?gpsbaud to find it (it leaves the port where the module answers), or");
    Serial.println("  power-cycle the remote and its GPS, then ?gpsreinit.");
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
// V2.5-Evo - 2026-10-27 - corrected: steps 3 and 4 write RAM and battery-backed memory, like boot (no
// Flash layer, see ubxValset()); step 5 asks the module to save, which is permanent only where it
// has flash (the HGLRC M100s have none). And boot does not merely verify: initTxGPS() re-applies
// every setting on every boot, so a module that lost them is put right at the next power-up.
// Step 3 now has three results (OK / STAYED / LOST, see gpsSetModuleBaud()); a module heard in UBX
// only gets its NMEA output switched back on in step 4 (gpsEnableNmeaOut()).
//
// Deliberately NOT stored in usrConf: adding a field would push sizeof(confStruct) past its
// pinned 136 bytes, forcing a SW_VERSION bump, which RESETS the TX SPIFFS config — pairing,
// calibration, everything. The module has its own non-volatile memory; the setting lives
// there, costs nothing, and follows the module if it is moved to another remote.
//
// ⚠️ Blocks the main loop for up to ~15 s. Bench only, USB only. 'quit' aborts between steps.
// V2.5-Evo - 2026-10-27 - ~15 s is the usual time; with every listen and poll running to its timeout
// the worst case is about 40 s (scan ~6 s, dialect ~3 s, move with retry and re-check ~8 s, writes
// and the NMEA repair ~7 s each at 9600, save ~2 s, readback ~6 s). A module that answers promptly
// takes a few seconds.
// ============================================================
void cmdGpsSetup(const String &args)
{
  (void)args;
  Serial.println("===== GPS one-time setup =====");

  // --- 1. Find the module ---
  Serial.println("[1/6] locating module...");
  // V2.5-Evo - 2026-10-27 - cur_proto: what the listen heard (NMEA / UBX), so step 4 can repair a
  // module heard in UBX only.
  uint8_t  cur_proto = 0;
  uint32_t cur = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &cur_proto);
  if (!cur) {
    // V2.5-Evo - 2026-10-23 - GPS-PIN-1: the "TX/RX are not swapped" advice is gone - the pin check handles the
    // orientation. The hint now reports what that check saw.
    Serial.println("  FAILED: nothing answered at any baud, in NMEA or UBX.");   // V2.5-Evo - 2026-10-27 - "in NMEA or UBX"
    gpsPrintNoNmeaHint("  ");
    Serial.printf("  Serial1 restored to %lu.\n", (unsigned long)GPS_BAUD_PREFERRED);
    return;
  }
  Serial.printf("  found at %lu baud (%s)\n", (unsigned long)cur, gpsProtoName(cur_proto));
  if (!(cur_proto & GPS_SAW_NMEA)) {
    gpsReportUbxOnly(cur);
    Serial.println("TX GPS: !! step [4/6] switches NMEA output back on if the module answers config");
    Serial.println("TX GPS: !! messages (see [2/6]); step [5/6] asks the module to save it.");
  }

  // --- 2. Which dialect? ---
  uint8_t dialect = gpsDetectDialect();
  Serial.printf("[2/6] dialect: %s\n",
                dialect == UBX_DIALECT_LEGACY ? "legacy UBX-CFG form"   // the format, not the chip (V2.5-Evo - 2026-10-27)
              : dialect == UBX_DIALECT_VALSET ? "CFG-VALSET (u-blox M9/M10)"
                                              : "UNKNOWN - answers no config poll");
  if (dialect == UBX_DIALECT_MUTE) {
    Serial.println("  The module replies to neither CFG-NAV5 nor CFG-VALGET, so nothing can");
    Serial.println("  be verified. Refusing to write blind — that is what got us here.");
    return;
  }
  if (checkSerialQuit()) { Serial.println("  aborted before any write."); return; }

  // --- 3. Raise the baud, if needed ---
  // V2.5-Evo - 2026-10-27 - THREE RESULTS, NO FLASH LAYER (see gpsSetModuleBaud()). STAYED: the module
  // is proven at the old baud; an old-form move is tried once more in CFG-VALSET there (the retry
  // boot makes), then the steps below write at whatever baud is proven. LOST: heard at neither baud,
  // so no baud is left that it is proven on - the command STOPS and sends nothing more. After a move
  // that lands, a dialect re-check that answers nothing stops the command the way step [2/6] does.
  Serial.printf("[3/6] baud: ");
  if (cur == GPS_BAUD_PREFERRED) {
    Serial.printf("already %lu\n", (unsigned long)GPS_BAUD_PREFERRED);
  } else {
    Serial.printf("%lu -> %lu (%s) ... ", (unsigned long)cur, (unsigned long)GPS_BAUD_PREFERRED,
                  dialect == UBX_DIALECT_VALSET ? "CFG-VALSET" : "CFG-PRT");
    uint8_t mv = gpsSetModuleBaud(dialect, cur, GPS_BAUD_PREFERRED);
    if (mv == GPS_MOVE_STAYED && dialect == UBX_DIALECT_LEGACY) {
      Serial.printf("not taken, heard again at %lu; trying once in CFG-VALSET ... ", (unsigned long)cur);
      mv = gpsSetModuleBaud(UBX_DIALECT_VALSET, cur, GPS_BAUD_PREFERRED);
    }
    if (mv == GPS_MOVE_OK) {
      Serial.println("OK");
      cur = GPS_BAUD_PREFERRED;
      // The dialect probe is cheap and the link just changed underneath us; re-confirm.
      dialect = gpsDetectDialect();
      if (dialect == UBX_DIALECT_MUTE) {
        Serial.println("  dialect at the new baud: UNKNOWN - answers no config poll");
        Serial.println("  Nothing can be verified, so nothing will be written. Refusing to write");
        Serial.println("  blind. Power-cycle the remote and its GPS, then run ?gpssetup again.");
        Serial.println("  NMEA output was not switched back on.");
        return;
      }
    } else if (mv == GPS_MOVE_STAYED) {
      Serial.printf("FAILED, heard again at %lu and staying there\n", (unsigned long)cur);
      Serial.println("  Continuing at the old baud — a slow link is better than none, but");
      Serial.println("  10 Hz will drop sentences. See the warning at the end.");
    } else if (mv == GPS_MOVE_LOST) {
      Serial.println("LOST");
      Serial.println("  The module was heard at neither baud after the move, so nothing more is");
      Serial.println("  sent. Run ?gpsbaud to find it (it leaves the port where the module");
      Serial.println("  answers), or power-cycle the remote and its GPS, then run ?gpssetup again.");
      return;
    }
  }

  // --- 4. Apply everything (RAM + battery-backed; step 5 saves) ---
  // V2.5-Evo - 2026-10-27 - was "(persisted)": the writes no longer ask for the Flash layer.
  Serial.println("[4/6] applying config (RAM + battery-backed; step [5/6] saves it)...");
  // V2.5-Evo - 2026-08-18 - GPS-DYN-TX. Second copy of the pre-checksummed Sea literal, now
  // built from usrConf.gps_dyn_model like the boot path. ?gpssetup writes to the module's OWN
  // non-volatile memory, so a hard-coded Sea here would have persisted the wrong model into the
  // hardware permanently — surviving reflash, and invisible until someone read it back.
  byte setNav5Sea[GPS_NAV5_FRAME_LEN];
  const uint8_t dyn_sel = gpsBuildNav5(setNav5Sea);

  static const byte disableGSV[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15};
  static const byte disableGLL[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11};
  static const byte disableVTG[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19};
  static const byte off = 0;
  const byte dyn_valset = dyn_sel;

  // Same ordering rule as boot: dynModel first (it is the setting that must not be missed),
  // then silence the NMEA chatter, then the rate once the link is quiet.
  // V2.5-Evo - 2026-10-27 - the trailing persist=true is gone from all five calls (no Flash layer).
  const char *s_nav5 = gpsApplyCfg(dialect, setNav5Sea, sizeof(setNav5Sea),
                                   KEY_NAVSPG_DYNMODEL, &dyn_valset, 1);
  const char *s_gsv  = gpsApplyCfg(dialect, disableGSV, sizeof(disableGSV),
                                   KEY_MSGOUT_NMEA_GSV_U1, &off, 1);
  const char *s_gll  = gpsApplyCfg(dialect, disableGLL, sizeof(disableGLL),
                                   KEY_MSGOUT_NMEA_GLL_U1, &off, 1);
  const char *s_vtg  = gpsApplyCfg(dialect, disableVTG, sizeof(disableVTG),
                                   KEY_MSGOUT_NMEA_VTG_U1, &off, 1);

  // V2.5-Evo - 2026-10-27 - gpsSelectedMeasMs() (was its own copy of the type 2 -> 100 ms rule).
  const uint16_t meas_ms = gpsSelectedMeasMs();
  byte setRate[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    (byte)(meas_ms & 0xFF), (byte)(meas_ms >> 8),
    0x01, 0x00, 0x01, 0x00, 0x00, 0x00
  };
  ubxAppendChecksum(setRate, sizeof(setRate));
  const byte rate_val[2] = { (byte)(meas_ms & 0xFF), (byte)(meas_ms >> 8) };
  const char *s_rate = gpsApplyCfg(dialect, setRate, sizeof(setRate),
                                   KEY_RATE_MEAS, rate_val, 2);

  Serial.printf("  dynModel=%s %s | GSV %s | GLL %s | VTG %s | rate=%ums %s\n",
                gpsDynModelName(dyn_sel), s_nav5, s_gsv, s_gll, s_vtg, meas_ms, s_rate);

  // V2.5-Evo - 2026-10-27 - NMEA output, the same rule as initTxGPS(): only when step 1 heard UBX but
  // no NMEA (the symptom), in whichever form the module answers; a module whose NMEA was heard is not
  // touched. MUTE cannot reach this line (steps 2 and 3 stop on it); the test is kept so the rule
  // reads the same in both places. RAM|BBR - step 5 saves it. Last, so the NMEA stream restarts only
  // once every write above has been answered.
  const bool ubx_only = !(cur_proto & GPS_SAW_NMEA);
  if (dialect != UBX_DIALECT_MUTE && ubx_only)
    gpsEnableNmeaOut(dialect);

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
