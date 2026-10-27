// V2.5-Evo - 2026-10-27 - RX GPS rate and baud on every dialect (audit 2026-10-09, Part 2): a SOURCE-TEXT GUARD over
//   Source/V2_Integration_Rx/GPS.ino. The host cannot compile the receiver's .ino files (they need Arduino), so this
//   reads GPS.ino as TEXT, with every comment and every string's contents blanked, and pins what the fix rests on:
//     R1 the rate goes out through gpsApplyCfg(..., KEY_RATE_MEAS, rate_val, 2) in configureGPS(), AFTER the dialect
//        probe, the baud move and the GSV/GLL/VTG filters; ?gpssetup makes the same call with persist=true;
//     R2 no blind rate write is left: no setRate5Hz / setRate10Hz, no pre-checksummed CFG-RATE frame, and setRate is
//        only ever declared, checksummed and handed to gpsApplyCfg();
//     R3 the period per type: gpsSelectedMeasMs() is 100 ms for gps_chip_type 2/3 and 200 ms otherwise, and both
//        writers and the ?gpscfg verdict read it; the runtime frame is the old pre-checksummed frame byte for byte
//        (200 ms -> DE 6A, 100 ms -> 7A 12);
//     R4 no constellation request: no CFG-SIGNAL key (0x1031xxxx, or a name with SIGNAL), no CFG-GNSS frame (06 3E);
//     R5 the boot line prints "rate=%ums %s" with meas_ms and rate_status, and so does ?gpssetup's line;
//     R6 types 2/3 transmit nothing in their branch, and after the switch nothing is transmitted before the listen
//        (gpsDetectBaud) and the "nothing heard -> return" gate; the type 0/1 branch writes only CFG-PRT (setBaud);
//     R7 the baud move: gpsSetModuleBaud() writes CFG-UART1-BAUDRATE on VALSET, proves the move by listening
//        (gpsProbeAt(to_baud, 1100)) and reads no ACK; configureGPS() calls it only below 115200 and never on a MUTE
//        module; ?gpssetup calls it persisted and writes no raw frame of its own;
//     R8 ?gpscfg reads the rate back (CFG-RATE poll 06 08 with an empty payload, CFG-VALGET KEY_RATE_MEAS with the key
//        echo checked) and polls UBX-MON-VER (0A 04).
//   RED RUNS: the same checker runs on the real file after one targeted edit per rule (a blind write put back, the
//   rate call commented out, a CFG-SIGNAL key added, ...) and each must FAIL the rule it breaks. Given a path as the
//   first argument it checks that file instead, which is how the pre-fix GPS.ino is shown to fail.
//   A file that cannot be read, or an anchor a red edit needs that is missing, is a FAIL, not a skip.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in the host-test note of the compile procedure):
//   g++ -std=c++17 -Wall -Wextra -o rx_gps_rate_baud_guard_test rx_gps_rate_baud_guard_test.cpp && ./rx_gps_rate_baud_guard_test

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// srcIdChar - true for a character that can be part of a C identifier.
static bool srcIdChar(char c) { return std::isalnum((unsigned char)c) || c == '_'; }

// srcCode - the text with every comment and the contents of every string and character literal replaced by spaces
// (newlines kept), so a search finds code only. Same length as the input, so a position in it is a position in the
// file. Handles //, /* */, "..." and '...' with escapes. (Same routine as throttle_byte_test.cpp, without raw strings,
// which GPS.ino does not use.)
static std::string srcCode(const std::string& t)
{
  std::string o = t;
  const size_t n = t.size();
  size_t i = 0;
  while (i < n) {
    const char c = t[i];
    if (c == '/' && i + 1 < n && t[i + 1] == '/') {
      while (i < n && t[i] != '\n') o[i++] = ' ';
    } else if (c == '/' && i + 1 < n && t[i + 1] == '*') {
      o[i] = ' '; o[i + 1] = ' '; i += 2;
      while (i < n && !(t[i] == '*' && i + 1 < n && t[i + 1] == '/')) { if (t[i] != '\n') o[i] = ' '; i++; }
      if (i < n) { o[i] = ' '; o[i + 1] = ' '; i += 2; }
    } else if (c == '"' || c == '\'') {
      i++;
      while (i < n && t[i] != c && t[i] != '\n') {
        if (t[i] == '\\' && i + 1 < n) o[i++] = ' ';
        o[i++] = ' ';
      }
      i++;
    } else {
      i++;
    }
  }
  return o;
}

// srcFindIdent - every position where the identifier id stands on its own in code (not inside a longer name).
static std::vector<size_t> srcFindIdent(const std::string& code, const char* id)
{
  std::vector<size_t> at;
  const size_t len = std::strlen(id);
  for (size_t p = code.find(id); p != std::string::npos; p = code.find(id, p + 1))
    if ((p == 0 || !srcIdChar(code[p - 1])) && (p + len >= code.size() || !srcIdChar(code[p + len]))) at.push_back(p);
  return at;
}

// srcSquash - every run of white space turned into one space, none at either end.
static std::string srcSquash(const std::string& s)
{
  std::string o;
  bool gap = false;
  for (char c : s) {
    if (std::isspace((unsigned char)c)) { gap = true; continue; }
    if (gap && !o.empty()) o += ' ';
    o += c;
    gap = false;
  }
  return o;
}

// srcTight - no white space at all, lower case: for byte sequences written with any spacing or hex case.
static std::string srcTight(const std::string& s)
{
  std::string o;
  for (char c : s) if (!std::isspace((unsigned char)c)) o += (char)std::tolower((unsigned char)c);
  return o;
}

// srcMatch - position of the bracket that closes the one at open ('(' or '{'), or npos.
static size_t srcMatch(const std::string& code, size_t open)
{
  const char o = code[open], c = (o == '(') ? ')' : '}';
  int depth = 0;
  for (size_t i = open; i < code.size(); i++) {
    if (code[i] == o) depth++;
    else if (code[i] == c && --depth == 0) return i;
  }
  return std::string::npos;
}

// Range - [b, e) of a piece of the file. ok=false when it was not found.
struct Range { bool ok; size_t b; size_t e; };

// srcFuncBody - the inside of the braces of the DEFINITION of fn (a call or a prototype is skipped).
static Range srcFuncBody(const std::string& code, const char* fn)
{
  for (size_t p : srcFindIdent(code, fn)) {
    size_t q = p + std::strlen(fn);
    while (q < code.size() && std::isspace((unsigned char)code[q])) q++;
    if (q >= code.size() || code[q] != '(') continue;
    const size_t close = srcMatch(code, q);
    if (close == std::string::npos) continue;
    size_t b = close + 1;
    while (b < code.size() && std::isspace((unsigned char)code[b])) b++;
    if (b >= code.size() || code[b] != '{') continue;
    const size_t e = srcMatch(code, b);
    if (e == std::string::npos) continue;
    return { true, b + 1, e };
  }
  return { false, 0, 0 };
}

// srcBlockAfter - the inside of the { } block that directly follows the text `label` (only white space between),
// searched from `from` up to `to`.
static Range srcBlockAfter(const std::string& code, const char* label, size_t from, size_t to)
{
  const size_t p = code.find(label, from);
  if (p == std::string::npos || p >= to) return { false, 0, 0 };
  size_t b = p + std::strlen(label);
  while (b < to && std::isspace((unsigned char)code[b])) b++;
  if (b >= to || code[b] != '{') return { false, 0, 0 };
  const size_t e = srcMatch(code, b);
  if (e == std::string::npos || e > to) return { false, 0, 0 };
  return { true, b + 1, e };
}

// srcCount - how many times needle occurs in the squashed code of [r.b, r.e).
static int srcCount(const std::string& code, Range r, const std::string& needle)
{
  if (!r.ok) return 0;
  const std::string s = srcSquash(code.substr(r.b, r.e - r.b));
  int n = 0;
  for (size_t p = s.find(needle); p != std::string::npos; p = s.find(needle, p + 1)) n++;
  return n;
}

// srcPos - position (in the file) of the first occurrence of needle in code inside r, or npos. needle is matched
// against the code with white space ignored, so the result is the position of its first character.
static size_t srcPos(const std::string& code, Range r, const std::string& needle)
{
  if (!r.ok) return std::string::npos;
  std::string tight;
  std::vector<size_t> map;
  for (size_t i = r.b; i < r.e; i++)
    if (!std::isspace((unsigned char)code[i])) { tight += code[i]; map.push_back(i); }
  std::string nt;
  for (char c : needle) if (!std::isspace((unsigned char)c)) nt += c;
  const size_t p = tight.find(nt);
  return (p == std::string::npos) ? std::string::npos : map[p];
}

static const char* kTx[] = { "Serial1.write(", "Serial1.begin(", "Serial1.print", "ubxSendAckedT(", "ubxValset(",
                             "gpsApplyCfg(", "gpsSetModuleBaud(", "ubxPoll(", "gpsSaveConfig(" };

// srcFirstTx - position of the first transmitting call in r, or npos.
static size_t srcFirstTx(const std::string& code, Range r)
{
  size_t best = std::string::npos;
  for (const char* t : kTx) {
    const size_t p = srcPos(code, r, t);
    if (p < best) best = p;
  }
  return best;
}

// fletcher - UBX Fletcher-8 over class..payload of a frame given without its two checksum bytes.
static uint16_t fletcher(const std::vector<int>& f)
{
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < f.size(); i++) { a = (uint8_t)(a + f[i]); b = (uint8_t)(b + a); }
  return (uint16_t)((a << 8) | b);
}

// hexList - the 0xNN tokens of an initializer, in order; false if any token is not a plain hex byte.
static bool hexList(const std::string& init, std::vector<int>* out)
{
  out->clear();
  size_t i = 0;
  while (i < init.size()) {
    while (i < init.size() && (std::isspace((unsigned char)init[i]) || init[i] == ',')) i++;
    if (i >= init.size()) break;
    size_t j = i;
    while (j < init.size() && init[j] != ',') j++;
    const std::string tok = srcSquash(init.substr(i, j - i));
    char* end = nullptr;
    const long v = std::strtol(tok.c_str(), &end, 16);
    if (tok.size() < 3 || tok[0] != '0' || (tok[1] != 'x' && tok[1] != 'X') || *end != '\0' || v < 0 || v > 255)
      return false;
    out->push_back((int)v);
    i = j;
  }
  return true;
}

// Guard - one flag per rule; all true means the file keeps every rule.
struct Guard { bool r1, r2, r3, r4, r5, r6, r7, r8; };
static bool guardOk(const Guard& g) { return g.r1 && g.r2 && g.r3 && g.r4 && g.r5 && g.r6 && g.r7 && g.r8; }

static Guard guard(const std::string& text)
{
  Guard g = { false, false, false, false, false, false, false, false };
  const std::string code = srcCode(text);
  const Range cfg   = srcFuncBody(code, "configureGPS");
  const Range setup = srcFuncBody(code, "cmdGpsSetup");
  const Range rdbk  = srcFuncBody(code, "cmdGpsCfg");
  const Range meas  = srcFuncBody(code, "gpsSelectedMeasMs");
  const Range smb   = srcFuncBody(code, "gpsSetModuleBaud");
  const Range prt   = srcFuncBody(code, "gpsBuildCfgPrt");
  if (!cfg.ok || !setup.ok || !rdbk.ok) return g;

  // ---- R1: one rate call in configureGPS(), after probe, baud move and filters; ?gpssetup persisted ----
  {
    const std::string call = "gpsApplyCfg(dialect, setRate, sizeof(setRate), KEY_RATE_MEAS, rate_val, 2)";
    const size_t rate  = srcPos(code, cfg, call);
    const size_t probe = srcPos(code, cfg, "ubxSendAckedT(setNav5Sea");
    const size_t baud  = srcPos(code, cfg, "gpsSetModuleBaud(dialect, heard");
    const size_t vtg   = srcPos(code, cfg, "KEY_MSGOUT_NMEA_VTG_U1");
    g.r1 = srcCount(code, cfg, call) == 1 && rate != std::string::npos && probe < rate && baud < rate && vtg < rate &&
           srcCount(code, setup, "gpsApplyCfg(dialect, setRate, sizeof(setRate), KEY_RATE_MEAS, rate_val, 2, true)") == 1;
  }

  // ---- R2: no blind rate write anywhere ----
  {
    bool ok = srcFindIdent(code, "setRate5Hz").empty() && srcFindIdent(code, "setRate10Hz").empty();
    // Every CFG-RATE frame with a 6-byte payload is the runtime-built one.
    const std::string tight = srcTight(code);
    const std::string hdr = "0xb5,0x62,0x06,0x08,0x06,0x00,";
    int frames = 0;
    for (size_t p = tight.find(hdr); p != std::string::npos; p = tight.find(hdr, p + 1)) {
      frames++;
      ok = ok && tight.compare(p + hdr.size(), 20, "(byte)(meas_ms&0xff)") == 0;
    }
    // setRate only declared, checksummed or handed to gpsApplyCfg().
    for (size_t p : srcFindIdent(code, "setRate")) {
      size_t b = p;
      while (b > 0 && code[b - 1] != ';' && code[b - 1] != '{' && code[b - 1] != '}') b--;
      const size_t e = code.find(';', p);
      const std::string st = srcSquash(code.substr(b, (e == std::string::npos ? code.size() : e) - b));
      ok = ok && (st.rfind("byte setRate[] = {", 0) == 0 || st == "ubxAppendChecksum(setRate, sizeof(setRate))" ||
                  st.find("gpsApplyCfg(dialect, setRate, sizeof(setRate), KEY_RATE_MEAS, rate_val, 2") != std::string::npos);
    }
    g.r2 = ok && frames == 2 && srcFindIdent(code, "setRate").size() >= 2;
  }

  // ---- R3: the period per type, read by both writers and the readback; the runtime frame is the old one ----
  {
    bool ok = meas.ok && srcSquash(code.substr(meas.b, meas.e - meas.b)) ==
              "return (usrConf.gps_chip_type == 2 || usrConf.gps_chip_type == 3) ? 100 : 200;";
    ok = ok && srcCount(code, cfg, "const uint16_t meas_ms = gpsSelectedMeasMs();") == 1 &&
         srcCount(code, setup, "const uint16_t meas_ms = gpsSelectedMeasMs();") == 1 &&
         srcCount(code, rdbk, "gpsSelectedMeasMs()") >= 1;
    const std::string layout = "byte setRate[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (byte)(meas_ms & 0xFF), "
                               "(byte)(meas_ms >> 8), 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 };";
    ok = ok && srcCount(code, cfg, layout) == 1 && srcCount(code, setup, layout) == 1;
    // The bytes that layout gives, against the two pre-checksummed literals the firmware used to send.
    const std::vector<int> f200 = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 200, 0, 1, 0, 1, 0 };
    const std::vector<int> f100 = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 100, 0, 1, 0, 1, 0 };
    ok = ok && fletcher(f200) == 0xDE6A && fletcher(f100) == 0x7A12;
    g.r3 = ok;
  }

  // ---- R4: no constellation request ----
  {
    bool ok = true;
    const std::string tight = srcTight(code);
    ok = ok && tight.find("0x1031") == std::string::npos;              // CFG-SIGNAL group 0x31, L-type keys
    ok = ok && tight.find("0x06,0x3e") == std::string::npos;           // UBX-CFG-GNSS
    ok = ok && srcFindIdent(code, "setGNSS").empty();
    for (size_t p = 0; p < code.size(); ) {                             // no identifier naming SIGNAL
      if (!srcIdChar(code[p])) { p++; continue; }
      size_t e = p;
      while (e < code.size() && srcIdChar(code[e])) e++;
      std::string id = code.substr(p, e - p);
      for (char& c : id) c = (char)std::toupper((unsigned char)c);
      if (id.find("SIGNAL") != std::string::npos) ok = false;
      p = e;
    }
    g.r4 = ok;
  }

  // ---- R5: the boot line and ?gpssetup's line print the rate ----
  {
    bool boot = false, setupline = false;
    for (int which = 0; which < 2; which++) {
      const Range r = which == 0 ? cfg : setup;
      for (size_t p = code.find("Serial.printf(", r.b); p != std::string::npos && p < r.e;
           p = code.find("Serial.printf(", p + 1)) {
        const size_t open = p + std::strlen("Serial.printf");
        const size_t close = srcMatch(code, open);
        if (close == std::string::npos) break;
        const std::string raw = text.substr(open, close - open + 1);
        const std::string args = srcSquash(code.substr(open, close - open + 1));
        const std::string tail = "(unsigned)meas_ms, rate_status)";
        if (which == 0 && raw.find("\"GPS config [%s]: dynModel=%s %s | GSV %s | GLL %s | VTG %s | rate=%ums %s\\n\"") !=
              std::string::npos &&
            args.size() >= tail.size() && args.compare(args.size() - tail.size(), tail.size(), tail) == 0)
          boot = true;
        if (which == 1 && raw.find("| rate=%ums %s\\n\"") != std::string::npos &&
            args.find("(unsigned)meas_ms, s_rate)") != std::string::npos)
          setupline = true;
      }
    }
    g.r5 = boot && setupline;
  }

  // ---- R6: types 2/3 send nothing in their branch; nothing is sent before the listen and its gate ----
  {
    const Range sw = srcBlockAfter(code, "switch (usrConf.gps_chip_type)", cfg.b, cfg.e);
    const Range m10 = sw.ok ? srcBlockAfter(code, "case 3:", sw.b, sw.e) : Range{ false, 0, 0 };
    const Range bn  = sw.ok ? srcBlockAfter(code, "case 1:", sw.b, sw.e) : Range{ false, 0, 0 };
    bool ok = sw.ok && m10.ok && bn.ok && srcFirstTx(code, m10) == std::string::npos;
    // after the switch: the listen, then the gate that returns when nothing was heard, then the first write
    const Range after = { sw.ok, sw.e + 1, cfg.e };
    const size_t listen = srcPos(code, after, "gpsDetectBaud(");
    const size_t gate   = srcPos(code, after, "if (!baud_confirmed)");
    const Range gateBlk = (gate != std::string::npos) ? srcBlockAfter(code, "if (!baud_confirmed)", gate, cfg.e)
                                                      : Range{ false, 0, 0 };
    const size_t first  = srcFirstTx(code, after);
    ok = ok && listen != std::string::npos && gateBlk.ok && srcCount(code, gateBlk, "return;") == 1 &&
         first != std::string::npos && listen < gate && gateBlk.e < first;
    // the type 0/1 branch writes CFG-PRT and nothing else
    int writes = 0;
    for (size_t p = code.find("Serial1.write(", bn.b); bn.ok && p != std::string::npos && p < bn.e;
         p = code.find("Serial1.write(", p + 1)) {
      writes++;
      const size_t close = srcMatch(code, p + std::strlen("Serial1.write"));
      ok = ok && close != std::string::npos &&
           srcSquash(code.substr(p, close - p + 1)) == "Serial1.write(setBaud, sizeof(setBaud))";
    }
    ok = ok && writes >= 1 && srcCount(code, bn, "ubxSendAckedT(") == 0 && srcCount(code, bn, "gpsApplyCfg(") == 0;
    g.r6 = ok;
  }

  // ---- R7: the baud move ----
  {
    bool ok = smb.ok && prt.ok;
    ok = ok && srcCount(code, smb, "KEY_UART1_BAUDRATE") >= 1 && srcCount(code, smb, "if (dialect == UBX_DIALECT_VALSET)") == 1 &&
         srcCount(code, smb, "gpsBuildCfgPrt(f, to_baud);") == 1 &&
         srcCount(code, smb, "if (gpsProbeAt(to_baud, 1100)) return true;") == 1 &&
         srcCount(code, smb, "ubxSendAckedT(") == 0 && srcCount(code, smb, "ubxPoll(") == 0;
    // configureGPS(): below 115200 only, never on a MUTE module
    const size_t guardAt = srcPos(code, cfg, "if (heard < GPS_BAUD_PREFERRED && dialect != UBX_DIALECT_MUTE) {");
    const size_t callAt  = srcPos(code, cfg, "gpsSetModuleBaud(dialect, heard, GPS_BAUD_PREFERRED, false)");
    const Range gb = (guardAt != std::string::npos)
                       ? srcBlockAfter(code, "if (heard < GPS_BAUD_PREFERRED && dialect != UBX_DIALECT_MUTE)", guardAt, cfg.e)
                       : Range{ false, 0, 0 };
    ok = ok && gb.ok && callAt != std::string::npos && gb.b <= callAt && callAt < gb.e &&
         srcCount(code, cfg, "gpsSetModuleBaud(") == 1;
    // ?gpssetup: persisted, in the established dialect, and no raw frame of its own
    ok = ok && srcCount(code, setup, "gpsSetModuleBaud(dialect, cur, GPS_BAUD_PREFERRED, true)") == 1 &&
         srcCount(code, setup, "Serial1.write(") == 0;
    // gpsBuildCfgPrt(115200) is the old CFG-PRT literal byte for byte (checksum C0 7E)
    if (ok) {
      const size_t t0 = code.find("tmpl[28] =", prt.b);
      const size_t ob = (t0 == std::string::npos) ? std::string::npos : code.find('{', t0);
      const size_t cb = (ob == std::string::npos) ? std::string::npos : srcMatch(code, ob);
      std::vector<int> f;
      ok = cb != std::string::npos && cb < prt.e && hexList(code.substr(ob + 1, cb - ob - 1), &f) && f.size() == 28;
      if (ok) {
        const uint32_t baud = 115200;
        f[14] = (int)(baud & 0xFF); f[15] = (int)((baud >> 8) & 0xFF); f[16] = (int)((baud >> 16) & 0xFF);
        f[17] = (int)((baud >> 24) & 0xFF);
        f.resize(26);
        const uint16_t ck = fletcher(f);
        f.push_back(ck >> 8); f.push_back(ck & 0xFF);
        const std::vector<int> old = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                       0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E };
        ok = (f == old);
      }
    }
    g.r7 = ok;
  }

  // ---- R8: ?gpscfg reads the rate back and asks for MON-VER ----
  {
    const std::string tight = srcTight(code.substr(rdbk.b, rdbk.e - rdbk.b));
    g.r8 = tight.find("{0xb5,0x62,0x06,0x08,0x00,0x00,") != std::string::npos &&
           srcCount(code, rdbk, "ubxPoll(pollRate, sizeof(pollRate), 0x06, 0x08, pl, sizeof(pl))") == 1 &&
           srcCount(code, rdbk, "(byte)( KEY_RATE_MEAS & 0xFF)") == 1 &&
           srcCount(code, rdbk, "gpsValgetOk(pl, n, KEY_RATE_MEAS)") == 1 &&
           tight.find("{0xb5,0x62,0x0a,0x04,0x00,0x00,") != std::string::npos &&
           srcCount(code, rdbk, "ubxPoll(pollVer, sizeof(pollVer), 0x0A, 0x04, ver, sizeof(ver))") == 1;
  }
  return g;
}

// readFile - the whole file as bytes. false when it cannot be opened.
static bool readFile(const std::filesystem::path& p, std::string* out)
{
  FILE* f = std::fopen(p.string().c_str(), "rb");
  if (!f) return false;
  out->clear();
  char buf[4096];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out->append(buf, n);
  std::fclose(f);
  return true;
}

// mutate - the text with the first occurrence of `from` replaced by `to`. *found says whether the anchor was there.
static std::string mutate(const std::string& t, const std::string& from, const std::string& to, bool* found)
{
  const size_t p = t.find(from);
  *found = (p != std::string::npos);
  if (!*found) return t;
  return t.substr(0, p) + to + t.substr(p + from.size());
}

// RED RUNS: one targeted edit of the real file per rule, each must fail that rule (and so the guard as a whole).
static void testRedRuns(const std::string& real)
{
  bool f = false;
  { // R1+R2: the rate written blind instead of through gpsApplyCfg()
    const std::string m = mutate(real, "gpsApplyCfg(dialect, setRate, sizeof(setRate), KEY_RATE_MEAS, rate_val, 2);",
                                 "\"blind\"; Serial1.write(setRate, sizeof(setRate));", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r1 && !g.r2 && !guardOk(g));
  }
  { // R1: the rate call commented out - a comment is not code
    const std::string m = mutate(real, "const char *rate_status = gpsApplyCfg(", "const char *rate_status = \"x\"; // gpsApplyCfg(", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r1 && !guardOk(g));
  }
  { // R1: the rate call moved in front of the dialect probe (the only change: where it sits)
    bool f2 = false;
    std::string m = mutate(real, "gpsApplyCfg(dialect, setRate, sizeof(setRate), KEY_RATE_MEAS, rate_val, 2);", "\"moved\";", &f);
    m = mutate(m, "uint8_t probe = ubxSendAckedT(setNav5Sea",
               "gpsApplyCfg(dialect, setRate, sizeof(setRate), KEY_RATE_MEAS, rate_val, 2); "
               "uint8_t probe = ubxSendAckedT(setNav5Sea", &f2);
    const Guard g = guard(m);
    CHECK(f && f2 && !g.r1 && g.r2 && !guardOk(g));
  }
  { // R2+R6: a blind 5 Hz write put back into the type 0/1 heard path
    const std::string m = mutate(real, "Serial.println(\"GPS [BN-220/880]: baud step done",
                                 "byte setRate5Hz[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, "
                                 "0x00, 0xDE, 0x6A }; Serial1.write(setRate5Hz, sizeof(setRate5Hz)); "
                                 "Serial.println(\"GPS [BN-220/880]: baud step done", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r2 && !g.r6 && !guardOk(g));
  }
  { // R3: 10 Hz asked of every type
    const std::string m = mutate(real, "usrConf.gps_chip_type == 3) ? 100 : 200;", "usrConf.gps_chip_type == 3) ? 100 : 100;", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r3 && !guardOk(g));
  }
  { // R3: the readback judged against a literal instead of the selector
    const std::string m = mutate(real, "const uint16_t rate_want = gpsSelectedMeasMs();", "const uint16_t rate_want = 200;", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r3 && !guardOk(g));
  }
  { // R4: a CFG-SIGNAL key added
    const std::string m = mutate(real, "static const uint32_t KEY_UART1_BAUDRATE",
                                 "static const uint32_t KEY_SIGNAL_GAL_ENA = 0x10310021UL;\nstatic const uint32_t KEY_UART1_BAUDRATE", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r4 && !guardOk(g));
  }
  { // R4: the same key written under an innocent name
    const std::string m = mutate(real, "static const uint32_t KEY_UART1_BAUDRATE",
                                 "static const uint32_t KEY_GAL = 0x10310021UL;\nstatic const uint32_t KEY_UART1_BAUDRATE", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r4 && !guardOk(g));
  }
  { // R4+R6: the four-constellation CFG-GNSS put back into the type 2/3 branch
    const std::string m = mutate(real, "Serial.println(\"GPS [M10]: listening first",
                                 "byte setGNSS[] = { 0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00 }; Serial1.write(setGNSS, sizeof(setGNSS)); "
                                 "Serial.println(\"GPS [M10]: listening first", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r4 && !g.r6 && !guardOk(g));
  }
  { // R5: the rate dropped from the boot line (its first occurrence; ?gpssetup's line comes later in the file)
    const std::string m = mutate(real, "| GLL %s | VTG %s | rate=%ums %s\\n\"", "| GLL %s | VTG %s\\n\"", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r5 && !guardOk(g));
  }
  { // R6: type 2/3 transmitting before the listen again
    const std::string m = mutate(real, "Serial.println(\"GPS [M10]: listening first",
                                 "Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX); Serial1.write(setNav5Sea, 1); "
                                 "Serial.println(\"GPS [M10]: listening first", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r6 && !guardOk(g));
  }
  { // R6: a write put between the switch and the listen
    const std::string m = mutate(real, "  bool     baud_confirmed = false;", "  Serial1.write(setNav5Sea, 1);\n  bool     baud_confirmed = false;", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r6 && !guardOk(g));
  }
  { // R7: the baud move made unconditional (would run on a MUTE module)
    const std::string m = mutate(real, "if (heard < GPS_BAUD_PREFERRED && dialect != UBX_DIALECT_MUTE) {", "if (heard < GPS_BAUD_PREFERRED) {", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r7 && !guardOk(g));
  }
  { // R7: the move "proven" by an ACK instead of by listening
    const std::string m = mutate(real, "  if (gpsProbeAt(to_baud, 1100)) return true;", "  if (ubxSendAckedT(f, 1, 1) == 0) return true;", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r7 && !guardOk(g));
  }
  { // R7: ?gpssetup back to a raw old-form CFG-PRT
    const std::string m = mutate(real, "if (gpsSetModuleBaud(dialect, cur, GPS_BAUD_PREFERRED, true)) {",
                                 "Serial1.write(setNav5Sea, 1); if (gpsProbeAt(GPS_BAUD_PREFERRED, 1100)) {", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r7 && !guardOk(g));
  }
  { // R8: the MON-VER poll removed
    const std::string m = mutate(real, "ubxPoll(pollVer, sizeof(pollVer), 0x0A, 0x04, ver, sizeof(ver))", "0", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r8 && !guardOk(g));
  }
  { // R8: the VALGET rate readback without the key-echo check
    const std::string m = mutate(real, "gpsValgetOk(pl, n, KEY_RATE_MEAS)", "true", &f);
    const Guard g = guard(m);
    CHECK(f && !g.r8 && !guardOk(g));
  }
  { // a file with no configureGPS() at all fails every rule
    const Guard g = guard("int x;\n");
    CHECK(!g.r1 && !g.r2 && !g.r3 && !g.r4 && !g.r5 && !g.r6 && !g.r7 && !g.r8);
  }
}

int main(int argc, char** argv)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::path gps;
  const bool given = (argc > 1);
  if (given) {
    gps = fs::path(argv[1]);
  } else {
    gps = fs::path("..") / ".." / "Source" / "V2_Integration_Rx" / "GPS.ino";
    if (!fs::is_regular_file(gps, ec))
      gps = fs::path(__FILE__).parent_path() / ".." / ".." / "Source" / "V2_Integration_Rx" / "GPS.ino";
  }
  std::printf("rx_gps_rate_baud_guard_test: reads %s\n", gps.string().c_str());
  std::string text;
  const bool read_ok = readFile(gps, &text);
  CHECK(read_ok && !text.empty());

  const Guard g = guard(text);
  std::printf("  R1 rate via gpsApplyCfg, after probe/baud/filters ... %s\n", g.r1 ? "ok" : "BROKEN");
  std::printf("  R2 no blind rate write ............................ %s\n", g.r2 ? "ok" : "BROKEN");
  std::printf("  R3 200 ms types 0/1, 100 ms types 2/3 ............. %s\n", g.r3 ? "ok" : "BROKEN");
  std::printf("  R4 no CFG-SIGNAL key, no CFG-GNSS ................. %s\n", g.r4 ? "ok" : "BROKEN");
  std::printf("  R5 boot line prints rate=%%ums %%s ................... %s\n", g.r5 ? "ok" : "BROKEN");
  std::printf("  R6 types 2/3 listen before any write .............. %s\n", g.r6 ? "ok" : "BROKEN");
  std::printf("  R7 baud move proven by listening .................. %s\n", g.r7 ? "ok" : "BROKEN");
  std::printf("  R8 ?gpscfg reads rate back + MON-VER .............. %s\n", g.r8 ? "ok" : "BROKEN");
  CHECK(g.r1); CHECK(g.r2); CHECK(g.r3); CHECK(g.r4); CHECK(g.r5); CHECK(g.r6); CHECK(g.r7); CHECK(g.r8);

  if (!given && read_ok) testRedRuns(text);   // the red edits are anchored on the fixed file

  if (fails) { std::printf("rx_gps_rate_baud_guard_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("rx_gps_rate_baud_guard_test: all %d checks passed\n", checks);
  return 0;
}
