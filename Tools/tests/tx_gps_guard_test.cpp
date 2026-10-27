// V2.5-Evo - 2026-10-27 - TX GPS pass (the remote's start-up brought in line with the receiver's 2026-10-27 GPS work): a
//   SOURCE-TEXT GUARD over Source/V2_Integration_Tx/GPS.ino. The host cannot compile the remote's .ino files (they need
//   Arduino), so this reads GPS.ino as TEXT, with every comment and every string's contents blanked, and pins:
//     T1 no constellation write: initTxGPS(), and every function it reaches through calls at any depth, carries no
//        CFG-SIGNAL key (a 0x1031xxxx literal, or a name bound to one anywhere in the file), no CFG-GNSS frame (06 3E) and
//        no identifier containing SIGNAL or GNSS. A read-only constellation check in ?gpscfg is allowed (green edit).
//     T2 the three-result baud move: gpsSetModuleBaud(dialect, from, to) returns GPS_MOVE_OK (heard at to_baud, 1100 ms),
//        GPS_MOVE_STAYED (heard again at from_baud, 1100 ms) or GPS_MOVE_LOST, reads no ACK, sends CFG-UART1-BAUDRATE or
//        gpsBuildCfgPrt(to_baud); initTxGPS() moves only inside "heard < 115200 and not MUTE", after the dialect probe
//        and before the GSV/GLL/VTG filters, with the one CFG-VALSET retry only after STAYED in the old form; the chip
//        branches move nothing; ?gpssetup makes the same move and retry; ?gpsbaud set tries a second form only after
//        STAYED; gpsBuildCfgPrt(115200) is the old CFG-PRT literal byte for byte.
//     T3 LOST sends nothing: initTxGPS()'s LOST block rescans with gpsDetectBaud() (listen only) and returns without
//        sending if nothing answers; ?gpssetup's LOST block and its "re-check answers nothing" block stop without
//        sending; nothing after ?gpsbaud set's move sends. "Sends" is worked out through the call chain: a function
//        defined in the file sends if its body writes to Serial1 or calls one that does, at any depth.
//     T4 the NMEA repair, keyed to the symptom: gpsProbeAt() recognises a checksum-valid UBX frame and still sends
//        nothing (the NMEA test and its early break come first); gpsEnableNmeaOut() sends old-form CFG-PRT built at
//        gps_current_baud through ubxSendAcked() first (old-form module only), then GGA and RMC through gpsApplyCfg()
//        (3-byte CFG-MSG F0 00 / F0 04, rate 1, checksums FB 10 / FF 18 computed at run time), then the CFG-VALSET
//        UBX-out and NMEA-out writes, and writes nothing unchecked; initTxGPS() calls it after the rate write when the
//        listen heard UBX but no NMEA and the module is not MUTE - and only then, so a healthy module is untouched; a
//        MUTE module with the symptom gets a "NOT switched back on" line; ?gpssetup step 4 makes the same call before
//        the step 5 save.
//     T5 no Flash layer: ubxValset() and gpsSetModuleBaud() write the layer byte 0x01 | 0x02; no (0x01|0x02|0x04)
//        anywhere; ubxValset() takes 4 arguments, gpsApplyCfg() 6 and gpsSetModuleBaud() 3, at every call too (a
//        persist argument cannot come back); the CFG-CFG save (gpsSaveConfig, deviceMask 0x17) is still called once
//        by ?gpssetup.
//     T6 the link-budget line: the share USED (need * 100 / gps_current_baud, no subtraction, so no underflow), printed
//        with that share, after the baud move and the rate write; checked numerically at 10 Hz on 9600 (147 %).
//     T7 ?gpscfg reads the rate back (CFG-RATE poll 06 08 with an empty payload, CFG-VALGET KEY_RATE_MEAS with the key
//        echo checked), polls UBX-MON-VER (0A 04), says "setting matches", judges against gpsSelectedMeasMs(), and
//        checks 'quit' before the rate poll and before the MON-VER poll.
//     T8 listen before any write: gpsSelectedMeasMs() is 100 ms for type 2 and 200 ms otherwise, read by initTxGPS()
//        (both branches), ?gpssetup and ?gpscfg; the type 2 branch sends nothing; the type 0 branch sends nothing when it
//        has heard the module and only CFG-PRT (setBaud) in the dual-baud dance; after the switch nothing is sent
//        before the listen and the "nothing heard -> return" gate.
//   RED RUNS: the same checker runs on the real file after one targeted edit per rule, and each must FAIL the rule it
//   breaks; green edits must pass. Given a path as the first argument it checks that file instead, which is how the
//   pre-fix GPS.ino is shown to fail. A file that cannot be read, or an anchor a red edit needs that is missing, is a
//   FAIL, not a skip. Carriage returns are dropped on reading.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in the host-test note of the compile procedure):
//   g++ -std=c++17 -Wall -Wextra -o tx_gps_guard_test tx_gps_guard_test.cpp && ./tx_gps_guard_test

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// srcIdChar - true for a character that can be part of a C identifier.
static bool srcIdChar(char c) { return std::isalnum((unsigned char)c) || c == '_'; }

// srcCode - the text with every comment and the contents of every string and character literal replaced by spaces
// (newlines kept), so a search finds code only. Same length as the input. Same routine as rx_gps_rate_baud_guard_test.cpp.
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
// searched from `from` up to `to`. The label is matched against the code as written (white space counts).
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

// srcPos - position (in the file) of the first occurrence of needle in code inside r, or npos. White space ignored.
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

// srcAllPos - every position (in the file) of needle in code inside r, white space ignored.
static std::vector<size_t> srcAllPos(const std::string& code, Range r, const std::string& needle)
{
  std::vector<size_t> out;
  if (!r.ok) return out;
  std::string tight;
  std::vector<size_t> map;
  for (size_t i = r.b; i < r.e; i++)
    if (!std::isspace((unsigned char)code[i])) { tight += code[i]; map.push_back(i); }
  std::string nt;
  for (char c : needle) if (!std::isspace((unsigned char)c)) nt += c;
  for (size_t p = tight.find(nt); p != std::string::npos; p = tight.find(nt, p + 1)) out.push_back(map[p]);
  return out;
}

// kTx - calls that put bytes on the GPS line (or reopen it at another baud), matched as written. Any other function
// defined in the file that sends is found by srcSenders() below, so this list is the floor, not the whole.
static const char* kTx[] = { "Serial1.write(", "Serial1.begin(", "Serial1.print", "ubxSendAcked(", "ubxValset(",
                             "gpsApplyCfg(", "gpsSetModuleBaud(", "ubxPoll(", "gpsSaveConfig(",
                             "gpsDetectDialect(", "gpsEnableNmeaOut(", "cmdGpsCfg(" };

// srcCalls - every call in r: (name, position) for each identifier followed by '(' that is not a keyword.
static std::vector<std::pair<std::string, size_t>> srcCalls(const std::string& code, Range r)
{
  static const char* kKeywords[] = { "if", "for", "while", "switch", "return", "sizeof", "do", "else", "case" };
  std::vector<std::pair<std::string, size_t>> out;
  if (!r.ok) return out;
  for (size_t p = r.b; p < r.e; ) {
    if (!srcIdChar(code[p]) || (p > 0 && srcIdChar(code[p - 1]))) { p++; continue; }
    size_t e = p;
    while (e < r.e && srcIdChar(code[e])) e++;
    size_t q = e;
    while (q < r.e && std::isspace((unsigned char)code[q])) q++;
    if (q < r.e && code[q] == '(') {
      const std::string fn = code.substr(p, e - p);
      bool kw = false;
      for (const char* k : kKeywords) if (fn == k) kw = true;
      if (!kw) out.push_back({ fn, p });
    }
    p = e;
  }
  return out;
}

static bool srcHas(const std::vector<std::string>& v, const std::string& s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}

// srcSenders - every function defined in the file that SENDS on the GPS line: its body writes to Serial1
// (Serial1.write / Serial1.print), or calls a function that does, all the way down. Opening, closing or reading the
// port is not sending, so gpsOpenAt(), gpsProbeAt() and gpsDetectBaud() are not in it.
static std::vector<std::string> srcSenders(const std::string& code)
{
  std::vector<std::pair<std::string, Range>> defs;
  std::vector<std::string> tried;
  for (const auto& c : srcCalls(code, Range{ true, 0, code.size() })) {
    if (srcHas(tried, c.first)) continue;
    tried.push_back(c.first);
    const Range b = srcFuncBody(code, c.first.c_str());
    if (b.ok) defs.push_back({ c.first, b });
  }
  std::vector<std::string> out;
  for (bool grew = true; grew; ) {
    grew = false;
    for (const auto& d : defs) {
      if (srcHas(out, d.first)) continue;
      bool s = srcPos(code, d.second, "Serial1.write(") != std::string::npos ||
               srcPos(code, d.second, "Serial1.print") != std::string::npos;
      for (const auto& c : srcCalls(code, d.second)) if (srcHas(out, c.first)) s = true;
      if (s) { out.push_back(d.first); grew = true; }
    }
  }
  return out;
}

// srcFirstTx - position of the first transmitting call in r, or npos: a kTx call as written, or a call to any function
// in senders.
static size_t srcFirstTx(const std::string& code, Range r, const std::vector<std::string>& senders)
{
  size_t best = std::string::npos;
  if (!r.ok) return best;
  for (const char* t : kTx) {
    const size_t p = srcPos(code, r, t);
    if (p < best) best = p;
  }
  for (const auto& c : srcCalls(code, r))
    if (c.second < best && srcHas(senders, c.first)) best = c.second;
  return best;
}

// srcCallArgs - for every call of fn in r (fn followed by '('), the number of top-level arguments. A definition's
// parameter list counts too when r covers it.
static std::vector<int> srcCallArgs(const std::string& code, Range r, const char* fn)
{
  std::vector<int> out;
  if (!r.ok) return out;
  for (size_t p : srcFindIdent(code, fn)) {
    if (p < r.b || p >= r.e) continue;
    size_t q = p + std::strlen(fn);
    while (q < code.size() && std::isspace((unsigned char)code[q])) q++;
    if (q >= code.size() || code[q] != '(') continue;
    const size_t close = srcMatch(code, q);
    if (close == std::string::npos) continue;
    int depth = 0, commas = 0;
    bool any = false;
    for (size_t i = q + 1; i < close; i++) {
      const char c = code[i];
      if (c == '(' || c == '{' || c == '[') depth++;
      else if (c == ')' || c == '}' || c == ']') depth--;
      else if (c == ',' && depth == 0) commas++;
      if (!std::isspace((unsigned char)c)) any = true;
    }
    out.push_back(any ? commas + 1 : 0);
  }
  return out;
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

// srcHasConstellationWrite - true when r holds a CFG-SIGNAL key (a 0x1031xxxx literal, or one of the names in sig,
// which are bound to such a literal somewhere in the file), a CFG-GNSS frame (06 3E), or an identifier whose name
// contains SIGNAL or GNSS.
static bool srcHasConstellationWrite(const std::string& code, Range r, const std::vector<std::string>& sig)
{
  if (!r.ok) return false;
  const std::string tight = srcTight(code.substr(r.b, r.e - r.b));
  if (tight.find("0x1031") != std::string::npos || tight.find("0x06,0x3e") != std::string::npos) return true;
  for (size_t p = r.b; p < r.e; ) {
    if (!srcIdChar(code[p])) { p++; continue; }
    size_t e = p;
    while (e < r.e && srcIdChar(code[e])) e++;
    std::string id = code.substr(p, e - p);
    for (const std::string& s : sig) if (id == s) return true;
    for (char& c : id) c = (char)std::toupper((unsigned char)c);
    if (id.find("SIGNAL") != std::string::npos || id.find("GNSS") != std::string::npos) return true;
    p = e;
  }
  return false;
}

// Guard - one flag per rule; all true means the file keeps every rule.
struct Guard { bool t1, t2, t3, t4, t5, t6, t7, t8; };
static bool guardOk(const Guard& g) { return g.t1 && g.t2 && g.t3 && g.t4 && g.t5 && g.t6 && g.t7 && g.t8; }

static const size_t NPOS = std::string::npos;

static Guard guard(const std::string& text)
{
  Guard g = { false, false, false, false, false, false, false, false };
  const std::string code = srcCode(text);
  const Range init  = srcFuncBody(code, "initTxGPS");
  const Range setup = srcFuncBody(code, "cmdGpsSetup");
  const Range rdbk  = srcFuncBody(code, "cmdGpsCfg");
  const Range bcmd  = srcFuncBody(code, "cmdGpsBaud");
  const Range smb   = srcFuncBody(code, "gpsSetModuleBaud");
  const Range prt   = srcFuncBody(code, "gpsBuildCfgPrt");
  const Range nmea  = srcFuncBody(code, "gpsEnableNmeaOut");
  const Range probe = srcFuncBody(code, "gpsProbeAt");
  const Range vset  = srcFuncBody(code, "ubxValset");
  const Range meas  = srcFuncBody(code, "gpsSelectedMeasMs");
  const Range save  = srcFuncBody(code, "gpsSaveConfig");
  if (!init.ok || !setup.ok || !rdbk.ok || !bcmd.ok) return g;
  const std::vector<std::string> senders = srcSenders(code);
  const Range all = { true, 0, code.size() };

  // Shared anchors in initTxGPS(): the dialect probe, the baud-move block, the filters, the rate write.
  const size_t probeAt = srcPos(code, init, "uint8_t probe = ubxSendAcked(setNav5Sea");
  const char*  moveLbl = "if (heard < GPS_BAUD_PREFERRED && dialect != UBX_DIALECT_MUTE)";
  const Range  moveBlk = srcBlockAfter(code, moveLbl, init.b, init.e);
  const size_t gsvAt   = srcPos(code, init, "KEY_MSGOUT_NMEA_GSV_U1");
  const size_t rateAt  = srcPos(code, init, "rate_status = gpsApplyCfg(dialect, setRate");

  // ---- T1: no constellation write in initTxGPS() or anything it reaches ----
  {
    std::vector<std::string> sig;   // names bound to a CFG-SIGNAL key anywhere in the file (NAME = 0x1031....)
    std::string low = code;
    for (char& c : low) c = (char)std::tolower((unsigned char)c);
    for (size_t p = low.find("0x1031"); p != NPOS; p = low.find("0x1031", p + 1)) {
      size_t q = p;
      while (q > 0 && std::isspace((unsigned char)code[q - 1])) q--;
      if (q == 0 || code[q - 1] != '=') continue;
      q--;
      while (q > 0 && std::isspace((unsigned char)code[q - 1])) q--;
      const size_t e = q;
      while (q > 0 && srcIdChar(code[q - 1])) q--;
      if (q < e) sig.push_back(code.substr(q, e - q));
    }
    bool ok = true;
    std::vector<std::string> seen = { "initTxGPS" }, todo = { "initTxGPS" };
    while (ok && !todo.empty()) {
      const std::string fn = todo.back();
      todo.pop_back();
      const Range body = srcFuncBody(code, fn.c_str());
      if (!body.ok) continue;
      if (srcHasConstellationWrite(code, body, sig)) ok = false;
      for (const auto& c : srcCalls(code, body))
        if (!srcHas(seen, c.first)) { seen.push_back(c.first); todo.push_back(c.first); }
    }
    g.t1 = ok;
  }

  // ---- T2: the three-result baud move ----
  {
    const std::string sq = srcSquash(code);
    bool ok = smb.ok && prt.ok &&
              sq.find("static const uint8_t GPS_MOVE_OK = 0; static const uint8_t GPS_MOVE_STAYED = 1; "
                      "static const uint8_t GPS_MOVE_LOST = 2;") != NPOS &&
              sq.find("static uint8_t gpsSetModuleBaud(uint8_t dialect, uint32_t from_baud, uint32_t to_baud)") != NPOS;
    const size_t pOk = srcPos(code, smb, "if (gpsProbeAt(to_baud, 1100)) return GPS_MOVE_OK;");
    const size_t pSt = srcPos(code, smb, "if (gpsProbeAt(from_baud, 1100)) return GPS_MOVE_STAYED;");
    const size_t pLo = srcPos(code, smb, "return GPS_MOVE_LOST;");
    ok = ok && pOk != NPOS && pSt != NPOS && pLo != NPOS && pOk < pSt && pSt < pLo &&
         srcCount(code, smb, "gpsProbeAt(") == 2 && srcCount(code, smb, "return true") == 0 &&
         srcCount(code, smb, "return false") == 0 && srcCount(code, smb, "ubxSendAcked(") == 0 &&
         srcCount(code, smb, "ubxPoll(") == 0 && srcCount(code, smb, "KEY_UART1_BAUDRATE") >= 1 &&
         srcCount(code, smb, "if (dialect == UBX_DIALECT_VALSET)") == 1 && srcCount(code, smb, "gpsBuildCfgPrt(f, to_baud);") == 1;
    // initTxGPS(): two calls, both in the "below 115200, not MUTE" block, after the probe and before the filters.
    const Range retry = moveBlk.ok ? srcBlockAfter(code, "if (mv == GPS_MOVE_STAYED && dialect == UBX_DIALECT_LEGACY)", moveBlk.b, moveBlk.e)
                                   : Range{ false, 0, 0 };
    const size_t first = srcPos(code, moveBlk, "uint8_t mv = gpsSetModuleBaud(dialect, heard, GPS_BAUD_PREFERRED);");
    ok = ok && moveBlk.ok && probeAt != NPOS && gsvAt != NPOS && probeAt < moveBlk.b && moveBlk.e < gsvAt &&
         srcCount(code, init, "gpsSetModuleBaud(") == 2 && srcCount(code, moveBlk, "gpsSetModuleBaud(") == 2 &&
         first != NPOS && retry.ok && first < retry.b &&
         srcCount(code, retry, "mv = gpsSetModuleBaud(UBX_DIALECT_VALSET, heard, GPS_BAUD_PREFERRED);") == 1;
    // ?gpssetup: the same move and the same retry, no raw frame of its own.
    const Range sretry = srcBlockAfter(code, "if (mv == GPS_MOVE_STAYED && dialect == UBX_DIALECT_LEGACY)", setup.b, setup.e);
    const size_t sfirst = srcPos(code, setup, "uint8_t mv = gpsSetModuleBaud(dialect, cur, GPS_BAUD_PREFERRED);");
    ok = ok && srcCount(code, setup, "gpsSetModuleBaud(") == 2 && sfirst != NPOS && sretry.ok && sfirst < sretry.b &&
         srcCount(code, sretry, "mv = gpsSetModuleBaud(UBX_DIALECT_VALSET, cur, GPS_BAUD_PREFERRED);") == 1 &&
         srcCount(code, setup, "Serial1.write(") == 0;
    // ?gpsbaud set: a second form only after STAYED.
    ok = ok && srcCount(code, bcmd, "gpsSetModuleBaud(") == 3 &&
         srcCount(code, bcmd, "if (mv == GPS_MOVE_STAYED && dialect != UBX_DIALECT_VALSET) mv = gpsSetModuleBaud(") == 1;
    // gpsBuildCfgPrt(115200) is the old CFG-PRT literal byte for byte (checksum C0 7E).
    if (ok) {
      const size_t t0 = code.find("tmpl[28] =", prt.b);
      const size_t ob = (t0 == NPOS) ? NPOS : code.find('{', t0);
      const size_t cb = (ob == NPOS) ? NPOS : srcMatch(code, ob);
      std::vector<int> f;
      ok = cb != NPOS && cb < prt.e && hexList(code.substr(ob + 1, cb - ob - 1), &f) && f.size() == 28;
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
    g.t2 = ok;
  }

  // ---- T3: LOST sends nothing ----
  {
    const Range lost = moveBlk.ok ? srcBlockAfter(code, "if (mv == GPS_MOVE_LOST)", moveBlk.b, moveBlk.e) : Range{ false, 0, 0 };
    const Range none = lost.ok ? srcBlockAfter(code, "if (!heard)", lost.b, lost.e) : Range{ false, 0, 0 };
    bool ok = lost.ok && srcFirstTx(code, lost, senders) == NPOS &&
              srcCount(code, lost, "heard = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &heard_proto);") == 1 &&
              none.ok && srcCount(code, none, "return;") == 1;
    // ?gpssetup: LOST stops; after a move that landed, a re-check that answers nothing stops; neither sends.
    const Range slost = srcBlockAfter(code, "if (mv == GPS_MOVE_LOST)", setup.b, setup.e);
    const Range sok   = srcBlockAfter(code, "if (mv == GPS_MOVE_OK)", setup.b, setup.e);
    const size_t recheck = srcPos(code, sok, "dialect = gpsDetectDialect();");
    const Range smute = sok.ok ? srcBlockAfter(code, "if (dialect == UBX_DIALECT_MUTE)", sok.b, sok.e) : Range{ false, 0, 0 };
    ok = ok && slost.ok && srcFirstTx(code, slost, senders) == NPOS && srcCount(code, slost, "return;") == 1 &&
         sok.ok && recheck != NPOS && smute.ok && recheck < smute.b && srcFirstTx(code, smute, senders) == NPOS &&
         srcCount(code, smute, "return;") == 1;
    // ?gpsbaud set: nothing after the move (and its one retry) sends.
    const std::vector<size_t> moves = srcAllPos(code, bcmd, "gpsSetModuleBaud(");
    if (ok && !moves.empty()) {
      const size_t lastCall = moves.back();
      const size_t semi = code.find(';', lastCall);
      ok = semi != NPOS && semi < bcmd.e && srcFirstTx(code, Range{ true, semi + 1, bcmd.e }, senders) == NPOS;
    } else {
      ok = false;
    }
    g.t3 = ok;
  }

  // ---- T4: the NMEA repair, keyed to the symptom, behind a listen-only detector that hears UBX ----
  {
    // gpsProbeAt(): sends nothing; the NMEA test and its early break come first; UBX only on a verified CK_B.
    const size_t nmeaHit = srcPos(code, probe, "seen |= GPS_SAW_NMEA; break;");
    const size_t ubxSw   = srcPos(code, probe, "switch (ubx_state)");
    bool ok = probe.ok && srcFirstTx(code, probe, senders) == NPOS && !srcHas(senders, "gpsProbeAt") &&
              nmeaHit != NPOS && ubxSw != NPOS && nmeaHit < ubxSw &&
              srcCount(code, probe, "case 7: ubx_state = (c == ubx_ckA) ? 8 : 0; break;") == 1 &&
              srcCount(code, probe, "if (c == ubx_ckB) seen |= GPS_SAW_UBX;") == 1 &&
              srcCount(code, probe, "GPS_SAW_UBX;") == 1 &&
              srcSquash(code).find("static uint32_t gpsDetectBaud(uint16_t window_ms, uint32_t fallback_baud, "
                                   "uint8_t *seen_out = nullptr)") != NPOS;
    // gpsEnableNmeaOut(): CFG-PRT at the proven baud (old form only), then GGA and RMC, then UBX-out, then NMEA-out.
    const Range leg = nmea.ok ? srcBlockAfter(code, "if (dialect == UBX_DIALECT_LEGACY)", nmea.b, nmea.e) : Range{ false, 0, 0 };
    const std::string ggaCall = "gpsApplyCfg(dialect, enableGGA, sizeof(enableGGA), KEY_MSGOUT_NMEA_GGA_U1, &on, 1)";
    const std::string rmcCall = "gpsApplyCfg(dialect, enableRMC, sizeof(enableRMC), KEY_MSGOUT_NMEA_RMC_U1, &on, 1)";
    const size_t atGga  = srcPos(code, nmea, ggaCall);
    const size_t atRmc  = srcPos(code, nmea, rmcCall);
    const size_t atUbx  = srcPos(code, nmea, "ubxValset(KEY_UART1OUTPROT_UBX, &on, 1, 2)");
    const size_t atNmea = srcPos(code, nmea, "ubxValset(KEY_UART1OUTPROT_NMEA, &on, 1, 2)");
    auto msgFrame = [&](const char* name, int id, uint16_t ck) {
      const size_t d = srcPos(code, nmea, std::string("byte ") + name + "[] = {");
      const size_t ob = (d == NPOS) ? NPOS : code.find('{', d);
      const size_t cb = (ob == NPOS) ? NPOS : srcMatch(code, ob);
      std::vector<int> f;
      if (cb == NPOS || cb > nmea.e || !hexList(code.substr(ob + 1, cb - ob - 1), &f)) return false;
      const std::vector<int> want = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, id, 0x01, 0x00, 0x00 };
      if (f != want) return false;
      f.resize(9);
      return fletcher(f) == ck &&
             srcCount(code, nmea, std::string("ubxAppendChecksum(") + name + ", sizeof(" + name + "));") == 1;
    };
    ok = ok && nmea.ok && srcSquash(code).find("static void gpsEnableNmeaOut(uint8_t dialect)") != NPOS &&
         leg.ok && srcCount(code, leg, "gpsBuildCfgPrt(f, gps_current_baud);") == 1 &&
         srcCount(code, leg, "ubxSendAcked(f, sizeof(f), 2)") == 1 &&
         atGga != NPOS && atRmc != NPOS && atUbx != NPOS && atNmea != NPOS &&
         leg.e < atGga && atGga < atRmc && atRmc < atUbx && atUbx < atNmea &&
         srcCount(code, nmea, "gpsApplyCfg(") == 2 && srcCount(code, nmea, "ubxValset(") == 2 &&
         srcCount(code, nmea, "KEY_MSGOUT_NMEA_GGA_U1") == 1 && srcCount(code, nmea, "KEY_MSGOUT_NMEA_RMC_U1") == 1 &&
         msgFrame("enableGGA", 0x00, 0xFB10) && msgFrame("enableRMC", 0x04, 0xFF18) &&
         srcCount(code, nmea, "Serial1.write(") == 0;
    // initTxGPS(): the symptom from the last listen, the call after the rate write, ONLY on the symptom.
    const std::string cond = "if (dialect != UBX_DIALECT_MUTE && ubx_only) gpsEnableNmeaOut(dialect);";
    const size_t callAt = srcPos(code, init, cond);
    ok = ok && srcCount(code, init, "const bool ubx_only = !(heard_proto & GPS_SAW_NMEA);") == 1 &&
         srcCount(code, init, cond) == 1 && srcCount(code, init, "gpsEnableNmeaOut(") == 1 &&
         srcCount(code, init, "else if (ubx_only) Serial.println(") == 1 &&
         rateAt != NPOS && callAt != NPOS && rateAt < callAt;
    // ?gpssetup step 4: the same rule on step 1's listen, before the step 5 save.
    const size_t sCall = srcPos(code, setup, cond);
    const size_t sSave = srcPos(code, setup, "gpsSaveConfig(");
    ok = ok && srcCount(code, setup, "const bool ubx_only = !(cur_proto & GPS_SAW_NMEA);") == 1 &&
         srcCount(code, setup, cond) == 1 && srcCount(code, setup, "gpsEnableNmeaOut(") == 1 &&
         srcCount(code, setup, "uint32_t cur = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &cur_proto);") == 1 &&
         sCall != NPOS && sSave != NPOS && sCall < sSave;
    // Nowhere else: the definition and those two calls.
    ok = ok && srcFindIdent(code, "gpsEnableNmeaOut").size() == 3;
    g.t4 = ok;
  }

  // ---- T5: no Flash layer, and no persist argument can come back ----
  {
    bool ok = vset.ok && smb.ok &&
              srcCount(code, vset, "f[n++] = 0x01 | 0x02;") == 1 && srcTight(code.substr(vset.b, vset.e - vset.b)).find("0x04") == NPOS &&
              srcCount(code, smb, "f[n++] = 0x01 | 0x02;") == 1 && srcTight(code.substr(smb.b, smb.e - smb.b)).find("0x04") == NPOS &&
              srcTight(code).find("0x01|0x02|0x04") == NPOS &&
              srcSquash(code).find("static uint8_t ubxValset(uint32_t key, const byte *val, uint8_t valLen, uint8_t tries)") != NPOS;
    for (int n : srcCallArgs(code, all, "ubxValset"))        ok = ok && n == 4;
    for (int n : srcCallArgs(code, all, "gpsApplyCfg"))      ok = ok && n == 6;
    for (int n : srcCallArgs(code, all, "gpsSetModuleBaud")) ok = ok && n == 3;
    ok = ok && !srcCallArgs(code, all, "gpsApplyCfg").empty() && !srcCallArgs(code, all, "gpsSetModuleBaud").empty();
    // The CFG-CFG save is untouched and still asked for by ?gpssetup.
    ok = ok && save.ok && srcTight(code.substr(save.b, save.e - save.b)).find("0x17,") != NPOS &&
         srcCount(code, setup, "gpsSaveConfig(") == 1;
    g.t5 = ok;
  }

  // ---- T6: the link-budget line prints the share used, on the baud after the move ----
  {
    const size_t usedAt = srcPos(code, init, "const uint32_t used = need * 100UL / gps_current_baud;");
    bool ok = srcCount(code, init, "const uint32_t need = (1000UL / meas_ms) * 142UL * 10UL;") == 1 &&
              srcCount(code, init, "const uint32_t used = need * 100UL / gps_current_baud;") == 1 &&
              srcCount(code, init, "if (used > 70UL)") == 1 && srcCount(code, init, "(unsigned long)used);") == 1 &&
              srcTight(code.substr(init.b, init.e - init.b)).find("100ul-") == NPOS &&
              usedAt != NPOS && moveBlk.ok && moveBlk.e < usedAt && rateAt != NPOS && rateAt < usedAt;
    // The arithmetic the line now does, at the worst case the old one wrapped on: 10 Hz on 9600 is ~147 % used.
    const uint32_t need = (1000UL / 100) * 142UL * 10UL;
    ok = ok && need * 100UL / 9600UL == 147UL && need * 100UL / 115200UL == 12UL;
    g.t6 = ok;
  }

  // ---- T7: ?gpscfg reads the rate back and asks MON-VER, with a quit check before each new poll ----
  {
    const std::string tight = srcTight(code.substr(rdbk.b, rdbk.e - rdbk.b));
    const size_t posDynV  = srcPos(code, rdbk, "ubxPoll(pollDyn");
    const size_t posRate  = srcPos(code, rdbk, "ubxPoll(pollRate,");
    const size_t posRateV = srcPos(code, rdbk, "ubxPoll(pollRateV");
    const size_t posVer   = srcPos(code, rdbk, "ubxPoll(pollVer");
    bool ok = tight.find("{0xb5,0x62,0x06,0x08,0x00,0x00,") != NPOS &&
              srcCount(code, rdbk, "ubxPoll(pollRate, sizeof(pollRate), 0x06, 0x08, pl, sizeof(pl))") == 1 &&
              srcCount(code, rdbk, "gpsBuildValget(pollRateV, KEY_RATE_MEAS);") == 1 &&
              srcCount(code, rdbk, "gpsValgetOk(pl, n, KEY_RATE_MEAS)") == 1 &&
              tight.find("{0xb5,0x62,0x0a,0x04,0x00,0x00,") != NPOS &&
              srcCount(code, rdbk, "ubxPoll(pollVer, sizeof(pollVer), 0x0A, 0x04, ver, sizeof(ver))") == 1 &&
              srcCount(code, rdbk, "const uint16_t rate_want = gpsSelectedMeasMs();") == 1 &&
              text.substr(rdbk.b, rdbk.e - rdbk.b).find("setting matches the") != NPOS &&
              posDynV != NPOS && posRate != NPOS && posRateV != NPOS && posVer != NPOS;
    if (ok) {
      bool beforeRate = false, beforeVer = false;
      const size_t rateLo = std::min(posRate, posRateV), rateHi = std::max(posRate, posRateV);
      for (size_t q : srcAllPos(code, rdbk, "checkSerialQuit()")) {
        if (q > posDynV && q < rateLo) beforeRate = true;
        if (q > rateHi && q < posVer) beforeVer = true;
      }
      ok = beforeRate && beforeVer;
    }
    g.t7 = ok;
  }

  // ---- T8: listen before any write; the rate selector ----
  {
    bool ok = meas.ok && srcSquash(code.substr(meas.b, meas.e - meas.b)) == "return (usrConf.gps_chip_type == 2) ? 100 : 200;" &&
              srcCount(code, init, "meas_ms = gpsSelectedMeasMs();") == 2 &&
              srcCount(code, setup, "const uint16_t meas_ms = gpsSelectedMeasMs();") == 1;
    const Range sw = srcBlockAfter(code, "switch (usrConf.gps_chip_type)", init.b, init.e);
    const Range c2 = sw.ok ? srcBlockAfter(code, "case 2:", sw.b, sw.e) : Range{ false, 0, 0 };
    const Range c0 = sw.ok ? srcBlockAfter(code, "case 0:", sw.b, sw.e) : Range{ false, 0, 0 };
    const Range heardBlk = c0.ok ? srcBlockAfter(code, "if (seen_baud)", c0.b, c0.e) : Range{ false, 0, 0 };
    ok = ok && sw.ok && c2.ok && c0.ok && heardBlk.ok && srcFirstTx(code, c2, senders) == NPOS &&
         srcFirstTx(code, heardBlk, senders) == NPOS && srcCount(code, heardBlk, "break;") == 1;
    // the type 0 branch: the listen first, and only CFG-PRT (setBaud) in the dance after it
    const size_t listen0 = srcPos(code, c0, "gpsDetectBaud(");
    const size_t tx0 = srcFirstTx(code, c0, senders);
    ok = ok && listen0 != NPOS && tx0 != NPOS && listen0 < tx0 && heardBlk.e < tx0 &&
         srcCount(code, c0, "gpsSetModuleBaud(") == 0 && srcCount(code, c0, "ubxSendAcked(") == 0 &&
         srcCount(code, c0, "gpsApplyCfg(") == 0;
    int writes = 0;
    for (size_t p = code.find("Serial1.write(", c0.b); c0.ok && p != NPOS && p < c0.e; p = code.find("Serial1.write(", p + 1)) {
      writes++;
      const size_t close = srcMatch(code, p + std::strlen("Serial1.write"));
      ok = ok && close != NPOS && srcSquash(code.substr(p, close - p + 1)) == "Serial1.write(setBaud, sizeof(setBaud))";
    }
    ok = ok && writes == 2;
    // after the switch: the listen block, then the gate that returns when nothing was heard, then the first write
    const Range after = { sw.ok, sw.e + 1, init.e };
    Range gate = { false, 0, 0 };
    size_t listenAfter = NPOS;
    for (size_t p = code.find("if (!baud_confirmed)", after.b); sw.ok && p != NPOS && p < init.e;
         p = code.find("if (!baud_confirmed)", p + 1)) {
      const Range blk = srcBlockAfter(code, "if (!baud_confirmed)", p, init.e);
      if (!blk.ok) continue;
      if (listenAfter == NPOS && srcPos(code, blk, "gpsDetectBaud(") != NPOS) listenAfter = blk.b;
      if (srcCount(code, blk, "return;") == 1) { gate = blk; break; }
    }
    const size_t firstAfter = srcFirstTx(code, after, senders);
    ok = ok && listenAfter != NPOS && gate.ok && listenAfter < gate.b && firstAfter != NPOS && gate.e < firstAfter &&
         srcFirstTx(code, gate, senders) == NPOS;
    g.t8 = ok;
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

// RED RUNS: one targeted edit of the real file per rule; each red must fail that rule (and so the guard as a whole),
// each green must pass every rule.
static int reds = 0, greens = 0;
#define RED(rule, m, f) do { const Guard g_ = guard(m); reds++; CHECK((f) && !g_.rule && !guardOk(g_)); } while (0)
#define GREEN(m, f) do { const Guard g_ = guard(m); greens++; CHECK((f) && guardOk(g_)); } while (0)

static void testRedRuns(const std::string& real)
{
  bool f = false, f2 = false;
  const std::string c2listen = "      Serial.println(\"TX GPS [M10]: listening first; nothing is sent until the module is heard\");";
  const std::string offLine  = "    static const byte off = 0;   // U1 output rate 0 = disabled, for the VALSET path";
  const std::string lostLine = "        Serial.println(\"LOST - heard at neither baud. Rescanning; nothing is sent until it is heard.\");";
  const std::string bootCond = "    if (dialect != UBX_DIALECT_MUTE && ubx_only)\n      gpsEnableNmeaOut(dialect);        // RAM|BBR";

  // ---- T1 ----
  { // the four-constellation CFG-GNSS put back into the type 2 branch (also a write before the listen: T8)
    const std::string m = mutate(real, c2listen, "      { byte setGNSS[] = { 0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00 }; "
                                 "ubxSendAcked(setGNSS, sizeof(setGNSS), 2); }\n" + c2listen, &f);
    RED(t1, m, f);
    CHECK(!guard(m).t8);
  }
  { // a CFG-SIGNAL key declared and written in initTxGPS()
    std::string m = mutate(real, "static const uint32_t KEY_UART1_BAUDRATE",
                           "static const uint32_t KEY_SIGNAL_GLO_ENA = 0x10310025UL;\nstatic const uint32_t KEY_UART1_BAUDRATE", &f);
    m = mutate(m, offLine, offLine + "\n    ubxValset(KEY_SIGNAL_GLO_ENA, &off, 1, 2);", &f2);
    RED(t1, m, f && f2);
  }
  { // the same key under an innocent name
    std::string m = mutate(real, "static const uint32_t KEY_UART1_BAUDRATE",
                           "static const uint32_t KEY_GLO = 0x10310025UL;\nstatic const uint32_t KEY_UART1_BAUDRATE", &f);
    m = mutate(m, offLine, offLine + "\n    ubxValset(KEY_GLO, &off, 1, 2);", &f2);
    RED(t1, m, f && f2);
  }
  { // the key hidden two calls down (initTxGPS -> gpsSetModuleBaud / gpsEnableNmeaOut -> gpsBuildCfgPrt)
    const std::string m = mutate(real, "  memcpy(f, tmpl, sizeof(tmpl));",
                                 "  { static const byte z = 0; ubxValset(0x10310025UL, &z, 1, 2); }\n  memcpy(f, tmpl, sizeof(tmpl));", &f);
    RED(t1, m, f);
  }
  { // GREEN: a read-only CFG-SIGNAL VALGET in ?gpscfg is allowed
    std::string m = mutate(real, "static const uint32_t KEY_UART1_BAUDRATE",
                           "static const uint32_t KEY_SIGNAL_BDS_ENA = 0x10310022UL;\nstatic const uint32_t KEY_UART1_BAUDRATE", &f);
    m = mutate(m, "  const uint16_t rate_want = gpsSelectedMeasMs();",
               "  { byte pollSig[16]; gpsBuildValget(pollSig, KEY_SIGNAL_BDS_ENA); "
               "n = ubxPoll(pollSig, sizeof(pollSig), 0x06, 0x8B, pl, sizeof(pl)); }\n"
               "  const uint16_t rate_want = gpsSelectedMeasMs();", &f2);
    GREEN(m, f && f2);
  }

  // ---- T2 ----
  { // the listen back shortened to 300 ms and its answer ignored (the code before this pass)
    const std::string m = mutate(real, "  if (gpsProbeAt(from_baud, 1100)) return GPS_MOVE_STAYED;",
                                 "  gpsProbeAt(from_baud, 300); return GPS_MOVE_STAYED;", &f);
    RED(t2, m, f);
  }
  { // the move made unconditional - it would run on a MUTE module
    const std::string m = mutate(real, "if (heard < GPS_BAUD_PREFERRED && dialect != UBX_DIALECT_MUTE) {",
                                 "if (heard < GPS_BAUD_PREFERRED) {", &f);
    RED(t2, m, f);
  }
  { // the CFG-VALSET retry after a LOST move too (first occurrence = initTxGPS)
    const std::string m = mutate(real, "if (mv == GPS_MOVE_STAYED && dialect == UBX_DIALECT_LEGACY) {",
                                 "if (mv != GPS_MOVE_OK && dialect == UBX_DIALECT_LEGACY) {", &f);
    RED(t2, m, f);
  }
  { // ?gpssetup retrying after LOST too
    const std::string from = "if (mv == GPS_MOVE_STAYED && dialect == UBX_DIALECT_LEGACY) {\n      Serial.printf(\"not taken, "
                             "heard again at %lu; trying once in CFG-VALSET ... \", (unsigned long)cur);";
    const std::string to   = "if (mv != GPS_MOVE_OK && dialect == UBX_DIALECT_LEGACY) {\n      Serial.printf(\"not taken, "
                             "heard again at %lu; trying once in CFG-VALSET ... \", (unsigned long)cur);";
    const std::string m = mutate(real, from, to, &f);
    RED(t2, m, f);
  }
  { // the move put back in the type 0 branch, before the probe (also a write in the heard block: T8)
    const std::string m = mutate(real, "        heard = seen_baud;",
                                 "        gpsSetModuleBaud(UBX_DIALECT_LEGACY, seen_baud, GPS_BAUD_PREFERRED);\n        heard = seen_baud;", &f);
    RED(t2, m, f);
    CHECK(!guard(m).t8);
  }
  { // the move put back in the type 2 branch, in both forms, before the dialect is known (the code before this pass)
    const std::string m = mutate(real, "        heard = found;",
                                 "        heard = found;\n        gpsSetModuleBaud(UBX_DIALECT_VALSET, found, GPS_BAUD_PREFERRED);", &f);
    RED(t2, m, f);
  }
  { // ?gpsbaud set retrying on any failure
    const std::string m = mutate(real, "if (mv == GPS_MOVE_STAYED && dialect != UBX_DIALECT_VALSET)",
                                 "if (mv != GPS_MOVE_OK && dialect != UBX_DIALECT_VALSET)", &f);
    RED(t2, m, f);
  }

  // ---- T3 ----
  { // LOST carries on at the old baud without a rescan
    const std::string m = mutate(real, "        heard = gpsDetectBaud(1100, GPS_BAUD_PREFERRED, &heard_proto);",
                                 "        heard = GPS_BAUD_PREFERRED;", &f);
    RED(t3, m, f);
  }
  { // a write slipped into the LOST block before the rescan
    const std::string m = mutate(real, lostLine, "        Serial1.write(setNav5Sea, 1);\n" + lostLine, &f);
    RED(t3, m, f);
  }
  { // the "nothing answers" exit after the rescan made unreachable
    const std::string m = mutate(real, "        if (!heard) {", "        if (false) {", &f);
    RED(t3, m, f);
  }
  { // ?gpssetup carries on after LOST
    const std::string m = mutate(real, "answers), or power-cycle the remote and its GPS, then run ?gpssetup again.\");\n      return;",
                                 "answers), or power-cycle the remote and its GPS, then run ?gpssetup again.\");", &f);
    RED(t3, m, f);
  }
  { // ?gpssetup's "re-check answers nothing" stop sending something on its way out
    const std::string m = mutate(real, "        Serial.println(\"  NMEA output was not switched back on.\");",
                                 "        gpsSaveConfig();\n        Serial.println(\"  NMEA output was not switched back on.\");", &f);
    RED(t3, m, f);
  }
  { // a dialect poll slipped into the LOST block - gpsDetectDialect() sends (it polls)
    const std::string m = mutate(real, lostLine, "        gpsDetectDialect();\n" + lostLine, &f);
    RED(t3, m, f);
  }
  { // a sending helper that is on no list, slipped into the LOST block - caught only through the call chain
    std::string m = mutate(real, "void initTxGPS()\n{",
                           "static void gpsNudge() { static const byte z[2] = { 0xB5, 0x62 }; Serial1.write(z, sizeof(z)); }\n"
                           "void initTxGPS()\n{", &f);
    m = mutate(m, lostLine, "        gpsNudge();\n" + lostLine, &f2);
    RED(t3, m, f && f2);
  }
  { // ?gpsbaud set writing again after LOST
    const std::string m = mutate(real, "    Serial.println(\"  power-cycle the remote and its GPS, then ?gpsreinit.\");",
                                 "    gpsSaveConfig();\n    Serial.println(\"  power-cycle the remote and its GPS, then ?gpsreinit.\");", &f);
    RED(t3, m, f);
  }
  { // GREEN: listening is not sending - a gpsProbeAt() in the LOST block passes
    const std::string m = mutate(real, lostLine, "        gpsProbeAt(GPS_BAUD_PREFERRED, 10);\n" + lostLine, &f);
    GREEN(m, f);
  }

  // ---- T4 ----
  { // the repair keyed to the format again (CFG-VALSET only)
    const std::string m = mutate(real, bootCond, "    if (dialect == UBX_DIALECT_VALSET)\n      gpsEnableNmeaOut(dialect);        // RAM|BBR", &f);
    RED(t4, m, f);
  }
  { // the MUTE exclusion dropped - an unverifiable module would be written to
    const std::string m = mutate(real, bootCond, "    if (ubx_only)\n      gpsEnableNmeaOut(dialect);        // RAM|BBR", &f);
    RED(t4, m, f);
  }
  { // a healthy CFG-VALSET module touched on every boot (the receiver's condition): not on the remote
    const std::string m = mutate(real, bootCond, "    if (dialect != UBX_DIALECT_MUTE && (ubx_only || dialect == UBX_DIALECT_VALSET))\n"
                                                 "      gpsEnableNmeaOut(dialect);        // RAM|BBR", &f);
    RED(t4, m, f);
  }
  { // ?gpssetup's repair dropped
    const std::string m = mutate(real, "  if (dialect != UBX_DIALECT_MUTE && ubx_only)\n    gpsEnableNmeaOut(dialect);\n",
                                 "  (void)ubx_only;\n", &f);
    RED(t4, m, f);
  }
  { // CFG-PRT built at a fixed baud - on a module heard at 9600 that is a baud move, not a repair
    const std::string m = mutate(real, "    gpsBuildCfgPrt(f, gps_current_baud);", "    gpsBuildCfgPrt(f, GPS_BAUD_PREFERRED);", &f);
    RED(t4, m, f);
  }
  { // the old-form CFG-PRT step dropped
    const std::string m = mutate(real, "    const uint8_t r = ubxSendAcked(f, sizeof(f), 2);", "    const uint8_t r = UBX_NOREPLY;", &f);
    RED(t4, m, f);
  }
  { // GGA and RMC CFG-VALSET only - an M8 would get the protocol back and both sentences left off
    std::string m = mutate(real, "gpsApplyCfg(dialect, enableGGA, sizeof(enableGGA), KEY_MSGOUT_NMEA_GGA_U1, &on, 1);",
                           "(ubxValset(KEY_MSGOUT_NMEA_GGA_U1, &on, 1, 2) == UBX_ACK ? \"OK/valset\" : \"REJECTED\");", &f);
    m = mutate(m, "gpsApplyCfg(dialect, enableRMC, sizeof(enableRMC), KEY_MSGOUT_NMEA_RMC_U1, &on, 1);",
               "(ubxValset(KEY_MSGOUT_NMEA_RMC_U1, &on, 1, 2) == UBX_ACK ? \"OK/valset\" : \"REJECTED\");", &f2);
    RED(t4, m, f && f2);
  }
  { // the GGA frame asking for the wrong sentence (F0 01 is GLL)
    const std::string m = mutate(real, "0xF0, 0x00, 0x01, 0x00, 0x00 };", "0xF0, 0x01, 0x01, 0x00, 0x00 };", &f);
    RED(t4, m, f);
  }
  { // the listen transmitting (the GPS-BAUD-4 hazard)
    const std::string m = mutate(real, "  uint8_t  ubx_state = 0;", "  Serial1.write((const byte *)0, 1);\n  uint8_t  ubx_state = 0;", &f);
    RED(t4, m, f);
  }
  { // a UBX frame believed without its checksum
    const std::string m = mutate(real, "      case 7: ubx_state = (c == ubx_ckA) ? 8 : 0; break;",
                                 "      case 7: seen |= GPS_SAW_UBX; ubx_state = 0; break;", &f);
    RED(t4, m, f);
  }

  // ---- T5 ----
  { // the Flash layer back in ubxValset()
    const std::string m = mutate(real, "  f[n++] = 0x01 | 0x02;                              // layers",
                                 "  f[n++] = 0x01 | 0x02 | 0x04;                       // layers", &f);
    RED(t5, m, f);
  }
  { // the Flash layer back in gpsSetModuleBaud()
    const std::string m = mutate(real, "    f[n++] = 0x01 | 0x02;                                          // RAM|BBR (never Flash)",
                                 "    f[n++] = persist ? (0x01 | 0x02 | 0x04) : (0x01 | 0x02);", &f);
    RED(t5, m, f);
  }
  { // a persist argument back on a ?gpssetup write
    const std::string m = mutate(real, "                                   KEY_NAVSPG_DYNMODEL, &dyn_valset, 1);",
                                 "                                   KEY_NAVSPG_DYNMODEL, &dyn_valset, 1, true);", &f);
    RED(t5, m, f);
  }

  // ---- T6 ----
  { // the underflow put back ("100 - used" printed)
    const std::string m = mutate(real, "                      (unsigned long)used);", "                      (unsigned long)(100UL - used));", &f);
    RED(t6, m, f);
  }
  { // computed on the baud before the move
    const std::string m = mutate(real, "const uint32_t used = need * 100UL / gps_current_baud;", "const uint32_t used = need * 100UL / heard;", &f);
    RED(t6, m, f);
  }

  // ---- T7 ----
  { // the MON-VER poll removed
    const std::string m = mutate(real, "ubxPoll(pollVer, sizeof(pollVer), 0x0A, 0x04, ver, sizeof(ver))", "0", &f);
    RED(t7, m, f);
  }
  { // the VALGET rate readback without the key-echo check
    const std::string m = mutate(real, "gpsValgetOk(pl, n, KEY_RATE_MEAS)", "true", &f);
    RED(t7, m, f);
  }
  { // no quit check before the MON-VER poll
    const std::string m = mutate(real, "  if (!aborted && checkSerialQuit()) {\n    Serial.println(\"  aborted before the MON-VER poll.\");",
                                 "  if (false) {\n    Serial.println(\"  aborted before the MON-VER poll.\");", &f);
    RED(t7, m, f);
  }

  // ---- T8 ----
  { // type 2 transmitting before the listen
    const std::string m = mutate(real, c2listen, "      { static const byte z = 0; Serial1.write(&z, 1); }\n" + c2listen, &f);
    RED(t8, m, f);
  }
  { // a write put between the switch and the listen
    const std::string m = mutate(real, "    if (!baud_confirmed) {\n      Serial.println(\"TX GPS: baud not confirmed yet",
                                 "    Serial1.write(setNav5Sea, 1);\n    if (!baud_confirmed) {\n      Serial.println(\"TX GPS: baud not confirmed yet", &f);
    RED(t8, m, f);
  }
  { // 10 Hz asked of every type
    const std::string m = mutate(real, "return (usrConf.gps_chip_type == 2) ? 100 : 200;", "return (usrConf.gps_chip_type == 2) ? 100 : 100;", &f);
    RED(t8, m, f);
  }
  { // a file with no initTxGPS() at all fails every rule
    const Guard g = guard("int x;\n");
    CHECK(!g.t1 && !g.t2 && !g.t3 && !g.t4 && !g.t5 && !g.t6 && !g.t7 && !g.t8);
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
    gps = fs::path("..") / ".." / "Source" / "V2_Integration_Tx" / "GPS.ino";
    if (!fs::is_regular_file(gps, ec))
      gps = fs::path(__FILE__).parent_path() / ".." / ".." / "Source" / "V2_Integration_Tx" / "GPS.ino";
  }
  std::printf("tx_gps_guard_test: reads %s\n", gps.string().c_str());
  std::string text;
  const bool read_ok = readFile(gps, &text);
  CHECK(read_ok && !text.empty());
  text.erase(std::remove(text.begin(), text.end(), '\r'), text.end());   // same text whatever the line endings

  const Guard g = guard(text);
  std::printf("  T1 no constellation write in initTxGPS() ......... %s\n", g.t1 ? "ok" : "BROKEN");
  std::printf("  T2 baud move: OK / STAYED / LOST, after the probe . %s\n", g.t2 ? "ok" : "BROKEN");
  std::printf("  T3 LOST sends nothing ............................. %s\n", g.t3 ? "ok" : "BROKEN");
  std::printf("  T4 NMEA repair keyed to the symptom ............... %s\n", g.t4 ? "ok" : "BROKEN");
  std::printf("  T5 no Flash layer, no persist argument ............ %s\n", g.t5 ? "ok" : "BROKEN");
  std::printf("  T6 link budget: share used, after the move ........ %s\n", g.t6 ? "ok" : "BROKEN");
  std::printf("  T7 ?gpscfg reads rate back + MON-VER, quit checks .. %s\n", g.t7 ? "ok" : "BROKEN");
  std::printf("  T8 listen before any write; rate selector ......... %s\n", g.t8 ? "ok" : "BROKEN");
  CHECK(g.t1); CHECK(g.t2); CHECK(g.t3); CHECK(g.t4); CHECK(g.t5); CHECK(g.t6); CHECK(g.t7); CHECK(g.t8);

  if (!given && read_ok) {
    testRedRuns(text);   // the red edits are anchored on the fixed file
    std::printf("  red edits: %d, green edits: %d\n", reds, greens);
  }

  if (fails) { std::printf("tx_gps_guard_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("tx_gps_guard_test: all %d checks passed\n", checks);
  return 0;
}
