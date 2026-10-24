// V2.5-Evo - 2026-10-24 - throttle-byte audit T-1, fix (b): host test for Source/Common/ThrottleByte.h. The remote's
//   scaling and the receiver's stretch over all 256 values (0 -> 0, never decreasing, rounded, never a meta type, never
//   below the input), the round trip (0 -> 0, 255 -> 255, within 1 count: 241 exact, 15 one count up), the one
//   constant, and the PWM the pair gives against the old clip (full squeeze 1941 -> 2000 us).
// V2.5-Evo - 2026-10-24 - throttle-byte audit T-1, fix (b), receiver half: a SOURCE-TEXT GUARD over the receiver's code
//   (the host cannot compile PWM.ino): the first statement of calcPWM() is the stretch line, thrByteToCmd appears once in
//   the receiver's code (PWM.ino) and once in Source/Common (its definition in ThrottleByte.h), and no map() call takes
//   it. Made-up snippets that break each rule prove the guard bites. Reads the files, so run it from Tools/tests.
// Build and run (Git Bash, from Tools/tests, PATH and TMP as in the host-test note of the compile procedure):
//   g++ -std=c++17 -Wall -Wextra -o throttle_byte_test throttle_byte_test.cpp && ./throttle_byte_test

#include <cctype>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "../../Source/Common/ThrottleByte.h"

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

// Arduino's map(), copied, as the receiver's efoil branch uses it with PWM 1000-2000.
static long arduinoMap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static int pwmUs(uint8_t cmd) { return (int)arduinoMap(cmd, 0, 255, 1000, 2000); }

// ---- The remote: thrTriggerToByte(), all 256 throttles ----
static void testRemote()
{
  CHECK(kThrCtrlMax == 0xF0);
  bool ok = true;
  for (int t = 0; t < 256; t++) {
    const uint8_t b = thrTriggerToByte((uint8_t)t);
    ok = ok && b <= kThrCtrlMax;                                            // never a meta type (0xF1-0xF4) or above
    ok = ok && (t == 0 ? b == 0 : b >= 1);                                  // 0 stays 0; a squeeze is never sent as 0
    const int err17 = 17 * (int)b - 16 * t;                                 // b against 240t/255 = 16t/17, in 17ths
    ok = ok && err17 >= -8 && err17 <= 8;                                   // rounded to nearest
    if (t > 0) ok = ok && b >= thrTriggerToByte((uint8_t)(t - 1));          // never decreasing
  }
  CHECK(ok);
  CHECK(thrTriggerToByte(0) == 0 && thrTriggerToByte(1) == 1 && thrTriggerToByte(254) == 239 &&
        thrTriggerToByte(255) == kThrCtrlMax);
}

// ---- The receiver: thrByteToCmd(), all 256 bytes ----
static void testReceiver()
{
  bool ok = true;
  for (int b = 0; b < 256; b++) {
    const uint8_t c = thrByteToCmd((uint8_t)b);
    ok = ok && c >= b;                                                      // never below its input
    if (b >= kThrCtrlMax) ok = ok && c == 255;                              // 240 or more = full output (0xF5-0xFF too)
    else {
      const int err = 240 * (int)c - 255 * b;                               // c against 255b/240, in 240ths
      ok = ok && c < 255 && err > -120 && err <= 120;                       // full only from 240; rounded (half up)
    }
    if (b > 0) {
      const uint8_t p = thrByteToCmd((uint8_t)(b - 1));
      ok = ok && c >= p;                                                    // never decreasing ...
      if (b <= kThrCtrlMax) ok = ok && c > p;                               // ... and every byte 0..240 has its own command
    }
  }
  CHECK(ok);
  CHECK(thrByteToCmd(0) == 0 && thrByteToCmd(1) == 1 && thrByteToCmd(8) == 9 && thrByteToCmd(239) == 254 &&
        thrByteToCmd(240) == 255 && thrByteToCmd(255) == 255);
}

// ---- The pair: the round trip, all 256 throttles: 0 -> 0, 255 -> 255, never decreasing, within 1 count ----
static void testRoundTrip()
{
  bool ok = true;
  int exact = 0, plus1 = 0;
  for (int t = 0; t < 256; t++) {
    const uint8_t r = thrByteToCmd(thrTriggerToByte((uint8_t)t));
    ok = ok && (r == t || r == t + 1);                                      // within 1 count, and never below the throttle
    if (r == t) exact++; else plus1++;
    if (t > 0) ok = ok && r >= thrByteToCmd(thrTriggerToByte((uint8_t)(t - 1)));
  }
  CHECK(ok && exact == 241 && plus1 == 15);
  CHECK(thrByteToCmd(thrTriggerToByte(0)) == 0 && thrByteToCmd(thrTriggerToByte(255)) == 255);
}

// ---- What the rider gets, PWM 1000-2000 us: the old clip against the pair (the audit's section 4 table) ----
static void testPwmTable()
{
  struct Row { uint8_t thr; int before; int fixed; };
  const Row rows[] = {
    { 255, 1941, 2000 },   // full squeeze: the finding (bench: 1941, never higher)
    { 240, 1941, 1941 },   // trigger 240: the old full squeeze, now 94 % of the travel
    { 128, 1501, 1501 },   // half trigger
    { 229, 1898, 1901 },   // gear 9 of 10
    { 216, 1847, 1847 },   // dynamic cap 85 % at a full squeeze (255 x 85 / 100)
    { 178, 1698, 1701 },   // the remote's return-to-me cap at 70 % (70 x 255 / 100 = 178)
    {   0, 1000, 1000 },   // released
  };
  for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    const uint8_t clip = (rows[i].thr > kThrCtrlMax) ? kThrCtrlMax : rows[i].thr;   // the old remote line
    CHECK(pwmUs(clip) == rows[i].before);                                           // the old receiver: 255 = full
    CHECK(pwmUs(thrByteToCmd(thrTriggerToByte(rows[i].thr))) == rows[i].fixed);
  }
}

// ============================================================
// SOURCE-TEXT GUARD: the stretch runs once, first, before every cap
// ============================================================
// WHY: the host tests cannot compile RX PWM.ino (it needs Arduino). A later edit to the firmware could move the stretch
// below a cap, call it a second time, or fold it into a map() line, and every test above would still pass. So this reads
// the receiver's source as TEXT, with every comment and every string's contents blanked, and checks the three things the
// safety of fix (b) rests on:
//   (1) the FIRST statement of calcPWM() is exactly the stretch line
//         uint8_t effective_thr = rtm_rx_emergency_stop ? 0 : thrByteToCmd(thr_received);
//       so the stretch runs before every cap (each cap is a later statement, a min() on effective_thr);
//   (2) thrByteToCmd appears ONCE in the receiver's code - in PWM.ino, nowhere else under Source/V2_Integration_Rx
//       (sub-folders included) - and ONCE in Source/Common, its definition in ThrottleByte.h (no double stretch, no
//       second path that stretches, and no shared header that stretches again);
//   (3) no map() call takes it (a stretch inside the map() lines would raise every cap by 6.25 %).
// The same checker runs first on made-up snippets that break each rule, so the guard is shown to bite. A source folder
// that cannot be read, or a folder walk that stops on an error, is a FAIL, not a skip.

// srcIdChar - true for a character that can be part of a C identifier.
static bool srcIdChar(char c) { return std::isalnum((unsigned char)c) || c == '_'; }

// srcCode - the text with every comment and the contents of every string and character literal replaced by spaces
// (newlines kept), so that a search finds code only. Handles //, /* */, "..." and '...' with escapes, and R"d(...)d".
// Input: the file's text. Output: the blanked copy, the same length. No side effects.
static std::string srcCode(const std::string& t)
{
  std::string o = t;
  const size_t n = t.size();
  size_t i = 0;
  while (i < n) {
    const char c = t[i];
    if (c == '/' && i + 1 < n && t[i + 1] == '/') {                                   // line comment
      while (i < n && t[i] != '\n') o[i++] = ' ';
    } else if (c == '/' && i + 1 < n && t[i + 1] == '*') {                            // block comment
      o[i] = ' '; o[i + 1] = ' '; i += 2;
      while (i < n && !(t[i] == '*' && i + 1 < n && t[i + 1] == '/')) { if (t[i] != '\n') o[i] = ' '; i++; }
      if (i < n) { o[i] = ' '; o[i + 1] = ' '; i += 2; }
    } else if (c == 'R' && i + 1 < n && t[i + 1] == '"' && (i == 0 || !srcIdChar(t[i - 1]))) {   // raw string
      const size_t open = t.find('(', i + 2);
      if (open == std::string::npos) { i++; continue; }
      const std::string close = ")" + t.substr(i + 2, open - (i + 2)) + "\"";
      size_t end = t.find(close, open + 1);
      end = (end == std::string::npos) ? n : end + close.size();
      for (size_t k = i + 2; k + 1 < end; k++) if (t[k] != '\n') o[k] = ' ';
      i = end;
    } else if (c == '"' || c == '\'') {                                               // string or character literal
      i++;
      while (i < n && t[i] != c && t[i] != '\n') {
        if (t[i] == '\\' && i + 1 < n) o[i++] = ' ';
        o[i++] = ' ';
      }
      i++;                                                                            // the closing quote stays
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

// srcFirstStatement - the first statement (up to and including its ';', white space squashed) of the DEFINITION of the
// function fn in code; a call or a prototype (no '{' after the ')') is skipped. "" when there is no definition.
static std::string srcFirstStatement(const std::string& code, const char* fn)
{
  for (size_t p : srcFindIdent(code, fn)) {
    size_t q = p + std::strlen(fn);
    while (q < code.size() && std::isspace((unsigned char)code[q])) q++;
    if (q >= code.size() || code[q] != '(') continue;
    const size_t close = code.find(')', q);
    if (close == std::string::npos) continue;
    size_t b = close + 1;
    while (b < code.size() && std::isspace((unsigned char)code[b])) b++;
    if (b >= code.size() || code[b] != '{') continue;
    const size_t semi = code.find(';', b + 1);
    return (semi == std::string::npos) ? std::string() : srcSquash(code.substr(b + 1, semi - b));
  }
  return std::string();
}

// srcMapCallsTaking - how many map( ... ) calls in code have the identifier id anywhere inside their parentheses.
static int srcMapCallsTaking(const std::string& code, const char* id)
{
  int hits = 0;
  for (size_t p : srcFindIdent(code, "map")) {
    size_t q = p + 3;
    while (q < code.size() && std::isspace((unsigned char)code[q])) q++;
    if (q >= code.size() || code[q] != '(') continue;
    int depth = 0;
    size_t e = q;
    for (; e < code.size(); e++) { if (code[e] == '(') depth++; else if (code[e] == ')' && --depth == 0) break; }
    if (!srcFindIdent(code.substr(q, e - q + 1), id).empty()) hits++;
  }
  return hits;
}

// StretchGuard - the guard's findings for one receiver source tree: PWM.ino's text, every other receiver file's, and
// every Source/Common file's.
struct StretchGuard { bool first_is_stretch; int pwm_count; int other_count; int common_count; int map_takes; };
static const char* kStretchLine = "uint8_t effective_thr = rtm_rx_emergency_stop ? 0 : thrByteToCmd(thr_received);";
static StretchGuard stretchGuard(const std::string& pwm_text, const std::vector<std::string>& other_texts,
                                 const std::vector<std::string>& common_texts)
{
  StretchGuard g;
  const std::string pwm = srcCode(pwm_text);
  g.first_is_stretch = (srcFirstStatement(pwm, "calcPWM") == kStretchLine);
  g.pwm_count    = (int)srcFindIdent(pwm, "thrByteToCmd").size();
  g.map_takes    = srcMapCallsTaking(pwm, "thrByteToCmd");
  g.other_count  = 0;
  g.common_count = 0;
  for (const std::string& t : other_texts) {
    const std::string code = srcCode(t);
    g.other_count += (int)srcFindIdent(code, "thrByteToCmd").size();
    g.map_takes   += srcMapCallsTaking(code, "thrByteToCmd");
  }
  for (const std::string& t : common_texts) {
    const std::string code = srcCode(t);
    g.common_count += (int)srcFindIdent(code, "thrByteToCmd").size();
    g.map_takes    += srcMapCallsTaking(code, "thrByteToCmd");
  }
  return g;
}
static bool stretchGuardOk(const StretchGuard& g)
{
  return g.first_is_stretch && g.pwm_count == 1 && g.other_count == 0 && g.common_count == 1 && g.map_takes == 0;
}

// srcReadFile - the whole file as bytes. false when it cannot be opened.
static bool srcReadFile(const std::filesystem::path& p, std::string* out)
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

// srcReadTree - every .ino / .h / .cpp file under dir (sub-folders included) into texts, except dir's own file named
// skip. Returns false when the walk could not start, stopped on an error, or a file could not be read; *files counts
// the source files seen (skip included).
static bool srcReadTree(const std::filesystem::path& dir, const char* skip, std::vector<std::string>* texts, int* files)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  bool ok = true;
  *files = 0;
  fs::recursive_directory_iterator it(dir, ec), end;
  if (ec) return false;
  for (; it != end; it.increment(ec)) {
    if (ec) break;
    std::error_code fec;
    const bool regular = it->is_regular_file(fec);
    if (fec) { ok = false; continue; }
    if (!regular) continue;
    const std::string ext = it->path().extension().string();
    if (ext != ".ino" && ext != ".h" && ext != ".cpp") continue;
    (*files)++;
    if (skip && it->path().filename() == skip && it->path().parent_path() == dir) continue;
    std::string t;
    ok = srcReadFile(it->path(), &t) && ok;
    texts->push_back(t);
  }
  return ok && !ec;
}

static void testGuardSnippets()
{
  // ---- The checker on made-up snippets: one that keeps every rule, then one per way of breaking a rule ----
  const std::string head = "void calcPWM()\n{\n  // the head\n  uint8_t effective_thr   = rtm_rx_emergency_stop ? 0 : "
                           "thrByteToCmd(thr_received);\n  if (cap < effective_thr) effective_thr = cap;\n";
  const std::string tail = "  PWM0_time = map(ramped_thr, 0, 255, lo, hi);\n}\n";
  const std::vector<std::string> none;
  const std::vector<std::string> def = { "static inline uint8_t thrByteToCmd(uint8_t b) { return b; }\n" };
  { const StretchGuard g = stretchGuard(head + tail, none, def);                  // keeps every rule
    CHECK(stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard("calcPWM();\n" + head + tail, none, def); // a call before the definition is skipped
    CHECK(stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard(                                          // (1) the stretch moved below a cap
      "void calcPWM()\n{\n  uint8_t effective_thr = rtm_rx_emergency_stop ? 0 : thr_received;\n"
      "  if (cap < effective_thr) effective_thr = cap;\n  effective_thr = thrByteToCmd(effective_thr);\n}\n", none, def);
    CHECK(!g.first_is_stretch && g.pwm_count == 1 && !stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard(                                          // (1) a statement put in front of it
      "void calcPWM()\n{\n  uint8_t lim = 255;\n  uint8_t effective_thr = rtm_rx_emergency_stop ? 0 : "
      "thrByteToCmd(thr_received);\n}\n", none, def);
    CHECK(!g.first_is_stretch && !stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard(                                          // (1)+(2) commented out: a comment is not code
      "void calcPWM()\n{\n  // uint8_t effective_thr = rtm_rx_emergency_stop ? 0 : thrByteToCmd(thr_received);\n"
      "  uint8_t effective_thr = rtm_rx_emergency_stop ? 0 : thr_received;\n}\n", none, def);
    CHECK(!g.first_is_stretch && g.pwm_count == 0 && !stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard(                                          // (2) a second stretch in PWM.ino
      head + "  effective_thr = thrByteToCmd(effective_thr);\n" + tail, none, def);
    CHECK(g.first_is_stretch && g.pwm_count == 2 && !stretchGuardOk(g)); }
  { const std::vector<std::string> other = { "uint8_t shown = thrByteToCmd(thr_received);\n" };
    const StretchGuard g = stretchGuard(head + tail, other, def);                 // (2) a stretch in another receiver file
    CHECK(g.other_count == 1 && !stretchGuardOk(g)); }
  { const std::vector<std::string> common = { def[0], "static inline uint8_t f(uint8_t b) { return thrByteToCmd(b); }\n" };
    const StretchGuard g = stretchGuard(head + tail, none, common);               // (2) a second use in a shared header
    CHECK(g.common_count == 2 && !stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard(head + tail, none, none);                 // (2) the definition is missing
    CHECK(g.common_count == 0 && !stretchGuardOk(g)); }
  { const StretchGuard g = stretchGuard(                                          // (3) a map() that takes it
      head + "  PWM0_time = map(thrByteToCmd(ramped_thr), 0, 255, lo, hi);\n}\n", none, def);
    CHECK(g.map_takes == 1 && !stretchGuardOk(g)); }
  { const std::vector<std::string> other = {                                       // comments and strings are not code ...
      "/* thrByteToCmd(x) */ const char* a = \"thrByteToCmd(\"; char q = '\"'; // thrByteToCmd\n"
      "const char* page = R\"raw(<a href=\"//x\">thrByteToCmd(' /*</a>)raw\";\n" };
    const StretchGuard g = stretchGuard(head + tail, other, def);
    CHECK(g.other_count == 0 && stretchGuardOk(g)); }
  { const std::vector<std::string> other = {                                       // ... but code after a raw string is
      "const char* page = R\"raw(<a href=\"//x\">' /*</a>)raw\"; uint8_t c = thrByteToCmd(b);\n" };
    const StretchGuard g = stretchGuard(head + tail, other, def);
    CHECK(g.other_count == 1 && !stretchGuardOk(g)); }
  { std::vector<std::string> texts; int files = 0;                                 // a folder that is not there is a FAIL
    CHECK(!srcReadTree(std::filesystem::path("no-such-folder-here"), nullptr, &texts, &files) && files == 0); }
}

static void testGuardSource()
{
  // ---- The receiver's real source: Source/V2_Integration_Rx and Source/Common, from Tools/tests or next to this file ----
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::path src = fs::path("..") / ".." / "Source";
  if (!fs::is_regular_file(src / "V2_Integration_Rx" / "PWM.ino", ec))
    src = fs::path(__FILE__).parent_path() / ".." / ".." / "Source";
  const fs::path rx = src / "V2_Integration_Rx";
  const fs::path common = src / "Common";
  std::printf("throttle_byte_test: the guard reads %s and %s\n", rx.string().c_str(), common.string().c_str());
  std::string pwm_text;
  std::vector<std::string> other_texts, common_texts;
  int rx_files = 0, common_files = 0;
  const bool pwm_ok    = srcReadFile(rx / "PWM.ino", &pwm_text);
  const bool rx_ok     = srcReadTree(rx, "PWM.ino", &other_texts, &rx_files);
  const bool common_ok = srcReadTree(common, nullptr, &common_texts, &common_files);
  CHECK(pwm_ok && !pwm_text.empty());                // PWM.ino was read
  CHECK(rx_ok && rx_files >= 2);                     // the whole receiver folder was walked, no error
  CHECK(common_ok && common_files >= 1);             // and the whole shared folder
  const StretchGuard g = stretchGuard(pwm_text, other_texts, common_texts);
  CHECK(g.first_is_stretch);                         // (1) the stretch is calcPWM()'s first statement
  CHECK(g.pwm_count == 1);                           // (2) once in PWM.ino ...
  CHECK(g.other_count == 0);                         //     ... nowhere else in the receiver ...
  CHECK(g.common_count == 1);                        //     ... and only its definition in Source/Common
  CHECK(g.map_takes == 0);                           // (3) no map() takes it
}

int main()
{
  testRemote();
  testReceiver();
  testRoundTrip();
  testPwmTable();
  testGuardSnippets();
  testGuardSource();
  if (fails) { std::printf("throttle_byte_test: %d of %d checks FAILED\n", fails, checks); return 1; }
  std::printf("throttle_byte_test: all %d checks passed\n", checks);
  return 0;
}
