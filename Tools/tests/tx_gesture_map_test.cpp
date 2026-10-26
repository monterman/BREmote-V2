// V2.5-Evo - 2026-10-26 - Host test: a SOURCE-TEXT GUARD over the remote's toggle and magnet gesture map (audit "LEFT-hold
//   lock" F-1 to F-6, owner design 2026-10-26: no new gestures). The host cannot compile the remote's .ino files (they need
//   Arduino), so this reads Hall.ino and RTMState.ino as TEXT, with every comment and every string's contents blanked, and
//   checks the five things the new map rests on:
//     G1 the LEFT 2 s hold LOCKS in every unlocked state: its branch in handleGearToggle() has no Follow-Me test before
//        the lock and no station step, and it switches Follow-Me off (fmSilentDisarm()) before the stop flush, with the
//        P-7 Return-To-Me end (setRtmDisarmed()) kept before the flush too;
//     G2 the RIGHT 2 s hold steps the station only while Follow-Me is armed (cycleFmModeArmed()), and the display cycle
//        is the else branch (on a multi-remote build still behind the take-back suppression, mtxTxSeizeSuppress());
//        cycleFmModeArmed() is called from no other branch of handleGearToggle();
//     G3 cycleFmModeArmed() carries the F-4 guards - Return-To-Me active or arming, the auto-return coming back
//        (fmIsReturning() and the raw fmReturnMovingRaw()) - each returning before the station changes;
//     G4 the LEFT tap -> RIGHT hold combo (cycleFmMode()) steps the station on an armed remote and never disarms;
//     G5 the mag_mode 4 tap walks all five stations: fmStepStationFromMagnet() passes 0x1F and no remote code outside
//        the config layer (ConfigService.ino, BREmote_V2_Tx.h) reads mag_fm_set.
//   The same checker runs first on made-up snippets - one that keeps every rule, then one per way of breaking a rule -
//   so the guard is shown to bite. A source folder that cannot be read is a FAIL, not a skip.
// Build and run, from Tools/tests (pass another TX source folder to check that one instead):
//   g++ -std=c++17 -O1 -Wall -Wextra tx_gesture_map_test.cpp -o tx_gesture_map_test && ./tx_gesture_map_test [tx-source-dir]

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <filesystem>
#include <string>
#include <vector>

static int g_checks = 0;
static int g_fails  = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fails++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)

// ---- Text helpers (the same blanking rules as multi_tx_test.cpp T44) ----

// srcIdChar - true for a character that can be part of a C identifier.
static bool srcIdChar(char c) { return isalnum((unsigned char)c) || c == '_'; }

// srcCode - the text with every comment and the contents of every string and character literal replaced by spaces
// (newlines kept), so that a search finds code only. Handles //, /* */, "..." and '...' with escapes, and R"d(...)d".
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
    } else if (c == 'R' && i + 1 < n && t[i + 1] == '"' && (i == 0 || !srcIdChar(t[i - 1]))) {
      const size_t open = t.find('(', i + 2);
      if (open == std::string::npos) { i++; continue; }
      const std::string close = ")" + t.substr(i + 2, open - (i + 2)) + "\"";
      size_t end = t.find(close, open + 1);
      end = (end == std::string::npos) ? n : end + close.size();
      for (size_t k = i + 2; k + 1 < end; k++) if (t[k] != '\n') o[k] = ' ';
      i = end;
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
  const size_t len = strlen(id);
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
    if (isspace((unsigned char)c)) { gap = true; continue; }
    if (gap && !o.empty()) o += ' ';
    o += c;
    gap = false;
  }
  return o;
}

// srcBlockAt - the text inside the { } block whose '{' is at open (brace-matched, braces only in code). "" if unmatched.
static std::string srcBlockAt(const std::string& code, size_t open)
{
  if (open >= code.size() || code[open] != '{') return std::string();
  int depth = 0;
  for (size_t e = open; e < code.size(); e++) {
    if (code[e] == '{') depth++;
    else if (code[e] == '}' && --depth == 0) return code.substr(open + 1, e - open - 1);
  }
  return std::string();
}

// srcFuncBody - the body of the DEFINITION of fn (a call or a prototype - no '{' after the ')' - is skipped). "" if none.
static std::string srcFuncBody(const std::string& code, const char* fn)
{
  for (size_t p : srcFindIdent(code, fn)) {
    size_t q = p + strlen(fn);
    while (q < code.size() && isspace((unsigned char)code[q])) q++;
    if (q >= code.size() || code[q] != '(') continue;
    const size_t close = code.find(')', q);
    if (close == std::string::npos) continue;
    size_t b = close + 1;
    while (b < code.size() && isspace((unsigned char)code[b])) b++;
    if (b >= code.size() || code[b] != '{') continue;
    return srcBlockAt(code, b);
  }
  return std::string();
}

// srcTokensAt - does the token sequence toks start at position p (white space between tokens ignored)? On success *end is
// the position just after the last token. Identifier tokens must stand on their own.
static bool srcTokensAt(const std::string& code, size_t p, const std::vector<std::string>& toks, size_t* end)
{
  for (const std::string& t : toks) {
    while (p < code.size() && isspace((unsigned char)code[p])) p++;
    if (code.compare(p, t.size(), t) != 0) return false;
    if (srcIdChar(t[0]) && p > 0 && srcIdChar(code[p - 1])) return false;
    if (srcIdChar(t.back()) && p + t.size() < code.size() && srcIdChar(code[p + t.size()])) return false;
    p += t.size();
  }
  *end = p;
  return true;
}

// srcBlocksAfter - the { } block that follows every occurrence of the token sequence toks (whose last token is "{").
static std::vector<std::string> srcBlocksAfter(const std::string& code, const std::vector<std::string>& toks)
{
  std::vector<std::string> out;
  for (size_t p = code.find(toks[0]); p != std::string::npos; p = code.find(toks[0], p + 1)) {
    size_t e;
    if (srcTokensAt(code, p, toks, &e)) out.push_back(srcBlockAt(code, e - 1));
  }
  return out;
}

// srcPos - position of the first identifier id (or, for a non-identifier text, the first occurrence) in s; npos if none.
static size_t srcPos(const std::string& s, const char* id)
{
  if (srcIdChar(id[0])) { std::vector<size_t> v = srcFindIdent(s, id); return v.empty() ? std::string::npos : v[0]; }
  return s.find(id);
}
static bool srcHas(const std::string& s, const char* id) { return srcPos(s, id) != std::string::npos; }
// srcBefore - a and b both present, and the first a comes before the first b.
static bool srcBefore(const std::string& s, const char* a, const char* b)
{
  const size_t pa = srcPos(s, a), pb = srcPos(s, b);
  return pa != std::string::npos && pb != std::string::npos && pa < pb;
}

// ---- The guard ----

// MapGuard - the findings for one remote source: Hall.ino's text, RTMState.ino's text and every other TX file that is
// not the config layer.
struct MapGuard {
  // G1 - the LEFT 2 s hold
  bool left_found, left_locks, left_no_fm_before_lock, left_no_step, left_fm_clear, left_clear_before_flush, left_p7_kept;
  // G2 - the RIGHT 2 s hold
  bool right_found, right_steps_if_armed, right_display_is_else, right_takeback_ok, step_only_in_right;
  // G3 - cycleFmModeArmed()'s F-4 guards
  bool armed_found, armed_rtm_guard, armed_return_guard, armed_returns_before_step;
  // G4 - the combo on an armed remote
  bool combo_found, combo_steps, combo_never_disarms;
  // G5 - the magnet tap
  bool mag_found, mag_all_five, mag_set_unread;
};

static MapGuard mapGuard(const std::string& hall_text, const std::string& rtm_text, const std::vector<std::string>& other_texts)
{
  MapGuard g = {};
  const std::string hall = srcCode(hall_text);
  const std::string rtm  = srcCode(rtm_text);
  std::vector<std::string> others;
  for (const std::string& t : other_texts) others.push_back(srcCode(t));
  // A multi-remote build has the take-back: its suppression is named in Hall.ino or defined in another file (MultiRemote.ino).
  bool multi = srcHas(hall, "mtxTxSeizeSuppress");
  for (const std::string& c : others) if (srcHas(c, "mtxTxSeizeSuppress")) multi = true;

  const std::string hgt = srcFuncBody(hall, "handleGearToggle");
  const std::vector<std::string> left  = srcBlocksAfter(hgt, {"else", "if", "(", "direction", "<", "0", ")", "{"});
  const std::vector<std::string> right = srcBlocksAfter(hgt, {"else", "if", "(", "direction", ">", "0", ")", "{"});

  // G1
  g.left_found = (left.size() == 1);
  if (g.left_found) {
    const std::string& L = left[0];
    const size_t lock = L.find("system_locked = 1;");
    g.left_locks             = (lock != std::string::npos) && srcHas(L, "no_lock");
    g.left_no_fm_before_lock = g.left_locks && !srcHas(L.substr(0, lock), "isFmArmed") && !srcHas(L.substr(0, lock), "fm_armed");
    g.left_no_step           = !srcHas(L, "cycleFmModeArmed") && !srcHas(L, "cycleFmMode") && !srcHas(L, "fmStepStationFromMagnet");
    g.left_fm_clear          = g.left_locks && srcHas(L.substr(lock), "fmSilentDisarm");
    g.left_clear_before_flush = srcBefore(L, "fmSilentDisarm", "rtmFmStopFlush");
    g.left_p7_kept           = srcBefore(L, "setRtmDisarmed", "rtmFmStopFlush");
  }
  // G2
  g.right_found = (right.size() == 1);
  if (g.right_found) {
    const std::string& R = right[0];
    g.right_steps_if_armed = srcBefore(R, "isFmArmed", "cycleFmModeArmed");
    const size_t ps = srcPos(R, "cycleFmModeArmed"), pd = srcPos(R, "cycleDisplayMode");
    g.right_display_is_else = ps != std::string::npos && pd != std::string::npos && ps < pd &&
                              srcHas(R.substr(ps, pd - ps), "else");
    g.right_takeback_ok = !multi || (srcBefore(R, "cycleFmModeArmed", "mtxTxSeizeSuppress") &&
                                     srcBefore(R, "mtxTxSeizeSuppress", "cycleDisplayMode"));
    g.step_only_in_right = srcFindIdent(hgt, "cycleFmModeArmed").size() == 1 && srcFindIdent(R, "cycleFmModeArmed").size() == 1;
  }
  // G3
  const std::string armed = srcFuncBody(rtm, "cycleFmModeArmed");
  g.armed_found = !armed.empty();
  if (g.armed_found) {
    const size_t step = armed.find("last_fm_mode =");
    const std::string head = (step == std::string::npos) ? std::string() : armed.substr(0, step);
    g.armed_rtm_guard    = srcHas(head, "rtm_tx_active") && srcHas(head, "rtmIsArming");
    g.armed_return_guard = srcHas(head, "fmIsReturning") && srcHas(head, "fmReturnMovingRaw");
    g.armed_returns_before_step = srcFindIdent(head, "return").size() >= 3;   // not armed, RTM, returning
  }
  // G4
  const std::string combo = srcFuncBody(rtm, "cycleFmMode");
  g.combo_found = !combo.empty();
  if (g.combo_found) {
    const std::vector<std::string> armed_branch = srcBlocksAfter(combo, {"if", "(", "fm_armed", ")", "{"});
    g.combo_steps         = armed_branch.size() == 1 && srcHas(armed_branch[0], "cycleFmModeArmed");
    g.combo_never_disarms = !srcHas(combo, "fmDisarm") && !srcHas(combo, "fmSilentDisarm");
  }
  // G5
  const std::string mag = srcFuncBody(rtm, "fmStepStationFromMagnet");
  g.mag_found = !mag.empty();
  g.mag_all_five = g.mag_found && srcSquash(mag).find("fmNextStationInSet(last_fm_mode, 0x1F)") != std::string::npos;
  g.mag_set_unread = !srcHas(hall, "mag_fm_set") && !srcHas(rtm, "mag_fm_set");
  for (const std::string& c : others) if (srcHas(c, "mag_fm_set")) g.mag_set_unread = false;
  return g;
}

static bool g1(const MapGuard& g) { return g.left_found && g.left_locks && g.left_no_fm_before_lock && g.left_no_step && g.left_fm_clear && g.left_clear_before_flush && g.left_p7_kept; }
static bool g2(const MapGuard& g) { return g.right_found && g.right_steps_if_armed && g.right_display_is_else && g.right_takeback_ok && g.step_only_in_right; }
static bool g3(const MapGuard& g) { return g.armed_found && g.armed_rtm_guard && g.armed_return_guard && g.armed_returns_before_step; }
static bool g4(const MapGuard& g) { return g.combo_found && g.combo_steps && g.combo_never_disarms; }
static bool g5(const MapGuard& g) { return g.mag_found && g.mag_all_five && g.mag_set_unread; }
static bool mapGuardOk(const MapGuard& g) { return g1(g) && g2(g) && g3(g) && g4(g) && g5(g); }

// ---- Made-up sources: one that keeps every rule (single-remote shape), and the pieces a mutation swaps ----

static const char* kRightGood =
  "        else if (direction > 0)\n        {\n          if (isFmArmed())\n          {\n            cycleFmModeArmed();\n          }\n"
  "          else cycleDisplayMode(1);\n        }\n";
static const char* kLeftGood =
  "        else if (direction < 0)\n        {\n          if (!usrConf.no_lock)\n          {\n            system_locked = 1;\n"
  "            if (isFmArmed()) { Serial.println(\"FM [TX] lock\"); fmSilentDisarm(); }\n"
  "            if (rtm_tx_active) setRtmDisarmed();\n            displayLock();\n            rtmFmStopFlush();\n          }\n        }\n";

static std::string fakeHall(const std::string& right, const std::string& left)
{
  return std::string("void handleGearToggle(int direction)\n{\n  if (has_combo)\n  {\n    if (direction < 0 && last_tap_dir == 1) returnGesture();\n"
                     "    else if (direction > 0 && last_tap_dir == -1) cycleFmMode();\n  }\n") + right + left + "}\n";
}

static const char* kArmedGood =
  "void cycleFmModeArmed()\n{\n  if (!fm_armed) return;\n  if (rtm_tx_active || rtmIsArming())\n  {\n    Serial.println(\"x\");\n    return;\n  }\n"
  "  if (fmIsReturning() || fmReturnMovingRaw())\n  {\n    return;\n  }\n  last_fm_mode = (last_fm_mode < 5) ? last_fm_mode + 1 : 1;\n}\n";
static const char* kComboGood =
  "void cycleFmMode()\n{\n  if (!usrConf.fm_override_enabled || !usrConf.gps_en) return;\n  if (fm_armed)\n  {\n    cycleFmModeArmed();\n    return;\n  }\n"
  "  fm_armed = true;\n}\n";
static const char* kMagGood =
  "bool fmStepStationFromMagnet()\n{\n  if (!fm_armed) return false;\n  uint8_t next = fmNextStationInSet(last_fm_mode, 0x1F);\n  return next != 0;\n}\n";

static std::string fakeRtm(const std::string& armed, const std::string& combo, const std::string& mag)
{
  return combo + armed + mag;
}

// replaceOnce - s with the first `from` replaced by `to` (from must be present: a made-up mutation that does not apply is a bug).
static std::string replaceOnce(std::string s, const std::string& from, const std::string& to)
{
  const size_t p = s.find(from);
  if (p == std::string::npos) { printf("  test bug: mutation text not found: %s\n", from.c_str()); g_fails++; return s; }
  return s.replace(p, from.size(), to);
}

static bool readFile(const std::filesystem::path& p, std::string* out)
{
  FILE* f = fopen(p.string().c_str(), "rb");
  if (!f) return false;
  out->clear();
  char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) out->append(buf, n);
  fclose(f);
  return true;
}

int main(int argc, char** argv)
{
  printf("tx_gesture_map_test - the remote's gesture map as source text (LEFT hold locks, RIGHT hold steps while armed)\n");

  // ---- The checker on made-up sources ----
  const std::string hall_ok = fakeHall(kRightGood, kLeftGood);
  const std::string rtm_ok  = fakeRtm(kArmedGood, kComboGood, kMagGood);
  const std::vector<std::string> none;
  { const MapGuard g = mapGuard(hall_ok, rtm_ok, none); CHECK(mapGuardOk(g)); }                 // keeps every rule
  { // a multi-remote RIGHT block with the take-back suppression in the display branch keeps the rules too
    const std::string r = replaceOnce(kRightGood, "else cycleDisplayMode(1);", "else if (!mtxTxSeizeSuppress()) cycleDisplayMode(1);");
    const MapGuard g = mapGuard(fakeHall(r, kLeftGood), rtm_ok, none); CHECK(mapGuardOk(g)); }
  { // G1 bites: the old P8.1 intercept is back (FM armed -> station step, else lock)
    const std::string l = replaceOnce(kLeftGood, "if (!usrConf.no_lock)", "if (isFmArmed()) cycleFmModeArmed();\n          else if (!usrConf.no_lock)");
    const MapGuard g = mapGuard(fakeHall(kRightGood, l), rtm_ok, none);
    CHECK(!g.left_no_fm_before_lock && !g.left_no_step && !g1(g) && !mapGuardOk(g)); }
  { // G1 bites: the lock no longer switches Follow-Me off (F-6)
    const std::string l = replaceOnce(kLeftGood, "fmSilentDisarm();", "");
    const MapGuard g = mapGuard(fakeHall(kRightGood, l), rtm_ok, none);
    CHECK(!g.left_fm_clear && !g1(g)); }
  { // G1 bites: Follow-Me is switched off only AFTER the flush (the keepalive could slip in between)
    std::string l = replaceOnce(kLeftGood, "if (isFmArmed()) { Serial.println(\"FM [TX] lock\"); fmSilentDisarm(); }\n", "");
    l = replaceOnce(l, "rtmFmStopFlush();", "rtmFmStopFlush();\n            fmSilentDisarm();");
    const MapGuard g = mapGuard(fakeHall(kRightGood, l), rtm_ok, none);
    CHECK(!g.left_clear_before_flush && !g1(g)); }
  { // G1 bites: P-7 (the lock ends the remote's own return) dropped
    const std::string l = replaceOnce(kLeftGood, "if (rtm_tx_active) setRtmDisarmed();", "");
    const MapGuard g = mapGuard(fakeHall(kRightGood, l), rtm_ok, none);
    CHECK(!g.left_p7_kept && !g1(g)); }
  { // G1: the old intercept only in a COMMENT is not code - still passes
    const std::string l = replaceOnce(kLeftGood, "if (!usrConf.no_lock)", "// if (isFmArmed()) cycleFmModeArmed(); else\n          if (!usrConf.no_lock)");
    const MapGuard g = mapGuard(fakeHall(kRightGood, l), rtm_ok, none); CHECK(g1(g) && mapGuardOk(g)); }
  { // G2 bites: the RIGHT hold is the display cycle only (no station step)
    const MapGuard g = mapGuard(fakeHall("        else if (direction > 0)\n        {\n          cycleDisplayMode(1);\n        }\n", kLeftGood), rtm_ok, none);
    CHECK(!g.right_steps_if_armed && !g2(g)); }
  { // G2 bites: the step does not depend on Follow-Me being armed, and the display cycle runs as well
    const MapGuard g = mapGuard(fakeHall("        else if (direction > 0)\n        {\n          cycleFmModeArmed();\n          cycleDisplayMode(1);\n        }\n", kLeftGood), rtm_ok, none);
    CHECK(!g.right_steps_if_armed && !g.right_display_is_else && !g2(g)); }
  { // G2 bites (multi-remote shape): the take-back suppression dropped from the display branch
    const std::string r = replaceOnce(kRightGood, "else cycleDisplayMode(1);", "else cycleDisplayMode(1);\n          (void)mtxTxSeizeSuppress;");
    const MapGuard g = mapGuard(fakeHall(r, kLeftGood), rtm_ok, none);
    CHECK(!g.right_takeback_ok && !g2(g)); }
  { // G2 bites (multi-remote build: MultiRemote.ino defines the suppression): Hall.ino dropped it altogether
    const std::vector<std::string> mr = { "bool mtxTxSeizeSuppress()\n{\n  return (s_seize_out & 4) != 0;\n}\n" };
    CHECK(!g2(mapGuard(hall_ok, rtm_ok, mr)));
    const std::string r = replaceOnce(kRightGood, "else cycleDisplayMode(1);", "else if (!mtxTxSeizeSuppress()) cycleDisplayMode(1);");
    CHECK(mapGuardOk(mapGuard(fakeHall(r, kLeftGood), rtm_ok, mr))); }
  { // G2 bites: a second station step elsewhere in handleGearToggle() (say the LEFT tap)
    const std::string h = replaceOnce(hall_ok, "if (has_combo)", "if (direction < 0 && tapped) cycleFmModeArmed();\n  if (has_combo)");
    const MapGuard g = mapGuard(h, rtm_ok, none);
    CHECK(!g.step_only_in_right && !g2(g)); }
  { // G3 bites: no Return-To-Me guard
    const std::string a = replaceOnce(kArmedGood, "rtm_tx_active || rtmIsArming()", "false");
    const MapGuard g = mapGuard(hall_ok, fakeRtm(a, kComboGood, kMagGood), none);
    CHECK(!g.armed_rtm_guard && !g3(g)); }
  { // G3 bites: no raw returning guard (fmIsReturning() alone errs toward stepping over a moving return)
    const std::string a = replaceOnce(kArmedGood, "fmIsReturning() || fmReturnMovingRaw()", "fmIsReturning()");
    const MapGuard g = mapGuard(hall_ok, fakeRtm(a, kComboGood, kMagGood), none);
    CHECK(!g.armed_return_guard && !g3(g)); }
  { // G3 bites: the guards are there but only AFTER the station changed
    std::string a = replaceOnce(kArmedGood, "  last_fm_mode = (last_fm_mode < 5) ? last_fm_mode + 1 : 1;\n", "");
    a = replaceOnce(a, "  if (rtm_tx_active", "  last_fm_mode = (last_fm_mode < 5) ? last_fm_mode + 1 : 1;\n  if (rtm_tx_active");
    const MapGuard g = mapGuard(hall_ok, fakeRtm(a, kComboGood, kMagGood), none);
    CHECK(!g.armed_rtm_guard && !g.armed_return_guard && !g3(g)); }
  { // G4 bites: the old combo - disarm after the first squeeze
    const std::string c = replaceOnce(kComboGood, "    cycleFmModeArmed();\n", "    if (fm_throttle_seen) fmDisarm(true);\n    else cycleFmModeArmed();\n");
    const MapGuard g = mapGuard(hall_ok, fakeRtm(kArmedGood, c, kMagGood), none);
    CHECK(g.combo_steps && !g.combo_never_disarms && !g4(g)); }
  { // G4 bites: the armed combo does nothing (no step)
    const std::string c = replaceOnce(kComboGood, "    cycleFmModeArmed();\n", "");
    const MapGuard g = mapGuard(hall_ok, fakeRtm(kArmedGood, c, kMagGood), none);
    CHECK(!g.combo_steps && !g4(g)); }
  { // G5 bites: the magnet walks mag_fm_set again
    const std::string m = replaceOnce(kMagGood, "0x1F", "usrConf.mag_fm_set");
    const MapGuard g = mapGuard(hall_ok, fakeRtm(kArmedGood, kComboGood, m), none);
    CHECK(!g.mag_all_five && !g.mag_set_unread && !g5(g)); }
  { // G5 bites: another remote file reads mag_fm_set; a comment or a string naming it does not count
    const std::vector<std::string> bad  = { "uint16_t m = usrConf.mag_fm_set;\n" };
    const std::vector<std::string> fine = { "// mag_fm_set is no longer read\nconst char* k = \"mag_fm_set\";\n" };
    CHECK(!g5(mapGuard(hall_ok, rtm_ok, bad)));
    CHECK(g5(mapGuard(hall_ok, rtm_ok, fine))); }

  // ---- The remote's real source: argv[1], else Source/V2_Integration_Tx from Tools/tests or next to this file ----
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::path dir;
  if (argc > 1) dir = fs::path(argv[1]);
  else {
    dir = fs::path("..") / ".." / "Source" / "V2_Integration_Tx";
    if (!fs::is_regular_file(dir / "Hall.ino", ec))
      dir = fs::path(__FILE__).parent_path() / ".." / ".." / "Source" / "V2_Integration_Tx";
  }
  std::string hall_text, rtm_text;
  std::vector<std::string> other_texts;
  bool read_ok = readFile(dir / "Hall.ino", &hall_text) && readFile(dir / "RTMState.ino", &rtm_text);
  int files = 0;
  for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
    if (!it->is_regular_file(ec)) continue;
    const std::string ext = it->path().extension().string();
    if (ext != ".ino" && ext != ".h" && ext != ".cpp") continue;
    files++;
    const std::string name = it->path().filename().string();
    // The config layer keeps the field (struct, default, range check, web page); Hall.ino / RTMState.ino are read above.
    if (name == "Hall.ino" || name == "RTMState.ino" || name == "ConfigService.ino" || name == "BREmote_V2_Tx.h" ||
        name == "WebUiEmbedded.h") continue;
    std::string t;
    read_ok = readFile(it->path(), &t) && read_ok;
    other_texts.push_back(t);
  }
  printf("  source: %s (%d files)\n", dir.string().c_str(), files);
  if (!read_ok || files < 3) printf("  cannot read the remote source at %s (run from Tools/tests, or pass the folder)\n", dir.string().c_str());
  CHECK(read_ok && files >= 3 && !hall_text.empty() && !rtm_text.empty());
  const MapGuard g = mapGuard(hall_text, rtm_text, other_texts);
  CHECK(g.left_found);               // G1: one LEFT simple-hold branch in handleGearToggle()
  CHECK(g.left_locks);               //     it locks (behind no_lock only)
  CHECK(g.left_no_fm_before_lock);   //     no Follow-Me test in front of the lock (the P8.1 intercept is gone)
  CHECK(g.left_no_step);             //     no station step on the LEFT hold
  CHECK(g.left_fm_clear);            //     F-6: the lock switches Follow-Me off ...
  CHECK(g.left_clear_before_flush);  //     ... before the stop flush
  CHECK(g.left_p7_kept);             //     P-7: the lock still ends the remote's own return before the flush
  CHECK(g.right_found);              // G2: one RIGHT simple-hold branch
  CHECK(g.right_steps_if_armed);     //     Follow-Me armed -> station step
  CHECK(g.right_display_is_else);    //     otherwise the display cycle
  CHECK(g.right_takeback_ok);        //     (multi-remote) the take-back suppression still guards the display cycle
  CHECK(g.step_only_in_right);       //     and no other branch of handleGearToggle() steps the station
  CHECK(g.armed_found);              // G3: cycleFmModeArmed() exists
  CHECK(g.armed_rtm_guard);          //     Return-To-Me active or arming -> ignored
  CHECK(g.armed_return_guard);       //     auto-return coming back (confirmed or raw) -> ignored
  CHECK(g.armed_returns_before_step);//     and both return before the station changes
  CHECK(g.combo_found);              // G4: cycleFmMode() exists
  CHECK(g.combo_steps);              //     armed -> station step
  CHECK(g.combo_never_disarms);      //     and it never disarms
  CHECK(g.mag_found);                // G5: fmStepStationFromMagnet() exists
  CHECK(g.mag_all_five);             //     it walks all five stations
  CHECK(g.mag_set_unread);           //     and nothing outside the config layer reads mag_fm_set
  printf("  G1 lock %s, G2 RIGHT hold %s, G3 F-4 guards %s, G4 combo %s, G5 magnet %s\n",
         g1(g) ? "ok" : "FAIL", g2(g) ? "ok" : "FAIL", g3(g) ? "ok" : "FAIL", g4(g) ? "ok" : "FAIL", g5(g) ? "ok" : "FAIL");

  printf("%s: %d checks, %d failed\n", g_fails ? "FAILED" : "PASSED", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
