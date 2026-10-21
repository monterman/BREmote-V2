// V2.5-Evo - 2026-10-21 - STEP-3 AUDIT P3-1: the summon and take-back answers are judged on the first owner arrival
//   (Common/MultiTx.h mtxTxOwnerWait()) and polled from handleGearToggle()'s loops too (mtxTxAnswerPoll()); the summon
//   tick is split into the wait (mtxTxSummonWait()) and the deferred arm (mtxTxSummonGo(), loop task only).
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE REMOTE STEP (TX SW29): new file. The remote's side of several remotes on one
//   buggy (design v3.1 section 9, v3.2 sections 3.4.2 and 5) that is not radio code and not a gesture: the boot
//   identity line (design 9.13). The radio side (frames, status, rate, pairing) stays in Radio.ino, the gestures in
//   Hall.ino / RTMState.ino. Every function here runs on the loop task.
// V2.5-Evo - 2026-10-20 - and the loop side of the ownership rules: the X-1 activity repeat, the summon with its polled wait
//   (X-7, X-9), the master's take-back (C20), the refusal "St" / E9 (owner rules), "tA" and the local disarm on losing
//   control, the ownership dot, ?mtx and ?mtxdot.

// ============================================================
// THE BOOT IDENTITY LINE (design 9.13, MT-00)
// ============================================================
// mtxTxBootReport - one "MTX:" line after the config is loaded: this chip's ID (eFuse), the stored own_address, and the
//   link: new-style (proto 3: the buggy's ID and the link ID L this remote sends), old-style (proto 0: the buggy's
//   address, today's frames), or not paired. Two NOTEs where they apply: a stored own_address that is not this chip's
//   ID (a copied config - harmless on a new-style link, where the ID comes from the chip), and no_lock, which also
//   turns off the spare-remote auto-lock. Inputs: g_efuse_id, usrConf. Side effects: Serial only.
//   (Whether this remote is the master is the buggy's answer - SEIZABLE - so ?mtx shows it, not this line.)
void mtxTxBootReport()
{
  char me[9], own[9], dest[9];
  snprintf(me,   sizeof(me),   "%02X:%02X:%02X", g_efuse_id[0], g_efuse_id[1], g_efuse_id[2]);
  snprintf(own,  sizeof(own),  "%02X:%02X:%02X", usrConf.own_address[0], usrConf.own_address[1], usrConf.own_address[2]);
  snprintf(dest, sizeof(dest), "%02X:%02X:%02X", usrConf.dest_address[0], usrConf.dest_address[1], usrConf.dest_address[2]);
  if (!usrConf.paired)
  {
    Serial.printf("MTX: id %s (eFuse), own_address %s, not paired - hold RIGHT at power-on to pair\n", me, own);
  }
  else if (txIsV3())
  {
    MtxLinkRec r;
    txLinkRec(&r);
    uint8_t L[3];
    mtxUplinkAddr(&r, L);
    Serial.printf("MTX: id %s (eFuse), own_address %s, link v3 to %s, L %02X:%02X:%02X\n", me, own, dest, L[0], L[1], L[2]);
  }
  else
  {
    Serial.printf("MTX: id %s (eFuse), own_address %s, link legacy to %s (old-style frames, as before; pair again for the multi-remote link)\n",
                  me, own, dest);
  }
  if (!mtxIdEqual(usrConf.own_address, g_efuse_id))
    Serial.printf("NOTE: stored own_address %s differs from this board's eFuse id %s (config copied?) - a new-style link uses the chip ID\n", own, me);
  if (usrConf.no_lock && txIsV3())
    Serial.println("NOTE: no_lock is set - a spare remote will not lock itself");
}

// ============================================================
// V2.5-Evo - 2026-10-20 - THE REMOTE ON A MULTI-REMOTE BUGGY: GESTURES, ANSWERS, SCREEN (design v3.1 9.3-9.10, v3.2 3.4.2;
//   audit X-1, X-7, X-8, X-9, C9, C20; owner rules 2026-10-20). Loop task only. Every function returns at once, or does
//   today's thing, on an OLD-STYLE link (mtx_link_proto 0) and with ONE remote (status 000): there the buggy never sends
//   a multi-remote status, so g_mtx_tx stays "no reply" or "you control" and none of this changes behaviour.
// ============================================================
extern volatile uint8_t current_vib_pattern;   // System.ino
extern volatile bool    vib_stop_pending;      // System.ino: request the Pattern 7 stop buzz (never write 7 directly)
static void gpsKeepAliveDelay(uint32_t ms);    // RTMState.ino (static there; this repeats its declaration, as Hall.ino does)

static MtxTxActivity   s_act;                  // the X-1 activity window (zero = closed)
static MtxSeizeGesture s_seize;                // the master's take-back gesture (zero = idle)
static uint8_t         s_seize_out       = 0;  // the gesture's last MTX_SGO_* answer
static bool            s_seize_wait      = false;
static uint32_t        s_seize_sent_ms   = 0;
static uint8_t         s_summon_kind     = 0;  // 0 = none, MTX_SUMMON_RTM / MTX_SUMMON_FM = a SUMMON is waiting for its answer
static uint32_t        s_summon_sent_ms  = 0;
static uint8_t         s_summon_go       = 0;  // granted: the arm to run once the owner-edge boot ID has gone out
static uint32_t        s_summon_go_ms    = 0;
static uint8_t         s_last_summon_v   = 0xFF;
static bool            s_e9_was_on       = false;
static uint32_t        s_n_activity = 0, s_n_summon = 0, s_n_seize = 0;
static const uint32_t  kMtxSummonBootIdWaitMs = 600;   // the longest the arm waits for the owner-edge boot ID to drain

// mtxTxNoteInput - the rate rule's "input" happened now (a toggle press, a magnet contact; the loop tick adds throttle
//   and steering). Side effects: mtx_input_ms.
void mtxTxNoteInput()
{
  const unsigned long t = millis();
  mtx_input_ms = (t != 0) ? t : 1;
}

// mtxActivityNote / mtxActivityTick - THE X-1 ACTIVITY REPEAT (Common/MultiTx.h section 17, host test T34). Note: a press
//   is in progress (opens or extends the window; the first repeat is due at once). Tick: queue one 0xF1/0 when one is
//   due - a FREE queue slot only (queueMetaPacketIfFree(): refused while any 0xF1 is pending, so a repeat can never
//   rewrite a queued 0xF1/1), every 2 s at most, only on a new-style link with the status OWNER (001: in control, others
//   paired), never locked or in setup, never with RTM active or an 0xF1/1 queued, not with the throttle byte at 8 or
//   more. Called from the toggle and magnet code, the blocking waits of the RTM ceremony and every loop pass.
//   Side effects: s_act, the meta queue.
void mtxActivityNote()
{
  mtxTxActivityNote(&s_act, millis());
}
void mtxActivityTick()
{
  if (!txIsV3()) return;
  MtxTxStatus t;
  mtxTxStatusGet(&t);
  MtxTxActIn in;
  in.v3            = true;
  in.owner_multi   = (t.count > 0 && t.last[0] == MTX_S_OWNER);
  in.locked        = system_locked;
  in.in_setup      = in_setup;
  in.rtm_active    = rtm_tx_active.load();
  in.rtm_on_queued = metaQueuePending(0xF1, 1, false);
  in.arming        = rtmIsArming();
  in.thr_byte      = thr_sent;
  const uint32_t now = millis();
  if (mtxTxActivityDue(&s_act, &in, now) && queueMetaPacketIfFree(0xF1, 0, 1))
  {
    mtxTxActivitySent(&s_act, now);
    s_n_activity++;
  }
}

// mtxTxIsNonOwner - a remote NOT in control on a multi-remote buggy right now (mtxTxNotOwner(): a new-style link and a
//   latest status other than "you control"). The six fresh-arm sites take the summon gate while this is true.
bool mtxTxIsNonOwner()
{
  if (!txIsV3()) return false;
  MtxTxStatus t;
  mtxTxStatusGet(&t);
  return mtxTxNotOwner(&t, true);
}

// mtxTxStopRefusal - THE REFUSAL SIGNAL (owner 2026-10-20: "if it cannot engage, show stop. That's the regular response
//   always"): "St" + the stop buzz. style: MTX_ST_BLOCK = the usual 2 s, blocking like every existing "St" (the trigger is
//   released and nothing runs); MTX_ST_HELD = 2 s held WITHOUT blocking (the trigger is held - the rider is driving);
//   MTX_ST_BRIEF = 1 s held without blocking, then the running mode's screen (this remote's own Follow-Me / return runs;
//   interim default 2). Inputs: why (one serial line, or NULL), style. Side effects: the stop buzz, the screen, Serial.
void mtxTxStopRefusal(const char* why, uint8_t style)
{
  if (why) Serial.println(why);
  vib_stop_pending = true;
  if (style == MTX_ST_BLOCK)
  {
    DISP_LOCK(); displayDigits(LET_S, LET_T); updateDisplay(); DISP_UNLOCK();
    gpsKeepAliveDelay(2000);
  }
  else
  {
    showStopHeld(style == MTX_ST_BRIEF ? kMtxStBriefMs : kMtxStHeldMs);
  }
}

// mtxTxShowE9 - a takeover try against a remote that is busy in control (owner rule: E9, as for a squeeze). Opens the
//   1.5 s E9 window the screen draws; the loop tick gives the one Pattern A buzz on its rising edge.
static void mtxTxShowE9()
{
  const unsigned long u = millis() + kMtxE9HoldMs;
  mtx_e9_until_ms = (u != 0) ? u : 1;
}

static const char* mtxSummonVerdictText(uint8_t v)
{
  switch (v)
  {
    case MTX_SV_OK:       return "ok";
    case MTX_SV_NOT_FREE: return "the buggy does not say FREE (another remote is in control and not claimable, or no reply for 5 s)";
    case MTX_SV_NO_FIX:   return "no fresh GPS fix (manual control only - squeeze from zero to take the buggy)";
    case MTX_SV_TRIGGER:  return "the trigger is not released";
    case MTX_SV_DISABLED: return "that mode is switched off on this remote, or gps_en is 0";
    case MTX_SV_LOCKED:   return "the remote is locked";
    case MTX_SV_SETUP:    return "the remote is in setup";
    default:              return "-";
  }
}

// mtxTxSummonTry - V2.5-Evo - 2026-10-20 - THE SUMMON (design v3.2 3.4.2; audit X-7, X-9). Called by the six fresh-arm sites
//   while this remote is NOT in control, BEFORE their own enable checks. The gate (mtxSummonVerdict(), host test T29): the
//   latest status FREE and at most 5 s old; a fresh own fix - the exact test that lets this remote stream its rider GPS;
//   the trigger released; the feature on; not locked, not in setup.
//   REFUSED: NOTHING IS SENT. A refusal because another remote is busy in control is E9 (owner rule: a takeover try
//   against a busy owner); every other refusal is "St" + the stop buzz (held without blocking if the trigger is held).
//   PASSED: the SUMMON burst (internal queue type kMtxQueueSummon, which only the builder turns into 0xF1/0x3C, proto 3
//   only), and THE WAIT IS A POLLED STATE (X-9), NOT A BLOCKING ONE: mtxTxLoopTick() watches for an owner status for up to
//   kMtxSummonWaitMs (1500 ms) while loop() keeps running (GPS, display, every state machine). One summon at a time.
//   Inputs: kind (MTX_SUMMON_RTM / MTX_SUMMON_FM). Side effects: the meta queue, the summon state, the screen, Serial.
void mtxTxSummonTry(uint8_t kind)
{
  if (s_summon_kind != 0 || s_summon_go != 0)
  {
    Serial.println("MTX [TX] a summon is already waiting for the buggy's answer - this gesture is ignored");
    return;
  }
  MtxTxStatus t;
  mtxTxStatusGet(&t);
  MtxSummonIn in;
  in.gps_en           = (usrConf.gps_en != 0);
  in.loc_valid        = gps_tx.location.isValid();
  in.loc_age_ms       = gps_tx.location.age();
  in.gps_stale_ms     = usrConf.tx_gps_stale_timeout_ms;
  in.trigger_released = triggerReleased();
  in.feature_enabled  = (kind == MTX_SUMMON_RTM) ? rtmEnabledEffective() : (usrConf.fm_override_enabled != 0);
  in.locked           = system_locked;
  in.in_setup         = in_setup;
  const uint32_t now = millis();
  const uint8_t  v   = mtxSummonVerdict(&t, &in, now);
  s_last_summon_v = v;
  const char* what = (kind == MTX_SUMMON_RTM) ? "Return-To-Me" : "Follow-Me";
  if (v != MTX_SV_OK)
  {
    if (v == MTX_SV_NOT_FREE && mtxTxRefusalSignal(&t, now) == MTX_RS_E9)
    {
      Serial.printf("MTX [TX] summon (%s) refused: another remote is in control and busy - E9\n", what);
      mtxTxShowE9();
      return;
    }
    Serial.printf("MTX [TX] summon (%s) refused: %s\n", what, mtxSummonVerdictText(v));
    mtxTxStopRefusal(NULL, (v == MTX_SV_TRIGGER) ? MTX_ST_HELD : MTX_ST_BLOCK);
    return;
  }
  queueMetaPacketBurst(kMtxQueueSummon, 0);
  s_summon_kind    = kind;
  s_summon_sent_ms = mtxStampOf(now);
  s_n_summon++;
  Serial.printf("MTX [TX] summon (%s) sent - waiting up to %lu ms for the buggy to name this remote the owner\n",
                what, (unsigned long)kMtxSummonWaitMs);
}

// mtxTxSummonGo - the summon's DEFERRED ARM, once per loop pass (loop task only, never from a gesture handler: the arm
//   can be the blocking RTM ceremony). Once the wait below has seen OWNER, the gesture's own action runs exactly as today -
//   setRtmArmed() (the "rn" ceremony and the squeezes) or cycleFmMode() (a fresh arm) - once the owner-edge boot ID has
//   gone out (at most kMtxSummonBootIdWaitMs): the ceremony's own 0xF1/0 would otherwise replace it in the queue (same
//   type), and the ceremony then blocks loop() for seconds. A lock in between: nothing is armed.
static void mtxTxSummonGo(uint32_t now, const MtxTxStatus* t)
{
  (void)t;
  if (s_summon_go == 0) return;
  if (txBootIdPending() && mtxAgeMs(now, s_summon_go_ms) < kMtxSummonBootIdWaitMs) return;
  const uint8_t kind = s_summon_go;
  s_summon_go = 0;
  if (system_locked) { Serial.println("MTX [TX] summon: the remote locked before the arm - nothing armed"); return; }
  if (kind == MTX_SUMMON_RTM) setRtmArmed();   // today's ceremony (blocks loop(), as from the toggle)
  else                        cycleFmMode();   // today's fresh arm (Follow-Me is not armed here)
}

// mtxTxSummonWait - the summon's polled wait (X-9). OWNER within 1500 ms of the send: the boot ID is queued AT ONCE (X-8;
//   the telemetry task already queued it on the edge, this is the belt) and the arm is handed to mtxTxSummonGo(). NO
//   ANSWER in 1500 ms: E9 if the status now says busy, else "St" + the stop buzz (held, not blocking). THE LATE GRANT
//   (documented, X-9): if the buggy granted but its FIRST owner status arrived after the 1500 ms, the remote has shown its
//   refusal and armed nothing, yet it IS in control - the ownership dot goes SOLID and the rider simply makes the gesture
//   again, now on the owner path (today's arm); the old owner's Follow-Me had already ended at the claim (accepted with
//   F33). A lock cancels the wait.
//   V2.5-Evo - 2026-10-21 - P3-1: the verdict is the arrival time of the first owner status (mtxTxOwnerWait()), so a poll
//   made late gives the same answer; and this runs from mtxTxAnswerPoll() inside handleGearToggle()'s loops as well,
//   because a toggle summon is answered while the rider still holds the button and loop() is blocked.
static void mtxTxSummonWait(uint32_t now, const MtxTxStatus* t)
{
  if (s_summon_kind == 0) return;
  if (system_locked)
  {
    s_summon_kind = 0;
    Serial.println("MTX [TX] summon cancelled: the remote locked");
    return;
  }
  const uint8_t w = mtxTxOwnerWait(t, s_summon_sent_ms, now, kMtxSummonWaitMs);
  if (w == MTX_W_WAIT) return;
  const uint8_t kind = s_summon_kind;
  s_summon_kind = 0;
  if (w == MTX_W_OWNER)
  {
    Serial.printf("MTX [TX] summon accepted: this remote is in control - %s\n",
                  (kind == MTX_SUMMON_RTM) ? "the Return-To-Me arm ceremony starts" : "Follow-Me arms");
    txBootIdOwnerEdge();
    s_summon_go    = kind;
    s_summon_go_ms = mtxStampOf(now);
    return;
  }
  if (mtxTxRefusalSignal(t, now) == MTX_RS_E9)
  {
    Serial.println("MTX [TX] summon refused by the buggy: another remote is in control and busy - E9");
    mtxTxShowE9();
  }
  else
  {
    Serial.printf("MTX [TX] summon: no owner answer within %lu ms - St (if the buggy granted it late, this remote is in control with nothing armed: make the gesture again)\n",
                  (unsigned long)kMtxSummonWaitMs);
    mtxTxStopRefusal(NULL, MTX_ST_HELD);
  }
}

// mtxTxSeizeWait - the take-back's answer (C20): an owner status that FIRST arrived within 1 s of the SEIZE burst ->
//   the arm-confirm buzz (Pattern 4: in control again); none -> "St" (held, not blocking).
//   V2.5-Evo - 2026-10-21 - P3-1: judged on that first arrival (mtxTxOwnerWait()), and polled inside handleGearToggle()'s
//   hold and release loops too (mtxTxAnswerPoll()): the SEIZE fires at mtx_seize_hold_ms while RIGHT is STILL held, so
//   the confirm buzz now comes while the rider holds the button instead of a false "St" when he lets go after 2 s.
static void mtxTxSeizeWait(uint32_t now, const MtxTxStatus* t)
{
  if (!s_seize_wait) return;
  const uint8_t w = mtxTxOwnerWait(t, s_seize_sent_ms, now, kMtxSeizeConfirmMs);
  if (w == MTX_W_OWNER)
  {
    s_seize_wait = false;
    if (current_vib_pattern == 0) current_vib_pattern = 4;   // the arm-confirm buzz: in control again
    Serial.println("MTX [TX] take-back: this remote is in control");
  }
  else if (w == MTX_W_TIMEOUT)
  {
    s_seize_wait = false;
    Serial.println("MTX [TX] take-back: the buggy did not hand control back within 1 s - St");
    mtxTxStopRefusal(NULL, MTX_ST_HELD);
  }
}

// mtxTxAnswerPoll - V2.5-Evo - 2026-10-21 - P3-1: the summon and take-back ANSWERS only (no arm, no edge handling), for
//   handleGearToggle()'s hold and release loops (Hall.ino), which block loop() - and so mtxTxLoopTick() - for as long as
//   the button is held. Every 10 ms in the hold loop and every 100 ms in the release wait, against the first owner
//   reply's arrival (about 0.1-0.4 s after the burst: the burst leaves on the next 100 ms cycles, the buggy answers the
//   frame within 40 ms). Both waits are idempotent, so mtxTxLoopTick() running them again is harmless. The deferred
//   summon arm is NOT run here (it can be the blocking ceremony); it starts at the next loop pass, as before.
//   Side effects: the wait states, the confirm buzz, the screen ("St" / E9 held without blocking), Serial.
void mtxTxAnswerPoll()
{
  if (!txIsV3()) return;
  if (s_summon_kind == 0 && !s_seize_wait) return;
  MtxTxStatus t;
  mtxTxStatusGet(&t);
  const uint32_t now = millis();
  mtxTxSummonWait(now, &t);
  mtxTxSeizeWait(now, &t);
}

// mtxTxSeizePoll - THE MASTER'S TAKE-BACK (design 9.7, audit C20; Common/MultiTx.h mtxSeizeGestureStep(), host test T16):
//   trigger released, a RIGHT tap, RIGHT pressed again within 400 ms and held mtx_seize_hold_ms (1000-1800). It starts
//   only at a RIGHT press while the latest status is SEIZABLE (this remote is the master and a guest is in control), makes
//   the remote fast at once, aborts on throttle >= 5, a release before the time, a lock or another status, and at the
//   hold time re-reads the verdict (the last two statuses SEIZABLE, the newer at most 5 s old): then the SEIZE burst
//   (internal type kMtxQueueSeize -> 0xF1/0x5A). The second press is SUPPRESSED until released (no display cycle).
//   Answer: an owner status within 1 s -> the arm-confirm buzz; none -> "St" (mtxTxLoopTick()).
//   Polled every 10 ms while a toggle is handled and on every runMenu() / loop pass. Side effects: the gesture state,
//   the meta queue, mtx_gesture_armed, Serial.
void mtxTxSeizePoll()
{
  if (!txIsV3()) { s_seize_out = 0; return; }
  MtxTxStatus t;
  mtxTxStatusGet(&t);
  const uint32_t now = millis();
  const uint8_t  out = mtxSeizeGestureStep(&s_seize, now, ctplus(), thr_scaled, system_locked != 0, &t, usrConf.mtx_seize_hold_ms);
  s_seize_out = out;
  if (out & MTX_SGO_FIRE)
  {
    queueMetaPacketBurst(kMtxQueueSeize, 0);
    s_seize_wait    = true;
    s_seize_sent_ms = mtxStampOf(now);
    s_n_seize++;
    Serial.println("MTX [TX] take-back: SEIZE sent (the guest's throttle goes to zero) - waiting for the buggy");
  }
  if (out & MTX_SGO_ABORT) Serial.println("MTX [TX] take-back gesture ended without a take-back");
  mtx_gesture_armed = ((out & MTX_SGO_ARMED) != 0) || s_seize_wait || s_summon_kind != 0 || s_summon_go != 0;
}

// mtxTxSeizeSuppress - is the current RIGHT press the take-back's held second press (its 2 s display cycle must not run)?
bool mtxTxSeizeSuppress()
{
  return (s_seize_out & MTX_SGO_SUPPRESS) != 0;
}

// mtxTxLoopTick - once per loop() pass, after runMenu(): the rate's input stamp, the edges the telemetry task raised
//   (control lost -> this remote's own Follow-Me / RTM end here, no packet; 2 SEIZED -> "tA" 3 s + one Pattern A buzz),
//   the E9 buzz on its rising edge, the summon and take-back answers, the ownership dot, and the activity repeat.
//   Side effects: as listed. Loop task only.
void mtxTxLoopTick()
{
  const uint32_t now = millis();
  if (thr_scaled >= kMtxZeroThr || tog_input != 0 || steer_scaled != 127) mtxTxNoteInput();

  if (!txIsV3())
  {
    mtx_dot_state     = (mtx_dot_override != 0xFF) ? mtx_dot_override : MTX_DOT_OFF;   // ?mtxdot works on any link
    mtx_gesture_armed = false;
    return;
  }

  MtxTxStatus t;
  mtxTxStatusGet(&t);
  uint8_t ev;
  portENTER_CRITICAL(&g_mtx_tx_mux);
  ev = mtx_tx_events;
  mtx_tx_events = 0;
  portEXIT_CRITICAL(&g_mtx_tx_mux);

  if (ev & MTX_TXE_SEIZED_NOTICE)
  {
    // Design 9.9: control was taken from this remote (a claim, or the master's take-back). "tA" for 3 s, one Pattern A buzz.
    const unsigned long u = now + 3000UL;
    mtx_ta_until_ms = (u != 0) ? u : 1;
    if (current_vib_pattern == 0) current_vib_pattern = 1;
    Serial.println("MTX [TX] control was taken by another remote: tA");
  }
  if (ev & MTX_TXE_LOST_OWNER)
  {
    // Design 9.5: the buggy has already ended this remote's Follow-Me / RTM; end them here too (no packet). Without a
    // "tA", and with something armed, the honest screen is "St" (it stopped - for example a buggy reboot).
    const bool was_armed = mtxLostOwnerDisarm();
    Serial.println("MTX [TX] this remote is no longer in control of the buggy");
    if (was_armed && !(ev & MTX_TXE_SEIZED_NOTICE)) mtxTxStopRefusal(NULL, MTX_ST_HELD);
  }

  // E9 (design 9.8): one Pattern A buzz on the rising edge of the window.
  const unsigned long e9 = mtx_e9_until_ms;
  const bool e9_on = (e9 != 0) && (long)(now - e9) < 0;
  if (e9_on && !s_e9_was_on && current_vib_pattern == 0) current_vib_pattern = 1;
  s_e9_was_on = e9_on;

  // The summon: the deferred arm first (granted on an earlier pass), then the wait; then the take-back's answer. The same
  // order as before the P3-1 split (the arm, when it runs, leaves no summon waiting).
  mtxTxSummonGo(now, &t);
  mtxTxSummonWait(now, &t);
  mtxTxSeizeWait(now, &t);

  mtx_gesture_armed = ((s_seize_out & MTX_SGO_ARMED) != 0) || s_seize_wait || s_summon_kind != 0 || s_summon_go != 0;
  mtx_dot_state     = (mtx_dot_override != 0xFF) ? mtx_dot_override : mtxTxDotState(&t, now, true);
  mtxActivityTick();
}

// mtxTxPrintStatus - ?mtx on the remote (design 14.4): this chip's ID, the link, the last two statuses, the rate, the
//   effective auto-lock, the take-back and summon state, the screen notices and the counts. One-shot. Loop task.
void mtxTxPrintStatus()
{
  static const char* const kS[8] = { "OWNER (one remote)", "OWNER", "FREE", "BUSY", "SEIZABLE", "REFUSED (E9)", "SEIZED (tA)", "-" };
  MtxTxStatus t;
  mtxTxStatusGet(&t);
  const uint32_t now = millis();
  Serial.println("=== Multi-remote (?mtx, remote) ===");
  mtxTxBootReport();
  if (!txIsV3())
  {
    Serial.println("old-style link (or not paired): today's frames and behaviour - no multi-remote status, rate, take-back or summon");
  }
  else if (t.count == 0)
  {
    Serial.println("status: no reply from the buggy yet");
  }
  else
  {
    Serial.printf("status: %s %lu ms ago", kS[t.last[0] & 7], (unsigned long)mtxAgeMs(now, t.last_ms[0]));
    if (t.count > 1) Serial.printf(", before it %s %lu ms ago", kS[t.last[1] & 7], (unsigned long)mtxAgeMs(now, t.last_ms[1]));
    Serial.printf(" | in control: %s | lost control: %s | rate: %s\n", t.owner ? "yes" : "no", t.lost_owner ? "yes" : "no",
                  mtx_tx_slow ? "SLOW (1 send in 10)" : "fast");
  }
  Serial.printf("auto-lock: %u s now (autolock_timeout_s %u, spare mtx_autolock_s %u)%s\n",
                (unsigned)mtxTxAutolockTimeoutS(&t, usrConf.mtx_autolock_s, usrConf.autolock_timeout_s),
                (unsigned)usrConf.autolock_timeout_s, (unsigned)usrConf.mtx_autolock_s, usrConf.no_lock ? " - OFF: no_lock is set" : "");
  Serial.printf("take-back: phase %u, hold %u ms, verdict now %s%s | summon: %s, last verdict: %s\n",
                (unsigned)s_seize.phase, (unsigned)usrConf.mtx_seize_hold_ms, mtxTxSeizeVerdict(&t, now) ? "yes" : "no",
                s_seize_wait ? " (waiting for the answer)" : "",
                (s_summon_kind || s_summon_go) ? "waiting" : "none",
                (s_last_summon_v == 0xFF) ? "-" : mtxSummonVerdictText(s_last_summon_v));
  const unsigned long e9 = mtx_e9_until_ms, ta = mtx_ta_until_ms;
  Serial.printf("screen: E9 %s, tA %s, dot %u (%s)\n", (e9 && (long)(now - e9) < 0) ? "on" : "off",
                (ta && (long)(now - ta) < 0) ? "on" : "off", (unsigned)mtx_dot_state,
                (mtx_dot_override != 0xFF) ? "forced by ?mtxdot" : "from the status");
  Serial.printf("sent: activity repeats %lu, summons %lu, take-backs %lu | packets sent %lu, replies %lu\n",
                (unsigned long)s_n_activity, (unsigned long)s_n_summon, (unsigned long)s_n_seize,
                (unsigned long)num_sent_packets, (unsigned long)num_rcv_packets);
}

// mtxTxDotCommand - ?mtxdot [0-3|auto]: force the ownership dot for the one-call-site hardware check (project display rule):
//   0 off, 1 solid (in control), 2 slow (another remote in control), 3 pulse (control available); auto = from the status.
//   Bare: print the state. It shows only while no Follow-Me dot uses C7 R2. Side effects: mtx_dot_override, Serial.
void mtxTxDotCommand(const String& args)
{
  if (args.length() == 0)
  {
    Serial.printf("mtxdot: %u (%s)\n", (unsigned)mtx_dot_state, (mtx_dot_override != 0xFF) ? "forced" : "auto");
    return;
  }
  if (args == "auto") { mtx_dot_override = 0xFF; Serial.println("mtxdot: auto (from the buggy's status)"); return; }
  if (args.length() == 1 && args[0] >= '0' && args[0] <= '3')
  {
    mtx_dot_override = (uint8_t)(args[0] - '0');
    mtx_dot_state    = mtx_dot_override;
    Serial.printf("mtxdot: forced %u (0 off, 1 solid, 2 slow blip, 3 pulse) - it shows at C7 R2 while no Follow-Me dot is lit; ?mtxdot auto to end\n",
                  (unsigned)mtx_dot_override);
    return;
  }
  Serial.println("ERR: usage: ?mtxdot [0-3|auto]");
}
