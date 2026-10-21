// V2.5-Evo - 2026-10-21 - STEP-3 AUDIT P3-4: a paired-list change from MULTI to SINGLE also publishes the takeover limit once
//   (mtxListChangeActions(), Common/MultiTx.h), so the remaining remote starts from zero under the soft start. No confStruct change.
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE, step-2 audit Z-1 / Z-6 / Z-7 a: (Z-1) a paired-list change zeroes the outputs
//   (throttle 0, steering 127, no failsafe stamp) whenever the list was or becomes MULTI, also with nobody in control -
//   a list change with one remote before and after stays today's (the owner's single-remote rule; Z-1 confirmed 2026-10-21);
//   (Z-6) the pairing power guard checks that -9 dBm was really set, and the window does not open if the radio
//   refused it; (Z-7 a) ?mtx counts the list-change releases. No control-path statement changed otherwise.
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE RECEIVER STEP (SW37): triggeredReceive() hands every 6-byte frame that passes
//   gate 0 and the CRC to mtxOnFrame() (Common/MultiTx.h) behind this file's hook table. With ONE remote paired
//   (SINGLE) the hooks ARE today's code: apply_owner is the old control branch, meta the old 0xF1 / 0xF2 / 0xF4 / 0xF3
//   branches, reply the old reply (status 000, today's address and rotation) - moved, not rewritten; the only
//   SINGLE differences are the approved ones (0xF5-0xFF and the 0xF1 SEIZE / SUMMON codes dropped, LOCKED from a
//   new-style remote read as 0/127). With 2-4 remotes (MULTI) the ownership rules decide who is obeyed. Gate 0 is
//   mtxMayAcceptControl(); the boot pairing window is rebuilt for up to four remotes (RSSI gate, -9 dBm power guard,
//   rebuild-never-append, commit or restore); the 15-byte rider GPS is accepted only from the slot that announced it
//   while it owns; the model's timers also run on the 2 s timeout; the "MTX:" lines are queued for the loop task.
//   The bodies of processRtmStatePacket(), processFmOverridePacket() and gpsPhaseBCheck() are untouched;
//   processMetaGpsPacket() changes in its address test only. GPS-integrity note (project rules 1-5): Phase A still
//   judges every accepted fix, the 0xF3 state machine still resets on every mismatch (now also a slot that does not
//   own) and a release restores 6-byte mode, positions stay int32 microdegrees, nothing is extrapolated.
// V2.5-Evo - 2026-10-12 - audit 2026-10-11 delta A-1: the 0xF3 GPS path accepts the v2 rider GPS packet (announcement
//   0xF3/0x03 -> a 15-byte packet with the remote's GPS fix counter) next to v1 (0xF3/0x01 or any other subtype -> 14
//   bytes, unchanged). processMetaGpsPacket() decodes both through Common/RiderGpsPacket.h and stamps
//   rx_tx_gps_solution_ms / _seq on every accepted NEW GPS solution (v2: the counter moved; v1: the position moved).
//   rx_tx_gps_fix_seq still counts distinct positions only. GPS-integrity note (project rule 5) at the function.
//   No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-08 - STICKY RETURN CAP (owner design): processRtmStatePacket() arms the sticky return cap on 0xF1/0
//   while RTM is running, before rtm_rx_active goes false. No packet format change, no confStruct change.
// V2.5-Evo - 2026-10-07 - P-3: gps_phase_b_dist_fail records that the last Phase B failure was the distance check. No
//   packet format change, no confStruct change.
// V2.5-Evo - 2026-10-07 - N-10: processRtmStatePacket() bumps rx_tx_boot_id_rx_seq on every boot-ID packet, so a parked auto-return can wait after a link gap until the remote has identified itself again. No confStruct change, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-07 - N-8(a): processFmOverridePacket() enforces the post-fault re-declaration block (M-1) only once a boot ID has been heard from this remote; a remote without the boot ID keeps the R-6 keepalive re-arm, so a link-loss fault it never saw cannot leave Follow-Me silently unavailable behind an ARMED screen. GPS-integrity note (project rule 5): the 0xF3 GPS state machine, Phase A/B and the freshness gates are untouched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-10-07 - S-8 + H-1 + M-1 (wire formats in Common/AutoReturnRules.h): processRtmStatePacket() accepts 0xF1 value 0x02 (RTM refresh: refreshes only, never activates; 0xF1/1 clears rtm_refresh_seen) and 0x80-0xFF (the remote boot ID; a change bumps rx_tx_boot_change_seq and clears the re-declaration block). processFmOverridePacket(): after a Follow-Me fault a mode 1-5 is ignored unless 0xF2 bit 7 (fresh declaration) is set; 0xF2/0 clears the block. Every older RX ignores the new values. GPS-integrity note (project rule 5): the 0xF3 GPS state machine, Phase A/B and the freshness gates are untouched. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-29 - REVERTED the RX-only standbyXOSC line that the 2026-09-28 commit (1803869) added to startupRadio(). It does not work on this hardware: with radio.standbyXOSC = true the SX1262 never initialises and boot prints "Starting Radio... Power: 22 Region: US/AU915 TOA: 0 Failed, code: -707", i.e. RADIOLIB_ERR_SPI_CMD_FAILED (RadioLib 7.1.2, src/TypeDef.h line 430) - the chip refused a command sent to it over SPI, so there is no radio, no link and no telemetry at all. Proven by a three-flash A/B on one bench RX in one sitting with nothing else changed: parent d9873cc gave "TOA: 7488 Done", 1803869 gave "TOA: 0 Failed, code: -707", and d9873cc rebuilt and reflashed gave "TOA: 7488 Done" again, with the failure repeating over two further power-on resets. LIKELY BUT UNPROVEN REASON: this module runs a TCXO powered from the chip's DIO3 pin (initRadioHardware() passes 1.8 V for it in ../Common/RadioCommon.h), and standing by on an oscillator that has not powered up and settled leaves the chip busy and rejecting the next command; a board wired for a plain crystal instead would not hit this, which is presumably why upstream can use it. The full write-up and a DO-NOT-RE-ADD note now sit in startupRadio() where the line used to be. The R-3 motor-gate fix from that same commit is KEPT and its logic is unchanged. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-29 - R-3 HARDENING (review finding L-1, LOW, explicitly not a regression): the last_control_packet = millis() stamp is MOVED so it happens immediately AFTER thr_received and steering_received are written, instead of nine lines before them. Same branch, same value, no logic change - but "the motor gate is open" now structurally implies "thr_received came from that same packet", rather than only happening to be true. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-28 - R-3 FIX, part 2 of 3 (see PWM.ino and BREmote_V2_Rx.h): the normal control-packet branch now also stamps last_control_packet, next to the existing last_packet. The three meta-packet branches (0xF1 RTM state, 0xF2 FM mode, 0xF4 aux) deliberately do NOT, and that is the whole fix - they refresh "the remote is alive" without ever refreshing "I have a fresh throttle command", because none of them carries a throttle byte. No confStruct change, sizeof stays 200, SW_VERSION stays 36.
// V2.5-Evo - 2026-09-19 - processFmOverridePacket() decodes the FULL 0xF2 mode byte: bits 0-2 = mode (a value above 5 fails closed to 0 = disarm, printed once), bits 5-6 = the remote's auto-return override (0 = none -> fm_return_mode_runtime 0xFF, 1 = OFF -> 0, 2 = ON -> 1, 3 = ignored). It used to mask with 0x03, which would have folded the override bits into the mode. fm_mode_last_rx_ms is stamped as before. GPS-integrity note (project rule 5): this touches only the 0xF2 control meta-packet; the 0xF3 GPS state machine, Phase A/B and the freshness gates are untouched, so rules 1-4 hold exactly as before.
// V2.5-Evo - 2026-07-24 - F9: cache last control-packet RSSI/SNR (g_last_rssi_dbm/g_last_snr_db) at receive so the logger task can add distance+link-quality CSV columns without racing the radio SPI bus. No confStruct/SW_VERSION change.
// V2.5-Evo - 2026-07-20 - FM engagement semantics: processFmOverridePacket() stamps fm_mode_last_rx_ms on every 0xF2 so RTMState.ino can expire an unrefreshed FM declaration (95 s). This handler is now the ONLY path that can arm FM — the RX no longer auto-arms from usrConf.followme_mode.
// V2.5-Evo - 2026-05-12 - Fix Phase B recovery: recheck gate reduces from 30s to 2s when gps_phase_b_ok=false, eliminating up to 30s RTM motor block after any TX GPS gap
// V2.5-Evo - 2026-05-03 - Removed if(0) dead code; checkAndAdjustAddress() TODO noted
// V2.5-Evo - 2026-04-29 - Bundle C: startTransmit() return value checked and logged on error
// V2.5-Evo - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine
// V2.5-Evo - 2026-04-24 - Added Phase B GPS handshake check: gpsPhaseBCheck() called from processMetaGpsPacket()
// V2.5-Evo - 2026-04-25 - P7: Added processRtmStatePacket(), processFmOverridePacket(); dispatch 0xF1/0xF2 in triggeredReceive()

// V2.5-Evo - 2026-07-24 - F9: cache last control-packet link quality for the logger CSV.
// radio.getRSSI()/getSNR() read SX1262 registers over SPI and are only valid right after a reception
// in triggeredReceive(). The logger runs in a separate FreeRTOS task, so it must NOT touch the radio
// SPI bus concurrently — it reads these cached copies instead (see extern in Logger.ino convertToLogData()).
float g_last_rssi_dbm = 0.0f;   // dBm; snapshot of the last control packet's RSSI
float g_last_snr_db   = 0.0f;   // dB;  snapshot of the last control packet's SNR

// ============================================================
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE: THE RADIO TASK'S OWNERSHIP STATE
// ============================================================
// Everything the per-packet rules (mtxOnFrame(), Common/MultiTx.h) need between frames. ONE WRITER: the radio task
// (triggeredReceive) - plus setup(), once, before that task exists (mtxBootReport()). Other tasks never touch it:
// they read the atomics in BREmote_V2_Rx.h, and ?mtx copies it with the scheduler suspended (mtxPrintStatus()).
struct MtxRxFw {
  MtxRxState   st;                  // the model's state (hook contract Y-1 e: no hook writes it)
  MtxRxBinding b;                   // this wake's copy of the binding from usrConf (paired, own, dest, the list)
  bool         b_valid;             // b has been filled at least once
  MtxRxCfg     cfg;                 // this wake's config for the model, clamped again here (Y-1 h)
  uint8_t      mode;                // MTX_MODE_UNPAIRED / SINGLE / MULTI of b
  uint8_t      tel_idx[kMtxSlots];  // MULTI: each remote's own telemetry rotation 0-19 (SINGLE keeps today's telemetry_index)
  uint8_t      linkq[kMtxSlots];    // MULTI: 0-10 link quality of each remote's own last frame (reply index 18)
  int16_t      rssi[kMtxSlots];     // MULTI: that frame's RSSI in dBm (?mtx)
  int8_t       gps_meta_slot;       // the slot whose 0xF3 announce switched the radio to 15 bytes; kMtxNone = none
  bool         skip_common_exit;    // set by the meta hook on the owner's 0xF3 announce: the radio already waits for 15 B
  uint32_t     rtm_off_quiet;       // MULTI: owner 0xF1/0 with RTM off, not handed to the RTM handler (see the meta hook)
  uint32_t     unknown;             // frames that passed the CRC but name no paired remote (another buggy's, a reply)
  uint32_t     lines_lost;          // "MTX:" lines dropped because the loop task had not printed the last 16 yet
};
static MtxRxFw g_mtx;               // zero-initialised; mtxRxStartFresh() sets the "nobody" values on the first wake
static bool    g_mtx_hooks_ok = false;   // mtxRxHooksComplete() at boot (Y-1); false = every frame is dropped

void radioErrorHalt(int type)
{
  if(type == 1) while(1) blinkErr(2, AP_L_BIND);
  while(1) blinkErr(3, AP_L_BIND);
}

void radioInitSuccess()
{
  // No extra init needed on RX side
}

void startupRadio()
{
  // ============================================================
  // V2.5-Evo - 2026-09-29 - DO NOT RE-ADD radio.standbyXOSC = true HERE
  // ============================================================
  // WHAT WAS TRIED (2026-09-28, commit 1803869). A single line sat here, immediately above the
  // initRadioHardware() call below: "radio.standbyXOSC = true;". The idea, taken from upstream's
  // 2026-07-04 "RF optimization (RadioLib bug #1827)" change, was to make the SX1262 idle on its
  // crystal between packets instead of dropping to the chip's own low-power internal oscillator, so
  // it would not have to spend about 5 ms waking that crystal back up before every receive and
  // every transmit.
  //
  // WHAT IT ACTUALLY DID ON THIS HARDWARE. The radio never started at all. Boot printed:
  //     Starting Radio... Power: 22 Region: US/AU915 TOA: 0 Failed, code: -707
  // -707 is RADIOLIB_ERR_SPI_CMD_FAILED (RadioLib 7.1.2, src/TypeDef.h line 430): the chip refused
  // a command we sent it over SPI. The "TOA: 0" says the same thing another way - with the radio
  // failing to come up there are no settings left to compute a transmit airtime from. That is no
  // link, no telemetry and no remote control whatsoever, so this is not a subtle regression.
  //
  // HOW WE KNOW IT WAS THIS LINE. Three flashes onto the same bench RX in one sitting, with nothing
  // changed but the firmware: parent commit d9873cc -> "TOA: 7488 Done"; commit 1803869 ->
  // "TOA: 0 Failed, code: -707"; d9873cc rebuilt and reflashed -> "TOA: 7488 Done" again. The
  // failure then repeated across two further power-on resets, so it is not a one-off boot glitch.
  // The only other radio-related line in that commit is a timestamp store, which cannot fail an
  // SPI command.
  //
  // WHY IT PROBABLY FAILS HERE BUT WORKS UPSTREAM - A HYPOTHESIS, NOT A PROVEN CAUSE.
  // initRadioHardware() (in ../Common/RadioCommon.h) calls begin(..., 1.8, false), and that 1.8 is
  // a supply voltage the SX1262 puts out on its DIO3 pin to power a TCXO - a temperature-compensated
  // oscillator module, which needs a moment to power up and settle before its clock is usable.
  // Telling the chip to idle on that oscillator requires it to be running and settled; told to do
  // so before it is ready, the chip stays busy and the next command we send is rejected, which is
  // exactly the -707 we saw. A board wired for a plain crystal rather than a TCXO would never hit
  // this, which would explain why upstream can use the setting.
  //
  // IF ANYONE WANTS TO TRY AGAIN: it has to be re-derived from scratch against this module's TCXO
  // start-up timing and proven on hardware before it goes back in. Note also that there is no
  // measured benefit to chase - the speed-up described above is upstream's claim and was never
  // measured on our board - so removing the line costs us nothing we actually had.
  initRadioHardware();
}

void ICACHE_RAM_ATTR packetReceived(void)
{
  if(rxIsrState && !rfInterrupt)
  {
    rfInterrupt = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(triggerReceiveSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    rfInterrupt = true;
  }
}

// ============================================================
// V2.5-Evo - 2026-08-05 - SAFETY: may this RX obey a control packet at all?
// ============================================================
// THE BUG THIS CLOSES: the only gate on an inbound control packet was the
// 3-byte destination-address compare in triggeredReceive(). usrConf.paired was
// never consulted anywhere in the control path.
//
// Why that mattered: unpairing ZEROES own_address (Common/WebConfigEngine.h), so
// an unpaired RX listens on 00:00:00 — the one address every unbound unit in the
// world shares. An unpaired TX transmits to 00:00:00 for the same reason. So two
// unpaired units in radio range would match, pass CRC, and drive the motor,
// with nothing in between. No pairing, no consent, no gesture.
//
// TWO conditions, because either one alone still leaves the hole open:
//   !paired             - never bound, or deliberately unpaired
//   own_address all 00  - the factory/unpaired address. Even if `paired` were
//                         somehow true (corrupt config, a half-finished bind),
//                         00:00:00 must never be commandable.
//
// Deliberately placed so it gates the WHOLE control path, not just throttle:
// every writer of thr_received AND every writer of last_packet (the failsafe
// feed) lives inside that address block, including the 0xF1/0xF2/0xF4 meta
// packets. Gating only the throttle would still have let a stranger hold the
// failsafe open, which is the more dangerous half.
//
// Costs nothing on the wire, changes nothing for a paired board.
//
// NOTE ON PAIRING: waitForPairing() sets usrConf.paired = false while it runs.
// V2.5-Evo - 2026-10-20 - CORRECTED (design V-21): the receive task is NOT live during
// the boot pairing window. checkButtons() runs it from runBootSequence(), which setup()
// calls BEFORE initTasks() (V2_Integration_Rx.ino), so no task exists yet and nothing
// can be driven - which is what the old sentence wanted, for a different reason. (It
// said the receive task was live because checkButtons() first ran from loop(); its
// first call is in fact the boot one, and the runtime calls never open the window.)
//
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE (SW37): GATE 0 IS mtxMayAcceptControl()
// (Common/MultiTx.h, host test T20): the two conditions above, AND at least one
// paired remote - a valid list entry, or the implicit old-style entry of a migrated
// board (paired, a dest_address, own_address = this chip's ID). The binding it reads
// is this wake's copy (mtxRxRefresh()), taken from usrConf just before this call.
// Every unpair path clears the list (the validator), so they all close this gate.
static bool rxMayAcceptControl()
{
  return mtxMayAcceptControl(&g_mtx.b, g_efuse_id);
}

// ============================================================
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE: THE BOOT PAIRING WINDOW (design 11.1; audit C10, C13, C14, C21)
// ============================================================
// BIND held at boot opens ONE window for up to four remotes. The list is REBUILT from this window, never appended to:
// whoever enrols is the whole list, in enrolment order, and the FIRST one is the MASTER (slot 1) - so the rider's own
// remote must be paired first. A legacy (old-style, 0xAB) remote is accepted only as the single remote and closes
// the window at once (C13). Both kinds are judged by the pure mtxPairOnRequest() / mtxPairOnConfirm() (host test T13):
// a request below -60 dBm (about 1 m at the -9 dBm both ends use here), from 00:00:00 or from this buggy's own ID, a
// fifth remote, or a legacy remote after any enrolment is refused with one line saying why.
// THE WINDOW: 60 s, +20 s per new remote, never past 120 s; it closes early on four remotes, a legacy enrolment, or
// a fresh BIND press. AT THE CLOSE: with at least one enrolment the new list is committed (own_address = this chip's
// ID, paired = 1) and saved ONCE; with none, the binding from before the window is restored and nothing is written.
// WHY BLOCKING IS SAFE: this runs inside setup() via runBootSequence() -> checkButtons(), BEFORE initTasks(), so no
// receive or PWM task exists, PWM_active is 0 and nothing can be driven. The loop yields every pass (delay(10)): the
// core starts the task watchdog before setup() (audit W-8), and the idle task must keep running for up to 120 s.
// THE RADIO POWER: -9 dBm for the whole window, and the stored rf_power back on EVERY exit - the guard's destructor
// runs on whichever path leaves this function (C14). The RX radio is never put to sleep in here, so the restore is a
// plain SPI write. MT-05 reads the "radio power back" line.

// RxPairPowerGuard - RAII: -9 dBm while it lives, usrConf.rf_power (unchanged by pairing) when it dies, with a line.
// V2.5-Evo - 2026-10-20 - Z-6: THE CONSTRUCTOR'S RESULT IS KEPT. It was discarded, so if the radio refused -9 dBm the
// window ran at the stored rf_power (22 dBm by default) and nothing said so - the buggy's own RSSI gate still held
// (it measures the remote), but its OFFER then reached about 31 dB further than the -60 dBm budget assumes.
// waitForPairing() reads `ok` and does not open the window when it is false (one PAIR line; MT-05 reads it).
struct RxPairPowerGuard {
  bool    ok;     // -9 dBm was really set
  int16_t err;    // RadioLib's answer to that request
  RxPairPowerGuard()  { err = radio.setOutputPower((int8_t)kMtxPairPowerDbm); ok = (err == RADIOLIB_ERR_NONE); }
  ~RxPairPowerGuard()
  {
    const int16_t e = radio.setOutputPower((int8_t)usrConf.rf_power);
    Serial.printf("PAIR: radio power back to the stored rf_power, %d dBm%s\n", (int)usrConf.rf_power,
                  (e == RADIOLIB_ERR_NONE) ? "" : " - the radio REFUSED it, reboot before riding");
  }
};

// mtxFmtId - "5C:4A:7B" for a 3-byte ID. Output: out (9 bytes). Side effects: none.
static void mtxFmtId(char out[9], const uint8_t id[3])
{
  snprintf(out, 9, "%02X:%02X:%02X", id[0], id[1], id[2]);
}

// mtxRxBindingFromConf - this board's binding (paired, own_address, dest_address, the list) copied out of usrConf,
//   padding zeroed so two copies compare with memcmp(). Output: *b. Side effects: none.
static void mtxRxBindingFromConf(MtxRxBinding* b)
{
  memset(b, 0, sizeof(*b));
  b->paired = usrConf.paired;
  memcpy(b->own_address,  usrConf.own_address,  3);
  memcpy(b->dest_address, usrConf.dest_address, 3);
  memcpy(b->mtx_id,       usrConf.mtx_id,       sizeof(b->mtx_id));
  memcpy(b->mtx_flags,    usrConf.mtx_flags,    sizeof(b->mtx_flags));
}

// mtxRxBindingToConf - write a binding back into usrConf (the pairing window's commit or restore). Side effects: usrConf.
static void mtxRxBindingToConf(const MtxRxBinding* b)
{
  usrConf.paired = b->paired;
  memcpy(usrConf.own_address,  b->own_address,  3);
  memcpy(usrConf.dest_address, b->dest_address, 3);
  memcpy(usrConf.mtx_id,       b->mtx_id,       sizeof(usrConf.mtx_id));
  memcpy(usrConf.mtx_flags,    b->mtx_flags,    sizeof(usrConf.mtx_flags));
}

// rxPairBindDown / rxPairLed / rxPairFlash - the BIND button and its LED during the window (AW9523, active LOW).
//   Boot only, so the unbounded i2cMutex take cannot meet the PWM task (it does not exist yet). Side effects: I2C.
static bool rxPairBindDown()
{
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  const bool down = !aw.digitalRead(AP_S_BIND);
  xSemaphoreGive(i2cMutex);
  return down;
}
static void rxPairLed(bool on)
{
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  aw.digitalWrite(AP_L_BIND, on ? LOW : HIGH);
  xSemaphoreGive(i2cMutex);
}
static void rxPairFlash(uint8_t n, uint16_t on_ms, uint16_t off_ms)
{
  for (uint8_t i = 0; i < n; i++) { rxPairLed(true); delay(on_ms); rxPairLed(false); delay(off_ms); }
}

// rxPairRefusalText - one plain-English reason per mtxPairOnRequest() refusal. Returns: a constant string.
static const char* rxPairRefusalText(uint8_t v)
{
  switch (v)
  {
    case MTX_PR_ZERO:        return "its ID is 00:00:00";
    case MTX_PR_RX_ID:       return "its ID is this buggy's own ID";
    case MTX_PR_FULL:        return "four remotes are already paired in this window";
    case MTX_PR_RSSI:        return "the signal is too weak";
    case MTX_PR_LEGACY_LATE: return "an old-style remote can only be the one remote, and a remote is already paired in this window";
    case MTX_PR_CLOSED:      return "the window is closed";
    default:                 return "refused";
  }
}

// rxPairExchange - one request: refuse it with a line, or OFFER (0xDA new-style, 0xBA old-style) 100 ms later and wait
//   up to 1.5 s for the matching CONFIRM (0xAE / 0xAC naming this buggy and that remote, CRC ok), then enrol it.
//   Frames (design 5.6): REQ [code][remote ID 3][CRC 0-3]; OFFER [code][remote ID 3][buggy ID 3][CRC 0-6];
//   CONFIRM [code][buggy ID 3][remote ID 3][CRC 0-6] - the old-style frames have the same layout, so an SW27 remote
//   pairs exactly as before. Inputs: w, legacy (0xAB), id (the remote's ID or address), rssi (of the request, dBm).
//   Side effects: *w, the radio (8-byte mode while it waits; the caller goes back to 5), the LED, Serial.
static void rxPairExchange(MtxPairWindow* w, bool legacy, const uint8_t id[3], float rssi)
{
  char ib[9];
  mtxFmtId(ib, id);
  const char* kind = legacy ? "old-style remote" : "remote";
  const uint8_t v = mtxPairOnRequest(w, legacy, id, g_efuse_id, rssi);
  if (v != MTX_PR_OFFER)
  {
    if (v == MTX_PR_RSSI)
      Serial.printf("PAIR: request from %s %s refused - RSSI %.0f dBm is below %d (bring the remote within about 1 m)\n",
                    kind, ib, (double)rssi, (int)kMtxPairMinRssiDbm);
    else
      Serial.printf("PAIR: request from %s %s refused - %s\n", kind, ib, rxPairRefusalText(v));
    return;
  }
  uint8_t offer[8];
  offer[0] = legacy ? 0xBA : 0xDA;
  memcpy(offer + 1, id, 3);
  memcpy(offer + 4, g_efuse_id, 3);
  offer[7] = esp_crc8(offer, 7);
  delay(100);
  radio.implicitHeader(8);
  {
    int16_t _txErr = radio.startTransmit(offer, 8);
    if (_txErr != RADIOLIB_ERR_NONE)
      Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
  }
  delay(10);
  radio.startReceive();
  rfInterrupt = false;
  const uint8_t confirm_code = legacy ? 0xAC : 0xAE;
  const unsigned long t0 = millis();
  bool confirmed = false;
  while (!confirmed && (millis() - t0) < kMtxConfirmWaitMs)
  {
    delay(10);   // C14: yield on every pass here too
    if (!rfInterrupt) continue;
    uint8_t cf[8];
    const int16_t rd = radio.readData(cf, 8);
    rfInterrupt = false;
    confirmed = (rd == RADIOLIB_ERR_NONE && cf[0] == confirm_code && memcmp(cf + 1, g_efuse_id, 3) == 0
                 && memcmp(cf + 4, id, 3) == 0 && cf[7] == esp_crc8(cf, 7));
    if (!confirmed) radio.startReceive();
  }
  if (!confirmed)
  {
    Serial.printf("PAIR: offered to %s %s, but no confirmation came within %lu ms - try again\n", kind, ib,
                  (unsigned long)kMtxConfirmWaitMs);
    return;
  }
  const int8_t slot = mtxPairOnConfirm(w, legacy, id);
  if (slot < 0)
  {
    Serial.printf("PAIR: %s %s confirmed but not enrolled - the window is closed or already holds four\n", kind, ib);
    return;
  }
  Serial.printf("PAIR: %s %s paired as slot %d (%s), request RSSI %.0f dBm%s\n", kind, ib, (int)slot + 1,
                (slot == 0) ? "MASTER" : "guest", (double)rssi,
                legacy ? " - an old-style remote is the only remote, the window closes" : "");
  rxPairLed(false);
  delay(400);
  rxPairFlash((uint8_t)(slot + 1), 250, 250);   // N flashes = slot N (1 = master)
}

// waitForPairing - the boot pairing window (see the block comment above). Called from checkButtons() at boot only.
//   Returns: true if at least one remote was paired (the new list is saved), false if the old binding was kept.
//   Side effects: usrConf's binding, one SPIFFS save on success, the radio (power, mode), the BIND LED, Serial.
bool waitForPairing()
{
  // V2.5-Evo - 2026-10-20 - Z-6: the power guard FIRST, so a radio that refuses -9 dBm leaves everything as it was:
  // no window, the binding untouched (usrConf.paired is not cleared yet), 3 fast blinks, 6-byte receive mode. The
  // guard's destructor still runs on this return and puts the stored rf_power back with its own line.
  RxPairPowerGuard power_guard;
  if (!power_guard.ok)
  {
    Serial.printf("PAIR: the radio REFUSED the pairing power of %d dBm (error %d) - the window is NOT opened, the previous pairing is kept. Reboot and try again; if it repeats, the radio needs checking\n",
                  (int)kMtxPairPowerDbm, (int)power_guard.err);
    rxPairFlash(3, 50, 50);
    radio.implicitHeader(6);
    return false;
  }
  MtxRxBinding cur;
  mtxRxBindingFromConf(&cur);
  MtxPairWindow w;
  mtxPairOpen(&w, &cur, millis());
  usrConf.paired = false;   // nothing is drivable while the window is open (no task exists yet either)
  char me[9];
  mtxFmtId(me, g_efuse_id);
  Serial.printf("PAIR: window open - this buggy's ID %s. Up to %u remotes; %lu s, +%lu s per new remote, %lu s at most. Radio at %d dBm; a request must arrive at %d dBm or stronger (about 1 m). Pair the remote you RIDE WITH FIRST: it becomes the master. Press BIND again to close the window.\n",
                me, (unsigned)kMtxSlots, (unsigned long)(kMtxPairWindowMs / 1000), (unsigned long)(kMtxPairExtendMs / 1000),
                (unsigned long)(kMtxPairMaxMs / 1000), (int)kMtxPairPowerDbm, (int)kMtxPairMinRssiDbm);
  radio.implicitHeader(5);
  radio.startReceive();
  rfInterrupt = false;
  // BIND was held to get here, so a press only counts once the button has been RELEASED for 300 ms (a bouncing
  // release must not close the window it just opened), and then only when it reads pressed on 3 passes (~30 ms).
  unsigned long bind_up_since = 0;
  bool          bind_armed    = false;
  uint8_t       bind_down_n   = 0;
  bool led_on = false;
  unsigned long led_ms = 0;
  for (;;)
  {
    delay(10);   // C14: yield on every pass
    const unsigned long now = millis();
    if (!rxPairBindDown())
    {
      bind_down_n = 0;
      if (bind_up_since == 0) bind_up_since = (now != 0) ? now : 1;
      if (now - bind_up_since >= 300UL) bind_armed = true;
    }
    else
    {
      bind_up_since = 0;
      if (bind_armed && bind_down_n < 255) bind_down_n++;
    }
    if (mtxPairPoll(&w, (uint32_t)now, bind_armed && bind_down_n >= 3)) break;
    if (now - led_ms >= 500UL) { led_ms = now; led_on = !led_on; rxPairLed(led_on); }   // slow blink: window open
    if (!rfInterrupt) continue;
    uint8_t req[5];
    const int16_t rd = radio.readData(req, 5);
    const float rssi = radio.getRSSI();
    rfInterrupt = false;
    if (rd == RADIOLIB_ERR_NONE && (req[0] == 0xAB || req[0] == 0xAD) && req[4] == esp_crc8(req, 4))
      rxPairExchange(&w, req[0] == 0xAB, req + 1, rssi);
    radio.implicitHeader(5);   // back to listening for 5-byte requests after every exchange
    radio.startReceive();
    rfInterrupt = false;
  }
  rxPairLed(false);
  const char* why = (w.close_reason == MTX_PC_FULL) ? "four remotes paired" : (w.close_reason == MTX_PC_LEGACY) ? "an old-style remote paired"
                  : (w.close_reason == MTX_PC_BIND) ? "BIND pressed" : "time up";
  MtxRxBinding nb = cur;
  const bool enrolled = mtxPairCommit(&w, &nb, g_efuse_id);
  if (enrolled) mtxValidateRxBinding(&nb, g_efuse_id);   // belt and braces: the window refused every bad ID already
  mtxRxBindingToConf(&nb);
  if (enrolled)
  {
    saveConfToSPIFFS(usrConf);
    Serial.printf("PAIR: window closed (%s) - %u remote(s) paired and saved; mode %s:\n", why,
                  (unsigned)mtxValidEntryCount(&nb), (mtxValidEntryCount(&nb) >= 2) ? "MULTI (ownership rules)" : "SINGLE (today's behaviour)");
    for (uint8_t i = 0; i < kMtxSlots; i++)
    {
      if (!mtxEntryValid(&nb, i)) continue;
      char ib[9];
      mtxFmtId(ib, mtxEntryId(&nb, i));
      Serial.printf("PAIR:   slot %u  %s  %s  %s\n", (unsigned)(i + 1), ib, (i == 0) ? "MASTER" : "guest ",
                    mtxEntryLegacy(&nb, i) ? "old-style" : "new-style");
    }
    rxPairLed(true);
    delay(1000);   // solid for 1 s: saved
    rxPairLed(false);
  }
  else
  {
    Serial.printf("PAIR: window closed (%s) with nobody paired - the previous pairing is kept, nothing was written\n", why);
    rxPairFlash(3, 50, 50);
  }
  // V2.5-Evo - 2026-10-20 - the old function left the radio in 8-byte mode, so the first frame after a bind was read
  // with the wrong length; normal operation listens for 6-byte frames.
  radio.implicitHeader(6);
  return enrolled;
}

// ============================================================
// PHASE B GPS HANDSHAKE STATE
//
// gps_phase_b_ok: set true when Phase B distance and speed checks
// both pass; set false when either fails. Initialized false so
// RTM arming is blocked until the first successful handshake.
// NOT static — the RTM state machine (Priority 7) reads this flag.
//
// The static variables below are internal to gpsPhaseBCheck() and
// track timing and the previous TX position snapshot across calls.
// ============================================================
bool gps_phase_b_ok = false;  // Phase B handshake result; false = RTM arming blocked
// V2.5-Evo - 2026-10-07 - P-3: true while the LAST Phase B result is a failure of the DISTANCE check (rider farther than
// gps_max_pair_dist_m); false after a pass or a speed-check failure. Read by runFmReturnTick() (RTMState.ino): a parked
// auto-return waits (NOT READY) through a distance-only failure instead of ending. NOT static, like gps_phase_b_ok.
bool gps_phase_b_dist_fail = false;

// Last time Phase B check ran (ms). 0 = never run this session.
static unsigned long gps_phase_b_last_check_ms = 0;

// Previous TX GPS position snapshot used to compute TX implied speed.
// Updated each time Phase B check runs. 0 = no prior snapshot.
static double        gps_phase_b_prev_tx_lat = 0.0;
static double        gps_phase_b_prev_tx_lng = 0.0;
static unsigned long gps_phase_b_prev_tx_ms  = 0;

// ============================================================
// gpsPhaseBCheck - Phase B GPS handshake anti-spoofing validation
// ============================================================
//
// What it does:
//   Called after every successful 0xF3 GPS meta-packet decode.
//   Validates that TX and RX are physically plausible partners:
//
//   1) Distance check: Haversine distance between the last accepted
//      RX GPS position and the just-received TX GPS position must
//      be < usrConf.gps_max_pair_dist_m. Catches a spoofed TX GPS
//      report placing TX far from RX.
//
//   2) Speed consistency check: TX implied speed, computed from two
//      consecutive Phase B check positions, must differ from RX GPS
//      speed by < usrConf.gps_max_speed_diff_kmh. Catches GPS
//      replay attacks that report implausible TX movement.
//      Skipped on the very first check (no prior TX snapshot yet).
//
//   Time-gated: runs only on the first call after boot and every
//   30 seconds thereafter, regardless of how often meta-packets
//   arrive (they come at 2Hz).
//
//   Skipped entirely if:
//   - GPS disabled (usrConf.gps_en == 0)
//   - RX has no valid GPS reading (gps_last_ms == 0)
//
// Inputs:
//   Reads globals: rx_tx_gps_lat/lng (just updated by processMetaGpsPacket),
//   gps_last_lat/lng/speed_kmh/ms (from GPS.ino), usrConf fields.
//
// Side effects:
//   Sets gps_phase_b_ok (true = pass, false = fail).
//   Updates gps_phase_b_last_check_ms and gps_phase_b_prev_tx_* snapshot.
//   Prints diagnostics to Serial.
// ============================================================
static void gpsPhaseBCheck()
{
  // ---- Prerequisite: GPS must be enabled in config ----
  if (!usrConf.gps_en)
  {
    // GPS disabled — Phase B cannot run; do not change gps_phase_b_ok.
    return;
  }

  // ---- Prerequisite: RX must have at least one valid accepted GPS reading ----
  if (gps_last_ms == 0)
  {
    rxprintln("GPS [PhB] Skipped — RX has no valid GPS reading yet");
    return;
  }

  // ---- Time gate: 30s when Phase B good (anti-spoofing); 2s when failed (recovery) ----
  unsigned long now = millis();
  unsigned long recheck_ms = gps_phase_b_ok ? 30000UL : 2000UL;
  if (gps_phase_b_last_check_ms != 0 &&
      (now - gps_phase_b_last_check_ms) < recheck_ms)
  {
    return;  // Not due yet; skip silently
  }
  gps_phase_b_last_check_ms = now;

  // ---- Check 1: TX-RX Haversine distance ----
  // TinyGPSPlus::distanceBetween() returns metres between two WGS84 lat/lng pairs.
  float dist_m = (float)TinyGPSPlus::distanceBetween(
      gps_last_lat, gps_last_lng,
      rx_tx_gps_lat, rx_tx_gps_lng);

  if (dist_m > usrConf.gps_max_pair_dist_m)
  {
    Serial.printf("GPS [PhB] FAIL distance: %.0f m > max %.0f m — RTM arming blocked\n",
                  dist_m, (double)usrConf.gps_max_pair_dist_m);
    gps_phase_b_ok = false;
    gps_phase_b_dist_fail = true;    // V2.5-Evo - 2026-10-07 - P-3: this failure is the distance check
    // Update snapshot so the next check has a fresh reference point.
    gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
    gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
    gps_phase_b_prev_tx_ms  = now;
    return;
  }

  // ---- Check 2: TX-RX speed consistency ----
  // Compute TX implied speed from the position change since the last Phase B snapshot.
  // Skip on the very first run because there is no prior snapshot to measure from.
  if (gps_phase_b_prev_tx_ms > 0)
  {
    float dt_s = (float)(now - gps_phase_b_prev_tx_ms) / 1000.0f;

    // Guard against near-zero dt (shouldn't happen due to 30s gate, but be safe).
    if (dt_s > 0.1f)
    {
      float tx_delta_m   = (float)TinyGPSPlus::distanceBetween(
          gps_phase_b_prev_tx_lat, gps_phase_b_prev_tx_lng,
          rx_tx_gps_lat, rx_tx_gps_lng);
      float tx_speed_kmh = (tx_delta_m / dt_s) * 3.6f;
      float speed_diff   = fabsf(tx_speed_kmh - gps_last_speed_kmh);

      if (speed_diff > usrConf.gps_max_speed_diff_kmh)
      {
        Serial.printf("GPS [PhB] FAIL speed: TX %.1f km/h, RX %.1f km/h, diff %.1f km/h > max %.1f km/h\n",
                      tx_speed_kmh, gps_last_speed_kmh, speed_diff,
                      (double)usrConf.gps_max_speed_diff_kmh);
        gps_phase_b_ok = false;
        gps_phase_b_dist_fail = false;   // V2.5-Evo - 2026-10-07 - P-3: a speed-check failure is never waited through
        gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
        gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
        gps_phase_b_prev_tx_ms  = now;
        return;
      }
    }
  }

  // ---- All checks passed ----
  Serial.printf("GPS [PhB] PASS: dist %.0f m (max %.0f m)\n",
                dist_m, (double)usrConf.gps_max_pair_dist_m);
  gps_phase_b_ok = true;
  gps_phase_b_dist_fail = false;     // V2.5-Evo - 2026-10-07 - P-3
  gps_phase_b_prev_tx_lat = rx_tx_gps_lat;
  gps_phase_b_prev_tx_lng = rx_tx_gps_lng;
  gps_phase_b_prev_tx_ms  = now;
}

// V2.5-Evo - 2026-04-25 - P7: Handle 0xF1 RTM state meta-packet from TX.
// pkt: 6-byte buffer. byte[3]=0xF1, byte[4]: 0=RTM deactivate, 1=RTM activate.
// Sets rtm_rx_active. Safety gates in RTMState.ino may override during active RTM.
// V2.5-Evo - 2026-10-07 - two more 0xF1 VALUES (wire format in Common/AutoReturnRules.h). Every older
// RX ignores any value other than 0 and 1, so a remote may send these to any RX safely:
//   0x02        H-1 REFRESH: "RTM is still active on the remote". Refreshes only - it never sets
//               rtm_rx_active, so a refresh that arrives after the buggy ended RTM by itself (Gate 9,
//               H-2, Phase C) cannot switch it back on. The first one in a run arms the expiry.
//   0x80-0xFF   S-8 BOOT ID: 0x80 | the remote's random 7-bit power-on ID. A different ID from the one
//               stored means the remote was switched off and on: rx_tx_boot_change_seq is bumped and
//               the loop cancels any standing return; the M-1 re-declaration block is cleared too
//               (a rebooted remote's first declaration comes from a gesture).
//   3-0x7F      still ignored.
// 0xF1/1 now also clears rtm_refresh_seen, so each RTM run must prove for itself that the remote
// refreshes before the expiry can apply.
static void processRtmStatePacket(const uint8_t *pkt)
{
  uint8_t new_state = pkt[4];
  if (new_state == 0)
  {
    // V2.5-Evo - 2026-10-08 - STICKY RETURN CAP (owner design). The remote ending a RUNNING return (its max-runtime
    // gate, its own GPS going stale, its answer to the buggy's fault bit, or any older remote's exit) used to leave
    // the buggy with nothing: the next pass handed the motor the full held trigger, buggy pointed at the rider. Now the
    // return's live limit is armed FIRST - at the cap in force (0 under an emergency stop, else RTM's own cap) - and
    // only then does RTM go off, so the motor task never sees a pass with neither cap standing. It clears on the first
    // pass with the trigger below 25 counts, so a released trigger (the arm ceremony's 0xF1/0, a stop after a long
    // release) is unaffected. Not armed when RTM is not running: after a Gate 9 arrival the buggy has already ended
    // RTM itself and keeps the arrival cap instead.
    // SCOPE (owner, 2026-10-08: fault ends only). The buggy cannot tell WHY the remote stopped. On this firmware's remote
    // every 0xF1/0 under a held trigger is a fault end (Gate 1 max runtime, Gate 2 its GPS stale); the rider's own stops
    // (Gate 3 after a 4 s release, the magnet, the lock) come with the trigger released, so the cap clears at once. Only
    // an OLDER remote's stick exit (Gate 4) would also arm it.
    if (rtm_rx_active) stickyCapArm(rtm_rx_emergency_stop ? 0 : rtm_approach_cap.load());
    rtm_rx_active         = false;
    rtm_rx_emergency_stop = false;
    Serial.println("RTM [RX] deactivated by TX");
  }
  else if (new_state == 1)
  {
    // RTM state machine in RTMState.ino will run safety gates on next iteration.
    rtm_refresh_last_ms = millis();   // V2.5-Evo - 2026-10-07 - H-1: a new run has not proved refreshing yet
    rtm_refresh_seen    = false;
    rtm_rx_active = true;
    Serial.println("RTM [RX] activation requested by TX");
  }
  else if (new_state == kRtmRefreshValue)
  {
    // V2.5-Evo - 2026-10-07 - H-1 refresh: only while RTM is active; never an activation.
    if (rtm_rx_active)
    {
      rtm_refresh_last_ms = millis();
      if (!rtm_refresh_seen) Serial.println("RTM [RX] the remote refreshes RTM - the 5 s refresh expiry is armed for this run");
      rtm_refresh_seen    = true;
    }
  }
  else if (txBootIdIsBootIdValue(new_state))
  {
    // V2.5-Evo - 2026-10-07 - S-8 boot ID. Single writer of rx_tx_boot_id (this task).
    uint8_t stored = rx_tx_boot_id.load();
    const uint8_t before = stored;
    const bool rebooted = txBootIdStep(&stored, new_state);
    rx_tx_boot_id.store(stored);
    // V2.5-Evo - 2026-10-07 - N-10: count every boot-ID packet, changed or not. Bumped AFTER the ID is
    // stored (and before the reboot counter below), so a reader that sees the count move also sees the
    // ID it belongs to; a changed ID still cancels the return through rx_tx_boot_change_seq.
    rx_tx_boot_id_rx_seq.store((uint8_t)(rx_tx_boot_id_rx_seq.load() + 1));
    if (rebooted)
    {
      fm_redeclare_blocked = false;
      rx_tx_boot_change_seq.store((uint8_t)(rx_tx_boot_change_seq.load() + 1));
      Serial.printf("RX [S-8] remote boot ID changed %u -> %u: the remote was switched off and on - any standing return is cancelled\n",
                    (unsigned)before, (unsigned)stored);
    }
    else if (before == kTxBootIdNone)
    {
      Serial.printf("RX [S-8] remote boot ID %u heard - a remote power cycle will now cancel a standing return\n", (unsigned)stored);
    }
  }
}

// V2.5-Evo - 2026-04-25 - P7: Handle 0xF2 FM override meta-packet from TX.
// pkt: 6-byte buffer. byte[3]=0xF2, byte[4]: the FM mode byte (layout below).
// Updates runtime FM mode without writing SPIFFS.
// V2.5-Evo - 2026-07-20 - R0/R2: 0xFF (the reboot value) no longer means "use the SPIFFS
// default" — it now means "the TX has made no declaration this session", which RTMState.ino
// treats as FM_IDLE. This handler is therefore the ONLY way FM can ever become armed.
// Each packet also stamps fm_mode_last_rx_ms so the RX can expire a declaration that stops
// being refreshed (the TX repeats 0xF2/mode every 30 s while armed).
//
// V2.5-Evo - 2026-09-19 - THE FULL MODE BYTE. byte[4] is no longer a bare 0-3:
//   bits 0-2  FM mode. 0 = disarm, 1-3 = the modes this firmware knows, 4-5 reserved for the
//             station work. Anything above 5 cannot be a mode this remote meant, so it FAILS CLOSED
//             to 0 (disarm) and says so once - never "the nearest mode".
//   bits 3-4  reserved (ignored).
//   bits 5-6  the remote's SESSION override of auto-return (fm_return_mode):
//             00 = none  -> fm_return_mode_runtime = 0xFF (use the stored default)
//             01 = OFF   -> 0
//             10 = ON    -> 1
//             11 = ignored (the runtime value is left as it was)
//   bit 7     V2.5-Evo - 2026-10-07 - M-1: "FRESH DECLARATION" - the remote sets it on a declaration
//             made by a rider gesture (arm, mode change), never on the 30 s keepalive or a disarm. It
//             is read only while the post-fault re-declaration block stands (see below); otherwise
//             ignored, so a remote that never sets it behaves exactly as before.
// The old `& 0x03` mask would have folded bit 5 into the mode; it is gone. Because the override
// rides on EVERY 0xF2 (declaration, 30 s keepalive, disarm burst), a lost packet is repaired by the
// next keepalive and a remote power cycle - which forgets its RAM override - returns the RX to its
// stored default within one keepalive. Order: the runtime override is written BEFORE the mode, so
// a 0xF2/0 disarm (bits 5-6 = 00) lands the runtime at 0xFF on the same packet that idles FM.
static void processFmOverridePacket(const uint8_t *pkt)
{
  const uint8_t raw  = pkt[4];
  uint8_t       mode = raw & 0x07;
  if (mode > 5) {
    static bool bad_mode_printed = false;   // one-shot: a stuck sender must not flood the log
    if (!bad_mode_printed) {
      bad_mode_printed = true;
      Serial.printf("FM [RX] 0xF2 mode %u is not a mode this firmware knows - treated as 0 (disarm)\n", (unsigned)mode);
    }
    mode = 0;
  }
  // ============================================================
  // V2.5-Evo - 2026-10-07 - M-1: AFTER A FOLLOW-ME FAULT, A KEEPALIVE MAY NOT RE-ARM THE BUGGY.
  // A remote that missed the fault edge (fm_flags bit 3) keeps its fm_armed and its 30 s keepalive
  // re-declared the mode, silently re-arming the buggy the fault had just disarmed. While
  // fm_redeclare_blocked stands (set by RTMState.ino when a fault ends Follow-Me), a mode 1-5 is
  // accepted only with bit 7 set - the "fresh declaration from a rider gesture" bit, which a remote
  // must set on its arm / mode-change packets and never on a keepalive. Today's remote never sets
  // bit 7, but it DOES disarm with 0xF2/0 when it sees the fault, and 0xF2/0 clears the block, so its
  // next arm gesture is accepted exactly as before. A blocked packet still counts as "the remote is
  // alive" (the caller stamped last_packet) but changes nothing else: no mode, no override bits, no
  // declaration age.
  // ============================================================
  // V2.5-Evo - 2026-10-07 - N-8(a): THE BLOCK IS ENFORCED ONLY ONCE THIS REMOTE HAS SENT A BOOT ID. Today's
  // remote sends none, and with it the block made Follow-Me silently unavailable after any link-loss fault
  // longer than the 6 s sticky window: the remote never saw fm_flags bit 3, never disarmed (so never sent
  // the 0xF2/0 that clears the block), kept showing ARMED, and every keepalive was ignored. Until a boot ID
  // is heard the RX therefore keeps the old behaviour (the keepalive may re-arm after a fault the remote
  // missed, audit R-6), and the block still records the fault for ?diag. A remote that sends a boot ID is
  // the one built with the full protocol (bit 7 on gesture declarations), and for it the block applies.
  if (mode == 0)
  {
    fm_redeclare_blocked = false;
  }
  else if (fm_redeclare_blocked.load() && rx_tx_boot_id.load() != kTxBootIdNone)
  {
    if (raw & 0x80)
    {
      fm_redeclare_blocked = false;   // a marked gesture declaration: accept it below
    }
    else
    {
      static unsigned long redeclare_msg_ms = 0;   // rate limit: one line per 10 s
      const unsigned long t = millis();
      if (redeclare_msg_ms == 0 || (t - redeclare_msg_ms) >= 10000UL)
      {
        redeclare_msg_ms = (t != 0) ? t : 1;
        Serial.printf("FM [RX] 0xF2 mode %u IGNORED: Follow-Me ended on a fault; disarm and arm again on the remote to re-declare\n", (unsigned)mode);
      }
      return;
    }
  }
  const uint8_t ret = (raw >> 5) & 0x03;
  if (ret == 0)      fm_return_mode_runtime.store(0xFF, std::memory_order_relaxed);
  else if (ret == 1) fm_return_mode_runtime.store(0,    std::memory_order_relaxed);
  else if (ret == 2) fm_return_mode_runtime.store(1,    std::memory_order_relaxed);
  // ret == 3: ignored on purpose (reserved encoding) - the runtime override keeps its value.
  fm_mode_runtime = mode;
  fm_mode_last_rx_ms.store(millis(), std::memory_order_relaxed);
  Serial.printf("FM [RX] mode override: %u (auto-return override: %s)\n", (unsigned)mode,
                ret == 0 ? "none - stored default" : ret == 1 ? "OFF" : ret == 2 ? "ON" : "ignored");
}

// V2.5-Evo - 2026-04-24 - GPS meta-packet state and handler for 0xF3 protocol

// gps_meta_pending: set true when a 0xF3 announcement (6-byte) is received.
// On the NEXT wakeup of triggeredReceive, read the GPS data packet (gps_meta_len bytes) instead
// of the normal 6-byte control packet.
static bool gps_meta_pending = false;
// V2.5-Evo - 2026-10-12 - audit delta A-1: the data length the last announcement promised - 15 for a v2 remote
// (announcement subtype 0x03, the packet carries the GPS fix counter), 14 for anything else (v1, unchanged).
// riderGpsDataLen() (Common/RiderGpsPacket.h) decides. Written and read on the radio task only.
static uint8_t gps_meta_len = kRiderGpsLenV1;

// ============================================================
// processMetaGpsPacket - Decode a received 14-byte (v1) or 15-byte (v2) GPS data packet
// ============================================================
//
// What it does:
//   Validates destination address, CRC8 (over bytes 0-12 stored in byte 13),
//   packet type (0xF3) and subtype (0x02). On success, extracts TX lat/lng
//   stored as int32_t microdegrees (little-endian) and writes the three
//   rx_tx_gps_* globals declared in BREmote_V2_Rx.h.
//   V2.5-Evo - 2026-10-12 - audit delta A-1: a 15-byte v2 packet has subtype 0x04, the remote's GPS fix counter in
//   byte 13 and the CRC (over 0-13) in byte 14 (Common/RiderGpsPacket.h). A NEW solution also stamps
//   rx_tx_gps_solution_ms / _seq.
//
// Inputs:
//   pkt - pointer to the received GPS data packet; len - 14 (v1) or 15 (v2), as the announcement promised
//
// Side effects:
//   On success: updates rx_tx_gps_lat, rx_tx_gps_lng, rx_tx_gps_timestamp.
//   Always: prints diagnostics to Serial.
// ============================================================
// ============================================================
// gpsPhaseATxCheck - Phase A validation for the RIDER's position
// ============================================================
// V2.5-Evo - 2026-08-27 - PHASE-A-TX-1.
//
// WHAT WAS MISSING, and it was a gap in a feature that was supposed to be symmetric. Phase A
// anti-spoofing (HDOP / teleport / acceleration) has existed since 2026-04-22, but
// gpsPhaseACheck() has exactly ONE call site - GPS.ino, on the RX's OWN GPS. The rider's
// position arrives over the radio and was written straight into rx_tx_gps_lat/lng with no
// plausibility test of any kind. Phase B cross-checks the two tracks and the FM controller
// EMA-filters the rider, but nothing ever asked "could a human have moved that far in that
// time?" about the very position every Follow-Me and RTM distance is computed from.
//
// IT IS NOT THEORETICAL. Beta tester heiguga/robertzach's 37-minute level-4 log contains nine
// position spikes; the worst moves the rider 24 -> 52 m in 0.34 s, an implied 301 km/h. The
// teleport limit is usrConf.gps_max_teleport_kmh, default 80. That spike is 3.7x over it and
// would have been rejected outright.
//
// WHY IT COSTS NOTHING IN TRACKING LATENCY, which is the objection worth answering up front:
// this REJECTS, it does not SMOOTH. A smoothing filter delays every sample, good ones included -
// which is why computeFmTarget() carries a lag anchor pushing the target forward by up to 2x
// d_follow purely to cancel the EMA's own delay. A rejection test adds nothing to a good fix:
// it passes through on the same tick it always did. Only an already-impossible reading is
// dropped.
//
// WHAT IS AND IS NOT CHECKED. The 0xF3/0x02 meta packet carries lat/lng only - no HDOP, no
// speed - so check 1 (HDOP) has no input and is skipped; the TX runs its own Phase A on its own
// module before transmitting, which is where HDOP is properly judged. Checks 2 and 3 are both
// reconstructed from consecutive positions.
//
// THREE GUARDS, ALL BIASED TOWARD ACCEPTING. A wrongly-rejected fix ages the rider's position
// and can trip FM's Gate 4 staleness timeout, so a false reject costs more than a missed spike -
// and a missed spike is caught downstream anyway by the 3-fix separation dwell. Phase A here is
// a second net, never the only one.
//   DT FLOOR   - rx_tx_gps_timestamp is stamped at PACKET RECEIPT, not at fix time, so after a
//     link dropout two packets can arrive back-to-back with a genuine position change between
//     them. The raw dt is then tiny and the implied speed enormous: good data rejected as a
//     teleport. Flooring dt makes the implied speed SMALLER, i.e. more permissive.
//   MAX GAP    - after a long silence the baseline is too old to say anything useful, so the
//     reading is accepted and the baseline re-seeded rather than judged against stale data.
//   REJECT RUN - if a bad position ever becomes the baseline, every good fix afterwards looks
//     like a teleport away from it and the rider would be frozen out permanently. After
//     kTxPhaseAMaxRejects consecutive rejections the checker assumes ITSELF to be the problem,
//     accepts, and re-seeds.
// ============================================================
// AUDIT PA-4: this floor and usrConf.gps_max_teleport_kmh MULTIPLY into a blind spot -
// (gps_max_teleport_kmh / 3.6) * kTxPhaseADtFloorS metres can always slip through regardless of
// arrival timing. At the 80 km/h default that is 5.56 m: below the follow station, comparable to
// ordinary GPS scatter, and 5x smaller than the worst spike in the beta logs. The validator allows
// gps_max_teleport_kmh up to 500, where the blind spot becomes 34.7 m and would swallow that spike
// whole. Do not raise the teleport limit without re-reading this line.
static const float    kTxPhaseADtFloorS   = 0.25f;  // s; smallest dt used for the speed maths
static const float    kTxPhaseAMaxGapS    = 10.0f;  // s; older than this, re-seed instead of judge
static const uint8_t  kTxPhaseAMaxRejects = 3;      // consecutive rejects before force-accepting

static double        tx_pa_prev_lat   = 0.0;
static double        tx_pa_prev_lng   = 0.0;
static unsigned long tx_pa_prev_ms    = 0;
static uint8_t       tx_pa_reject_run = 0;

uint16_t gps_tx_phase_a_rejects = 0;   // lifetime counter, surfaced by ?diag


// Previously accepted rider position, kept as the RAW int32 microdegrees off the wire rather than
// as reconstructed doubles: integer comparison is exact by construction and needs no reasoning
// about float representation. These feed rx_tx_gps_fix_seq, which is defined and fully explained
// in BREmote_V2_Rx.h beside the other rx_tx_gps_* globals.
// V2.5-Evo - 2026-10-12 - audit delta A-1: the three statics that held it (tx_seq_prev_lat_ud / _lng_ud /
// tx_seq_seeded) moved into this one struct, which also remembers the last v2 fix counter seen. The rules that read
// and write it are pure in Common/RiderGpsPacket.h (riderGpsClassify / NoteCounter / Commit, host-tested).
static RiderGpsRxState tx_gps_rx_state = {false, 0, 0, false, 0};

static bool gpsPhaseATxCheck(double cur_lat, double cur_lng, unsigned long now_ms)
{
  // No baseline yet - nothing to compare against. Seed and accept.
  if (tx_pa_prev_ms == 0) {
    tx_pa_prev_lat = cur_lat;
    tx_pa_prev_lng = cur_lng;
    tx_pa_prev_ms  = now_ms;
    return true;
  }

  float dt_raw = (float)(now_ms - tx_pa_prev_ms) / 1000.0f;

  // Baseline too old to judge against, or the clock went backwards - re-seed, do not reject.
  if (dt_raw > kTxPhaseAMaxGapS || dt_raw <= 0.0f) {
    tx_pa_prev_lat = cur_lat;
    tx_pa_prev_lng = cur_lng;
    tx_pa_prev_ms  = now_ms;
    return true;
  }

  float dt_s = (dt_raw < kTxPhaseADtFloorS) ? kTxPhaseADtFloorS : dt_raw;

  float dist_m      = (float)TinyGPSPlus::distanceBetween(
                               tx_pa_prev_lat, tx_pa_prev_lng, cur_lat, cur_lng);
  float implied_kmh = (dist_m / dt_s) * 3.6f;

  // ---- Check 2: teleport. THE ONLY CHECK, DELIBERATELY. ----
  // AUDIT PA-2 KILLED CHECK 3 (acceleration), and the reasoning is worth keeping because the same
  // trap will catch the next person who tries to add it back.
  //
  // Acceleration needs two successive SPEED measurements. What is available here is two successive
  // position differences over a RECEIVE clock, not a fix clock - so computing acceleration from it
  // differentiates the packet cadence twice and measures an artifact.
  //
  // Concretely: the TX transmits at 2 Hz off a 1 Hz GPS, so consecutive packets alternate REAL MOVE
  // / REPEATED POSITION. On a move packet the implied speed is 2v; on the repeat it is 0. The old
  // guard `tx_pa_prev_kmh > 0.0f` was false after a repeat, so the check SKIPPED every packet
  // carrying information and RAN it on every packet carrying an artifact - judging a synthetic 2v -> 0 step that no
  // rider ever performed. It crossed gps_max_accel_g (3.0) at a rider speed of 26.5 km/h at 2 Hz,
  // and at the 3.0 Hz rate actually measured in the beta logs it tripped at 11.8 km/h - which is
  // WELL INSIDE ordinary riding speed, and barely above the 10 km/h a rider must already exceed
  // for Follow-Me to engage at all. It would have rejected good rider fixes during normal use,
  // in exactly the speed band where the separation dwell is trying to accumulate them.
  //
  // The teleport check alone catches the observed spikes with 3.7x margin (worst logged spike
  // implies 301 km/h against an 80 km/h limit), and it needs only ONE position difference. Since
  // R4-1 the caller does not even invoke this function on a re-broadcast, so the alternating
  // pattern never reaches it and the dt it measures is always between two DISTINCT positions.
  bool bad = (implied_kmh > usrConf.gps_max_teleport_kmh);

  if (bad) {
    tx_pa_reject_run++;
    if (gps_tx_phase_a_rejects < 65535) gps_tx_phase_a_rejects++;
    Serial.printf("GPS [PhA-TX] rider position rejected: %.0f m in %.2f s = %.0f km/h "
                  "(max %.0f), run %u/%u\n",
                  (double)dist_m, (double)dt_raw, (double)implied_kmh,
                  (double)usrConf.gps_max_teleport_kmh,
                  (unsigned)tx_pa_reject_run, (unsigned)kTxPhaseAMaxRejects);

    // REJECT RUN escape: the baseline is more likely wrong than the whole world.
    if (tx_pa_reject_run >= kTxPhaseAMaxRejects) {
      Serial.println("GPS [PhA-TX] reject run hit the limit - re-seeding the baseline and "
                     "accepting. The stored reference was probably the bad one.");
      tx_pa_prev_lat   = cur_lat;
      tx_pa_prev_lng   = cur_lng;
      tx_pa_prev_ms    = now_ms;
      tx_pa_reject_run = 0;
      return true;
    }
    return false;    // baseline deliberately NOT advanced - next fix is judged from the last good one
  }

  tx_pa_prev_lat   = cur_lat;
  tx_pa_prev_lng   = cur_lng;
  tx_pa_prev_ms    = now_ms;
  tx_pa_reject_run = 0;
  return true;
}

// V2.5-Evo - 2026-10-12 - audit delta A-1: takes the LENGTH the announcement promised (14 = v1, 15 = v2) and decodes
// both through riderGpsParse() (Common/RiderGpsPacket.h). A v2 packet carries the remote's GPS fix counter, which moves
// once per new GPS solution - so a rider lying still with a good fix (identical coordinates) is a NEW solution every
// time, where the old coordinate test saw only re-broadcasts and the 30 s clock switched Follow-Me off.
// GPS-INTEGRITY NOTE (project GPS rules 1-4, rule 5): (1) every NEW solution is still judged by Phase A before anything
// is written, a rejected one writes nothing, and the repeat of a rejected solution is dropped, not re-judged; (2) the
// state machine still resets cleanly - gps_meta_pending is cleared before the read, a bad length / CRC / type writes
// nothing, and the radio returns to 6-byte mode on the common exit; (3) positions stay int32 microdegrees until the
// same double conversion as before; (4) nothing is extrapolated or interpolated: a re-broadcast or a dropped packet
// leaves the last good position to age exactly as before.
// rxGpsDataAddressOk - V2.5-Evo - 2026-10-20 - MULTI-REMOTE (design 5.4): may this 15-byte rider GPS packet be used?
//   Its bytes 0-2 must be the uplink address of the slot whose 0xF3 announce switched the radio to 15 bytes - this
//   buggy's own address for the old-style (or implicit) remote, exactly today's test; the link ID (remote ID XOR this
//   chip's ID) for a new-style remote - and in MULTI that slot must still own the buggy (mtxGpsDataOk(), host test
//   T30), so a non-owner's position can never become the rider's. GPS-integrity rule 2: a mismatch writes nothing.
//   Input: pkt (bytes 0-2). Returns: bool. Side effects: none (radio task, which also wrote the state it reads).
static bool rxGpsDataAddressOk(const uint8_t* pkt)
{
  const int8_t s = g_mtx.gps_meta_slot;
  uint8_t slot_addr[3];
  if (s == kMtxSlotImplicit || (s >= 0 && s < (int8_t)kMtxSlots && mtxEntryLegacy(&g_mtx.b, (uint8_t)s)))
    memcpy(slot_addr, usrConf.own_address, 3);
  else if (s >= 0 && s < (int8_t)kMtxSlots)
    mtxIdXor(mtxEntryId(&g_mtx.b, (uint8_t)s), g_efuse_id, slot_addr);
  else
    return false;
  return mtxGpsDataOk(&g_mtx.st, &g_mtx.cfg, s, pkt, slot_addr);
}

static void processMetaGpsPacket(uint8_t *pkt, uint8_t len)
{
  // V2.5-Evo - 2026-10-20 - MULTI-REMOTE: the address test is rxGpsDataAddressOk() (above). For the one old-style
  // remote it is the same comparison with own_address as before; the rest of this function is untouched.
  if (!rxGpsDataAddressOk(pkt))
  {
    rxprintln("META GPS: address mismatch, discarding");
    return;
  }

  // CRC (over every byte but the last; stored in the last), type 0xF3 and the subtype of THIS length (0x02 for v1,
  // 0x04 for v2) - riderGpsParse() checks all three, in that order, exactly as the 14-byte code did.
  RiderGpsFields f;
  const uint8_t parsed = riderGpsParse(pkt, len, esp_crc8, &f);
  if (parsed != RGP_OK)
  {
    rxprintln(parsed == RGP_CRC ? "META GPS: CRC fail, discarding" : "META GPS: unexpected length/type/subtype, discarding");
    return;
  }

  // lat/lng arrive as int32_t microdegrees, little-endian (memcpy inside riderGpsParse - no strict-aliasing cast).
  const int32_t lat_ud = f.lat_ud;
  const int32_t lng_ud = f.lng_ud;

  double   cand_lat = (double)lat_ud / 1e6;
  double   cand_lng = (double)lng_ud / 1e6;
  unsigned long now = millis();

  // AUDIT R4-1 / Q6: is this a genuinely NEW position, or the TX re-broadcasting the last one?
  // The TX transmits at 2 Hz off a 1 Hz GPS (3.0 Hz measured against 1 Hz in the beta logs), so
  // roughly every other packet carries a position the RX has already seen. Compared on the RAW
  // integers, which is exact.
  // V2.5-Evo - 2026-10-12 - audit delta A-1: the question is now "a NEW GPS SOLUTION?" (riderGpsClassify()):
  //   v2 - the remote's fix counter moved (identical coordinates included: a still rider, speed 0); the same counter
  //        with the same coordinates is a re-broadcast (REPEAT), the same counter with OTHER coordinates is dropped;
  //   v1 - the position moved: the R4-1 rule above, unchanged.
  // The counter is noted before Phase A, so the repeat of a solution Phase A rejects is a DROP, never judged twice.
  const uint8_t cls = riderGpsClassify(&tx_gps_rx_state, &f);
  riderGpsNoteCounter(&tx_gps_rx_state, &f);
  if (cls == RGF_DROP)
  {
    rxprintln("META GPS: fix counter repeated with a different position, discarding");
    return;
  }
  const bool is_new_fix = (cls == RGF_NEW);   // a new GPS SOLUTION (since 2026-10-12, A-1)

  // PHASE-A-TX-1: judge the rider's position before it becomes the number every FM and RTM
  // distance is computed from. On rejection NOTHING is written - not the position and not the
  // timestamp - so the previous good fix simply ages, exactly as it does when a packet is lost.
  // That is the honest representation: we do not have a fresh rider position this tick.
  //
  // A RE-BROADCAST IS NOT JUDGED AT ALL. It carries no new evidence, so evaluating it would only
  // advance the Phase A baseline clock and shorten the dt seen by the next genuine fix - which
  // inflates that fix's implied speed and risks rejecting good data. Skipping keeps the teleport
  // check measuring between DISTINCT positions, which is the only interval where speed means
  // anything.
  // V2.5-Evo - 2026-10-12 - A-1: "between distinct SOLUTIONS" for a v2 remote. A new solution at the same coordinates
  // is judged too (0 m, 0 km/h - always passes) and moves the baseline time to its own receipt, so the next moving fix
  // is measured over the real interval since the last solution, not since the rider last moved.
  bool distinct = false;
  if (is_new_fix) {
    if (!gpsPhaseATxCheck(cand_lat, cand_lng, now)) {
      return;
    }
    distinct = riderGpsCommit(&tx_gps_rx_state, &f);   // the reference moves; true only for a new POSITION
  }

  rx_tx_gps_lat       = cand_lat;
  rx_tx_gps_lng       = cand_lng;
  rx_tx_gps_timestamp = now;

  // AUDIT S-1: the counter is published LAST, after the position it advertises. Bumping it before
  // the writes meant a loop-task read landing in between saw "a new fix is available" with the
  // PREVIOUS position still in place. Harmless for today's only consumer, which reads the counter
  // and nothing else - but this is now the SSOT for "is there a new rider fix", and the next
  // consumer will read lat/lng on the strength of it. Same publish-last discipline the timestamp
  // already follows two lines up, for the same reason.
  // V2.5-Evo - 2026-10-12 - A-1: rx_tx_gps_fix_seq keeps its meaning - one step per DISTINCT position (the separation
  // proof needs real distinct positions, unchanged). The new SOLUTION stamp and counter are published after it, last of
  // all: they are what Follow-Me's "rider GPS alive" rule reads (BREmote_V2_Rx.h).
  if (distinct) rx_tx_gps_fix_seq++;
  if (is_new_fix) {
    rx_tx_gps_solution_ms  = (now != 0) ? (uint32_t)now : 1u;
    rx_tx_gps_solution_seq = rx_tx_gps_solution_seq + 1;   // explicit RMW (single writer), not ++ on a volatile
  }
  rx_tx_gps_proto = f.has_ctr ? 2 : 1;

  #ifdef DEBUG_RX
  Serial.printf("META GPS received: lat=%.6f lng=%.6f\n",
                rx_tx_gps_lat, rx_tx_gps_lng);
  #endif

  // Run Phase B anti-spoofing check against the freshly received TX GPS position.
  // gpsPhaseBCheck() is time-gated (first call + every 30s) and self-throttles.
  gpsPhaseBCheck();
}

// ============================================================
// V2.5-Evo - 2026-10-20 - MULTI-REMOTE: THE HOOK TABLE BEHIND mtxOnFrame() (Common/MultiTx.h)
// ============================================================
// The model decides WHAT happens to a frame and IN WHICH ORDER (release, grant, claim; host-tested). These hooks do
// the real work, on the radio task, and keep the contract written on MtxRxHooks (audit Y-1 a-h):
//   (a) mtxHkOwnerChanged() publishes the owner atomic, THEN moves the change counter;
//   (b) mtxHkReadEnv() reads the live atomics on every call;
//   (c) mtxHkZeroOutputs() is two byte stores + the compiler barrier and stamps no failsafe clock; mtxHkPublishLimit()
//       is the grant-stamp store + the barrier (the C6 order: the limit is visible before the new owner's first byte);
//   (d) mtxHkEvent() never blocks or prints: it queues for the loop task (mtxEventService()), and never with one remote;
//   (e) no hook calls back into the model or writes MtxRxState;
//   (f) mtxHkReply() yields (the two 10 ms delays of today's reply) and the model calls it last for a frame;
//   (g) one `now` per wake, read right after the semaphore (triggeredReceive()); mtxTimers() also runs on the timeout;
//   (h) the config is clamped in mtxRxRefresh() before MtxRxCfg is built; negative slots never reach mtxOnFrame().
// WHY THE ORDER HOLDS ON THIS CHIP (C6): single core, in-order; the PWM task (priority 10) is never preempted by this
// task (priority 5), so one calcPWM() pass sees either none of a grant, or the limit with or without the new throttle
// byte - never the byte without the limit. calcPWM() reads thr_received before the limit; keep it so (PWM.ino).

static void mtxHkReadEnv(void* c, MtxRxEnv* out)
{
  (void)c;
  out->pwm_active      = PWM_active;
  out->fm_mode_runtime = fm_mode_runtime.load();
  out->rtm_rx_active   = rtm_rx_active.load();
  out->fm_rx_active    = fm_rx_active.load();
}

// Release step 1 (C7, V-19): throttle 0 and steering centre FIRST, then the barrier. Neither last_packet nor
// last_control_packet is stamped, so the motor gate stays open at neutral until failsafe_time and shuts on its own.
static void mtxHkZeroOutputs(void* c)
{
  (void)c;
  thr_received      = 0;
  steering_received = 127;
  std::atomic_signal_fence(std::memory_order_seq_cst);
}

// Release step 2: today's RTM handler with a synthetic 0xF1/0. It arms the sticky return cap first if a return was
// running (Radio.ino processRtmStatePacket(), untouched), then RTM is off - at once, on this task.
static void mtxHkRtmOff(void* c)
{
  (void)c;
  const uint8_t f[6] = { 0, 0, 0, 0xF1, 0x00, 0 };
  processRtmStatePacket(f);
}

// Release step 3: today's Follow-Me handler with a synthetic 0xF2/0: mode 0, the override back to "none", the M-1
// block cleared. fm_rx_active is the loop's flag and drops on its next Follow-Me tick (the claim token covers that).
static void mtxHkFmOff(void* c)
{
  (void)c;
  const uint8_t f[6] = { 0, 0, 0, 0xF2, 0x00, 0 };
  processFmOverridePacket(f);
}

// Release / grant step 4: the rider-GPS state of the old owner. The fix's TIME STAMPS are cleared (every reader of the
// coordinates gates on them - design 3.4.1, audit X-10 a; the coordinates themselves are left), the Phase A baseline and
// Phase B are reset (the new owner's first GPS packet runs Phase B at once, distance check included), and a pending
// 15-byte receive goes back to 6 bytes (V-18). GPS-integrity rules 1-2: nothing stale can pass as the new owner's fix.
static void mtxHkResetRiderGps(void* c)
{
  MtxRxFw* x = (MtxRxFw*)c;
  static const RiderGpsRxState kRiderGpsRxStateNone = { false, 0, 0, false, 0 };
  rx_tx_gps_timestamp       = 0;
  rx_tx_gps_solution_ms     = 0;
  tx_gps_rx_state           = kRiderGpsRxStateNone;
  tx_pa_prev_lat            = 0.0;   // Phase A (rider): no baseline - the next fix seeds it
  tx_pa_prev_lng            = 0.0;
  tx_pa_prev_ms             = 0;
  tx_pa_reject_run          = 0;
  gps_phase_b_ok            = false; // Phase B: not passed until the new owner's own check
  gps_phase_b_dist_fail     = false;
  gps_phase_b_last_check_ms = 0;     // 0 = "never run": the next GPS packet runs it at once
  gps_phase_b_prev_tx_lat   = 0.0;
  gps_phase_b_prev_tx_lng   = 0.0;
  gps_phase_b_prev_tx_ms    = 0;
  x->gps_meta_slot          = kMtxNone;
  if (gps_meta_pending)
  {
    gps_meta_pending = false;
    radio.implicitHeader(6);
    radio.startReceive();
  }
}

// Release / grant step 5: the remote boot ID (S-8). The next owner's first ID is a seed, not a reboot.
static void mtxHkResetBootId(void* c)
{
  (void)c;
  rx_tx_boot_id.store(kTxBootIdNone);
}

// Grant step 1 (C6, V-12): the limit's start stamp (never 0) and the barrier, BEFORE any granted byte is applied.
static void mtxHkPublishLimit(void* c, uint32_t grant_ms)
{
  (void)c;
  mtx_grant_ms.store(grant_ms, std::memory_order_seq_cst);
  std::atomic_signal_fence(std::memory_order_seq_cst);
}

// The owner changed (a release or a grant). Y-1 (a): the owner atomic FIRST, then the counter the loop watches.
static void mtxHkOwnerChanged(void* c)
{
  MtxRxFw* x = (MtxRxFw*)c;
  mtx_owner_slot.store(x->st.owner, std::memory_order_seq_cst);
  mtx_owner_change_seq.fetch_add(1, std::memory_order_seq_cst);
}

// TODAY'S CONTROL BRANCH, MOVED HERE VERBATIM - the only writer of thr_received besides the release's zero, and the only
// stamp of last_control_packet (R-3). Same statements, same order, same millis() calls as the old inline code: the
// liveness stamp, the WiFi notice, the two bytes, THEN the motor gate's stamp (L-1), then the RSSI snapshot (F9) and
// the owner's link quality. Reached for the owner only (or the one remote, SINGLE).
static void mtxHkApplyOwner(void* c, int8_t slot, const uint8_t f[6])
{
  (void)c; (void)slot;
  last_packet = millis();
#ifdef WIFI_ENABLED
  webCfgNotifyRxConnected();
#endif
  rxprint("RSSI: ");
  rxprint(radio.getRSSI());
  rxprint(", SNR: ");
  rxprintln(radio.getSNR());

  thr_received      = f[3];
  steering_received = f[4];

  // V2.5-Evo - 2026-09-28 - R-3 FIX / 2026-09-29 L-1: the motor gate's own timestamp, stamped HERE, after the two
  // bytes it vouches for, and in no other place (the full history is with the old inline branch's comments, git).
  last_control_packet = millis();

  // V2.5-Evo - 2026-07-24 - F9: snapshot RSSI/SNR while the radio SPI bus is valid (just after this reception).
  g_last_rssi_dbm = radio.getRSSI();
  g_last_snr_db   = radio.getSNR();
  telemetry.link_quality = getLinkQuality(g_last_rssi_dbm, g_last_snr_db);
}

// TODAY'S META BRANCHES (0xF1 RTM state, 0xF2 Follow-Me, 0xF4 aux, 0xF3 GPS announce), MOVED HERE VERBATIM. Reached for
// the owner only in MULTI (a non-owner's meta is dropped by the model), and for the one remote in SINGLE.
// ONE MULTI-ONLY DIFFERENCE (step-1 code audit Q3 item 7): an owner 0xF1/0 while RTM is NOT active is not handed to
// processRtmStatePacket(). The remote repeats 0xF1/0 every 2.5 s or less during an arm gesture and the ceremony, so the
// buggy can see the owner is busy (the X-1 activity rule); each would print "RTM [RX] deactivated by TX" from this
// task. With RTM inactive the handler changes nothing that matters: it does not arm the sticky cap (RTM not running),
// rtm_rx_active is already false, and the emergency stop it clears is cleared by the inactive RTM loop on every tick
// anyway (RTMState.ino) - leaving it up to 100 ms longer can only hold throttle at 0. The liveness stamp is kept.
// With one remote paired this branch is today's, byte for byte.
static void mtxHkMeta(void* c, int8_t slot, const uint8_t f[6])
{
  MtxRxFw* x = (MtxRxFw*)c;
  if (f[3] == 0xF1)
  {
    // ---- RTM state meta-packet ----
    last_packet = millis();  // meta-packet proves TX is alive
    if (!x->cfg.single && f[4] == 0 && !rtm_rx_active.load())
    {
      x->rtm_off_quiet++;    // MULTI, RTM off: nothing to deactivate (see above)
      return;
    }
    processRtmStatePacket(f);
  }
  else if (f[3] == 0xF2)
  {
    // ---- FM override meta-packet ----
    last_packet = millis();  // meta-packet proves TX is alive
    processFmOverridePacket(f);
  }
  else if (f[3] == 0xF4)
  {
    // ---- Aux control meta-packet ----
    last_packet = millis();
    rx_aux_flags = f[4];
  }
  else if (f[3] == 0xF3)
  {
    // ---- GPS announcement ---- (unchanged: switch to the promised length now, before the data arrives)
    gps_meta_len = riderGpsDataLen(f[4]);
    rxprintln(gps_meta_len == kRiderGpsLenV2 ? "META GPS: announcement v2, switching to 15-byte mode"
                                             : "META GPS: announcement, switching to 14-byte mode");
    gps_meta_pending = true;
    x->gps_meta_slot = slot;   // design 5.4: the data is accepted only from this slot (rxGpsDataAddressOk())
    radio.implicitHeader(gps_meta_len);
    radio.startReceive();
    rfInterrupt = false;
    x->skip_common_exit = true;   // the remote expects no reply; skip the 6-byte re-arm (today's `continue`)
  }
}

// THE REPLY. SINGLE: TODAY'S REPLY, byte for byte - byte 3 is the shared telemetry_index (status 000), byte 4 the raw
// telemetry byte, the address the old-style remote's dest_address (or the one new-style remote's own ID). MULTI: the
// remote's own ID, its own rotation (C15, F-11), its status in bits 7-5, and the masks (design 7.6, audit O-1): a
// non-owner never sees the owner's Follow-Me / RTM fields, index 18 is THAT remote's own link quality, and the new
// owner gets the non-owner mask too until the loop has rebuilt the bytes after an owner change. Then today's send:
// 10 ms, 6-byte mode, transmit, 10 ms (this is the yield, hook contract Y-1 f).
static void mtxHkReply(void* c, int8_t slot, uint8_t status)
{
  MtxRxFw* x = (MtxRxFw*)c;
  rxprintln("Sending response");
  uint8_t sendArray[6];
  uint8_t* ptr = (uint8_t*)&telemetry;
  if (x->cfg.single)
  {
    if (slot == kMtxSlotImplicit || mtxEntryLegacy(&x->b, (uint8_t)slot)) memcpy(sendArray, usrConf.dest_address, 3);
    else                                                                   memcpy(sendArray, mtxEntryId(&x->b, (uint8_t)slot), 3);
    sendArray[3] = telemetry_index;
    sendArray[4] = ptr[telemetry_index];
    telemetry_index++;
    if(telemetry_index >= sizeof(TelemetryPacket))
    {
      telemetry_index = 0;
    }
  }
  else
  {
    memcpy(sendArray, mtxEntryId(&x->b, (uint8_t)slot), 3);
    const uint8_t idx     = mtxTelIndexNext(&x->tel_idx[slot]);
    const bool    pending = (mtx_telemetry_owner_seq.load() != mtx_owner_change_seq.load());
    sendArray[3] = mtxReplyByte3(status, idx);
    sendArray[4] = mtxReplyValue(idx, ptr[idx], false, status == MTX_S_OWNER, pending, x->linkq[slot]);
  }
  sendArray[5] = esp_crc8(sendArray, 5);

  #ifdef DEBUG_RX
  printHexArray(sendArray, 6);
  #endif

  vTaskDelay(pdMS_TO_TICKS(10));
  radio.implicitHeader(6);
  {
    int16_t _txErr = radio.startTransmit(sendArray, 6);
    if (_txErr != RADIOLIB_ERR_NONE)
      Serial.printf("[Radio] startTransmit error %d at line %d\n", _txErr, __LINE__);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

// ---- The "MTX:" line queue: radio task in, loop task out (hook contract Y-1 d) ----
// A single-producer / single-consumer ring: the radio task only moves the head, the loop task only the tail, so no
// lock is needed and the radio task never waits. If the loop is 16 lines behind, a line is dropped and counted.
struct MtxEvRec {
  uint8_t  ev, arg;
  int8_t   slot;
  uint8_t  id[3];      // the slot's ID when the event happened
  uint8_t  id2[3];     // the CLAIM event's old owner's ID (arg = its slot), else 0
  uint32_t ms;
};
static const uint8_t kMtxEvRingLen = 16;
static MtxEvRec             g_mtx_ev_ring[kMtxEvRingLen];
static std::atomic<uint8_t> g_mtx_ev_head{0};   // next slot the radio task writes
static std::atomic<uint8_t> g_mtx_ev_tail{0};   // next slot the loop task reads

static void mtxHkEvent(void* c, uint8_t ev, int8_t slot, uint8_t arg)
{
  MtxRxFw* x = (MtxRxFw*)c;
  if (x->cfg.single) return;                                       // no "MTX:" line ever with one remote (MT-33)
  if (ev == MTX_EV_DROPPED || ev == MTX_EV_NON_OWNER_META) return; // counted by the model; ?mtx shows them
  const uint8_t h    = g_mtx_ev_head.load(std::memory_order_relaxed);
  const uint8_t next = (uint8_t)((h + 1) % kMtxEvRingLen);
  if (next == g_mtx_ev_tail.load(std::memory_order_acquire)) { x->lines_lost++; return; }
  MtxEvRec& r = g_mtx_ev_ring[h];
  r.ev = ev; r.arg = arg; r.slot = slot; r.ms = millis();
  memset(r.id, 0, 3); memset(r.id2, 0, 3);
  if (slot >= 0 && slot < (int8_t)kMtxSlots) memcpy(r.id, mtxEntryId(&x->b, (uint8_t)slot), 3);
  if (ev == MTX_EV_CLAIM && arg < kMtxSlots) memcpy(r.id2, mtxEntryId(&x->b, arg), 3);
  g_mtx_ev_head.store(next, std::memory_order_release);
}

static const MtxRxHooks g_mtx_hooks = {
  &g_mtx,
  mtxHkReadEnv,
  mtxHkZeroOutputs,
  mtxHkRtmOff,
  mtxHkFmOff,
  mtxHkResetRiderGps,
  mtxHkResetBootId,
  mtxHkPublishLimit,
  mtxHkOwnerChanged,
  mtxHkApplyOwner,
  mtxHkMeta,
  mtxHkReply,
  mtxHkEvent
};

// mtxRxStartFresh - "nobody owns, nothing heard" for the radio task's state; the counters are kept. Side effects: g_mtx.
static void mtxRxStartFresh()
{
  const MtxRxCounters n = g_mtx.st.n;
  mtxRxStateInit(&g_mtx.st);
  g_mtx.st.n = n;
  memset(g_mtx.tel_idx, 0, sizeof(g_mtx.tel_idx));
  memset(g_mtx.linkq,   0, sizeof(g_mtx.linkq));
  memset(g_mtx.rssi,    0, sizeof(g_mtx.rssi));
  g_mtx.gps_meta_slot = kMtxNone;
  mtx_owner_slot.store(kMtxNone);
}

// mtxRxRefresh - run once per radio-task wake (and once at boot), before anything reads g_mtx.b or g_mtx.cfg:
//   1. this wake's copy of the binding from usrConf. If the paired list CHANGED under a running owner (a config
//      import or ?applyconf rebuilt it, ?set paired 0 cleared it), the slot numbers may now name other remotes, so the
//      owner is released with the tested order (throttle 0 first, RTM / Follow-Me off, ...) and everything starts
//      again from "nobody owns". With one remote paired nobody ever owns in the model, so this is a no-op there.
//   2. the mode, and the model's config, CLAMPED again here (hook contract Y-1 h; the validator already clamps every
//      intake path): claim_s 6-60 s, failsafe 100-10000 ms. valid_mask = the valid NEW-STYLE entries only.
//   Input: now (this wake's clock). Side effects: g_mtx; on a list change under an owner, the release hooks.
// V2.5-Evo - 2026-10-20 - Z-1 (step-2 audit; confirmed by the owner 2026-10-21): THE OUTPUTS ARE ZEROED
//   ON A LIST CHANGE THAT INVOLVES MULTI. The release above runs only when someone owns, so a change with NOBODY in
//   control zeroed nothing: going from one remote to two (an import, ?applyconf) left the single remote's last byte
//   standing until failsafe_time, with nobody owning it any more. Now, whenever the list was MULTI before the change or
//   is MULTI after it, the release's own first step runs as well - throttle 0, steering 127, the barrier, NO failsafe
//   stamp, so the motor gate shuts on its own clock. It writes nothing else. A change from one remote to one remote
//   (or to none) is left exactly as today: the owner's single-remote rule.
// V2.5-Evo - 2026-10-21 - P3-4 (step-3 audit; the owner's "start from zero and with a soft start"): on a change from
//   MULTI (two or more remotes) to SINGLE (one), the takeover limit is ALSO published, once, after the zero and before
//   this wake's frame. The board is SINGLE from here on, so the remaining remote's next control frame is applied at once
//   with no zero proof: before this, its held trigger drove at full authority the instant a config write removed the
//   other remote. Now that first byte meets the limit (mtx_takeover_cap rising to full over mtx_takeover_ms), and the
//   motor ramp. The SINGLE frame path is untouched; the PWM task applies the limit in any mode (PWM.ino). Both decisions
//   are the pure mtxListChangeActions() (Common/MultiTx.h, host test T21). This is the one case where the limit runs with
//   a single remote paired (SOP-039's "never with a single remote" needs the owner's one-clause amendment).
static void mtxRxRefresh(uint32_t now)
{
  MtxRxBinding b;
  mtxRxBindingFromConf(&b);
  if (!g_mtx.b_valid)
  {
    g_mtx.b = b;
    g_mtx.b_valid = true;
    mtxRxStartFresh();
  }
  else if (memcmp(&b, &g_mtx.b, sizeof(b)) != 0)
  {
    const uint8_t old_mode = g_mtx.mode;                    // last wake's mode (of the old list)
    const uint8_t new_mode = mtxRxMode(&b, g_efuse_id);
    if (g_mtx.st.owner != kMtxNone && g_mtx_hooks_ok) mtxRelease(&g_mtx.st, &g_mtx_hooks, MTX_REL_REBIND, now);
    const uint8_t lc = mtxListChangeActions(old_mode, new_mode);
    if (lc & MTX_LC_ZERO)  mtxHkZeroOutputs(&g_mtx);                       // Z-1
    if (lc & MTX_LC_LIMIT) mtxHkPublishLimit(&g_mtx, mtxStampOf(now));     // P3-4: the soft start, once (MULTI -> SINGLE)
    g_mtx.b = b;
    mtxRxStartFresh();
  }
  g_mtx.mode = mtxRxMode(&g_mtx.b, g_efuse_id);
  uint16_t claim_s = usrConf.mtx_claim_s, seize = usrConf.mtx_seize_en;
  uint16_t cap = usrConf.mtx_takeover_cap, cap_ms = usrConf.mtx_takeover_ms;
  mtxClampRxConfig(&claim_s, &seize, &cap, &cap_ms);
  g_mtx.cfg.single     = (g_mtx.mode != MTX_MODE_MULTI);
  g_mtx.cfg.valid_mask = 0;
  for (uint8_t i = 0; i < kMtxSlots; i++)
    if (mtxEntryValid(&g_mtx.b, i) && !mtxEntryLegacy(&g_mtx.b, i)) g_mtx.cfg.valid_mask |= (uint8_t)(1u << i);
  g_mtx.cfg.claim_s     = claim_s;
  g_mtx.cfg.failsafe_ms = mtxClampU16(usrConf.failsafe_time, 100, 10000);
  g_mtx.cfg.seize_en    = (seize != 0);
}

// mtxModeText - the mode in words. Returns: a constant string.
static const char* mtxModeText(uint8_t mode)
{
  return (mode == MTX_MODE_MULTI) ? "MULTI (2-4 remotes: the ownership rules)"
       : (mode == MTX_MODE_SINGLE) ? "SINGLE (one remote: today's behaviour)" : "UNPAIRED (nothing is obeyed - hold BIND at boot to pair)";
}

// mtxBootReport - V2.5-Evo - 2026-10-20 - called once from setup() after the config is loaded and before any task
//   exists: the hook-table check (Y-1: an incomplete table drops every frame, so the motor gate stays shut), the first
//   fill of the radio task's state, and the "MTX:" boot lines - this chip's ID, the mode and the paired list (MT-00).
//   Side effects: g_mtx, g_mtx_hooks_ok, Serial.
void mtxBootReport()
{
  g_mtx_hooks_ok = mtxRxHooksComplete(&g_mtx_hooks);
  mtxRxRefresh(millis());
  char me[9];
  mtxFmtId(me, g_efuse_id);
  Serial.printf("MTX: this buggy's chip ID %s | %s | %u remote(s) in the list\n", me, mtxModeText(g_mtx.mode),
                (unsigned)mtxValidEntryCount(&g_mtx.b));
  for (uint8_t i = 0; i < kMtxSlots; i++)
  {
    if (!mtxEntryValid(&g_mtx.b, i)) continue;
    char ib[9];
    mtxFmtId(ib, mtxEntryId(&g_mtx.b, i));
    Serial.printf("MTX:   slot %u  %s  %s  %s\n", (unsigned)(i + 1), ib, (i == 0) ? "MASTER" : "guest ",
                  mtxEntryLegacy(&g_mtx.b, i) ? "old-style" : "new-style");
  }
  if (mtxImplicitLegacy(&g_mtx.b, g_efuse_id))
  {
    char ib[9];
    mtxFmtId(ib, g_mtx.b.dest_address);
    Serial.printf("MTX:   one old-style remote from the earlier pairing (replies go to %s); it pairs as new-style once both are on SW37 / SW29\n", ib);
  }
  if (!g_mtx_hooks_ok) Serial.println("MTX: ERROR - the receiver's hook table is incomplete: every frame is dropped and the motors stay off");
}

// mtxEventService - V2.5-Evo - 2026-10-20 - loop task, once per loop() pass: print the "MTX:" lines the radio task
//   queued (at most 8 per pass, so a burst can never hold the loop). Side effects: Serial, the ring's tail.
void mtxEventService()
{
  for (uint8_t n = 0; n < 8; n++)
  {
    const uint8_t t = g_mtx_ev_tail.load(std::memory_order_relaxed);
    if (t == g_mtx_ev_head.load(std::memory_order_acquire)) return;
    const MtxEvRec r = g_mtx_ev_ring[t];
    g_mtx_ev_tail.store((uint8_t)((t + 1) % kMtxEvRingLen), std::memory_order_release);
    char a[9], b[9];
    mtxFmtId(a, r.id);
    mtxFmtId(b, r.id2);
    switch (r.ev)
    {
      case MTX_EV_TAKES_CONTROL:
        Serial.printf("MTX: %s takes control (slot %d, %s)\n", a, (int)r.slot + 1, (r.slot == 0) ? "master" : "guest");
        break;
      case MTX_EV_RELEASED:
        Serial.printf("MTX: %s released (%s)\n", a, (r.arg == MTX_REL_LOCK) ? "LOCK" : (r.arg == MTX_REL_SEIZED) ? "SEIZED by the master"
                      : (r.arg == MTX_REL_CLAIM) ? "CLAIM" : (r.arg == MTX_REL_REBIND) ? "the paired list changed" : "?");
        break;
      case MTX_EV_CLAIM:
        if (r.arg == 0xFF) Serial.printf("MTX: claim by %s (nobody was in control)\n", a);
        else               Serial.printf("MTX: claim by %s - control taken from %s\n", a, b);
        break;
      case MTX_EV_CLAIMABLE:
        Serial.printf("MTX: %s claimable (%s)\n", a, (r.arg == MTX_CL_SILENT) ? "SILENT" : "IDLE");
        break;
      case MTX_EV_NOT_CLAIMABLE:
        Serial.printf("MTX: %s no longer claimable (%s)\n", a, (r.arg == MTX_NC_HEARD) ? "heard" : (r.arg == MTX_NC_INPUT) ? "input" : "nobody else on");
        break;
      case MTX_EV_TOKEN:
        Serial.printf("MTX: claim by %s waits for Follow-Me to idle (up to %lu ms)\n", a, (unsigned long)kMtxClaimGrantMs);
        break;
      case MTX_EV_TOKEN_LAPSED:
        if (r.arg == 1) Serial.printf("MTX: claim by %s ended - that remote locked\n", a);
        else            Serial.printf("MTX: claim by %s lapsed (Follow-Me did not idle in time) - squeeze from zero again\n", a);
        break;
      case MTX_EV_SUMMON:
        Serial.printf("MTX: %s summoned the buggy (its RTM or Follow-Me arm follows from it)\n", a);
        break;
      case MTX_EV_SUMMON_REFUSED:
        Serial.printf("MTX: summon by %s refused - the remote in control is not claimable, or no fresh zero from %s\n", a, a);
        break;
      default:
        break;
    }
  }
}

// mtxPrintStatus - V2.5-Evo - 2026-10-20 - the body of ?mtx (System.ino), loop task. A CONSISTENT snapshot: the radio
//   task's state is copied with the scheduler suspended (about a microsecond - nothing else can run, and the radio task
//   never writes it from an interrupt), then printed from the copy. Side effects: Serial.
void mtxPrintStatus()
{
  vTaskSuspendAll();
  const MtxRxFw x = g_mtx;
  const uint32_t grant = mtx_grant_ms.load();
  const uint8_t  seq_owner = mtx_owner_change_seq.load(), seq_tel = mtx_telemetry_owner_seq.load();
  xTaskResumeAll();
  const uint32_t now = millis();
  char me[9], own[9], dest[9];
  mtxFmtId(me, g_efuse_id);
  mtxFmtId(own, x.b.own_address);
  mtxFmtId(dest, x.b.dest_address);
  Serial.println("=== Multi-remote (?mtx) ===");
  Serial.printf("this buggy: chip ID %s | own_address %s | paired %u | dest_address %s | hook table %s\n",
                me, own, (unsigned)x.b.paired, dest, g_mtx_hooks_ok ? "OK" : "INCOMPLETE - every frame dropped");
  Serial.printf("mode: %s | %u remote(s) in the list\n", mtxModeText(x.mode), (unsigned)mtxValidEntryCount(&x.b));
  Serial.printf("settings: mtx_claim_s %u s (claimable after %lu ms of silence with failsafe_time %u ms) | take-back %s | takeover limit %u -> 255 over %u ms\n",
                (unsigned)x.cfg.claim_s, (unsigned long)mtxClaimMs(x.cfg.claim_s, x.cfg.failsafe_ms), (unsigned)x.cfg.failsafe_ms,
                x.cfg.seize_en ? "on" : "off", (unsigned)usrConf.mtx_takeover_cap, (unsigned)usrConf.mtx_takeover_ms);
  static const char* const kStatusText[8] = { "OWNER (solo)", "OWNER", "FREE", "BUSY", "SEIZABLE", "REFUSED (E9)", "SEIZED (tA)", "-" };
  for (uint8_t i = 0; i < kMtxSlots; i++)
  {
    if (!mtxEntryValid(&x.b, i)) continue;
    char ib[9];
    mtxFmtId(ib, mtxEntryId(&x.b, i));
    const bool multi = (x.mode == MTX_MODE_MULTI);
    if (!multi)
    {
      Serial.printf("slot %u  %s  %s  %s (one remote: no per-slot ownership state)\n", (unsigned)(i + 1), ib, (i == 0) ? "MASTER" : "guest ",
                    mtxEntryLegacy(&x.b, i) ? "old-style" : "new-style");
      continue;
    }
    Serial.printf("slot %u  %s  %s  new-style | heard %s%lu ms ago | zero streak %u | last status %s | RSSI %d dBm, link %u/10\n",
                  (unsigned)(i + 1), ib, (i == 0) ? "MASTER" : "guest ",
                  x.st.heard_ms[i] ? "" : "never/", x.st.heard_ms[i] ? (unsigned long)mtxAgeMs(now, x.st.heard_ms[i]) : 0UL,
                  (unsigned)x.st.zero_streak[i], kStatusText[x.st.last_status[i] & 0x07], (int)x.rssi[i], (unsigned)x.linkq[i]);
  }
  if (mtxImplicitLegacy(&x.b, g_efuse_id)) Serial.printf("the one old-style remote of a migrated pairing: frames to %s, replies to %s\n", own, dest);
  if (x.mode != MTX_MODE_MULTI)
  {
    Serial.println("(with one remote the buggy runs today's code: no owner, no claim, no limit, no MTX: lines)");
  }
  else
  {
    if (x.st.owner >= 0 && x.st.owner < (int8_t)kMtxSlots)
    {
      char ob[9];
      mtxFmtId(ob, mtxEntryId(&x.b, (uint8_t)x.st.owner));
      Serial.printf("owner: slot %d (%s)", (int)x.st.owner + 1, ob);
    }
    else
    {
      Serial.print("owner: nobody");
    }
    Serial.printf(" | claimable: %s", (x.st.claimable == MTX_CL_SILENT) ? "SILENT" : (x.st.claimable == MTX_CL_IDLE) ? "IDLE" : "no");
    if (x.st.claimable != MTX_CL_NO) Serial.printf(" for %lu ms", (unsigned long)mtxAgeMs(now, x.st.claimable_since_ms));
    if (x.st.token.slot >= 0) Serial.printf(" | claim token: slot %d, %lu ms old", (int)x.st.token.slot + 1, (unsigned long)mtxAgeMs(now, x.st.token.ms));
    else                      Serial.print(" | claim token: none");
    Serial.printf(" | takeover limit now %u%s\n", (unsigned)mtxTakeoverCap(now, grant, (uint8_t)mtxClampU16(usrConf.mtx_takeover_cap, 0, 255), usrConf.mtx_takeover_ms),
                  grant ? "" : " (none running)");
    Serial.printf("owner-change counter %u, telemetry rebuilt for %u%s\n", (unsigned)seq_owner, (unsigned)seq_tel,
                  (seq_owner != seq_tel) ? " - the new owner gets the masked bytes until the next loop tick" : "");
  }
  Serial.printf("counters: grants %u, claims %u, summons %u, summons refused %u, tokens lapsed %u | released: lock %u, take-back %u, claim %u, list change %u | non-owner metas %u, dropped %u, unknown %lu | quiet RTM-off repeats %lu, lines lost %lu\n",
                (unsigned)x.st.n.grants, (unsigned)x.st.n.claims, (unsigned)x.st.n.summons, (unsigned)x.st.n.summons_refused,
                (unsigned)x.st.n.tokens_lapsed, (unsigned)x.st.n.released_lock, (unsigned)x.st.n.released_seized,
                (unsigned)x.st.n.released_claim, (unsigned)x.st.n.released_rebind, (unsigned)x.st.n.non_owner_meta, (unsigned)x.st.n.dropped,
                (unsigned long)x.unknown, (unsigned long)x.rtm_off_quiet, (unsigned long)x.lines_lost);
}

void triggeredReceive(void *parameter) {
  while (1)
  {
    // Feed WDT on every iteration regardless of packet activity.
    // portMAX_DELAY would block indefinitely when TX is off, preventing WDT reset.
    // 2000ms timeout: short enough to feed the 3000ms WDT, long enough to avoid
    // busy-looping when the radio is quiet.
    // V2.5-Evo - 2026-07-31 - RX-WDT-2: gated, same reason as PWM.ino — this task starts
    // before initWatchdog() subscribes it. Feed behaviour after subscription is unchanged.
    if (g_wdt_active) esp_task_wdt_reset();
    if (xSemaphoreTake(triggerReceiveSemaphore, pdMS_TO_TICKS(2000)) == pdTRUE)
    {
      // V2.5-Evo - 2026-10-20 - MULTI-REMOTE, hook contract Y-1 (g): ONE clock for this wake, read right after the
      // semaphore, handed to every model call. (The control and meta branches keep their own millis() stamps.)
      const uint32_t now = millis();
      if (gps_meta_pending)
      {
        // ---- GPS data packet path (14 bytes v1, 15 bytes v2) ----
        // A 0xF3 announcement arrived on the previous wakeup.
        // Radio is already in implicitHeader(gps_meta_len) + startReceive mode.
        // TX has sent the GPS coordinate packet; read and decode it.
        // V2.5-Evo - 2026-10-12 - audit delta A-1: the length is the one the announcement promised (15 for a v2 remote,
        // whose packet carries the GPS fix counter; 14 otherwise), and the buffer holds the larger of the two.
        // V2.5-Evo - 2026-10-20 - MULTI-REMOTE: processMetaGpsPacket() now accepts it only from the slot that announced
        // it, while that slot owns (rxGpsDataAddressOk()); with one remote that is today's own_address test.
        gps_meta_pending = false;  // clear before any early return

        uint8_t gpsArray[kRiderGpsLenMax];
        const uint8_t gps_len = gps_meta_len;
        if (radio.readData(gpsArray, gps_len) == RADIOLIB_ERR_NONE)
        {
          processMetaGpsPacket(gpsArray, gps_len);
        }
        else
        {
          rxprintln("META GPS: readData error");
        }
        // Fall through to common exit below (implicitHeader(6) + startReceive + rfInterrupt=false)
      }
      else
      {
        // ---- Normal 6-byte packet path ----
        uint8_t rcvArray[6];
        if (radio.readData(rcvArray, 6) == RADIOLIB_ERR_NONE)
        {
          rxprint("Received packet: ");
          #ifdef DEBUG_RX
          printHexArray(rcvArray, 6);
          #endif

          // V2.5-Evo - 2026-10-20 - MULTI-REMOTE: this wake's binding and config (and, if the paired list changed
          // under an owner, the release - mtxRxRefresh()). Gate 0 below reads the binding copied here.
          mtxRxRefresh(now);

          // V2.5-Evo - 2026-08-05 - SAFETY GATE. Checked BEFORE the address compare,
          // because on an unpaired board the address IS 00:00:00 and would match a
          // stranger. See rxMayAcceptControl() above for the full reasoning.
          if (!rxMayAcceptControl())
          {
            rxprintln("Ignored: RX is not paired — hold BIND at boot to pair");
          }
          else if (rcvArray[5] != esp_crc8(rcvArray, 5))
          {
            rxprintln("CRC fail");
          }
          else
          {
            // ============================================================
            // V2.5-Evo - 2026-10-20 - MULTI-REMOTE: WHO SENT IT, THEN THE PER-PACKET RULES
            // ============================================================
            // The address compare is now mtxLookupSlot() (host test T2): bytes 0-2 XOR this chip's ID name the
            // remote. 00:00:00 is dropped (gate 1); bytes 0-2 equal to this buggy's own address (XOR = 0) are the
            // old-style remote - the implicit entry of a migrated board, or the one legacy entry - exactly today's
            // match; anything else must be a valid new-style entry's link ID. A frame built for another buggy, and
            // any reply, decode to an ID that is not on the list and are dropped silently, as before.
            // Then mtxOnFrame(): with one remote it calls this file's hooks exactly as the old branches ran (control
            // -> apply_owner + reply, metas -> meta); with 2-4 remotes it applies the ownership rules. Negative
            // lookups never reach it (hook contract Y-1 h), nor does anything while the hook table is incomplete.
            const int8_t s = mtxLookupSlot(&g_mtx.b, g_efuse_id, rcvArray);
            if (s < 0 || !g_mtx_hooks_ok)
            {
              if (s < 0) g_mtx.unknown++;
              rxprintln("Not for this buggy (no paired remote has this address)");
            }
            else
            {
              rxprintln("Address matches");
              const bool legacy_entry = (s == kMtxSlotImplicit) || mtxEntryLegacy(&g_mtx.b, (uint8_t)s);
              if (!g_mtx.cfg.single && s < (int8_t)kMtxSlots)
              {
                // MULTI: THIS remote's own link quality, for its own replies (index 18, C15 / V-17) and ?mtx.
                const float rssi = radio.getRSSI();
                g_mtx.rssi[s]  = (int16_t)rssi;
                g_mtx.linkq[s] = (uint8_t)getLinkQuality(rssi, radio.getSNR());
              }
              g_mtx.skip_common_exit = false;
              mtxOnFrame(&g_mtx.st, &g_mtx.cfg, &g_mtx_hooks, s, legacy_entry, rcvArray, now);
              // The owner's 0xF3 announce switched the radio to the GPS length and expects no reply: skip the
              // common exit, exactly as the old announce branch's `continue` did.
              if (g_mtx.skip_common_exit) continue;
            }
          }
        }
        else
        {
          rxprintln("Rx err");
        }
      }

      // Common exit: restore 6-byte receive mode and clear stale interrupt flag.
      // The GPS announcement path uses 'continue' and never reaches here.
      radio.implicitHeader(6);
      radio.startReceive();
      rfInterrupt = false;
    }
    else
    {
      // ============================================================
      // Feature C - SX1262 self-heal on the semaphore-timeout branch
      // ============================================================
      // V2.5-Evo - 2026-07-14 - Feature C: re-arm the radio when the 2000ms semaphore times out.
      // The bug: the took-semaphore branch above re-arms the SX1262 (implicitHeader(6)+startReceive)
      // on every packet, but the timeout branch previously did NOTHING except loop back to feed the
      // WDT. If the SX1262 wedges or drops its DIO IRQ, no packet ever fires the semaphore again, so
      // the receiver is never re-armed and the link stays dead until a power-cycle.
      // The fix: on timeout, re-run the same re-arm sequence used at the common exit so a wedged
      // radio self-heals. startReceive() resets the FIFO/IRQ state, so it doubles as the buffer flush.
      // Motor-safety: this runs only after 2000ms of RF silence, by which point the RX failsafe has
      // long since zeroed the motor (the gate in PWM.ino tests millis()-last_control_packet; it was
      // last_packet until the 2026-09-28 R-3 fix, and the stale line number it used to quote has been
      // dropped rather than re-guessed). This branch never writes thr_received, last_packet,
      // last_control_packet, or any PWM state — motor-safe by construction.
      // V2.5-Evo - 2026-10-20 - MULTI-REMOTE, hook contract Y-1 (g): the model's timers run here too, so a claim token
      // lapses after 1 s even when nothing is heard and the claimable edges are printed. mtxTimers() RELEASES NOTHING
      // and writes no motor state; with one remote it returns at once. The paired-list check (mtxRxRefresh()) runs
      // first: a list cleared while everyone is silent still releases a stale owner.
      {
        const uint32_t now = millis();
        mtxRxRefresh(now);
        if (g_mtx_hooks_ok) mtxTimers(&g_mtx.st, &g_mtx.cfg, &g_mtx_hooks, now);
      }
      radio.implicitHeader(6);
      radio.startReceive();
      rfInterrupt = false;
    }
  }
}

// getLinkQuality() is now in ../Common/RadioCommon.h
