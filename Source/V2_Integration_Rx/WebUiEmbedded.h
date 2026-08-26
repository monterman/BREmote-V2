// V2.5-Evo - 2026-08-26 - fm_engage_dist_m description now documents the stationary-near latch reset: fresh radial distance below effective D_engage plus filtered rider speed below 2 km/h for 2 s clears a set proof. Text only; no field/struct/SW_VERSION change. Kept byte-identical to the standalone tool.
// V2.5-Evo - 2026-08-25 - Follow-Me selector extended with F4 In Front (max 4) and its already-ahead engagement warning. The existing zone-angle descriptions now state both roles: F1/F3 diagonal blend and F4 front-cone latch/loss. near_diag_offset sign prose corrected to match the clockwise-bearing implementation (Right negative, Left positive). UI only; no confStruct/SW_VERSION change.
// V2.5-Evo - 2026-08-16 - DEFAULTS RECONCILED: 11 field "def:" values in the array below disagreed with the factory defaults the firmware actually ships in defaultConf (BREmote_V2_Rx.h), so the page told the rider the wrong thing in the grey hint under each control ("default: N") and in the value it falls back to. Corrected against defaultConf, which is the only source of truth for this — radio_preset 1->2, rf_power 0->22, steering_type 0->1, pwm0_min 1500->1000, pwm1_min 1500->1000, data_src 0->2, vesc_timeout_s 12->6, gps_en 0->1, kalman_en 0->1, tx_gps_stale_timeout_ms 1000->3000, rtm_approach_zone_m 15->12. Where the field's own description ALSO stated a default in prose it was corrected in the same edit, so the sentence and the number can no longer drift apart. Verified mechanically: confStruct has 66 fields and the live defaultConf initializer has 66 top-level tokens, a 1:1 positional map, and after this pass all 64 UI defaults match it. Reconciled against defaultConf ONLY — the docs HTML tool carries the same stale numbers and was deliberately not used as a reference. DISPLAY LAYER ONLY: "def" feeds the hint text and the pre-load fallback value; it is never sent to the board, so no stored setting changes and no behaviour changes. No confStruct change, sizeof stays 192, SW_VERSION stays 35.
// V2.5-Evo - 2026-08-16 - REPAIR: the embedded RX config page was DEAD in the published SW35 binary and is fixed here. A bad bulk edit had (1) deleted the fields array terminator "];" together with the const state / const openGroups declarations and the api() fetch helper that followed it, so the whole embedded script block failed to parse (SyntaxError at "function normAddr") and nothing rendered and no button worked; (2) spliced the gps_dyn_model field object INSIDE gps_chip_type's options array, between option 0 and option 1, so gps_dyn_model was not a field at all and the module dropdown carried a garbage option; (3) deleted four fields outright — log_level, logger_en, wifi_password, version — which also left the Logging and System groups empty. All four are restored, the array is terminated, gps_dyn_model is a proper sibling field and gps_chip_type's four options are back. Also in this pass: mag_orientation moved from group "Compass" (which is not in groupOrder, so the field never rendered) to "GPS" where the other mag_* fields live; gps_update_hz default corrected 2 -> 10 to match defaultConf; followme_mode description no longer claims the removed 0xFF fallback. Field count is now 64 = the 66 kCfgFields rows minus the 2 hidden RESERVED slots. UI LAYER ONLY — no confStruct change, sizeof stays 192, SW_VERSION stays 35, no config wipe.
// V2.5-Evo - 2026-07-25 - STAGE 0 PART A: the fm_steer_reposition_en control (bool, RESERVED, Follow-Me group) is REPLACED by log_level (enum 0-4, Logging group). The underlying confStruct slot is the same uint16_t renamed in place, so sizeof stays 184 and SW_VERSION stays 34 — no config wipe. Description states what each level captures, roughly how long a session fits on this board at 5 Hz and at the 3 Hz default, that levels 1 and 2 are accepted but CURRENTLY LOG AS LEVEL 3 (reserved for a future storage optimisation, not silently ignored), and that the FM v2 repositioning feature will claim a fresh field of its own when it lands. Parameter count is unchanged at 63 (a rename, not an addition). Byte-identical to the same field in docs/BREmote_V2.5-Evo_Web_Serial_Config_Tool.html per the three-surface sync rule.
// V2.5-Evo - 2026-07-25 - F3-c: fm_engage_dist_m description rewritten in plain English (owner request) — it now LEADS with the rule (minimum 8 m), then the REASON (it is the tow-rope safety floor: Follow-Me must never be able to engage while you are still on the rope), then HOW TO CHOOSE (measure your rope, add at least a metre; a 20 ft / 6.1 m rope → 8 m or more), and it now states explicitly that setting 0 (automatic) does NOT bypass the minimum — the firmware applies the same 8 m floor to the auto-computed value (1.5 x (Min Distance + Smoothing Band)), which is the F3-c firmware fix in RTMState.ino. Byte-identical to the same field in docs/BREmote_V2.5-Evo_Web_Serial_Config_Tool.html per the three-surface sync rule. Description text only — key, type, def, min/max/step and unit all unchanged; no struct/SW_VERSION change.
// V2.5-Evo - 2026-07-25 - F3-b: fm_engage_dist_m description updated to the raised floor — legal shape is now "0 = auto, or 8-50 m" (was 5-50; 5 m was below the tow rope it exists to clear), the WHY no longer quotes a specific rope length, and it adds the rider instruction to MEASURE THEIR OWN ROPE and set at least rope length + 1 m (a 20 ft / 6.1 m rope → 8 m or more). Firmware rejects (0, 8) m and clamps older stored values up to 8 m. Description text only — key, type, def, min/max/step and unit all unchanged; no struct/SW_VERSION change. Keep byte-identical to docs/BREmote_V2.5-Evo_Web_Serial_Config_Tool.html RX_FIELDS.
// V2.5-Evo - 2026-07-25 - F3: fm_engage_dist_m description now states the legal shape "0 = auto, or 5-50 m" and WHY (the value must exceed the 6.7-7.6 m tow rope, or the separation interlock is defeated and FM could engage with the rider still on the rope). Firmware rejects anything in between (cfgValidateCrossField) and clamps older stored values up to 5 m. Description text only — key, type, def, min/max/step and unit all unchanged; no struct/SW_VERSION change.
// V2.5-Evo - 2026-07-25 - A2: fm_engage_dist_m label/description rewritten — the field is LIVE now (firmware reads it), no longer "RESERVED — not read". Explains that the value IS the engage distance in metres (rope length x ~1.15), not the rope length, and that 0 = auto. Text only — key, type, def, min/max/step and unit all unchanged; no struct/SW_VERSION change.
// V2.5-Evo - 2026-07-24 - F4 fix: followme_mode web def 1→2 to match defaultConf (shipped default is Behind, not Near-Right); description + option labels moved the "(default)" tag to Behind. Text/default only — no struct/SW_VERSION change.
// V2.5-Evo - 2026-07-21 - FIX-WEB-1 (ported from TX): saveAll() now validates and sends only DIRTY fields (was: all fields every save). Unblocks RX web "Save All" — the orphaned compass mag_* fields could never be dirty, so they can no longer block a legitimate save with a false "Required" alert. No struct/SW_VERSION change.
// V2.5-Evo - 2026-07-20 - SW34 RX: added 3 reserved config fields to the web UI (fm_engage_dist_m 0=auto, auton_runtime_cap_s 0=disabled, fm_steer_reposition_en RESERVED Option C disabled-until-v2); param-count comment 57→63. Labels flag all three as RESERVED so the owner does not enable them.
// V2.5-Evo - 2026-07-18 - FM mode labels canonicalized to TX convention (1=Near-Right, 2=Behind, 3=Near-Left); followme_mode def 0→1; foiler_low_speed default text 5→8 km/h (matches defaultConf). Cosmetic — no struct/firmware change.
// V2.5-Evo - 2026-05-13 - SW44 RX: mag_offset_x/y + mag_scale_x/y added to WebUI GPS group (were in confStruct but invisible to UI)
// V2.5-Evo - 2026-05-13 - SW42 RX: Del All button in log modal header (red, small); fixed btn danger→btn warn on list body buttons
// V2.5-Evo - 2026-05-13 - SW41 RX: modal-overlay height 100%→100vh (Android Firefox centering fix)
// V2.5-Evo - 2026-05-13 - SW39 RX: Expand All / Collapse All buttons above config groups
// V2.5-Evo - 2026-05-13 - SW34: WebUI UX — reboot btn feedback, log Refresh btn, overflow-x fix, compact log rows
#ifndef WEB_UI_EMBEDDED_H
#define WEB_UI_EMBEDDED_H
// V2.5-Evo - 2026-05-03 - Log UI: Delete All + Delete Selected buttons;
//                   exportJsonFile() redirects to server endpoint (iPhone fix)
// V2.5-Evo - 2026-04-25 - P7: Added 5 RTM/FM RX fields; added RTM & Follow-Me group
// V2.5-Evo - 2026-04-28 - Security: added rtm_stop_distance_m field (was in ConfigService but missing from UI)
// V2.5-Evo - 2026-04-29 - TaskC: full description audit — bool 0/1 values, enum all options inline, int/float extremes explained
// V2.5-Evo - 2026-04-30 - RTM approach decel zone: rtm_approach_zone_m field added to RTM & Follow-Me group
// V2.5-Evo - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V2.5-Evo - 2026-04-30 - Bundle E: gps_update_hz field added to GPS & Follow-Me group; gps_max_teleport_kmh default 200→80
// V2.5-Evo - 2026-04-29 - Bundle A: radio_preset max clamped to 2; dead foil_speed != 99 sentinel removed
// V2.5-Evo - 2026-05-01 - fix: wet_det_active description corrected — warning-only (E71 + vibration), output is never cut
// V2.5-Evo - 2026-05-06 - D6: Added rtm_use_compass + rtm_cog_min_speed_kmh UI controls in RTM & Follow-Me group
// V2.5-Evo - 2026-05-22 - SW32: Added rtm_align_threshold_deg + rtm_target_speed_kmh controls in RTM group
// V2.5-Evo - 2026-05-08 - Bundle 1: Replace dummy_delete_me with rtm_steer_response enum dropdown in RTM & Follow-Me group
// V2.5-Evo - 2026-05-07 - FIX-WEBUI-LOGS-CSS: minimal CSS-only mobile fix for Logs modal — flex-wrap on log-item, mobile media query for stacked layout
// V2.5-Evo - 2026-05-08 - Bundle 3a: Shrink per-row Download CSV + Delete buttons in Logs modal (smaller padding+font-size; flex:0 0 auto on mobile so filename gets more space)
// V2.5-Evo - 2026-05-08 - Bundle 4: Web UI parameter regrouping. Renamed "GPS & Follow-Me" → "GPS"; "RTM & Follow-Me" → "RTM"; split RTM and Follow-Me into separate groups; moved followme_mode + rtm_vesc_speed_diff_kmh + vesc_erpm_per_kmh out of GPS group; merged "Follow-Me Tuning" into "Follow-Me". Cosmetic only — no struct/firmware changes.

#include <Arduino.h>

static const char WEB_UI_INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>BREmote V2 RX Web Config</title>
  <style>
    :root{--bg:#0b1220;--panel:#101828;--panel2:#1e293b;--txt:#e5e7eb;--muted:#9ca3af;--pri:#60a5fa;--err:#ef4444}
    *{box-sizing:border-box}
    body{margin:0;overflow-x:hidden;background:radial-gradient(1200px 700px at 10% -10%,#1e3a5f66 0,transparent 45%),linear-gradient(180deg,#0a1120 0,#0b1220 40%);color:var(--txt);font-family:"Avenir Next","Montserrat","Segoe UI",sans-serif; padding-top: 15px;}
    .wrap{max-width:980px;margin:0 auto;padding:14px 14px 110px}
    .card{background:linear-gradient(180deg,#121b2e,#101828);border:1px solid #243042;border-radius:14px;padding:12px;box-shadow:0 8px 24px #00000033; margin-bottom: 15px;}
    .top{position:sticky;top:0;backdrop-filter:blur(6px);padding-top:6px;z-index:9}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .sp{justify-content:space-between}
    .title{font-weight:700;font-size:18px; color:#38bdf8;}
    .sub{color:var(--muted);font-size:12px}
    .btn{border:0;border-radius:10px;padding:9px 12px;background:var(--pri);color:#08111f;font-weight:700;cursor:pointer;transition:all 0.2s;}
    .btn:disabled{opacity:.55;cursor:not-allowed}
    .btn.sec{background:#243042;color:var(--txt)}
    .btn.warn{background:#b91c1c;color:#fff}
    
    /* --- INTERACTIVE FEEDBACK UI --- */
    .btn.success { background: #22c55e !important; color: #000 !important; box-shadow: 0 0 10px rgba(34, 197, 94, 0.5); }
    .dirty { background-color: #451a1a !important; border: 1px solid #ef4444 !important; box-shadow: 0 0 8px rgba(239, 68, 68, 0.4); }
    .active-save { background: #f97316 !important; color: white !important; animation: pulse 1.5s infinite; border: 1px solid #fff; }
    @keyframes pulse { 0% { transform: scale(1); box-shadow: 0 0 0 0 rgba(249,115,22,0.7); } 70% { transform: scale(1.02); box-shadow: 0 0 0 10px rgba(249,115,22,0); } 100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(249,115,22,0); } }

    .groups{margin-top:10px;display:flex;flex-direction:column;gap:8px}
    details{background:var(--panel2);border:1px solid #334155;border-radius:12px}
    summary{cursor:pointer;padding:10px 12px;font-weight:700;color:#cbd5e1}
    .items{padding:0 10px 10px;display:flex;flex-direction:column;gap:8px}
    .field{background:var(--panel);border:1px solid #243042;border-radius:10px;padding:10px}
    .label{font-weight:700}.desc,.hint,.err{font-size:12px}.desc,.hint{color:var(--muted)}.err{color:var(--err)}
    input,select{width:100%;padding:8px;border-radius:8px;border:1px solid #334155;background:#0f172a;color:var(--txt); transition: 0.3s;}
    input[type='checkbox']{width:auto} input[type='range']{width:100%}
    .triple{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.foot{margin-top:10px}
    
    /* --- JSON EDITOR STYLE --- */
    textarea { width:100%; height:180px; font-family:monospace; font-size:12px; background:#0f172a; color:#38bdf8; border:1px solid #334155; border-radius:8px; padding:10px; resize:vertical; outline:none;}
    
    .modal-overlay{position:fixed;top:0;left:0;width:100%;height:100vh;background:#0b1220e6;display:none;align-items:center;justify-content:center;z-index:99;padding:20px;overflow-y:auto}
    .modal{background:linear-gradient(180deg,#121b2e,#101828);border:1px solid #243042;border-radius:14px;padding:20px;width:100%;max-width:500px;max-height:80vh;overflow-y:auto;box-shadow:0 8px 24px #00000066}
    .modal-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;font-size:18px;font-weight:700}
    .log-item{display:flex;align-items:center;gap:5px;padding:3px 0;border-bottom:1px solid #1a2535;}
    .log-item:last-child{border-bottom:none;}
    .log-info{flex:1;overflow:hidden;min-width:0;}
    .log-name{font-family:ui-monospace,SFMono-Regular,monospace;font-size:11px;color:var(--txt);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;display:block;}
    .log-size{font-size:10px;color:var(--muted);}
    .log-actions{display:flex;gap:3px;flex-shrink:0;}
    @media (max-width:600px){.modal-overlay{padding:8px;}.modal{padding:10px;}}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div class="card">
        <div class="row sp">
          <div>
            <div class="title">BREmote V2.5-Evo RX Config</div>
            <div class="sub" id="status">Loading...</div>
            <div class="sub" id="loaded">Loaded 0/0</div>
          </div>
          <div class="row">
            <button class="btn sec" id="loadBtn" onclick="loadCfg()">Force Sync</button>
            <button class="btn" id="saveBtn" onclick="saveAll()">Save All</button>
            <button class="btn sec" onclick="openLogs()">Manage Logs</button>
            <button class="btn warn" onclick="rebootDev()">Reboot RX</button>
          </div>
        </div>
      </div>
    </div>
    
    <div class="row" style="margin:4px 0 6px">
      <button class="btn sec" style="font-size:12px;padding:5px 10px" onclick="expandAll()">Expand All</button>
      <button class="btn sec" style="font-size:12px;padding:5px 10px" onclick="collapseAll()">Collapse All</button>
    </div>
    <div class="groups" id="groups"></div>
    
    <div class="card" style="margin-top: 20px;">
        <div class="title" style="margin-bottom: 10px;">Raw JSON Backup & Restore</div>
        <div class="desc" style="margin-bottom: 10px;">Copy/Paste configurations manually or use files. Click 'Load from Text' to apply imported settings to the UI above before saving to the board.</div>
        <textarea id="jsonBox" spellcheck="false"></textarea>
        
        <div class="row" style="margin-top: 10px;">
            <button class="btn sec" id="btnCopy" onclick="copyJson()">Copy to Clipboard</button>
            <button class="btn" style="background:#f59e0b; color:black;" onclick="loadFromJsonText()">Load from Text</button>
            <div style="flex-grow:1;"></div>
            <button class="btn sec" onclick="exportJsonFile()">Export File</button>
            <button class="btn sec" onclick="document.getElementById('importFile').click()">Import File</button>
            <input type="file" id="importFile" accept=".json" style="display:none" onchange="importJsonFile(this)">
        </div>
    </div>
    <div class="card foot"><div class="sub mono" id="last">Last: -</div></div>
  </div>

  <div class="modal-overlay" id="logModal">
    <div class="modal">
      <div class="modal-header">
        <span>Data Logs</span>
        <div style="display:flex;gap:6px"><button class="btn sec" style="font-size:13px;padding:5px 10px" onclick="openLogs()" title="Refresh log list">↺</button><button class="btn warn" style="font-size:11px;padding:4px 8px" onclick="deleteAllLogs()" title="Delete all log files">Del All</button><button class="btn sec" onclick="document.getElementById('logModal').style.display='none'">Close</button></div>
      </div>
      <div id="logList"></div>
    </div>
  </div>

<script>
// 64 Parameters for RX. This is every row of kCfgFields (ConfigService.ino) except the two
// intentionally-hidden RESERVED slots, rsvd_u16_1 and rsvd_f32_1: 66 - 2 = 64. If you add,
// rename or remove a kCfgFields row, this count and the array below must move with it.
const groupOrder=["Radio","Steering","PWM","Motor & Safety","VESC","Sensors","Battery","GPS","RTM","Follow-Me","Logging","System"];
const fields=[
{key:"radio_preset",label:"Radio Preset",description:"Radio frequency band. 1=EU 868 MHz, 2=US/AU 915 MHz. Only these two bands exist. Must match the TX setting. (Any invalid value now safely defaults to EU868 instead of halting boot.) Factory default 2 (US/AU 915 MHz).",group:"Radio",type:"enum",def:2,min:1,max:2,options:[{v:1,l:"EU868"},{v:2,l:"US/AU915"}]},
{key:"rf_power",label:"RF Power",description:"RF transmit power in dBm. -9=minimum (shortest range, lowest interference), 22=maximum (longest range). Must match TX setting. Default 22 (maximum).",group:"Radio",type:"int",def:22,min:-9,max:22,unit:"dBm"},
{key:"paired",label:"Paired",description:"0=not paired (no TX address stored), 1=paired. Set automatically by the pairing sequence — do not edit manually.",group:"Radio",type:"bool",def:0,min:0,max:1},
{key:"own_address",label:"Own Address",description:"This RX unit's 3-byte LoRa radio address (hex, e.g. 01,A2,FF). Set automatically during pairing — do not edit manually.",group:"Radio",type:"address3",def:"00,00,00"},
{key:"dest_address",label:"Dest Address",description:"Paired TX remote's 3-byte LoRa radio address (hex, e.g. 01,A2,FF). Set automatically during pairing — do not edit manually.",group:"Radio",type:"address3",def:"00,00,00"},
{key:"steering_type",label:"Steering Type",description:"0=Single Motor (one motor, no differential steering), 1=Differential (two motors, speed difference creates turning), 2=Servo (dedicated servo output on PWM1). Default 1.",group:"Steering",type:"enum",def:1,min:0,max:2,options:[{v:0,l:"Single Motor"},{v:1,l:"Diff Motor"},{v:2,l:"Servo"}]},
{key:"steering_influence",label:"Steering Influence",description:"How much the steering input modifies motor speed in differential mode. 0=no steering effect (straight only), 100=full differential (one motor fully cut on sharp turn). Default 50.",group:"Steering",type:"int",def:50,min:0,max:100,unit:"%"},
{key:"steering_inverted",label:"Steering Inverted",description:"0=normal steering direction, 1=inverted (left/right swapped). Use if buggy steers opposite to expectation. Default 0.",group:"Steering",type:"bool",def:0,min:0,max:1},
{key:"trim",label:"Trim",description:"Steering trim offset applied to the centre position. -500=full left trim, 0=centre, 500=full right trim. Adjust if buggy pulls left or right at neutral. Default 0.",group:"Steering",type:"int",def:0,min:-500,max:500},
{key:"pwm0_min",label:"PWM0 Min",description:"Motor 0 (main drive) minimum PWM pulse width — maps to zero throttle / idle. 500=shortest pulse, 2500=longest. Typical ESC idle is 1000-1500 µs. Default 1000.",group:"PWM",type:"int",def:1000,min:500,max:2500,unit:"us"},
{key:"pwm0_max",label:"PWM0 Max",description:"Motor 0 (main drive) maximum PWM pulse width — maps to full throttle. 500=shortest pulse, 2500=longest. Typical ESC full throttle is 1800-2000 µs. Default 2000.",group:"PWM",type:"int",def:2000,min:500,max:2500,unit:"us"},
{key:"pwm1_min",label:"PWM1 Min",description:"Motor 1 / servo minimum PWM pulse width — maps to zero or full-left position. 500=shortest pulse, 2500=longest. Default 1000.",group:"PWM",type:"int",def:1000,min:500,max:2500,unit:"us"},
{key:"pwm1_max",label:"PWM1 Max",description:"Motor 1 / servo maximum PWM pulse width — maps to full throttle or full-right position. 500=shortest pulse, 2500=longest. Default 2000.",group:"PWM",type:"int",def:2000,min:500,max:2500,unit:"us"},
{key:"failsafe_time",label:"Failsafe Time",description:"Time after the last received LoRa packet before the RX cuts motor output to zero. 100ms=very fast failsafe (noisy environment risk), 10000ms=10 second delay (dangerous — motor runs on after signal loss). Default 1000ms (1 second).",group:"Motor & Safety",type:"int",def:1000,min:100,max:10000,unit:"ms"},
{key:"motor_ramp_s",label:"Motor Ramping (secs)",description:"Seconds for a motor to ramp from 0 to full. Smooths the throttle AND prevents a single motor from taking off (throttle- or steering-driven). WARNING: this also ramps differential steering — a sharp turn builds up over this time (you can always straighten instantly; only starting a hard turn is ramped). 0=instant/off. Default 0.75s. Range 0-4s; higher=slower/smoother.",group:"Motor & Safety",type:"float",def:0.75,min:0.0,max:4.0,step:0.05,unit:"s"},
{key:"data_src",label:"Data Source",description:"0=Off (no telemetry sent to TX), 1=Analog (battery voltage via ADC on UBAT pin), 2=VESC UART (full VESC telemetry including speed, ERPM, and motor current). Default 2.",group:"Motor & Safety",type:"enum",def:2,min:0,max:2,options:[{v:0,l:"Off"},{v:1,l:"Analog"},{v:2,l:"VESC UART"}]},
{key:"vesc_timeout_s",label:"VESC Timeout",description:"Seconds without an ESC UART packet before battery % and temperature show as unavailable. Default 10s. Range 5-60s. A VESC cold restart takes about 8-9s, so the old 6s default blanked battery and temperature across every restart; 10s rides through it. Lower it toward 5s only if you would rather see N/A quickly than hold a possibly stale reading.",group:"VESC",type:"int",def:10,min:5,max:60,unit:"s"},
{key:"foil_num_cells",label:"Battery Cells",description:"Number of LiPo/Li-Ion cells in series (S-count). Used to compute cell voltage from pack voltage. 1=1S (4.2V full), 14=14S (58.8V full). Example: a 14S4P pack is 14 cells in series. Default 10.",group:"Battery",type:"int",def:10,min:1,max:50},
{key:"ubat_cal",label:"Battery Cal Factor",description:"ADC-to-voltage calibration multiplier. Multiply raw ADC reading by this factor to get pack voltage. Calibrate by measuring real voltage with a multimeter and adjusting until they match. Range 0.000001-1.0, default 0.0095554.",group:"Battery",type:"float",def:0.0095554,min:0.000001,max:1.0,step:0.000001},
{key:"ubat_offset",label:"Battery Voltage Offset",description:"Fixed voltage offset added to the calibrated ADC reading. Used to correct for resistor divider bias. -100.0V to +100.0V. Adjust in small steps (e.g. ±0.1V) until measured voltage matches multimeter. Default 0.0.",group:"Battery",type:"float",def:0.0,min:-100.0,max:100.0,step:0.0001},
{key:"bms_det_active",label:"BMS Detection",description:"0=BMS detection disabled, 1=BMS cutoff detection enabled (RX monitors BMS signal pin and shuts output if BMS trips). Default 0.",group:"Sensors",type:"bool",def:0,min:0,max:1},
{key:"wet_det_active",label:"Water Detection",description:"0=wetness detection disabled, 1=enabled (RX monitors moisture sensor and sends E71 warning to TX display with vibration alert if water ingress detected — motor output is NOT cut, user can continue riding and return to shore). Default 1.",group:"Sensors",type:"bool",def:1,min:0,max:1},
{key:"rtm_steer_response",label:"RTM/FM Steering Response Preset",description:"Steering response preset for RTM and Follow-Me heading controller. Selects P+D gain set and target-position filter time constant. Lower index = more damped (better for rough water/heavy surf). Higher index = more aggressive turn-in (better for calm/flat-water sessions). 0=Very Soft (big waves, aggressive surfer), 1=Soft (choppy water), 2=Normal (DEFAULT, mixed conditions), 3=Sharp (calm water), 4=Very Sharp (glass-flat, no waves). Pick before each session based on conditions.",group:"RTM",type:"enum",def:2,min:0,max:4,options:[{v:0,l:"0 — Very Soft (heavy surf, aggressive surfer)"},{v:1,l:"1 — Soft (choppy water)"},{v:2,l:"2 — Normal (DEFAULT, mixed conditions)"},{v:3,l:"3 — Sharp (calm water)"},{v:4,l:"4 — Very Sharp (glass-flat, no waves)"}]},
{key:"gps_en",label:"GPS Enabled",description:"0=GPS module disabled (no UART polling, all GPS-dependent features blocked), 1=GPS enabled. RTM and Follow-Me require GPS enabled. Default 1.",group:"GPS",type:"bool",def:1,min:0,max:1},
{key:"gps_update_hz",label:"GPS Update Rate",description:"How many times per second the RX drains the GPS UART for new NMEA data. 10=100ms interval (default), 5=200ms, 2=500ms. Range 1-10 Hz. Does not change the GPS module output rate — only how often the firmware reads it. The module streams continuously at 5Hz (BN-220/BN-880) or 10Hz (M10), so draining at 10Hz keeps the serial ring far below capacity; lower values let sentences pile up between reads.",group:"GPS",type:"int",def:10,min:1,max:10,unit:"Hz"},
{key:"gps_chip_type",label:"GPS Module Type",description:"GPS module type — determines init sequence, baud rate, and update rate. 0=BN-220 no compass, 1=BN-880 with compass (default for RX), 2=M10 no compass, 3=M10 with compass. Reboot required after change.",group:"GPS",type:"enum",def:1,min:0,max:3,options:[{v:0,l:"BN-220 no compass (9600→115200, 5Hz)"},{v:1,l:"BN-880 + compass (default, 9600→115200, 5Hz)"},{v:2,l:"M10 no compass (115200 direct, 10Hz, all constellations)"},{v:3,l:"M10 + compass (115200 direct, 10Hz, all constellations)"}]},
{key:"gps_dyn_model",label:"GPS Dynamic Model",description:"u-blox navigation filter platform model. 0=default (Sea) — leave this unless you are above 500 m. 4=Automotive — REQUIRED for lakes above ~500 m altitude, because the Sea model has a 500 m ceiling and fixes degrade above it. 5=Sea (explicit) — best below 500 m: it constrains the filter to ~25 m/s and pins altitude near the surface, which sharpens course-over-ground, and COG is what Follow-Me steers on. Portable (0 in u-blox terms) is deliberately NOT offered — it permits 310 m/s and is what produced bogus 254 km/h high-confidence fixes.",group:"GPS",type:"enum",def:0,min:0,max:5,options:[{v:0,l:"Default (Sea)"},{v:4,l:"Automotive (above 500 m)"},{v:5,l:"Sea (explicit)"}]},
{key:"followme_mode",label:"Follow-Me Mode",description:"Stored FM mode preference on this RX. IT IS NOT AN AUTO-ARM SOURCE: the RX arms Follow-Me only from the TX's live 0xF2 mode declaration, and an undeclared mode means IDLE — it never falls back to this value. Kept for backup/restore and documentation. 0=Disabled, 1=Near Right, 2=Behind (default), 3=Near Left, 4=In Front. In Front engages only after the buggy is already proven ahead of the rider.",group:"Follow-Me",type:"enum",def:2,min:0,max:4,options:[{v:0,l:"Disabled"},{v:1,l:"Near Right"},{v:2,l:"Behind (default)"},{v:3,l:"Near Left"},{v:4,l:"In Front"}]},
{key:"kalman_en",label:"Kalman Filter",description:"0=Kalman filter disabled (raw GPS position used), 1=Kalman filter enabled (smooths GPS position noise for follow-me and RTM). Enable for autonomous use; disable to debug raw GPS output. Default 1.",group:"GPS",type:"bool",def:1,min:0,max:1},
{key:"tx_gps_stale_timeout_ms",label:"TX GPS Stale Timeout",description:"Maximum age of TX GPS data before RX considers it stale and blocks Phase B/C anti-spoofing checks. 0=disabled (never stale), 65535=65 seconds. Default 3000ms (3 seconds). Reduce if TX GPS updates fall behind.",group:"GPS",type:"int",def:3000,min:0,max:65535,unit:"ms"},
{key:"gps_max_hdop",label:"GPS Max HDOP",description:"Maximum HDOP for a valid GPS fix — lower is stricter. Range 0.5-5.0, default 2.0. Readings above this threshold are rejected.",group:"GPS",type:"float",def:2.0,min:0.5,max:5.0,step:0.1},
{key:"gps_max_accel_g",label:"GPS Max Acceleration",description:"Maximum implied acceleration between consecutive GPS readings (G-force). Range 1.0-10.0 G, default 3.0 G. Higher-than-max implies a spoofed jump.",group:"GPS",type:"float",def:3.0,min:1.0,max:10.0,step:0.1,unit:"G"},
{key:"gps_max_teleport_kmh",label:"GPS Max Teleport Speed",description:"Maximum speed implied by position change between readings. Range 50-500 km/h, default 80 km/h. Larger implies GPS teleport.",group:"GPS",type:"float",def:80.0,min:50.0,max:500.0,step:1.0,unit:"km/h"},
{key:"gps_suspect_threshold",label:"GPS Suspect Threshold",description:"Consecutive anti-spoofing failures before GPS is marked rejected. Range 1-10, default 3. While rejected, RTM arming is blocked.",group:"GPS",type:"int",def:3,min:1,max:10},
{key:"gps_max_pair_dist_m",label:"Phase B: Max Pair Distance",description:"Maximum plausible TX-RX distance during GPS handshake check. Range 50-2000 m, default 500 m. If TX and RX are further apart than this, RTM arming is blocked.",group:"GPS",type:"float",def:500.0,min:50.0,max:2000.0,step:10.0,unit:"m"},
{key:"gps_max_speed_diff_kmh",label:"Phase B: Max Speed Difference",description:"Maximum TX-RX GPS speed difference during handshake check. Range 10-200 km/h, default 50 km/h. If speeds differ more than this, RTM arming is blocked.",group:"GPS",type:"float",def:50.0,min:10.0,max:200.0,step:1.0,unit:"km/h"},
{key:"mag_offset_x",label:"Compass Offset X",description:"Hard-iron calibration offset for X axis (raw magnetometer units). 0=no correction. Set automatically by ?compasscal — do not edit manually unless restoring a known-good backup. Survives firmware flashes; resets to 0 only on SW_VERSION change.",group:"GPS",type:"int",def:0,min:-32768,max:32767},
{key:"mag_offset_y",label:"Compass Offset Y",description:"Hard-iron calibration offset for Y axis (raw magnetometer units). 0=no correction. Set automatically by ?compasscal — do not edit manually unless restoring a known-good backup.",group:"GPS",type:"int",def:0,min:-32768,max:32767},
{key:"mag_scale_x",label:"Compass Scale X",description:"Soft-iron calibration scale for X axis. 1.0=unity gain (no correction). Set automatically by ?compasscal — do not edit manually unless restoring a known-good backup. Range -10.0 to 10.0.",group:"GPS",type:"float",def:1.0,min:-10.0,max:10.0,step:0.001},
{key:"mag_scale_y",label:"Compass Scale Y",description:"Soft-iron calibration scale for Y axis. 1.0=unity gain (no correction). Set automatically by ?compasscal — do not edit manually unless restoring a known-good backup. Range -10.0 to 10.0. A NEGATIVE value is legal and meaningful: it means the sensor frame is MIRRORED, and ?compasscal sets it automatically when it sees the heading run backwards during a clockwise turn. Range -10.0 to 10.0.",group:"GPS",type:"float",def:1.0,min:-10.0,max:10.0,step:0.001},{key:"mag_orientation",label:"Compass Mounting Rotation",description:"How the compass module is rotated on the buggy, in degrees. Measured automatically by ?compasscal - point the nose at north, turn CLOCKWISE through two full circles, finish on north. 0/90/180/270 only: the idle noise floor is about 3 degrees, so finer resolution would be false precision. A MIRRORED module is handled separately, by a negative Compass Scale Y. Getting this wrong rotates every heading by the same amount, which makes Follow-Me steer off by that angle.",group:"GPS",type:"enum",def:0,min:0,max:270,options:[{v:0,l:"0 deg"},{v:90,l:"90 deg"},{v:180,l:"180 deg"},{v:270,l:"270 deg"}]},
{key:"rtm_vesc_speed_diff_kmh",label:"Phase C: Max VESC Speed Diff",description:"Phase C: max difference between GPS speed and VESC ERPM-implied speed during active RTM. Range 5-50 km/h, default 20.",group:"RTM",type:"float",def:20.0,min:5.0,max:50.0,step:1.0,unit:"km/h"},
{key:"vesc_erpm_per_kmh",label:"VESC ERPM per km/h",description:"Vehicle-specific: how many ERPM equals 1 km/h. Set by driving at known speed and reading ERPM from ?printtasks. 0=disable Phase C VESC check.",group:"VESC",type:"float",def:0.0,min:0.0,max:9999.0,step:1.0,unit:"ERPM/kmh"},
{key:"rtm_rx_enabled",label:"RTM RX Enabled",description:"RX-side RTM master enable (safety kill switch). 0=RTM blocked regardless of TX. Default 1.",group:"RTM",type:"bool",def:1,min:0,max:1},
{key:"rtm_rx_override_steering",label:"RTM Override Steering",description:"Allow RTM to override steering towards TX GPS position. 0=disable steering override (throttle limiting only). Default 1.",group:"RTM",type:"bool",def:1,min:0,max:1},
{key:"rtm_compass_required",label:"RTM Compass Required",description:"Requires a valid HEADING before RTM will arm - not specifically a compass, despite the name. It calls the same heading logic RTM steers on and accepts any source. 1=must have a heading to arm (default, recommended). 0=allow RTM to arm without one; steering does nothing until a heading appears. You do not normally set this yourself: switch Heading Source to 0 (GPS COG only) and the firmware clears this automatically and says so, because with the compass off there is no heading while the buggy sits still and RTM could never arm.",group:"RTM",type:"bool",def:1,min:0,max:1},
{key:"rtm_stop_distance_m",label:"RTM Hard Stop Distance",description:"Distance at which RTM triggers a hard stop and disengages (Gate 9). Range 1-50 m, default 10 m. GPS accuracy floor (~5 m) makes values below 10 m unreliable. Must be less than the arm distance to allow RTM to run.",group:"RTM",type:"int",def:10,min:1,max:50,unit:"m"},
{key:"rtm_approach_zone_m",label:"RTM Approach Zone",description:"Distance from TX at which RTM begins ramping throttle down toward zero. Throttle reduces linearly from full at this outer edge to zero at the Hard Stop Distance — smooth arrival instead of sudden cut. 0=disabled (hard stop only, no ramp). Must be greater than Hard Stop Distance to have effect. Range 0-100 m, default 12 m.",group:"RTM",type:"int",def:12,min:0,max:100,unit:"m"},
{key:"rtm_use_compass",label:"RTM Heading Source",description:"Selects which heading source RTM steering uses. 0=GPS COG only — compass disabled for steering (safest if compass is biased by motor current; what bench tests on this hardware showed). 1=Hybrid (default, recommended) — GPS COG primary above the speed threshold; clean compass snapshot at low speed. 2=Compass only — DIAGNOSTIC USE, DO NOT USE ON WATER. Bench tests show motor current biases compass by 100° or more during operation. Setting this on water risks RTM steering the buggy in the wrong direction. For non-EMI builds with proven clean compass behavior under load only.",group:"RTM",type:"enum",def:1,min:0,max:2,options:[{v:0,l:"GPS COG Only"},{v:1,l:"Hybrid (Default)"},{v:2,l:"Compass Only — DIAGNOSTIC, NOT FOR WATER"}]},
{key:"rtm_cog_min_speed_kmh",label:"RTM COG Min Speed",description:"Minimum GPS speed below which COG is considered too noisy to use as primary heading. Below this threshold, RTM falls back to compass snapshot (in Hybrid mode) or holds straight (in GPS COG Only mode). Range 1-15 km/h, default 3. Lower values use COG more aggressively at slow speeds; higher values rely more on compass snapshot. Above 10 km/h is unusual.",group:"RTM",type:"int",def:3,min:1,max:15,unit:"km/h"},
{key:"rtm_align_threshold_deg",label:"RTM Align Threshold",description:"Phase 1→2 transition angle. While heading error exceeds this value, throttle is suppressed to ~5% so the buggy pivots toward you without driving away first. Once heading error drops below this threshold, the run phase engages and the GPS speed governor takes over. Range 10-90°, default 45°. Lower = buggy aligns more precisely before accelerating; higher = quicker transition to run phase.",group:"RTM",type:"int",def:45,min:10,max:90,unit:"deg"},
{key:"rtm_target_speed_kmh",label:"RTM Target Speed",description:"Phase 2 GPS speed cap. Once aligned, RTM governor limits buggy speed to this value regardless of motor or prop size — makes behaviour consistent across different boogies. 0=disabled (user throttle passes through after alignment, approach decel zone only). Range 0-20 km/h, default 4 km/h. Start at 3-5 km/h for first water tests.",group:"RTM",type:"float",def:4.0,min:0,max:20,step:0.5,unit:"km/h"},
{key:"boogie_vmax_in_followme_kmh",label:"Boogie V-Max",description:"Maximum vehicle speed while Follow-Me is active. Throttle is capped so speed never exceeds this value during following. 0=no limit, 100=100 km/h cap. Default 25 km/h. Set to a safe value for your terrain.",group:"Follow-Me",type:"float",def:25.0,min:0,max:100,step:0.1,unit:"km/h"},
{key:"min_dist_m",label:"Min Distance",description:"Minimum safe distance the buggy must maintain from the foiler. Throttle is cut if the buggy gets closer than this. 0=no minimum, 1000=1000 m minimum (impractical). Default 10 m.",group:"Follow-Me",type:"float",def:10.0,min:0,max:1000,step:0.1,unit:"m"},
{key:"followme_smoothing_band_m",label:"Smoothing Band",description:"Distance band above min_dist_m over which throttle is linearly reduced to zero as the buggy approaches the minimum distance. Larger = smoother slowdown but less responsive. Default 10 m.",group:"Follow-Me",type:"float",def:10.0,min:0,max:1000,step:0.1,unit:"m"},
{key:"foiler_low_speed_kmh",label:"Foiler Low Speed",description:"If the foiler's GPS speed drops below this value, Follow-Me throttle is cut (foiler has stopped or crashed). 0=disabled, 100=100 km/h threshold (would always cut). Default 8 km/h.",group:"Follow-Me",type:"float",def:8.0,min:0,max:100,step:0.1,unit:"km/h"},
{key:"zone_angle_enter_deg",label:"Zone Angle Enter",description:"Geometry Schmitt enter half-angle. F1/F3: below this angle from directly behind, the diagonal side offset is applied (it is not their engagement gate). F4: the buggy must be within this angle of the rider's forward course to prove or re-enter In Front. Default 35°.",group:"Follow-Me",type:"float",def:35.0,min:0,max:180,step:0.1,unit:"deg"},
{key:"zone_angle_exit_deg",label:"Zone Angle Exit",description:"Geometry Schmitt exit half-angle. F1/F3: above it, the diagonal is dropped. F4: above it, the buggy loses In Front, stops in HOLD and must prove the front position again. Keep 5-15° wider than Zone Angle Enter. Default 45°.",group:"Follow-Me",type:"float",def:45.0,min:0,max:180,step:0.1,unit:"deg"},
{key:"near_diag_offset_deg",label:"Near Diag Offset",description:"Target bearing offset from directly behind for F1 Near Right and F3 Near Left. In the firmware's clockwise bearing convention, Right uses -offset and Left uses +offset. 0=directly behind, 90=beside the rider. F4 ignores this value. Default 45°.",group:"Follow-Me",type:"float",def:45.0,min:0,max:180,step:0.1,unit:"deg"},
{key:"fm_engage_dist_m",label:"FM Engage Distance",description:"LIVE. Minimum 8 m. This is how far you must get from the buggy before Follow-Me is allowed to engage for the first time. Valid entries: 0 = automatic, or 8-50 m — nothing between 0 and 8 is accepted. WHY THE 8 m MINIMUM: it is the tow-rope safety floor. Follow-Me must never be able to engage while you are still on the rope, and an engage distance shorter than your rope would let the buggy take over steering mid-tow. HOW TO CHOOSE YOURS: measure your tow rope and set at least one metre beyond it. Example: a 20 ft (6.1 m) rope → 8 m or more. A longer rope needs a bigger number. This value IS the engage distance in metres; it is NOT the rope length. SETTING 0 (AUTOMATIC) DOES NOT BYPASS THE MINIMUM: the firmware computes 1.5 x (Min Distance + Smoothing Band) and then applies the same 8 m floor to that computed value, so automatic can never drop below 8 m either. Any older stored value below 8 m is clamped up to 8 m. Once set, you must stay beyond this distance for 2 seconds before Follow-Me can engage. LATCH RESET: a set separation proof is cleared when fresh positions keep radial distance below this effective distance and filtered rider speed below 2 km/h for 2 seconds. The automatic value is used for this reset too.",group:"Follow-Me",type:"float",def:0,min:0,max:50,step:0.5,unit:"m"},
{key:"log_level",label:"Log Detail Level",description:"How much detail each log record carries. More detail means bigger records, so the on-board storage fills up faster. 0 = Automatic (DEFAULT) — behaves exactly like level 3, and is what every unit already stores, so leaving this alone changes nothing. 1 = Basic and 2 = VESC are RESERVED for a future storage optimisation (smaller records for longer sessions) and are NOT implemented yet: they are accepted and saved, but the firmware CURRENTLY LOGS AS LEVEL 3. They are not silently ignored — they simply do not save you any space yet. 3 = Developer — the full record this firmware has always written: VESC, GPS, RTM/Follow-Me heading, the steering actually applied, distance to the remote and LoRa link quality. 59 bytes per record, which is roughly 1 h 40 min of continuous logging at 5 Hz, or about 2 h 50 min at the 3 Hz default rate. 4 = Deep — everything in Developer plus four columns that explain WHY a session went wrong: GPS sentences parsed per second, seconds since the GPS course VALUE actually changed (not just its timestamp, which kept looking fresh straight through a frozen heading), a running count of UART-mux read-back failures, and the worst main-loop time since the previous record. 65 bytes per record, roughly 1 h 30 min at 5 Hz or about 2 h 30 min at 3 Hz. Use 4 when you are chasing a fault; 0 or 3 for normal riding. Those figures assume storage is otherwise empty — the logger always keeps a 500 KB free-space reserve and deletes older log files to hold it, so a session with nothing left to delete stops about that much short. Each log FILE records the level it was captured at in its own header, so changing this mid-session is safe: it takes effect on the next log file. NOTE: this setting replaces the old RESERVED FM Steer Reposition slot, which no firmware ever read. The Follow-Me v2 repositioning feature is not cancelled — it will get a new setting of its own when it lands.",group:"Logging",type:"enum",def:0,min:0,max:4,options:[{v:0,l:"0 — Automatic (same as Developer)"},{v:1,l:"1 — Basic (reserved — currently logs as Developer)"},{v:2,l:"2 — VESC (reserved — currently logs as Developer)"},{v:3,l:"3 — Developer (full record, 59 B)"},{v:4,l:"4 — Deep (full record + diagnostics, 65 B)"}]},
{key:"logger_en",label:"Logger Enabled",description:"0=data logger disabled at boot, 1=logger starts logging immediately at boot. RECOMMENDED: leave this at 0. The RX then boots with logging off, so it never fills SPIFFS while parked or on the bench. Short-press AUX any time the RX is running to START logging (AUX LED blinks 5x) and short-press again to STOP (blinks 2x). Logging per session, on demand — you do not need this set to 1.",group:"Logging",type:"bool",def:0,min:0,max:1},
{key:"wifi_password",label:"WiFi Password",description:"AP password (exactly 8 characters)",group:"System",type:"text",def:"12345678",minLen:8,maxLen:8},
{key:"version",label:"Config Version",description:"Config-format version stored in SPIFFS. DIAGNOSTIC — do not edit: the firmware accepts only a value equal to its own SW_VERSION (35 in this build), so the single legal entry is 35. If it reads anything else the board is running a different build than you think, and its stored settings were reset on the last boot.",group:"System",type:"int",def:35,min:35,max:35}
];

const state={values:{},loaded:{},saved:{},last:'-'};
const openGroups={};

// --- REAL ESP32 API ---
async function api(url,m='GET',b=null){const o={method:m};if(b){o.headers={'Content-Type':'application/x-www-form-urlencoded'};o.body=b;}try{const r=await fetch(url,o);const j=await r.json();state.last=JSON.stringify(j);document.getElementById('last').textContent='Last: '+state.last;return j;}catch(e){const j={ok:0,err:'ERR_HTTP'};state.last=JSON.stringify(j);document.getElementById('last').textContent='Last: '+state.last;return j;}}
function normAddr(v){let s=String(v||'').replace(/[\[\]]/g,'').replace(/[:;-]/g,',').trim();const p=s.split(',').map(x=>x.trim()).filter(Boolean).slice(0,3);while(p.length<3)p.push('00');return p.map(x=>x.replace(/^0x/i,'').replace(/[^0-9a-f]/gi,'').slice(0,2)).map(x=>x.length?Number.parseInt(x,16):0).map(n=>Number.isNaN(n)?0:Math.max(0,Math.min(255,n)));}
function toAddrHex(v){return normAddr(v).map(n=>n.toString(16).toUpperCase().padStart(2,'0'));}
function valueForSend(f,v){if(f.type==='address3'){const h=toAddrHex(v);return `${h[0]}:${h[1]}:${h[2]}`;}return String(v).trim();}
function canonValue(f,v){if(f.type==='address3')return toAddrHex(v).join(',');return String(v??'').trim();}

function hasUnsavedChanges(){
    for(const f of fields){
        if(canonValue(f,state.values[f.key])!==canonValue(f,state.saved[f.key])) return true;
    }
    return false;
}

function checkDirtyUI() {
    const hasChanges = hasUnsavedChanges();
    const btn = document.getElementById('saveBtn');
    
    if(hasChanges) {
        btn.classList.add('active-save');
        btn.innerText = '⚠️ Save Required';
    } else {
        btn.classList.remove('active-save');
        btn.innerText = 'Save All';
    }

    document.querySelectorAll('[data-key]').forEach(el => {
        const k = el.getAttribute('data-key');
        const f = fields.find(x => x.key === k);
        if(f) {
            if(canonValue(f, state.values[k]) !== canonValue(f, state.saved[k])) {
                el.classList.add('dirty');
            } else {
                el.classList.remove('dirty');
            }
        }
    });
}

function validate(f,v){const s=String(v??'').trim();if(!s.length)return'Required';if(f.type==='address3')return null;if(f.type==='float'){const n=parseFloat(s);if(Number.isNaN(n))return'Must be float';if(n<f.min||n>f.max)return`Range ${f.min}..${f.max}`;return null;}const n=parseInt(s,10);if(Number.isNaN(n))return'Must be integer';if(n<f.min||n>f.max)return`Range ${f.min}..${f.max}`;return null;}

// --- JSON SYNC ENGINE ---
function syncJsonBox() {
    let exportObj = {};
    for (const f of fields) {
        exportObj[f.key] = state.values[f.key];
    }
    document.getElementById('jsonBox').value = JSON.stringify(exportObj, null, 2);
}

function setVal(k,v){state.values[k]=String(v); updateHint(k); checkDirtyUI(); syncJsonBox();}
function setBool(k,c){setVal(k,c?'1':'0');}
function setEnum(k,v){setVal(k,v);}
function syncField(k,v){document.querySelectorAll(`[data-key='${k}']`).forEach(el=>{if(el!==document.activeElement)el.value=v;});}
function setAddrPart(k,i,v){const cur=toAddrHex(state.values[k]);const c=String(v||'').toUpperCase().replace(/[^0-9A-F]/g,'').slice(0,2);cur[i]=c;state.values[k]=cur.join(','); checkDirtyUI(); syncJsonBox();}
function updateHint(k){const f=fields.find(x=>x.key===k);if(!f)return;const err=validate(f,state.values[k]);const el=document.getElementById(`hint-${k}`);if(!el)return;const hint=`Range ${f.min}..${f.max}${f.unit?(' '+f.unit):''} | default: ${f.def}`;el.textContent=err||hint;el.className=err?'err':'hint';}

// --- JSON BUTTON FUNCTIONS ---
// V2.5-Evo - 2026-08-18 - CLIP-1. navigator.clipboard exists ONLY in a secure context: HTTPS,
// or localhost. This page is served by the board's own access point over plain
// http://192.168.4.1, which is neither - so on a phone connected to the remote, the API is
// either undefined (the call throws, the button does nothing at all) or present but rejecting
// (the promise fails unhandled and the button cheerfully says "Copied!" while the clipboard
// stays empty). The second is worse: the rider believes they have their config backed up.
// This is the ONLY way the page is ever reached in the field, so the button never worked there.
// It works off a local file, which is presumably how it came to be believed working.
// execCommand('copy') is deprecated but is not secure-context gated and still works on iOS
// Safari and Android Chrome over plain HTTP. Try the modern API first, fall back, and only
// claim success when a copy actually happened.
function legacyCopy(text) {
    const ta = document.createElement('textarea');
    ta.value = text;
    // Off-screen rather than hidden: iOS Safari will not select from a display:none element,
    // and readOnly stops it popping the on-screen keyboard.
    ta.style.position = 'fixed';
    ta.style.top = '-1000px';
    ta.setAttribute('readonly', '');
    document.body.appendChild(ta);
    let ok = false;
    try {
        ta.select();
        ta.setSelectionRange(0, ta.value.length);   // iOS needs the explicit range
        ok = document.execCommand('copy');
    } catch (e) { ok = false; }
    document.body.removeChild(ta);
    return ok;
}

function copyFeedback(worked) {
    const btn = document.getElementById('btnCopy');
    if (!btn) return;
    btn.innerText = worked ? "Copied!" : "Copy failed - select and copy by hand";
    if (worked) btn.classList.add('success');
    setTimeout(() => { btn.innerText = "Copy to Clipboard"; btn.classList.remove('success'); }, 2500);
}

function copyJson() {
    const text = document.getElementById('jsonBox').value;
    if (navigator.clipboard && window.isSecureContext) {
        navigator.clipboard.writeText(text)
            .then(() => copyFeedback(true))
            .catch(() => copyFeedback(legacyCopy(text)));   // never leave the failure unhandled
        return;
    }
    copyFeedback(legacyCopy(text));
}

function exportJsonFile() {
    window.location.href = '/api/config/export?format=json';
}

function importJsonFile(input) {
    const file = input.files[0];
    if(!file) return;
    const reader = new FileReader();
    reader.onload = function(e) {
        document.getElementById('jsonBox').value = e.target.result;
        loadFromJsonText(); 
        input.value = ''; 
    };
    reader.readAsText(file);
}

function loadFromJsonText() {
    try {
        const rawJson = JSON.parse(document.getElementById('jsonBox').value);
        const parsedData = rawJson.data || rawJson; 
        
        let count = 0;
        for (const f of fields) {
            if (parsedData[f.key] !== undefined) {
                state.values[f.key] = String(parsedData[f.key]);
                count++;
            }
        }
        render();
        checkDirtyUI();
        syncJsonBox(); 
        alert(`Successfully mapped ${count} values from JSON. \n\nPlease review the RED boxes, then click 'Save All' to apply them to the hardware.`);
    } catch(e) {
        alert("Error: Invalid JSON format. Make sure the brackets {} and quotes are correct.");
    }
}

// --- STANDARD API ACTIONS ---
async function saveAll(){
    const btn = document.getElementById('saveBtn');
    btn.innerText = 'Saving...';

    // V2.5-Evo - 2026-07-21 - FIX-WEB-1 (ported from TX): only validate and send DIRTY fields.
    // Previous behavior validated and sent ALL fields on every save. Any field with a stale or
    // unreadable value in state.values — notably the compass mag_* fields, which were orphaned
    // from /api/config until this build — failed validate() with "Required" and blocked the
    // entire save with a misleading alert (the owner's "compass deviation 0"). Restrict the loop
    // to fields the user actually changed since the last refresh (canonValue matches checkDirtyUI),
    // so untouched/edge-case fields can never block a legitimate save.
    const dirty = fields.filter(f => canonValue(f, state.values[f.key]) !== canonValue(f, state.saved[f.key]));

    if (dirty.length === 0) {
        // Nothing to save — surface this instead of silently doing nothing.
        btn.classList.add('success');
        btn.innerText = 'Nothing to save';
        setTimeout(() => { btn.classList.remove('success'); btn.innerText = 'Save All'; checkDirtyUI(); }, 1500);
        return;
    }

    // Validate dirty fields only.
    for(const f of dirty){
        const e = validate(f, state.values[f.key]);
        if(e){ alert(`${f.label}: ${e}`); checkDirtyUI(); btn.innerText='Save All'; return; }
    }

    // Send dirty fields only.
    for(const f of dirty){
        const body = `key=${encodeURIComponent(f.key)}&value=${encodeURIComponent(valueForSend(f, state.values[f.key]))}`;
        const r = await api('/api/set','POST', body);
        if(!r.ok){ alert(`SET ${f.key} failed: ${r.err||'ERR'}`); checkDirtyUI(); btn.innerText='Save All'; return; }
    }

    // Final commit to SPIFFS.
    const s = await api('/api/save','POST');
    if(!s.ok){ alert(s.err||'SAVE failed'); checkDirtyUI(); btn.innerText='Save All'; return; }

    await refreshAll();

    btn.classList.add('success');
    btn.innerText = `Saved OK (${dirty.length})`;
    setTimeout(() => { btn.classList.remove('success'); btn.innerText = 'Save All'; checkDirtyUI(); }, 1500);
}

async function loadCfg(){
    const btn = document.getElementById('loadBtn');
    btn.innerText = 'Syncing...';
    
    await api('/api/load','POST');
    await refreshAll();
    
    btn.classList.add('success');
    btn.innerText = 'Synced OK';
    setTimeout(() => { btn.classList.remove('success'); btn.innerText = 'Force Sync'; }, 1500);
}

function ctrlHtml(f){const v=state.values[f.key]??String(f.def);if(f.type==='bool'){const on=String(v).trim()==='1';return `<label class='row'><input type='checkbox' data-key='${f.key}' ${on?'checked':''} onchange="setBool('${f.key}',this.checked)"><span>${on?'Enabled':'Disabled'}</span></label>`;}if(f.type==='enum'){const opts=f.options.map(o=>`<option value='${o.v}' ${String(v)===String(o.v)?'selected':''}>${o.l}</option>`).join('');return `<select data-key='${f.key}' onchange="setEnum('${f.key}',this.value)">${opts}</select>`;}if(f.type==='address3'){const p=toAddrHex(v);return `<div class='triple'><input value='${p[0]}' maxlength='2' oninput="setAddrPart('${f.key}',0,this.value)"><input value='${p[1]}' maxlength='2' oninput="setAddrPart('${f.key}',1,this.value)"><input value='${p[2]}' maxlength='2' oninput="setAddrPart('${f.key}',2,this.value)"></div>`;}const step=f.type==='float'?(f.step||0.000001):1;const n=parseFloat(v);const showSlider=f.type==='int'&&(f.max-f.min)<=1000;const slider=showSlider?`<input type='range' data-key='${f.key}' min='${f.min}' max='${f.max}' step='1' value='${Number.isNaN(n)?f.min:Math.max(f.min,Math.min(f.max,n))}' oninput="setVal('${f.key}',this.value);syncField('${f.key}',this.value)">`:'';return `<input type='number' data-key='${f.key}' step='${step}' min='${f.min}' max='${f.max}' value='${v}' oninput="setVal('${f.key}',this.value);syncField('${f.key}',this.value)">${slider}`;}

function render(){
  document.querySelectorAll('.groups details').forEach(d=>{openGroups[d.dataset.group]=d.open;});
  const gEl=document.getElementById('groups');
  const loadedCount=fields.filter(f=>state.loaded[f.key]).length;
  document.getElementById('loaded').textContent=`Loaded ${loadedCount}/${fields.length}${hasUnsavedChanges()?' | Unsaved changes':''}`;
  const byGroup={};for(const g of groupOrder)byGroup[g]=[];for(const f of fields){if(!byGroup[f.group])byGroup[f.group]=[];byGroup[f.group].push(f);}
  gEl.innerHTML=groupOrder.map((g,gi)=>{
    const items=(byGroup[g]||[]).map(f=>{
      const v=state.values[f.key]??String(f.def);const err=validate(f,v);const hint=`Range ${f.min}..${f.max}${f.unit?(' '+f.unit):''} | default: ${f.def}`;
      return `<div class='field'><div class='label'>${f.label}</div><div class='desc'>${f.description}</div><div style='margin:8px 0'>${ctrlHtml(f)}</div><div id='hint-${f.key}' class='${err?'err':'hint'}'>${err||hint}</div></div>`;
    }).join('');
    // TRUE forces all folders to be open by default
    const isOpen = Object.prototype.hasOwnProperty.call(openGroups,g)?openGroups[g]:true;
    return `<details data-group='${g}' ${isOpen?'open':''}><summary>${g}</summary><div class='items'>${items}</div></details>`;
  }).join('');
}

async function refreshAll(){
  const s=await api('/api/state');
  document.getElementById('status').textContent=s.state||JSON.stringify(s);
  const c=await api('/api/config');
  if(c.ok&&c.data){
    for(const f of fields){
      if(Object.prototype.hasOwnProperty.call(c.data,f.key)){
        const raw=c.data[f.key];
        const val=Array.isArray(raw)?raw.join(','):String(raw);
        state.values[f.key]=val;
        state.saved[f.key]=val;
        state.loaded[f.key]=true;
      }
    }
  }
  render();
  checkDirtyUI();
  syncJsonBox(); // Load fresh data into the text box
}

async function rebootDev(){if(hasUnsavedChanges()){const ignore=confirm("Unsaved config changes detected. Press OK to ignore and reboot.");if(!ignore)return;}const b=document.querySelector('[onclick="rebootDev()"]');if(b){b.textContent='Rebooting…';b.classList.add('active-save');b.disabled=true;}await api('/api/reboot','POST');}
function expandAll(){document.querySelectorAll('.groups details').forEach(d=>{d.open=true;openGroups[d.dataset.group]=true;});}
function collapseAll(){document.querySelectorAll('.groups details').forEach(d=>{d.open=false;openGroups[d.dataset.group]=false;});}

// --- LOG MANAGEMENT ---
async function openLogs(){
  const m = document.getElementById('logModal');
  const l = document.getElementById('logList');
  m.style.display='flex';
  l.innerHTML='<div class="sub">Loading...</div>';
  const res = await api('/api/logs/list');
  if(res.ok){
    if(res.logs.length===0){
      l.innerHTML='<div class="sub">No logs found on device.</div>';
      return;
    }
    state.logs=res.logs;
    let h='<div class="sub" style="margin-bottom:6px">Sizes are on-device binary. CSV downloads are ~2× larger.</div><div style="margin-bottom:8px;display:flex;gap:8px"><button class="btn warn" onclick="deleteAllLogs()">Delete All</button><button class="btn warn" onclick="deleteSelected()">Delete Selected</button></div>';
    res.logs.forEach(x=>{
      const kb=(x.size/1024).toFixed(1);
      h+=`<div class="log-item"><input type="checkbox" class="log-check" data-name="${x.name}"><div class="log-info"><span class="log-name">${x.name}</span><span class="log-size">${kb} KB raw</span></div><div class="log-actions"><a class="btn" href="/api/logs/download?file=${x.name}" target="_blank" style="text-decoration:none;font-size:10px;padding:2px 6px">CSV</a><button class="btn warn" style="font-size:10px;padding:2px 6px" onclick="deleteLog('${x.name}')">Del</button></div></div>`;
    });
    l.innerHTML=h;
  }else{
    l.innerHTML='<div class="err">Failed to fetch logs.</div>';
  }
}
async function deleteLog(fname){
  if(!confirm('Delete '+fname+'?'))return;
  const res = await api('/api/logs/delete?file='+fname,'POST');
  if(res.ok) openLogs();
  else alert('Delete failed');
}
async function deleteAllLogs(){
  if(!confirm('Delete ALL log files? This cannot be undone.'))return;
  for(const f of state.logs||[]){
    await fetch('/api/logs/delete?file='+encodeURIComponent(f.name),{method:'POST'});
  }
  openLogs();
}
async function deleteSelected(){
  const checked=[...document.querySelectorAll('.log-check:checked')].map(el=>el.dataset.name);
  if(!checked.length)return;
  if(!confirm('Delete '+checked.length+' selected log(s)?'))return;
  for(const f of checked){
    await fetch('/api/logs/delete?file='+encodeURIComponent(f),{method:'POST'});
  }
  openLogs();
}

window.setVal=setVal;window.setBool=setBool;window.setEnum=setEnum;window.syncField=syncField;window.setAddrPart=setAddrPart;window.saveAll=saveAll;window.loadCfg=loadCfg;window.rebootDev=rebootDev;window.refreshAll=refreshAll;window.openLogs=openLogs;window.deleteLog=deleteLog;window.deleteAllLogs=deleteAllLogs;window.deleteSelected=deleteSelected;window.copyJson=copyJson;window.exportJsonFile=exportJsonFile;window.importJsonFile=importJsonFile;window.loadFromJsonText=loadFromJsonText;window.expandAll=expandAll;window.collapseAll=collapseAll;
refreshAll();
</script>
</body>
</html>
)HTML";

static const size_t WEB_UI_INDEX_HTML_LEN = sizeof(WEB_UI_INDEX_HTML) - 1;

#endif
