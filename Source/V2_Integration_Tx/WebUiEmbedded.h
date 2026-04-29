// V3 - 2026-04-25 - P7: Added 12 RTM/FM fields; added RTM & Follow-Me group; sizeof TX confStruct 96→120
// V2.5-Evo - 2026-04-28 - ChangeA/F: fm_arm_window max 60→120s; followme_mode labels updated, option 0 removed
// V2.5-Evo - 2026-04-29 - TaskB: full description audit — bool 0/1 values, enum all options inline, int/float extremes explained
#ifndef WEB_UI_EMBEDDED_H
#define WEB_UI_EMBEDDED_H

#include <Arduino.h>

static const char WEB_UI_INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>BREmote V2 TX Web Config</title>
  <style>
    :root{--bg:#0b1220;--panel:#101828;--panel2:#1e293b;--txt:#e5e7eb;--muted:#9ca3af;--pri:#60a5fa;--err:#ef4444}
    *{box-sizing:border-box}
    body{margin:0;background:radial-gradient(1200px 700px at 10% -10%,#1e3a5f66 0,transparent 45%),linear-gradient(180deg,#0a1120 0,#0b1220 40%);color:var(--txt);font-family:"Avenir Next","Montserrat","Segoe UI",sans-serif; padding-top: 15px;}
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
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div class="card">
        <div class="row sp">
          <div>
            <div class="title">BREmote V2.5-Evo TX Config</div>
            <div class="sub" id="status">Loading...</div>
            <div class="sub" id="loaded">Loaded 0/0</div>
          </div>
          <div class="row">
            <button class="btn sec" id="loadBtn" onclick="loadCfg()">Force Sync</button>
            <button class="btn" id="saveBtn" onclick="saveAll()">Save All</button>
            <button class="btn warn" onclick="rebootDev()">Reboot TX</button>
          </div>
        </div>
      </div>
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

<script>
// ALL 59 TX PARAMETERS (updated 2026-04-28 P9: +1 new field dist_unit)
const groupOrder=["Radio","Gears","Throttle","Steering","Toggle","Lock & Timing","Calibration","GPS & Follow-Me","RTM & Follow-Me","System"];
const fields=[
{key:"radio_preset",label:"Radio Preset",description:"Radio frequency band. 1=EU 868 MHz, 2=US/AU 915 MHz. Do not select 3 — it causes a boot error and the remote will not start.",group:"Radio",type:"enum",def:1,min:1,max:3,options:[{v:1,l:"EU868"},{v:2,l:"US/AU915"},{v:3,l:"Reserved"}]},
{key:"rf_power",label:"RF Power",description:"LoRa transmit power in dBm. -9=minimum range (bench testing only), 22=maximum range. Higher values draw more current.",group:"Radio",type:"int",def:0,min:-9,max:22,unit:"dBm"},
{key:"paired",label:"Paired",description:"0=not paired (no RX address stored), 1=paired. Set automatically by the pairing sequence — do not edit manually.",group:"Radio",type:"bool",def:0,min:0,max:1},
{key:"own_address",label:"Own Address",description:"This TX device's LoRa address. Set automatically by the pairing sequence — do not edit manually.",group:"Radio",type:"address3",def:"00,00,00"},
{key:"dest_address",label:"Dest Address",description:"Paired RX device address. Set automatically by the pairing sequence — do not edit manually.",group:"Radio",type:"address3",def:"00,00,00"},
{key:"max_gears",label:"Max Gears",description:"Total throttle gears available. 1=no gear system (single fixed power level), 10=maximum. startgear must be less than this value.",group:"Gears",type:"int",def:10,min:1,max:10},
{key:"startgear",label:"Start Gear",description:"Gear selected on power-on or after unlock. 0=gear 1 (lowest power). Must be less than max_gears.",group:"Gears",type:"int",def:0,min:0,max:9},
{key:"throttle_mode",label:"Throttle Mode",description:"0=Gears, 1=No Gears, 2=Dynamic Cap",group:"Gears",type:"enum",def:0,min:0,max:2,options:[{v:0,l:"Gears"},{v:1,l:"No Gears"},{v:2,l:"Dynamic Cap"}]},
{key:"dynamic_power_start",label:"Dynamic Power Start %",description:"Starting power cap in Dynamic Cap mode (throttle_mode=2). 10%=very conservative start, 100%=full power from the beginning.",group:"Gears",type:"int",def:85,min:10,max:100,unit:"%"},
{key:"dynamic_power_step",label:"Dynamic Power Step %",description:"Power cap change per toggle press in Dynamic Cap mode (throttle_mode=2). 1%=finest control, 25%=coarsest.",group:"Gears",type:"int",def:5,min:1,max:25,unit:"%"},
{key:"gear_change_waittime",label:"Gear Change Wait",description:"Toggle hold duration to register a gear change. 0=instant change, 65535=very long hold required. Milliseconds.",group:"Gears",type:"int",def:100,min:0,max:65535,unit:"ms"},
{key:"gear_display_time",label:"Gear Display Time",description:"How long the gear number flashes after a change. 0=no display, higher=longer flash. Milliseconds.",group:"Gears",type:"int",def:1000,min:0,max:65535,unit:"ms"},
{key:"thr_expo",label:"Throttle Expo",description:"Throttle response curve. 0=maximum exponential (gentle at low throttle, aggressive at high), 50=linear, 100=reverse exponential. Most riders prefer 30-50.",group:"Throttle",type:"int",def:50,min:0,max:100},
{key:"thr_expo1",label:"Throttle Expo 1",description:"Reserved/unused",group:"Throttle",type:"int",def:0,min:0,max:65535},
{key:"steer_enabled",label:"Steering Enabled",description:"0=steering disabled (toggle input controls gears/cap only), 1=steering enabled (toggle also steers vehicle).",group:"Steering",type:"bool",def:1,min:0,max:1},
{key:"steer_expo",label:"Steering Expo",description:"Reserved/unused",group:"Steering",type:"int",def:50,min:0,max:65535},
{key:"steer_expo1",label:"Steering Expo 1",description:"Reserved/unused",group:"Steering",type:"int",def:0,min:0,max:65535},
{key:"tog_deadzone",label:"Toggle Deadzone",description:"Raw ADC deadzone around center position. 100=very sensitive (small movements register), 3000=insensitive. Increase if false inputs occur while riding.",group:"Toggle",type:"int",def:500,min:100,max:3000},
{key:"tog_diff",label:"Toggle Diff",description:"Minimum ADC movement required to count as input. 1=most sensitive (may pick up electrical noise), 200=least sensitive.",group:"Toggle",type:"int",def:30,min:1,max:200},
{key:"tog_block_time",label:"Toggle Block Time",description:"After a steering input, block gear-change inputs for this long. 0=no block, 5000=5 second block. Milliseconds.",group:"Toggle",type:"int",def:500,min:0,max:5000},
{key:"no_lock",label:"No Lock",description:"0=boot locked (must squeeze trigger to unlock before riding), 1=boot unlocked (no unlock step). Default 0 — use 1 with caution.",group:"Lock & Timing",type:"bool",def:0,min:0,max:1},
{key:"trig_unlock_timeout",label:"Trigger Unlock Timeout",description:"Time allowed to complete the trigger unlock gesture. 0=very short window, 65535=very long squeeze allowed. Milliseconds.",group:"Lock & Timing",type:"int",def:5000,min:0,max:65535,unit:"ms"},
{key:"lock_waittime",label:"Lock Wait Time",description:"Toggle hold duration to trigger power-off. 0=instant power-off, 65535=effectively disabled. Milliseconds.",group:"Lock & Timing",type:"int",def:2000,min:0,max:65535,unit:"ms"},
{key:"menu_timeout",label:"Menu Timeout",description:"Delay after menu exit before steering re-enables. 0=immediate re-engage, 1000=1 second delay.",group:"Lock & Timing",type:"int",def:10,min:0,max:1000},
{key:"err_delete_time",label:"Error Delete Time",description:"How long an error code stays on display before auto-clearing. 0=clears instantly, 65535=persists until reboot. Milliseconds.",group:"Lock & Timing",type:"int",def:2000,min:0,max:65535},
{key:"cal_ok",label:"Calibration OK",description:"0=throttle not calibrated (trigger output may be incorrect — calibrate before riding), 1=calibrated. Set automatically by the calibration sequence.",group:"Calibration",type:"bool",def:0,min:0,max:1},
{key:"cal_offset",label:"Cal Offset",description:"Internal ADC calibration offset. Set automatically during calibration — do not edit manually.",group:"Calibration",type:"int",def:100,min:0,max:65535},
{key:"thr_idle",label:"Throttle Idle",description:"ADC reading at trigger idle (not squeezed). Set automatically during calibration — do not edit manually.",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"thr_pull",label:"Throttle Pull",description:"ADC reading at trigger fully squeezed. Set automatically during calibration — do not edit manually.",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"tog_left",label:"Toggle Left",description:"ADC reading at toggle fully left. Set automatically during calibration — do not edit manually.",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"tog_mid",label:"Toggle Mid",description:"ADC reading at toggle center. Set automatically during calibration — do not edit manually.",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"tog_right",label:"Toggle Right",description:"ADC reading at toggle fully right. Set automatically during calibration — do not edit manually.",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"ubat_cal",label:"Battery Cal Factor",description:"Multiplier to convert raw ADC to battery voltage. Adjust (tiny value, default ≈0.000186) until displayed voltage matches a multimeter reading.",group:"Calibration",type:"float",def:0.000185662,min:0.000001,max:1.0,step:0.000001},
{key:"gps_en",label:"GPS Enabled",description:"0=GPS disabled (RTM and Follow-Me unavailable), 1=GPS active. Requires GPS module physically installed.",group:"GPS & Follow-Me",type:"bool",def:0,min:0,max:1},
{key:"followme_mode",label:"Follow-Me Starting Mode",description:"FM starting mode loaded on first arm this session. RAM override only — never written back to SPIFFS. 1=Near Right (default), 2=Behind, 3=Near Left.",group:"GPS & Follow-Me",type:"enum",def:1,min:1,max:3,options:[{v:1,l:"Near Right (default)"},{v:2,l:"Behind"},{v:3,l:"Near Left"}]},
{key:"kalman_en",label:"Kalman Filter",description:"0=raw GPS coordinates used directly, 1=Kalman filter smooths GPS jitter (recommended when GPS is noisy).",group:"GPS & Follow-Me",type:"bool",def:0,min:0,max:1},
{key:"speed_src",label:"Speed Source",description:"Speed value shown in SP display mode. 0=RX GPS km/h, 1=RX GPS knots, 2=TX GPS km/h, 3=TX GPS knots, 4=RX GPS mph, 5=TX GPS mph.",group:"GPS & Follow-Me",type:"enum",def:0,min:0,max:5,options:[{v:0,l:"GPS RX km/h"},{v:1,l:"GPS RX knots"},{v:2,l:"GPS TX km/h"},{v:3,l:"GPS TX knots"},{v:4,l:"GPS RX mph"},{v:5,l:"GPS TX mph"}]},
{key:"tx_gps_stale_timeout_ms",label:"TX GPS Stale Timeout",description:"How long a TX GPS fix is valid before considered stale. 0=never expires, 65535=very lenient. Lower values = stricter RTM GPS safety. Milliseconds.",group:"GPS & Follow-Me",type:"int",def:1000,min:0,max:65535,unit:"ms"},
{key:"gps_max_hdop",label:"TX GPS Max HDOP",description:"Max GPS Signal Noise (HDOP×100): 150=excellent, 200=good (default), 300=lenient. Lower = stricter filter.",group:"GPS & Follow-Me",type:"int",def:200,min:50,max:500},
{key:"gps_chip_type",label:"GPS Module Type",description:"GPS module type — determines baud/rate/constellation init sequence. TX supports 0 and 2 only (no compass on TX hardware).",group:"GPS & Follow-Me",type:"enum",def:0,min:0,max:3,options:[{v:0,l:"BN-220 (default, 9600→11520​0, 5Hz)"},{v:2,l:"M10 no compass (115200 direct, 10Hz, all constellations)"}]},
{key:"rtm_enabled",label:"RTM Enabled",description:"Master enable for Return-to-Me feature. 0=off, 1=on.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_hold_duration_s",label:"RTM Arm Hold Time",description:"Gesture hold duration for RTM arming. Currently fixed at 5s in firmware — this field is reserved for a future configurable threshold.",group:"RTM & Follow-Me",type:"int",def:5,min:4,max:10,unit:"s"},
{key:"rtm_arm_window_s",label:"RTM Arm Window",description:"Time to complete throttle squeeze(s) after the RTM arm gesture. 5s=very tight window (hard to complete), 30s=generous. Default 10.",group:"RTM & Follow-Me",type:"int",def:10,min:5,max:30,unit:"s"},
{key:"rtm_double_squeeze_en",label:"RTM Double-Squeeze",description:"1=require two squeezes to engage (default), 0=hold throttle >30% for 500ms.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_throttle_start_pct",label:"RTM Start Throttle %",description:"Throttle cap at RTM engage moment. 10%=very gentle start, 50%=aggressive. Ramps to rtm_throttle_max_pct over rtm_ramp_duration_s.",group:"RTM & Follow-Me",type:"int",def:30,min:10,max:50,unit:"%"},
{key:"rtm_throttle_max_pct",label:"RTM Max Throttle %",description:"Throttle cap after the ramp completes. 30%=slow approach, 90%=fast approach. User throttle is limited to this cap during RTM.",group:"RTM & Follow-Me",type:"int",def:70,min:30,max:90,unit:"%"},
{key:"rtm_ramp_duration_s",label:"RTM Ramp Duration",description:"Time to ramp from rtm_throttle_start_pct to rtm_throttle_max_pct. 2s=quick ramp, 15s=very gradual. Default 5.",group:"RTM & Follow-Me",type:"int",def:5,min:2,max:15,unit:"s"},
{key:"rtm_disengage_distance_m",label:"RTM Disengage Distance",description:"Distance at which RTM stops and motor cuts to 0. 3m=stops very close to user, 20m=stops well before reaching user. Allow enough for braking distance. Default 10.",group:"RTM & Follow-Me",type:"int",def:10,min:3,max:20,unit:"m"},
{key:"rtm_max_runtime_s",label:"RTM Max Runtime",description:"Maximum continuous RTM runtime before auto-disengage. 0=disabled (safety gates handle all real scenarios), 300s=5 minute hard cap. Default 0.",group:"RTM & Follow-Me",type:"int",def:0,min:0,max:300,unit:"s"},
{key:"rtm_gps_timeout_ms",label:"RTM GPS Timeout",description:"How long TX GPS can be lost before RTM safety-stops. 500ms=very strict, 3000ms=lenient (handles brief outages). Default 2000.",group:"RTM & Follow-Me",type:"int",def:2000,min:500,max:3000,unit:"ms"},
{key:"fm_hold_duration_s",label:"FM Hold Time",description:"Gesture hold duration for Follow-Me arming. Currently fixed at 5s in firmware — this field is reserved for a future configurable threshold.",group:"RTM & Follow-Me",type:"int",def:5,min:4,max:10,unit:"s"},
{key:"fm_override_enabled",label:"FM Override Enabled",description:"Allow TX to override RX follow-me mode at runtime (no SPIFFS write). 0=off, 1=on.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_display_mode",label:"RTM/FM Display Mode",description:"Info shown on TX display while RTM or FM is active. 0=distance to TX (default), 1=speed, 2=alternating every 2.5s.",group:"RTM & Follow-Me",type:"enum",def:0,min:0,max:2,options:[{v:0,l:"Distance (default)"},{v:1,l:"Speed"},{v:2,l:"Alternating 2.5s"}]},
{key:"fm_warn_distance_m",label:"FM Warning Distance (not yet implemented)",description:"[NOT YET IMPLEMENTED] Reserved for future use: TX-to-RX distance that will trigger proximity warning vibration in Follow-Me mode. Setting this value has no effect in the current firmware. Default 150.",group:"RTM & Follow-Me",type:"int",def:150,min:50,max:1000,unit:"m"},
{key:"rtm_steer_exit_on_input",label:"RTM Exit on Steering",description:"1=any significant steering input exits RTM immediately (default). 0=steering used for correction only (blend mode).",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"fm_arm_window_s",label:"FM Arm Window",description:"How long FM stays armed before silently auto-disarming if no throttle input. 10s=short window, 120s=2 minute window. No display or haptic on expiry. Default 30.",group:"RTM & Follow-Me",type:"int",def:30,min:10,max:120,unit:"s"},
{key:"dist_unit",label:"Distance Unit",description:"Unit for all distance readouts on the TX display (RTM, FM). Internal math always uses metres. 0=Metres, 1=Feet/miles.",group:"RTM & Follow-Me",type:"enum",def:0,min:0,max:1,options:[{v:0,l:"Metres"},{v:1,l:"Feet"}]},
{key:"wifi_password",label:"WiFi Password",description:"AP password (exactly 8 characters)",group:"System",type:"text",def:"12345678",minLen:8,maxLen:8},
{key:"version",label:"Config Version",description:"Must match firmware SW_VERSION",group:"System",type:"int",def:25,min:0,max:65535}
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

function validate(f,v){const s=String(v??'').trim();if(!s.length)return'Required';if(f.type==='address3')return null;if(f.type==='float'){const n=parseFloat(s);if(Number.isNaN(n))return'Must be float';if(n<f.min||n>f.max)return`Range ${f.min}..${f.max}`;return null;}const n=parseInt(s,10);if(Number.isNaN(n))return'Must be integer';if(n<f.min||n>f.max)return`Range ${f.min}..${f.max}`;if(f.key==='startgear'){const mg=parseInt(state.values.max_gears??'10',10);if(!Number.isNaN(mg)&&n>=mg)return'Must be < max_gears';}return null;}

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
function copyJson() {
    navigator.clipboard.writeText(document.getElementById('jsonBox').value);
    const btn = document.getElementById('btnCopy');
    btn.innerText = "Copied!";
    btn.classList.add('success');
    setTimeout(() => { btn.innerText = "Copy to Clipboard"; btn.classList.remove('success'); }, 1500);
}

function exportJsonFile() {
    const blob = new Blob([document.getElementById('jsonBox').value], {type:'application/json'});
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'bremote_tx_backup.json';
    a.click();
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
    
    for(const f of fields){const e=validate(f,state.values[f.key]);if(e){alert(`${f.label}: ${e}`); checkDirtyUI(); return;}}
    for(const f of fields){const body=`key=${encodeURIComponent(f.key)}&value=${encodeURIComponent(valueForSend(f,state.values[f.key]))}`;const r=await api('/api/set','POST',body);if(!r.ok){alert(`SET ${f.key} failed: ${r.err||'ERR'}`); checkDirtyUI(); return;}}
    const s=await api('/api/save','POST');if(!s.ok){alert(s.err||'SAVE failed'); checkDirtyUI(); return;}
    await refreshAll();
    
    btn.classList.add('success');
    btn.innerText = 'Saved OK';
    setTimeout(() => { btn.classList.remove('success'); checkDirtyUI(); }, 1500);
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

async function rebootDev(){if(hasUnsavedChanges()){const ignore=confirm("Unsaved config changes detected. Press OK to ignore and reboot.");if(!ignore)return;}await api('/api/reboot','POST');}

window.setVal=setVal;window.setBool=setBool;window.setEnum=setEnum;window.syncField=syncField;window.setAddrPart=setAddrPart;window.saveAll=saveAll;window.loadCfg=loadCfg;window.refreshAll=refreshAll;window.copyJson=copyJson;window.exportJsonFile=exportJsonFile;window.importJsonFile=importJsonFile;window.loadFromJsonText=loadFromJsonText;window.rebootDev=rebootDev;
refreshAll();
</script>
</body>
</html>
)HTML";

static const size_t WEB_UI_INDEX_HTML_LEN = sizeof(WEB_UI_INDEX_HTML) - 1;

#endif