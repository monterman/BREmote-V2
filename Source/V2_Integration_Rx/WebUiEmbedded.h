#ifndef WEB_UI_EMBEDDED_H
#define WEB_UI_EMBEDDED_H
// V3 - 2026-04-25 - P7: Added 5 RTM/FM RX fields; added RTM & Follow-Me group
// V2.5-Evo - 2026-04-28 - Security: added rtm_stop_distance_m field (was in ConfigService but missing from UI)
// V2.5-Evo - 2026-04-29 - TaskC: full description audit — bool 0/1 values, enum all options inline, int/float extremes explained
// V3 - 2026-04-30 - RTM approach decel zone: rtm_approach_zone_m field added to RTM & Follow-Me group
// V3 - 2026-04-30 - Rename: gps_max_jump_kmh → gps_max_teleport_kmh (clarity)
// V3 - 2026-04-30 - Bundle E: gps_update_hz field added to GPS & Follow-Me group; gps_max_teleport_kmh default 200→80
// V3 - 2026-04-29 - Bundle A: radio_preset max clamped to 2; dead foil_speed != 99 sentinel removed
// V3 - 2026-05-01 - fix: wet_det_active description corrected — warning-only (E71 + vibration), output is never cut

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
    
    .modal-overlay{position:fixed;top:0;left:0;width:100%;height:100%;background:#0b1220e6;display:none;align-items:center;justify-content:center;z-index:99;padding:20px}
    .modal{background:linear-gradient(180deg,#121b2e,#101828);border:1px solid #243042;border-radius:14px;padding:20px;width:100%;max-width:500px;max-height:80vh;overflow-y:auto;box-shadow:0 8px 24px #00000066}
    .modal-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:15px;font-size:18px;font-weight:700}
    .log-item{display:flex;justify-content:space-between;align-items:center;padding:12px 0;border-bottom:1px solid #243042;}
    .log-item:last-child{border-bottom:none;}
    .log-name{font-family:ui-monospace,SFMono-Regular,monospace;font-size:14px;color:var(--txt)}
    .log-size{color:var(--muted);font-size:12px;margin-top:4px}
    .log-actions{display:flex;gap:8px;}
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
        <button class="btn sec" onclick="document.getElementById('logModal').style.display='none'">Close</button>
      </div>
      <div id="logList"></div>
    </div>
  </div>

<script>
// 41 Parameters for RX
const groupOrder=["Radio","Steering","PWM","Motor & Safety","VESC","Sensors","Battery","GPS & Follow-Me","Follow-Me Tuning","RTM & Follow-Me","Logging","System"];
const fields=[
{key:"radio_preset",label:"Radio Preset",description:"Radio frequency band. 1=EU 868 MHz, 2=US/AU 915 MHz. Do not select 3 — it causes a boot error and the RX will not start. Must match TX setting.",group:"Radio",type:"enum",def:1,min:1,max:2,options:[{v:1,l:"EU868"},{v:2,l:"US/AU915"}]},
{key:"rf_power",label:"RF Power",description:"RF transmit power in dBm. -9=minimum (shortest range, lowest interference), 22=maximum (longest range). Must match TX setting. Default 0.",group:"Radio",type:"int",def:0,min:-9,max:22,unit:"dBm"},
{key:"paired",label:"Paired",description:"0=not paired (no TX address stored), 1=paired. Set automatically by the pairing sequence — do not edit manually.",group:"Radio",type:"bool",def:0,min:0,max:1},
{key:"own_address",label:"Own Address",description:"This RX unit's 3-byte LoRa radio address (hex, e.g. 01,A2,FF). Set automatically during pairing — do not edit manually.",group:"Radio",type:"address3",def:"00,00,00"},
{key:"dest_address",label:"Dest Address",description:"Paired TX remote's 3-byte LoRa radio address (hex, e.g. 01,A2,FF). Set automatically during pairing — do not edit manually.",group:"Radio",type:"address3",def:"00,00,00"},
{key:"steering_type",label:"Steering Type",description:"0=Single Motor (one motor, no differential steering), 1=Differential (two motors, speed difference creates turning), 2=Servo (dedicated servo output on PWM1).",group:"Steering",type:"enum",def:0,min:0,max:2,options:[{v:0,l:"Single Motor"},{v:1,l:"Diff Motor"},{v:2,l:"Servo"}]},
{key:"steering_influence",label:"Steering Influence",description:"How much the steering input modifies motor speed in differential mode. 0=no steering effect (straight only), 100=full differential (one motor fully cut on sharp turn). Default 50.",group:"Steering",type:"int",def:50,min:0,max:100,unit:"%"},
{key:"steering_inverted",label:"Steering Inverted",description:"0=normal steering direction, 1=inverted (left/right swapped). Use if buggy steers opposite to expectation. Default 0.",group:"Steering",type:"bool",def:0,min:0,max:1},
{key:"trim",label:"Trim",description:"Steering trim offset applied to the centre position. -500=full left trim, 0=centre, 500=full right trim. Adjust if buggy pulls left or right at neutral. Default 0.",group:"Steering",type:"int",def:0,min:-500,max:500},
{key:"pwm0_min",label:"PWM0 Min",description:"Motor 0 (main drive) minimum PWM pulse width — maps to zero throttle / idle. 500=shortest pulse, 2500=longest. Typical ESC idle is 1000-1500 µs. Default 1500.",group:"PWM",type:"int",def:1500,min:500,max:2500,unit:"us"},
{key:"pwm0_max",label:"PWM0 Max",description:"Motor 0 (main drive) maximum PWM pulse width — maps to full throttle. 500=shortest pulse, 2500=longest. Typical ESC full throttle is 1800-2000 µs. Default 2000.",group:"PWM",type:"int",def:2000,min:500,max:2500,unit:"us"},
{key:"pwm1_min",label:"PWM1 Min",description:"Motor 1 / servo minimum PWM pulse width — maps to zero or full-left position. 500=shortest pulse, 2500=longest. Default 1500.",group:"PWM",type:"int",def:1500,min:500,max:2500,unit:"us"},
{key:"pwm1_max",label:"PWM1 Max",description:"Motor 1 / servo maximum PWM pulse width — maps to full throttle or full-right position. 500=shortest pulse, 2500=longest. Default 2000.",group:"PWM",type:"int",def:2000,min:500,max:2500,unit:"us"},
{key:"failsafe_time",label:"Failsafe Time",description:"Time after the last received LoRa packet before the RX cuts motor output to zero. 100ms=very fast failsafe (noisy environment risk), 10000ms=10 second delay (dangerous — motor runs on after signal loss). Default 1000ms (1 second).",group:"Motor & Safety",type:"int",def:1000,min:100,max:10000,unit:"ms"},
{key:"data_src",label:"Data Source",description:"0=Off (no telemetry sent to TX), 1=Analog (battery voltage via ADC on UBAT pin), 2=VESC UART (full VESC telemetry including speed, ERPM, and motor current). Default 0.",group:"Motor & Safety",type:"enum",def:0,min:0,max:2,options:[{v:0,l:"Off"},{v:1,l:"Analog"},{v:2,l:"VESC UART"}]},
{key:"vesc_timeout_s",label:"VESC Timeout",description:"Seconds without a VESC UART packet before battery % and temperature show as unavailable. Default 12s gives margin above the ~8-9s VESC cold-restart time. Range 5-60s.",group:"VESC",type:"int",def:12,min:5,max:60,unit:"s"},
{key:"foil_num_cells",label:"Battery Cells",description:"Number of LiPo/Li-Ion cells in series (S-count). Used to compute cell voltage from pack voltage. 1=1S (4.2V full), 14=14S (58.8V full). Example: a 14S4P pack is 14 cells in series. Default 10.",group:"Battery",type:"int",def:10,min:1,max:50},
{key:"ubat_cal",label:"Battery Cal Factor",description:"ADC-to-voltage calibration multiplier. Multiply raw ADC reading by this factor to get pack voltage. Calibrate by measuring real voltage with a multimeter and adjusting until they match. Range 0.000001-1.0, default 0.0095554.",group:"Battery",type:"float",def:0.0095554,min:0.000001,max:1.0,step:0.000001},
{key:"ubat_offset",label:"Battery Voltage Offset",description:"Fixed voltage offset added to the calibrated ADC reading. Used to correct for resistor divider bias. -100.0V to +100.0V. Adjust in small steps (e.g. ±0.1V) until measured voltage matches multimeter. Default 0.0.",group:"Battery",type:"float",def:0.0,min:-100.0,max:100.0,step:0.0001},
{key:"bms_det_active",label:"BMS Detection",description:"0=BMS detection disabled, 1=BMS cutoff detection enabled (RX monitors BMS signal pin and shuts output if BMS trips). Default 0.",group:"Sensors",type:"bool",def:0,min:0,max:1},
{key:"wet_det_active",label:"Water Detection",description:"0=wetness detection disabled, 1=enabled (RX monitors moisture sensor and sends E71 warning to TX display with vibration alert if water ingress detected — motor output is NOT cut, user can continue riding and return to shore). Default 1.",group:"Sensors",type:"bool",def:1,min:0,max:1},
{key:"dummy_delete_me",label:"Dummy (reserved)",description:"Reserved field, will be removed",group:"Sensors",type:"int",def:0,min:0,max:65535},
{key:"gps_en",label:"GPS Enabled",description:"0=GPS module disabled (no UART polling, all GPS-dependent features blocked), 1=GPS enabled. RTM and Follow-Me require GPS enabled. Default 0.",group:"GPS & Follow-Me",type:"bool",def:0,min:0,max:1},
{key:"gps_update_hz",label:"GPS Update Rate",description:"How many times per second the RX drains the GPS UART for new NMEA data. 2=500ms interval (default, safe for BN-880 at 5Hz), 5=200ms (poll every 200ms, catches all 5Hz sentences). Range 1-10 Hz. Does not change the GPS module output rate — only how often the firmware reads it.",group:"GPS & Follow-Me",type:"int",def:2,min:1,max:10,unit:"Hz"},
{key:"gps_chip_type",label:"GPS Module Type",description:"GPS module type — determines init sequence, baud rate, and update rate. 0=BN-220 no compass, 1=BN-880 with compass (default for RX), 2=M10 no compass, 3=M10 with compass. Reboot required after change.",group:"GPS & Follow-Me",type:"enum",def:1,min:0,max:3,options:[{v:0,l:"BN-220 no compass (9600→115200, 5Hz)"},{v:1,l:"BN-880 + compass (default, 9600→115200, 5Hz)"},{v:2,l:"M10 no compass (115200 direct, 10Hz, all constellations)"},{v:3,l:"M10 + compass (115200 direct, 10Hz, all constellations)"}]},
{key:"followme_mode",label:"Follow-Me Mode",description:"0=Disabled (RTM throttle-limit only, no steering), 1=Behind (directly behind foiler), 2=Near Right (rear-right diagonal), 3=Near Left (rear-left diagonal). Default 0.",group:"GPS & Follow-Me",type:"enum",def:0,min:0,max:3,options:[{v:0,l:"Disabled"},{v:1,l:"Behind"},{v:2,l:"Near Right"},{v:3,l:"Near Left"}]},
{key:"kalman_en",label:"Kalman Filter",description:"0=Kalman filter disabled (raw GPS position used), 1=Kalman filter enabled (smooths GPS position noise for follow-me and RTM). Enable for autonomous use; disable to debug raw GPS output. Default 0.",group:"GPS & Follow-Me",type:"bool",def:0,min:0,max:1},
{key:"tx_gps_stale_timeout_ms",label:"TX GPS Stale Timeout",description:"Maximum age of TX GPS data before RX considers it stale and blocks Phase B/C anti-spoofing checks. 0=disabled (never stale), 65535=65 seconds. Default 1000ms (1 second). Reduce if TX GPS updates fall behind.",group:"GPS & Follow-Me",type:"int",def:1000,min:0,max:65535,unit:"ms"},
{key:"gps_max_hdop",label:"GPS Max HDOP",description:"Maximum HDOP for a valid GPS fix — lower is stricter. Range 0.5-5.0, default 2.0. Readings above this threshold are rejected.",group:"GPS & Follow-Me",type:"float",def:2.0,min:0.5,max:5.0,step:0.1},
{key:"gps_max_accel_g",label:"GPS Max Acceleration",description:"Maximum implied acceleration between consecutive GPS readings (G-force). Range 1.0-10.0 G, default 3.0 G. Higher-than-max implies a spoofed jump.",group:"GPS & Follow-Me",type:"float",def:3.0,min:1.0,max:10.0,step:0.1,unit:"G"},
{key:"gps_max_teleport_kmh",label:"GPS Max Teleport Speed",description:"Maximum speed implied by position change between readings. Range 50-500 km/h, default 80 km/h. Larger implies GPS teleport.",group:"GPS & Follow-Me",type:"float",def:80.0,min:50.0,max:500.0,step:1.0,unit:"km/h"},
{key:"gps_suspect_threshold",label:"GPS Suspect Threshold",description:"Consecutive anti-spoofing failures before GPS is marked rejected. Range 1-10, default 3. While rejected, RTM arming is blocked.",group:"GPS & Follow-Me",type:"int",def:3,min:1,max:10},
{key:"gps_max_pair_dist_m",label:"Phase B: Max Pair Distance",description:"Maximum plausible TX-RX distance during GPS handshake check. Range 50-2000 m, default 500 m. If TX and RX are further apart than this, RTM arming is blocked.",group:"GPS & Follow-Me",type:"float",def:500.0,min:50.0,max:2000.0,step:10.0,unit:"m"},
{key:"gps_max_speed_diff_kmh",label:"Phase B: Max Speed Difference",description:"Maximum TX-RX GPS speed difference during handshake check. Range 10-200 km/h, default 50 km/h. If speeds differ more than this, RTM arming is blocked.",group:"GPS & Follow-Me",type:"float",def:50.0,min:10.0,max:200.0,step:1.0,unit:"km/h"},
{key:"rtm_vesc_speed_diff_kmh",label:"Phase C: Max VESC Speed Diff",description:"Phase C: max difference between GPS speed and VESC ERPM-implied speed during active RTM. Range 5-50 km/h, default 20.",group:"GPS & Follow-Me",type:"float",def:20.0,min:5.0,max:50.0,step:1.0,unit:"km/h"},
{key:"vesc_erpm_per_kmh",label:"VESC ERPM per km/h",description:"Vehicle-specific: how many ERPM equals 1 km/h. Set by driving at known speed and reading ERPM from ?printtasks. 0=disable Phase C VESC check.",group:"GPS & Follow-Me",type:"float",def:0.0,min:0.0,max:9999.0,step:1.0,unit:"ERPM/kmh"},
{key:"rtm_rx_enabled",label:"RTM RX Enabled",description:"RX-side RTM master enable (safety kill switch). 0=RTM blocked regardless of TX. Default 1.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_rx_override_steering",label:"RTM Override Steering",description:"Allow RTM to override steering towards TX GPS position. 0=disable steering override (throttle limiting only). Default 1.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_compass_required",label:"RTM Compass Required",description:"Require valid compass reading for RTM arming. 0=allow RTM without compass (steering disabled). Default 1.",group:"RTM & Follow-Me",type:"bool",def:1,min:0,max:1},
{key:"rtm_stop_distance_m",label:"RTM Hard Stop Distance",description:"Distance at which RTM triggers a hard stop and disengages (Gate 9). Range 1-50 m, default 3 m. Must be less than the arm distance to allow RTM to run.",group:"RTM & Follow-Me",type:"int",def:3,min:1,max:50,unit:"m"},
{key:"rtm_approach_zone_m",label:"RTM Approach Zone",description:"Distance from TX at which RTM begins ramping throttle down toward zero. Throttle reduces linearly from full at this outer edge to zero at the Hard Stop Distance — smooth arrival instead of sudden cut. 0=disabled (hard stop only, no ramp). Must be greater than Hard Stop Distance to have effect. Range 0-100 m, default 15 m.",group:"RTM & Follow-Me",type:"int",def:15,min:0,max:100,unit:"m"},
{key:"boogie_vmax_in_followme_kmh",label:"Boogie V-Max",description:"Maximum vehicle speed while Follow-Me is active. Throttle is capped so speed never exceeds this value during following. 0=no limit, 100=100 km/h cap. Default 25 km/h. Set to a safe value for your terrain.",group:"Follow-Me Tuning",type:"float",def:25.0,min:0,max:100,step:0.1,unit:"km/h"},
{key:"min_dist_m",label:"Min Distance",description:"Minimum safe distance the buggy must maintain from the foiler. Throttle is cut if the buggy gets closer than this. 0=no minimum, 1000=1000 m minimum (impractical). Default 10 m.",group:"Follow-Me Tuning",type:"float",def:10.0,min:0,max:1000,step:0.1,unit:"m"},
{key:"followme_smoothing_band_m",label:"Smoothing Band",description:"Distance band above min_dist_m over which throttle is linearly reduced to zero as the buggy approaches the minimum distance. Larger = smoother slowdown but less responsive. Default 10 m.",group:"Follow-Me Tuning",type:"float",def:10.0,min:0,max:1000,step:0.1,unit:"m"},
{key:"foiler_low_speed_kmh",label:"Foiler Low Speed",description:"If the foiler's GPS speed drops below this value, Follow-Me throttle is cut (foiler has stopped or crashed). 0=disabled, 100=100 km/h threshold (would always cut). Default 5 km/h.",group:"Follow-Me Tuning",type:"float",def:5.0,min:0,max:100,step:0.1,unit:"km/h"},
{key:"zone_angle_enter_deg",label:"Zone Angle Enter",description:"Half-angle of the acceptable follow zone behind the foiler. Follow-Me steering activates only when buggy is within ±this angle of directly behind. 0=very narrow corridor, 180=full circle. Default 35°.",group:"Follow-Me Tuning",type:"float",def:35.0,min:0,max:180,step:0.1,unit:"deg"},
{key:"zone_angle_exit_deg",label:"Zone Angle Exit",description:"Half-angle at which Follow-Me zone steering disengages (hysteresis). Must be ≥ zone_angle_enter_deg to avoid oscillation — typically set 5-15° wider. Default 45°.",group:"Follow-Me Tuning",type:"float",def:45.0,min:0,max:180,step:0.1,unit:"deg"},
{key:"near_diag_offset_deg",label:"Near Diag Offset",description:"Target bearing offset from directly-behind for Near Right and Near Left Follow-Me modes. 0=directly behind foiler, 90=beside foiler. Applied as +offset for Near Right, -offset for Near Left. Default 45°.",group:"Follow-Me Tuning",type:"float",def:45.0,min:0,max:180,step:0.1,unit:"deg"},
{key:"logger_en",label:"Logger Enabled",description:"0=data logger disabled (no CSV files written to SPIFFS), 1=logger enabled (records speed, GPS, VESC, and control data to SPIFFS at 10Hz). Disable if storage is full or to extend SPIFFS lifetime. Default 1.",group:"Logging",type:"bool",def:1,min:0,max:1},
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
    a.download = 'bremote_rx_backup.json';
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
    let h='';
    res.logs.forEach(x=>{
      const kb=(x.size/1024).toFixed(1);
      h+=`<div class="log-item"><div><div class="log-name">${x.name}</div><div class="log-size">${kb} KB</div></div><div class="log-actions"><a class="btn" href="/api/logs/download?file=${x.name}" target="_blank" style="text-decoration:none;font-size:12px;padding:7px 10px">Download CSV</a><button class="btn warn" style="font-size:12px;padding:7px 10px" onclick="deleteLog('${x.name}')">Delete</button></div></div>`;
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

window.setVal=setVal;window.setBool=setBool;window.setEnum=setEnum;window.syncField=syncField;window.setAddrPart=setAddrPart;window.saveAll=saveAll;window.loadCfg=loadCfg;window.rebootDev=rebootDev;window.refreshAll=refreshAll;window.openLogs=openLogs;window.deleteLog=deleteLog;window.copyJson=copyJson;window.exportJsonFile=exportJsonFile;window.importJsonFile=importJsonFile;window.loadFromJsonText=loadFromJsonText;
refreshAll();
</script>
</body>
</html>
)HTML";

static const size_t WEB_UI_INDEX_HTML_LEN = sizeof(WEB_UI_INDEX_HTML) - 1;

#endif