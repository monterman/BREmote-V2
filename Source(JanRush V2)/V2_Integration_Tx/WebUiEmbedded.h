#ifndef WEB_UI_EMBEDDED_H
#define WEB_UI_EMBEDDED_H

#include <Arduino.h>

static const char WEB_UI_INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>BREmote V2 Web Config</title>
  <style>
    :root{--bg:#0b1220;--panel:#101828;--panel2:#1e293b;--txt:#e5e7eb;--muted:#9ca3af;--pri:#60a5fa;--err:#ef4444}
    *{box-sizing:border-box}
    body{margin:0;background:radial-gradient(1200px 700px at 10% -10%,#1e3a5f66 0,transparent 45%),linear-gradient(180deg,#0a1120 0,#0b1220 40%);color:var(--txt);font-family:"Avenir Next","Montserrat","Segoe UI",sans-serif}
    .wrap{max-width:980px;margin:0 auto;padding:14px 14px 110px}
    .card{background:linear-gradient(180deg,#121b2e,#101828);border:1px solid #243042;border-radius:14px;padding:12px;box-shadow:0 8px 24px #00000033}
    .top{position:sticky;top:0;backdrop-filter:blur(6px);padding-top:6px;z-index:9}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .sp{justify-content:space-between}
    .title{font-weight:700;font-size:18px}
    .sub{color:var(--muted);font-size:12px}
    .btn{border:0;border-radius:10px;padding:9px 12px;background:var(--pri);color:#08111f;font-weight:700;cursor:pointer}
    .btn:disabled{opacity:.55;cursor:not-allowed}
    .btn.sec{background:#243042;color:var(--txt)}
    .btn.warn{background:#b91c1c;color:#fff}
    .groups{margin-top:10px;display:flex;flex-direction:column;gap:8px}
    details{background:var(--panel2);border:1px solid #334155;border-radius:12px}
    summary{cursor:pointer;padding:10px 12px;font-weight:700;color:#cbd5e1}
    .items{padding:0 10px 10px;display:flex;flex-direction:column;gap:8px}
    .field{background:var(--panel);border:1px solid #243042;border-radius:10px;padding:10px}
    .label{font-weight:700}.desc,.hint,.err{font-size:12px}.desc,.hint{color:var(--muted)}.err{color:var(--err)}
    input,select{width:100%;padding:8px;border-radius:8px;border:1px solid #334155;background:#0f172a;color:var(--txt)}
    input[type='checkbox']{width:auto} input[type='range']{width:100%}
    .triple{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.foot{margin-top:10px}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div class="card">
        <div class="row sp">
          <div>
            <div class="title">BREmote V2 Web Config</div>
            <div class="sub" id="status">Loading...</div>
            <div class="sub" id="loaded">Loaded 0/0</div>
          </div>
          <div class="row">
            <button class="btn sec" onclick="loadCfg()">Load</button>
            <button class="btn" onclick="saveAll()">Save All</button>
            <button class="btn sec" onclick="exportCfg()">Export</button>
            <button class="btn sec" onclick="document.getElementById('importFile').click()">Import</button>
            <input type="file" id="importFile" accept=".json" style="display:none" onchange="importCfg(this)">
            <button class="btn warn" onclick="rebootDev()">Reboot TX</button>
          </div>
        </div>
      </div>
    </div>
    <div class="groups" id="groups"></div>
    <div class="card foot"><div class="sub mono" id="last">Last: -</div></div>
  </div>
<script>
const groupOrder=["Radio","Gears","Throttle","Steering","Toggle","Lock & Timing","Calibration","GPS & Follow-Me","System"];
const fields=[
{key:"radio_preset",label:"Radio Preset",description:"Select your region",group:"Radio",type:"enum",def:1,min:1,max:3,options:[{v:1,l:"EU868"},{v:2,l:"US/AU915"},{v:3,l:"Reserved"}]},
{key:"rf_power",label:"RF Power",description:"RF transmit power (dBm)",group:"Radio",type:"int",def:0,min:-9,max:22,unit:"dBm"},
{key:"paired",label:"Paired",description:"Pairing state flag",group:"Radio",type:"bool",def:0,min:0,max:1},
{key:"own_address",label:"Own Address",description:"Local radio address",group:"Radio",type:"address3",def:"00,00,00"},
{key:"dest_address",label:"Dest Address",description:"Paired destination address",group:"Radio",type:"address3",def:"00,00,00"},
{key:"max_gears",label:"Max Gears",description:"Maximum number of user gears",group:"Gears",type:"int",def:10,min:1,max:10},
{key:"startgear",label:"Start Gear",description:"Gear after power-on/unlock",group:"Gears",type:"int",def:0,min:0,max:9},
{key:"throttle_mode",label:"Throttle Mode",description:"0=Gears, 1=No Gears, 2=Dynamic Cap",group:"Gears",type:"enum",def:0,min:0,max:2,options:[{v:0,l:"Gears"},{v:1,l:"No Gears"},{v:2,l:"Dynamic Cap"}]},
{key:"dynamic_power_start",label:"Dynamic Power Start %",description:"Starting power cap for Dynamic Cap mode",group:"Gears",type:"int",def:85,min:10,max:100,unit:"%"},
{key:"dynamic_power_step",label:"Dynamic Power Step %",description:"Cap adjustment per toggle press",group:"Gears",type:"int",def:5,min:1,max:25,unit:"%"},
{key:"gear_change_waittime",label:"Gear Change Wait",description:"Hold time to change gear",group:"Gears",type:"int",def:100,min:0,max:65535,unit:"ms"},
{key:"gear_display_time",label:"Gear Display Time",description:"Duration to show selected gear",group:"Gears",type:"int",def:1000,min:0,max:65535,unit:"ms"},
{key:"thr_expo",label:"Throttle Expo",description:"Throttle curve (50 = linear)",group:"Throttle",type:"int",def:50,min:0,max:100},
{key:"thr_expo1",label:"Throttle Expo 1",description:"Reserved/unused",group:"Throttle",type:"int",def:0,min:0,max:65535},
{key:"steer_enabled",label:"Steering Enabled",description:"Enable/disable steering",group:"Steering",type:"bool",def:1,min:0,max:1},
{key:"steer_expo",label:"Steering Expo",description:"Reserved/unused",group:"Steering",type:"int",def:50,min:0,max:65535},
{key:"steer_expo1",label:"Steering Expo 1",description:"Reserved/unused",group:"Steering",type:"int",def:0,min:0,max:65535},
{key:"tog_deadzone",label:"Toggle Deadzone",description:"Toggle center deadzone",group:"Toggle",type:"int",def:500,min:100,max:3000},
{key:"tog_diff",label:"Toggle Diff",description:"Delta to register input",group:"Toggle",type:"int",def:30,min:1,max:200},
{key:"tog_block_time",label:"Toggle Block Time",description:"Block time while steering",group:"Toggle",type:"int",def:500,min:0,max:5000},
{key:"no_lock",label:"No Lock",description:"Disable lock",group:"Lock & Timing",type:"bool",def:0,min:0,max:1},
{key:"trig_unlock_timeout",label:"Trigger Unlock Timeout",description:"Max trigger time during unlock",group:"Lock & Timing",type:"int",def:5000,min:0,max:65535,unit:"ms"},
{key:"lock_waittime",label:"Lock Wait Time",description:"Hold time for lock/power-off",group:"Lock & Timing",type:"int",def:2000,min:0,max:65535,unit:"ms"},
{key:"menu_timeout",label:"Menu Timeout",description:"Time before steering re-engages",group:"Lock & Timing",type:"int",def:10,min:0,max:1000},
{key:"err_delete_time",label:"Error Delete Time",description:"Error-clear screen duration",group:"Lock & Timing",type:"int",def:2000,min:0,max:65535},
{key:"cal_ok",label:"Calibration OK",description:"Calibration complete flag",group:"Calibration",type:"bool",def:0,min:0,max:1},
{key:"cal_offset",label:"Cal Offset",description:"Calibration offset",group:"Calibration",type:"int",def:100,min:0,max:65535},
{key:"thr_idle",label:"Throttle Idle",description:"Raw throttle ADC at idle",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"thr_pull",label:"Throttle Pull",description:"Raw throttle ADC at full pull",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"tog_left",label:"Toggle Left",description:"Raw toggle ADC at left",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"tog_mid",label:"Toggle Mid",description:"Raw toggle ADC at center",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"tog_right",label:"Toggle Right",description:"Raw toggle ADC at right",group:"Calibration",type:"int",def:0,min:0,max:65535},
{key:"ubat_cal",label:"Battery Cal Factor",description:"ADC-to-voltage calibration",group:"Calibration",type:"float",def:0.000185662,min:0.000001,max:1.0,step:0.000001},
{key:"gps_en",label:"GPS Enabled",description:"GPS runtime enable flag",group:"GPS & Follow-Me",type:"bool",def:0,min:0,max:1},
{key:"followme_mode",label:"Follow-Me Mode",description:"Follow-me positioning mode",group:"GPS & Follow-Me",type:"enum",def:0,min:0,max:3,options:[{v:0,l:"Disabled"},{v:1,l:"Behind"},{v:2,l:"Near Right"},{v:3,l:"Near Left"}]},
{key:"kalman_en",label:"Kalman Filter",description:"Kalman runtime enable",group:"GPS & Follow-Me",type:"bool",def:0,min:0,max:1},
{key:"speed_src",label:"Speed Source",description:"Speed data source",group:"GPS & Follow-Me",type:"enum",def:0,min:0,max:3,options:[{v:0,l:"GPS RX km/h"},{v:1,l:"GPS RX knots"},{v:2,l:"GPS TX km/h"},{v:3,l:"GPS TX knots"}]},
{key:"tx_gps_stale_timeout_ms",label:"TX GPS Stale Timeout",description:"TX GPS stale timeout",group:"GPS & Follow-Me",type:"int",def:1000,min:0,max:65535,unit:"ms"},
{key:"wifi_password",label:"WiFi Password",description:"AP password (exactly 8 characters)",group:"System",type:"text",def:"12345678",minLen:8,maxLen:8},
{key:"version",label:"Config Version",description:"Must match firmware SW_VERSION",group:"System",type:"int",def:3,min:0,max:65535}
];
const state={values:{},loaded:{},saved:{},last:'-'};
const openGroups={};
async function api(url,m='GET',b=null){const o={method:m};if(b){o.headers={'Content-Type':'application/x-www-form-urlencoded'};o.body=b;}try{const r=await fetch(url,o);const j=await r.json();state.last=JSON.stringify(j);document.getElementById('last').textContent='Last: '+state.last;return j;}catch(e){const j={ok:0,err:'ERR_HTTP'};state.last=JSON.stringify(j);document.getElementById('last').textContent='Last: '+state.last;return j;}}
function normAddr(v){
  let s=String(v||'').replace(/[\[\]]/g,'').replace(/[:;-]/g,',').trim();
  const p=s.split(',').map(x=>x.trim()).filter(Boolean).slice(0,3);
  while(p.length<3)p.push('00');
  return p
    .map(x=>x.replace(/^0x/i,'').replace(/[^0-9a-f]/gi,'').slice(0,2))
    .map(x=>x.length?Number.parseInt(x,16):0)
    .map(n=>Number.isNaN(n)?0:Math.max(0,Math.min(255,n)));
}
function toAddrHex(v){return normAddr(v).map(n=>n.toString(16).toUpperCase().padStart(2,'0'));}
function valueForSend(f,v){if(f.type==='address3'){const h=toAddrHex(v);return `${h[0]}:${h[1]}:${h[2]}`;}return String(v).trim();}
function canonValue(f,v){if(f.type==='address3')return toAddrHex(v).join(',');return String(v??'').trim();}
function hasUnsavedChanges(){for(const f of fields){const a=canonValue(f,state.values[f.key]);const b=canonValue(f,state.saved[f.key]);if(a!==b)return true;}return false;}
function validate(f,v){const s=String(v??'').trim();if(!s.length)return'Required';if(f.type==='address3')return null;if(f.type==='float'){const n=parseFloat(s);if(Number.isNaN(n))return'Must be float';if(n<f.min||n>f.max)return`Range ${f.min}..${f.max}`;return null;}const n=parseInt(s,10);if(Number.isNaN(n))return'Must be integer';if(n<f.min||n>f.max)return`Range ${f.min}..${f.max}`;if(f.key==='startgear'){const mg=parseInt(state.values.max_gears??'10',10);if(!Number.isNaN(mg)&&n>=mg)return'Must be < max_gears';}return null;}
function setVal(k,v){state.values[k]=String(v);updateHint(k);}
function setBool(k,c){setVal(k,c?'1':'0');}
function setEnum(k,v){setVal(k,v);}
function syncField(k,v){document.querySelectorAll(`[data-key='${k}']`).forEach(el=>{if(el!==document.activeElement)el.value=v;});}
function setAddrPart(k,i,v){const cur=toAddrHex(state.values[k]);const c=String(v||'').toUpperCase().replace(/[^0-9A-F]/g,'').slice(0,2);cur[i]=c;state.values[k]=cur.join(',');}
function updateHint(k){const f=fields.find(x=>x.key===k);if(!f)return;const err=validate(f,state.values[k]);const el=document.getElementById(`hint-${k}`);if(!el)return;const hint=`Range ${f.min}..${f.max}${f.unit?(' '+f.unit):''} | default: ${f.def}`;el.textContent=err||hint;el.className=err?'err':'hint';}
async function saveAll(){for(const f of fields){const e=validate(f,state.values[f.key]);if(e){alert(`${f.label}: ${e}`);return;}}for(const f of fields){const body=`key=${encodeURIComponent(f.key)}&value=${encodeURIComponent(valueForSend(f,state.values[f.key]))}`;const r=await api('/api/set','POST',body);if(!r.ok){alert(`SET ${f.key} failed: ${r.err||'ERR'}`);return;}}const s=await api('/api/save','POST');if(!s.ok){alert(s.err||'SAVE failed');return;}await refreshAll();}
function resetAll(){for(const f of fields)state.values[f.key]=String(f.def);render();}
async function loadCfg(){await api('/api/load','POST');await refreshAll();}
async function rebootDev(){if(hasUnsavedChanges()){const ignore=confirm("Unsaved config changes detected. Press OK to ignore and reboot, or Cancel to go back and Save All.");if(!ignore){alert("Hint: press Save All before reboot to keep your changes.");return;}}await api('/api/reboot','POST');}
function ctrlHtml(f){const v=state.values[f.key]??String(f.def);if(f.type==='bool'){const on=String(v).trim()==='1';return `<label class='row'><input type='checkbox' ${on?'checked':''} onchange="setBool('${f.key}',this.checked)"><span>${on?'Enabled':'Disabled'}</span></label>`;}if(f.type==='enum'){const opts=f.options.map(o=>`<option value='${o.v}' ${String(v)===String(o.v)?'selected':''}>${o.l}</option>`).join('');return `<select onchange="setEnum('${f.key}',this.value)">${opts}</select>`;}if(f.type==='address3'){const p=toAddrHex(v);return `<div class='triple'><input value='${p[0]}' maxlength='2' oninput="setAddrPart('${f.key}',0,this.value)"><input value='${p[1]}' maxlength='2' oninput="setAddrPart('${f.key}',1,this.value)"><input value='${p[2]}' maxlength='2' oninput="setAddrPart('${f.key}',2,this.value)"></div>`;}const step=f.type==='float'?(f.step||0.000001):1;const n=parseFloat(v);const showSlider=f.type==='int'&&(f.max-f.min)<=1000;const slider=showSlider?`<input type='range' data-key='${f.key}' min='${f.min}' max='${f.max}' step='1' value='${Number.isNaN(n)?f.min:Math.max(f.min,Math.min(f.max,n))}' oninput="setVal('${f.key}',this.value);syncField('${f.key}',this.value)">`:'';return `<input type='number' data-key='${f.key}' step='${step}' min='${f.min}' max='${f.max}' value='${v}' oninput="setVal('${f.key}',this.value);syncField('${f.key}',this.value)">${slider}`;}
function render(){document.querySelectorAll('.groups details').forEach(d=>{openGroups[d.dataset.group]=d.open;});const gEl=document.getElementById('groups');const loadedCount=fields.filter(f=>state.loaded[f.key]).length;document.getElementById('loaded').textContent=`Loaded ${loadedCount}/${fields.length}${hasUnsavedChanges()?' | Unsaved changes':''}`;const byGroup={};for(const g of groupOrder)byGroup[g]=[];for(const f of fields){if(!byGroup[f.group])byGroup[f.group]=[];byGroup[f.group].push(f);}gEl.innerHTML=groupOrder.map((g,gi)=>{const items=(byGroup[g]||[]).map(f=>{const v=state.values[f.key]??String(f.def);const err=validate(f,v);const hint=`Range ${f.min}..${f.max}${f.unit?(' '+f.unit):''} | default: ${f.def}`;return `<div class='field'><div class='label'>${f.label}</div><div class='desc'>${f.description}</div><div style='margin:8px 0'>${ctrlHtml(f)}</div><div id='hint-${f.key}' class='${err?'err':'hint'}'>${err||hint}</div></div>`;}).join('');const isOpen = Object.prototype.hasOwnProperty.call(openGroups,g)?openGroups[g]:(gi===0);return `<details data-group='${g}' ${isOpen?'open':''}><summary>${g}</summary><div class='items'>${items}</div></details>`;}).join('');}
async function refreshAll(){const s=await api('/api/state');document.getElementById('status').textContent=s.state||JSON.stringify(s);const c=await api('/api/config');if(c.ok&&c.data){for(const f of fields){if(Object.prototype.hasOwnProperty.call(c.data,f.key)){const raw=c.data[f.key];const val=Array.isArray(raw)?raw.join(','):String(raw);state.values[f.key]=val;state.saved[f.key]=val;state.loaded[f.key]=true;}}}render();}
async function exportCfg(){const r=await api('/api/config/export?format=json');if(!r.ok){alert('Export failed: '+(r.err||'ERR'));return;}const blob=new Blob([JSON.stringify(r,null,2)],{type:'application/json'});const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='bremote_tx_config_'+new Date().toISOString().slice(0,10)+'.json';a.click();URL.revokeObjectURL(a.href);}
async function importCfg(input){const file=input.files[0];if(!file)return;input.value='';try{const txt=await file.text();const json=JSON.parse(txt);const data=json.data||json;const body='format=json&data='+encodeURIComponent(JSON.stringify(data));const r=await api('/api/config/import','POST',body);if(!r.ok){alert('Import failed: '+(r.err||'ERR'));return;}await refreshAll();alert('Config imported. Review values then Save All.');}catch(e){alert('Invalid JSON file: '+e.message);}}
window.setVal=setVal;window.setBool=setBool;window.setEnum=setEnum;window.syncField=syncField;window.setAddrPart=setAddrPart;window.saveAll=saveAll;window.loadCfg=loadCfg;window.rebootDev=rebootDev;window.refreshAll=refreshAll;window.exportCfg=exportCfg;window.importCfg=importCfg;refreshAll();
</script>
</body>
</html>
  

)HTML";

static const size_t WEB_UI_INDEX_HTML_LEN = sizeof(WEB_UI_INDEX_HTML) - 1;

#endif