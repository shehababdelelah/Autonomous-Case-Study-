// TestLoggerWeb.h - phone page for the zone test logger (v4, Team B4 layout).
//
// Open http://192.168.<10+BOX_ID>.1/test on a phone connected to the box's Wi-Fi
// (best on the Logger's phone; the Operator keeps the driving page).
//
// Four tabs:
//   Setup     sensor check, zone (1..4, full path, sensor bench), test type (sensors only,
//             manual drive, AUTO), before-start checklist (on the phone only), Start
//   Run       REC timer, E-STOP, next checkpoint, Mark checkpoint / Log obstacle / Undo,
//             checkpoint list, Stop and save
//   Sensors   live LiDAR top view, ultrasonic bars, IMU, GPS (from /values of WebHMI)
//   Results   the selected run read back from its events file(s): every waypoint, ramp, abort
//             and summary marked pass / warn / fail by the logger's own rules, plus the files
//             to download or delete
//
// Only AUTO drives (forward at setTestDrive() speed, see TestLogger.h); Sensors only and Manual
// drive just record. E-STOP sends a zero drive command and stops the run at once.
//
// Add the routes once, where your AsyncWebServer is set up (before server.begin()):
//   #include "TestLoggerWeb.h"
//   extern TestLogger testLog;
//   addTestLogRoutes(server, testLog);
// Use the same HTTP_GET / HTTP_POST names your WebHMI routes already use.
#pragma once
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <stdlib.h>
#include <string.h>
#include "TestLogger.h"

static const char kTestLogPage[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Zone tests</title>
<style>
:root{--navy:#1f3864;--navy2:#2b4a80;--bg:#eef1f6;--card:#fff;--line:#e3e7ee;--text:#1b2433;--muted:#6b7587;
 --red:#e53935;--redbg:#fde8e8;--green:#2e7d32;--greenbg:#e5f4e6;--amber:#e08a00;--amberbg:#fff3dc;--grey:#eef0f4;--blue:#3d6fd6}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
.app{max-width:520px;margin:0 auto;min-height:100vh;padding-bottom:76px}
header{background:var(--navy);color:#fff;padding:12px 14px;display:flex;align-items:center;gap:8px;position:sticky;top:0;z-index:5}
header .t{flex:1;min-width:0}header h1{font-size:19px;margin:0}header small{opacity:.75;font-size:12px}
.pill{background:rgba(255,255,255,.14);border-radius:999px;padding:5px 11px;font-size:13px;white-space:nowrap}
.pill i{display:inline-block;width:8px;height:8px;border-radius:50%;background:#9aa4b2;margin-right:6px}
.pill.on i{background:#4cd964}.pill.off i{background:var(--red)}
.estop{background:var(--red);color:#fff;border:0;border-radius:10px;font-weight:800;padding:9px 12px;font-size:15px;letter-spacing:.5px}
.msg{background:#e2e7f0;color:#34405a;font-size:14px;padding:8px 14px;text-align:center}
main{padding:10px 12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:12px 14px;margin-bottom:12px}
.ch{display:flex;justify-content:space-between;align-items:baseline;gap:8px;margin-bottom:8px}
.ch b{font-size:13px;letter-spacing:.6px;color:#4a5468}.ch span{font-size:13px;color:var(--muted)}
.chips{display:flex;flex-wrap:wrap;gap:6px}
.chip{border-radius:999px;padding:5px 10px;font-size:13.5px;background:var(--grey);color:#3d4658}
.chip.g{background:var(--greenbg);color:var(--green)}.chip.r{background:var(--redbg);color:#c62828}.chip.a{background:var(--amberbg);color:#9a5d00}
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.opt{border:1px solid var(--line);background:#fff;border-radius:10px;padding:9px 10px;text-align:left;font-size:15px;font-weight:650;color:var(--text);min-height:58px;min-width:0;overflow-wrap:anywhere}
.opt small{display:block;font-weight:400;font-size:12px;color:var(--muted);margin-top:2px}
.opt.sel{background:var(--navy);color:#fff;border-color:var(--navy)}.opt.sel small{color:#c9d3e8}
.seg{display:flex;border:1px solid var(--line);border-radius:10px;overflow:hidden}
.seg button{flex:1;border:0;background:#fff;padding:11px 4px;font-size:15px;color:var(--text)}
.seg button.sel{background:var(--navy);color:#fff;font-weight:650}
.seg button:disabled{color:#b3b9c4}
.hint{font-size:13px;color:var(--muted);margin-top:8px}
.row{display:grid;grid-template-columns:42% minmax(0,1fr);align-items:center;margin:6px 0}
.row label{color:#4a5468;font-size:15px}
input[type=text]{width:100%;border:1px solid var(--line);border-radius:9px;padding:10px;font-size:15px}
.chk{display:flex;gap:10px;align-items:flex-start;margin:9px 0;font-size:15px}
.chk input{width:22px;height:22px;margin:0;flex:none}
.btn{display:block;width:100%;border:0;border-radius:12px;padding:15px;font-size:17px;font-weight:700;background:var(--navy);color:#fff}
.btn:disabled{opacity:.4}
.btn.ghost{background:#fff;color:var(--red);border:2px solid var(--red)}
.rec{background:var(--redbg);border-color:#f5c2c2;display:flex;justify-content:space-between;align-items:center}
.rec b{color:#c62828;font-size:18px}.rec b:before{content:"";display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--red);margin-right:8px}
.rec .tm{font-size:26px;font-weight:800;color:#c62828;font-variant-numeric:tabular-nums;text-align:right}
.rec small{display:block;color:#a33;font-size:13px}
.big-stop{width:100%;border:0;border-radius:14px;background:var(--red);color:#fff;padding:14px;margin-bottom:12px;box-shadow:0 3px 0 #b71c1c}
.big-stop b{display:block;font-size:26px;letter-spacing:1px}.big-stop small{font-size:13px}
.bar{height:9px;background:var(--grey);border-radius:9px;overflow:hidden;margin:8px 0 6px}.bar i{display:block;height:100%;background:#7b3fc4;border-radius:9px}
h3{margin:2px 0 2px;font-size:19px}
.acts{display:grid;grid-template-columns:1.2fr 1fr 1fr;gap:8px;margin-bottom:12px}
.acts button{border-radius:12px;padding:14px 6px;font-size:16px;font-weight:700;border:0}
.b-mark{background:var(--navy);color:#fff}.b-obs{background:#e69500;color:#fff}.b-undo{background:#fff;color:var(--navy);border:2px solid var(--navy)!important}
.acts button:disabled{opacity:.4}
.cp{display:flex;align-items:center;gap:10px;padding:8px 0;border-top:1px solid var(--line)}
.cp:first-of-type{border-top:0}
.num{width:28px;height:28px;border-radius:50%;display:grid;place-items:center;font-size:13px;font-weight:700;flex:none;background:var(--grey);color:#6b7587}
.num.p{background:var(--green);color:#fff}.num.f{background:var(--red);color:#fff}.num.n{background:#fff;border:2px solid var(--navy);color:var(--navy)}.num.w{background:#7b3fc4;color:#fff}
.cp .x{flex:1;min-width:0}.cp .x small{display:block;color:var(--muted);font-size:13px}
.tag{font-size:13px;font-weight:650;border-radius:999px;padding:3px 10px;background:var(--grey);color:#4a5468;white-space:nowrap}
.tag.Pass{background:var(--greenbg);color:var(--green)}.tag.Fail{background:var(--redbg);color:#c62828}.tag.Warn{background:var(--amberbg);color:#9a5d00}
canvas{display:block;width:100%;max-width:360px;margin:0 auto;background:#f6f8fb;border-radius:10px}
.kv{display:grid;grid-template-columns:1fr 1fr;gap:4px 18px;margin-top:10px}.kv div{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid var(--line)}
.us{display:grid;grid-template-columns:96px 1fr 44px;align-items:center;gap:8px;margin:6px 0;font-size:15px}
.us .tr{height:16px;background:var(--grey);border-radius:4px;overflow:hidden}.us .tr i{display:block;height:100%;background:var(--blue)}
.us b{text-align:right}
.two{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.bigv{font-size:26px;font-weight:800}.bigv small{font-size:14px;font-weight:400;color:var(--muted)}
table{width:100%;border-collapse:collapse;font-size:14px;table-layout:fixed;overflow-wrap:anywhere}
th:nth-child(1){width:28%}th:nth-child(2){width:24%}th:nth-child(4){width:58px}
th{text-align:left;font-size:12px;letter-spacing:.5px;color:#4a5468;padding:6px 4px;border-bottom:1px solid var(--line)}
td{padding:8px 4px;border-bottom:1px solid var(--line);vertical-align:top}td small{display:block;color:var(--muted);font-size:12px}
select{width:100%;padding:10px;border-radius:9px;border:1px solid var(--line);font-size:15px;background:#fff;margin-bottom:10px}
.fr{display:flex;align-items:center;gap:8px;padding:9px 0;border-top:1px solid var(--line)}.fr:first-child{border-top:0}
.fr .fn{flex:1;min-width:0;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:14px;overflow-wrap:anywhere}
.fr .fn small{font-family:system-ui,sans-serif;color:var(--muted);font-size:13px;margin-left:4px}
.dl{border:1px solid var(--line);background:#fff;border-radius:8px;padding:6px 12px;font-size:14px;font-weight:650;color:#34405a;text-decoration:none;white-space:nowrap}
.fbtns{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
.fbtns button{border-radius:12px;padding:14px 6px;font-size:16px;font-weight:700;background:var(--navy);color:#fff;border:2px solid var(--navy)}
.fbtns button.o{background:#fff;color:var(--navy)}.fbtns button:disabled{opacity:.4}
.small-btn{border:1px solid var(--line);background:#fff;border-radius:8px;padding:7px 11px;font-size:14px;margin:10px 6px 0 0}
.small-btn.d{color:#c62828;border-color:#f0b4b4}
nav{position:fixed;bottom:0;left:0;right:0;background:#fff;border-top:1px solid var(--line);display:flex;justify-content:center;z-index:5}
nav div{display:flex;width:100%;max-width:520px}
nav button{flex:1;border:0;background:none;padding:10px 0 12px;font-size:15px;color:#8a93a3;position:relative}
nav button:before{content:"";display:block;width:30px;height:4px;border-radius:4px;background:#d5dae3;margin:0 auto 6px}
nav button.sel{color:var(--navy);font-weight:700}nav button.sel:before{background:var(--navy)}
.hide{display:none!important}
</style></head><body><div class="app">
<header><div class="t"><h1>Zone tests</h1><small id="host"></small></div>
 <span class="pill" id="conn"><i></i>...</span><button class="estop" id="estop1">E-STOP</button></header>
<div class="msg" id="msg">Connecting...</div>
<main>
<!-- ================= SETUP ================= -->
<section id="t-setup">
 <div class="card"><div class="ch"><b>SENSORS</b><span id="sReady">-</span></div><div class="chips" id="chips"></div></div>
 <div class="card"><div class="ch"><b>1 · ZONE</b></div><div class="grid3" id="zones"></div></div>
 <div class="card"><div class="ch"><b>2 · TEST TYPE</b></div>
  <div class="seg" id="types"><button data-t="0">Sensors only</button><button data-t="1">Manual drive</button><button data-t="2">AUTO</button></div>
  <div class="hint" id="typeHint"></div></div>
 <div class="card"><div class="ch"><b>3 · BEFORE START</b><span id="ckCount">0 of 4</span></div><div id="checks"></div></div>
 <button class="btn" id="start">Start</button>
</section>
<!-- ================= RUN ================= -->
<section id="t-run" class="hide">
 <div class="card" id="idle"><b>No run is recording.</b><div class="hint">Choose a zone and test type on Setup, then press Start.</div></div>
 <div id="live">
  <div class="card rec"><div><b id="recRun">REC</b><small id="recSub"></small></div><div class="tm"><span id="recTime">0:00</span><small id="recOf"></small></div></div>
  <button class="big-stop" id="estop2"><b>E-STOP</b><small>stops the motors at once</small></button>
  <div class="card"><div class="ch"><b>NEXT CHECKPOINT</b><span id="nextCount"></span></div>
   <h3 id="nextTitle">-</h3><div class="hint" id="nextHint" style="margin-top:2px"></div>
   <div class="bar"><i id="zoneBar" style="width:0"></i></div><div class="hint" id="liveInfo" style="margin-top:0"></div></div>
  <div class="acts"><button class="b-mark" id="mark">Mark checkpoint</button><button class="b-obs" id="obs">Log obstacle</button><button class="b-undo" id="undo">Undo</button></div>
  <button class="btn hide" id="next" style="margin-bottom:12px">Next zone</button>
  <div class="card"><div class="ch"><b>CHECKPOINTS</b><span id="cpCount"></span></div><div id="cps"></div></div>
  <button class="btn ghost" id="stop">Stop and save</button>
 </div>
</section>
<!-- ================= SENSORS ================= -->
<section id="t-sensors" class="hide">
 <div class="card"><div class="ch"><b>LIDAR</b><span id="lidInfo">-</span></div>
  <canvas id="cv" width="360" height="360"></canvas>
  <div class="kv"><div><span>Front</span><b id="lF">-</b></div><div><span>Back</span><b id="lB">-</b></div>
   <div><span>Left</span><b id="lL">-</b></div><div><span>Right</span><b id="lR">-</b></div></div></div>
 <div class="card"><div class="ch"><b>ULTRASONIC</b><span>cm · red under 40, amber under 70</span></div><div id="usBars"></div></div>
 <div class="two">
  <div class="card"><div class="ch"><b>IMU</b></div><div id="imuBox">-</div></div>
  <div class="card"><div class="ch"><b>GPS</b></div><div id="gpsBox">-</div></div></div>
</section>
<!-- ================= RESULTS ================= -->
<section id="t-results" class="hide">
 <select id="runSel"></select>
 <div class="card" id="resCard"><div class="ch"><b>LAST RUN</b><span id="resHead">-</span></div>
  <div class="chips" id="resChips" style="margin-bottom:8px"></div>
  <table><thead><tr><th>CHECK</th><th>EXPECTED</th><th>MEASURED</th><th></th></tr></thead><tbody id="resRows"></tbody></table>
  <div class="hint" id="resInfo"></div></div>
 <div class="card"><div class="ch"><b>FILES</b><span id="filesInfo"></span></div><div id="files">Loading...</div>
  <div class="fbtns"><button id="dlall">Download all</button><button class="o" id="copyRow">Copy row for sheet</button></div>
  <button class="small-btn d" id="delrun">Delete this run</button><button class="small-btn d" id="delall">Delete all saved runs</button></div>
</section>
</main>
<nav><div><button data-tab="setup" class="sel">Setup</button><button data-tab="run">Run</button><button data-tab="sensors">Sensors</button><button data-tab="results">Results</button></div></nav>
</div>
<script>
const $=i=>document.getElementById(i);
const esc=s=>(''+s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
const mmss=ms=>{const s=Math.max(0,Math.floor(ms/1000));return Math.floor(s/60)+':'+String(s%60).padStart(2,'0');};
const store={get:k=>{try{return localStorage.getItem('tl.'+k)}catch(e){return null}},set:(k,v)=>{try{localStorage.setItem('tl.'+k,v)}catch(e){}}};
// mode -> label, section, record time, first and last waypoint (same tables as TestLogger.h)
const ZONES={1:['Zone 1','B door to C · R1',90,0,2],2:['Zone 2','F to E · R2',180,2,3],3:['Zone 3','D/E to K · R3',120,3,5],
 4:['Zone 4','K to H',60,5,7],0:['Full path','B door to H',450,0,7],5:['Sensor bench','no driving',600,0,0]};
const TYPES=['Sensors only','Manual drive','AUTO'];
const CHECKS=['Robot on the start point, facing the route','Battery charged, wheels free',
 'Operator phone on the driving page','Route clear of people and bags'];
let S=null,V=null,tab='setup',zone=+(store.get('zone')||1),type=+(store.get('type')||1),wasRec=null,runs=[],resRun='';
$('host').textContent=location.host||'box';

// ---------- tabs
function show(t){tab=t;['setup','run','sensors','results'].forEach(x=>$('t-'+x).classList.toggle('hide',x!==t));
 document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('sel',b.dataset.tab===t));
 if(t==='results')files();window.scrollTo(0,0);}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>show(b.dataset.tab));

// ---------- setup
function drawZones(){$('zones').innerHTML=[1,2,3,0,4,5].map(z=>{const Z=ZONES[z];
 return `<button class="opt${z===zone?' sel':''}" data-z="${z}">${Z[0]}<small>${Z[1]} · ${mmss(Z[2]*1000)}</small></button>`;}).join('');
 document.querySelectorAll('#zones .opt').forEach(b=>b.onclick=()=>{zone=+b.dataset.z;store.set('zone',zone);drawZones();drawTypes();});}
function drawTypes(){const bench=zone===5;
 document.querySelectorAll('#types button').forEach(b=>{const t=+b.dataset.t;b.disabled=bench&&t!==0;b.classList.toggle('sel',(bench?0:type)===t);});
 const d=S?S.cfgDrive:0,t=bench?0:type;
 $('typeHint').textContent=bench?'Sensor bench always records in place, the box never drives.':
  t===2?(d>0?'AUTO drives straight forward at '+d.toFixed(2)+' and holds the heading. Stops on its timer, Stop or E-STOP.':'AUTO is switched off on the box (setTestDrive is 0): it will only record.'):
  t===1?'Records only. The operator drives with the driving page.':'Records only. The robot stays where it is.';
 $('start').textContent='Start '+ZONES[zone][0]+' · '+TYPES[t];}
document.querySelectorAll('#types button').forEach(b=>b.onclick=()=>{type=+b.dataset.t;store.set('type',type);drawTypes();});
$('checks').innerHTML=CHECKS.map((c,i)=>`<label class="chk"><input type="checkbox" data-i="${i}"><span>${c}</span></label>`).join('');
const ticks=()=>[...document.querySelectorAll('#checks input')];
ticks().forEach(c=>c.onchange=()=>{$('ckCount').textContent=ticks().filter(x=>x.checked).length+' of '+CHECKS.length;});
$('start').onclick=()=>{
 const missing=CHECKS.length-ticks().filter(x=>x.checked).length;
 if(missing&&!confirm(missing+' before-start item(s) not ticked. Start anyway?'))return;
 const t=zone===5?0:type;
 if(t===2&&S&&S.cfgDrive>0&&!confirm('AUTO: the robot drives forward by itself. Start?'))return;
 post('start?zone='+zone+'&type='+t).then(()=>{ticks().forEach(x=>x.checked=false);$('ckCount').textContent='0 of '+CHECKS.length;show('run');});};

// ---------- commands
function post(u){return fetch('/test/'+u,{method:'POST'}).then(()=>setTimeout(poll,250)).catch(()=>{});}
const PAGE_ID=Math.floor(Math.random()*4e9)+1;let seq=0;
function estop(){seq++;fetch('/drive?l=0.00&r=0.00&id='+PAGE_ID+'&n='+seq).catch(()=>{});post('stop');}
$('estop1').onclick=estop;$('estop2').onclick=estop;
$('mark').onclick=()=>post('mark');$('undo').onclick=()=>post('undo');$('obs').onclick=()=>post('obstacle');$('next').onclick=()=>post('next');
$('stop').onclick=()=>{if(confirm('Stop and save this run?'))post('stop');};

// ---------- run tab
function drawRun(){
 const rec=S&&S.rec;$('idle').classList.toggle('hide',!!rec);$('live').classList.toggle('hide',!rec);if(!rec)return;
 const Z=ZONES[S.mode]||ZONES[1];
 $('recRun').textContent='REC '+S.run;
 $('recSub').textContent=(S.mode===0?'Full path, zone '+S.zone:Z[0])+' · '+TYPES[S.type]+(S.drive>0?' '+S.drive.toFixed(2):'');
 $('recTime').textContent=mmss(S.zoneMs);$('recOf').textContent='of '+mmss(S.totalMs);
 $('zoneBar').style.width=Math.min(100,100*S.zoneMs/S.totalMs)+'%';
 const first=S.wpFirst,last=S.wpLast,n=last-first+1,done=Math.min(n,Math.max(0,S.wp-first));
 const res={};S.wps.forEach(w=>res[w.wp]=w);
 if(S.avgLeft>=0){$('nextTitle').textContent='Averaging WP'+S.wp;$('nextHint').textContent=S.avgLeft+' s left - keep the robot still';}
 else if(S.wp>last){$('nextTitle').textContent='All checkpoints marked';$('nextHint').textContent='Press Stop and save, or let the timer end the run.';}
 else{$('nextTitle').textContent='CP'+(S.wp-first+1)+' · WP'+S.wp;$('nextHint').textContent='Stop on the point, then press Mark: it averages GPS and heading for 20 s.';}
 $('nextCount').textContent=Math.min(n,done+1)+' of '+n;
 $('liveInfo').textContent=mmss(S.zoneMs)+' of '+mmss(S.totalMs)+' · ramps '+S.ramps+' of '+S.rampsExp+' · front '+S.front+' cm'+
  (S.drive>0?(S.yawRef?' · heading '+(S.yawErr>=0?'+':'')+S.yawErr.toFixed(1)+'°':' · heading: no reference yet'):'')+
  (S.obstacles?' · '+S.obstacles+' obstacle(s) logged':'')+(S.fix?'':' · no GPS fix');
 $('mark').disabled=S.avgLeft>=0||S.wp>last;$('undo').disabled=S.avgLeft<0&&S.wp<=first;
 $('next').classList.toggle('hide',S.mode!==0||S.zone>=4);
 let h='';for(let w=first;w<=last;w++){const r=res[w];let c='',tag='',sub='GPS + heading, 20 s still';
  if(r){c=r.res?'f':'p';tag=r.res?'Fail':'Pass';sub=r.res===2?'No GPS fix - undo and mark again':r.res===1?'Moved while averaging - undo and mark again':'GPS spread '+r.spread.toFixed(1)+' m';}
  else if(w===S.wp&&S.avgLeft>=0){c='w';tag='Averaging';sub=S.avgLeft+' s left';}
  else if(w===S.wp){c='n';tag='Next';}
  h+=`<div class="cp"><div class="num ${c}">${w-first+1}</div><div class="x">WP${w}<small>${sub}</small></div>${tag?`<span class="tag ${tag}">${tag}</span>`:''}</div>`;}
 $('cps').innerHTML=h;$('cpCount').textContent=S.wps.filter(w=>!w.res).length+' of '+n+' marked';}

// ---------- sensors (live, from the driving page's /values)
function sector(d,a,b){let m=0;for(let i=a;i<=b;i++){const v=d[(i+90)%90];if(v&&(!m||v<m))m=v;}return m;}
function drawSensors(){
 if(!V||!V.sensors)return;const s=V.sensors,L=s.lidar||{},d=L.d||[];
 $('lidInfo').textContent=L.ok?L.pps+' packets/s · rings 1 m':'no data';
 const cv=$('cv'),g=cv.getContext('2d'),W=cv.width,c=W/2,MAX=300,k=(c-14)/MAX;
 g.clearRect(0,0,W,W);g.strokeStyle='#d5dae3';g.lineWidth=1.5;
 for(let m=100;m<=MAX;m+=100){g.beginPath();g.arc(c,c,m*k,0,2*Math.PI);g.stroke();}
 g.fillStyle='#3d6fd6';const N=d.length,st=360/Math.max(N,1);
 d.forEach((cm,i)=>{if(!cm||cm>MAX)return;const a=(i*st+st/2)*Math.PI/180;g.beginPath();g.arc(c+cm*k*Math.sin(a),c-cm*k*Math.cos(a),3,0,2*Math.PI);g.fill();});
 g.fillStyle='#7b3fc4';g.beginPath();g.moveTo(c,c-16);g.lineTo(c-10,c+10);g.lineTo(c+10,c+10);g.fill();
 const f=v=>v?(v/100).toFixed(2)+' m':'-';
 $('lF').textContent=f(sector(d,-2,2));$('lR').textContent=f(sector(d,20,25));$('lB').textContent=f(sector(d,43,47));$('lL').textContent=f(sector(d,65,70));
 const us=s.us||[],names=['Front','Left','Right','Back','Front angled'],mx=s.usMax||300;
 $('usBars').innerHTML=names.map((n,i)=>{const v=us[i],far=v===undefined||v>=mx,col=far?'#3d6fd6':v<40?'#e53935':v<70?'#e69500':'#3d6fd6';
  return `<div class="us"><span>${n}</span><div class="tr"><i style="width:${far?100:Math.min(100,100*v/mx)}%;background:${col}"></i></div><b>${v===undefined?'-':far?mx+'+':v}</b></div>`;}).join('');
 const I=s.imu||{},G=s.gps||{};
 $('imuBox').innerHTML=I.ok?`<div class="bigv">${I.yaw}°<small> yaw</small></div><div class="bigv" style="font-size:20px">${I.pitch}°<small> pitch</small></div><div class="hint">roll ${I.roll}° · cal ${(I.cal||[]).join('/')}</div>`:'<div class="hint">IMU not found</div>';
 $('gpsBox').innerHTML=G.chars?`<div class="bigv">${G.sats}<small> sats</small></div><div class="bigv" style="font-size:20px">${G.hdop}<small> HDOP</small></div><div class="hint">${G.fix?G.lat+', '+G.lon:'no fix'}</div>`:'<div class="hint">no data from the module</div>';}
function drawChips(){
 const c=[];if(V&&V.sensors){const s=V.sensors,I=s.imu||{},L=s.lidar||{},G=s.gps||{},mx=s.usMax||300;
  c.push(I.ok?['g','IMU cal '+(I.cal||[]).join('/')]:['r','IMU not found']);
  c.push(L.ok?['g','LiDAR '+L.pps+' pkt/s']:['r','LiDAR no data']);
  const near=(s.us||[]).filter(v=>v<mx).length;c.push(['','Ultrasonic '+near+' of 5 see an object']);
  c.push(!G.chars?['r','GPS no data']:G.fix?['g','GPS fix ('+G.sats+' sats)']:['r','GPS no fix ('+G.sats+' sats)']);
  const v=(V.values||[]).find(x=>x.name==='rover.voltage_V'),volt=v?parseFloat(v.value):0;
  c.push(volt>0?[volt<10.5?'a':'g','Rover '+volt.toFixed(1)+' V']:['a','Rover not connected']);
  $('sReady').textContent=I.ok&&L.ok?'ready':'check sensors';}
 if(S)c.push(!S.fs?['r','No flash file system']:[S.freeMin<3?'a':'g','Flash '+S.freeKb+' kB · ~'+S.freeMin+' min']);
 $('chips').innerHTML=c.map(x=>`<span class="chip ${x[0]}">${esc(x[1])}</span>`).join('');}

// ---------- results: read the run's events file(s) back and judge each row
function link(run,n){return '/test/file?path='+encodeURIComponent('/'+run+'/'+n);}
function parseCsv(t){const L=t.trim().split(/\r?\n/),h=L.shift().split(',');
 return L.map(l=>{const p=l.split(','),o={};h.forEach((k,i)=>o[k]=i<h.length-1?p[i]:p.slice(i).join(','));return o;});}
function modeOf(run){return run[0]==='F'?0:run[0]==='S'?5:+run[1];}
function judge(rows,run){
 const out=[],ok={},z=ZONES[modeOf(run)]||ZONES[1];let dur=0,type='',rampSeen=0;
 for(const e of rows){const t=+e.t_ms||0;if(t>dur)dur=t;const n=e.note||'';
  switch(e.event){
  case 'TYPE':type=n;break;
  case 'WP':{const pass=n==='ok';if(pass)ok[e.wp]=1;
   out.push(['WP'+e.wp,'still, GPS fix','spread '+e.spread_m+' m · rms '+e.rms_m+' m · yaw '+e.yaw_deg+'°',pass?'Pass':'Fail',pass?'':n,t]);break;}
  case 'RAMP':{rampSeen++;const w=/^route ramp/.test(n)?'Pass':'Warn';
   out.push(['Ramp '+e.ramp,'route ramp, peak ≤ 4.7°','pitch '+e.pitch_min+' to '+e.pitch_max+'° · '+(e.duration_ms/1000).toFixed(1)+' s · '+e.first+' first',w,n,t]);break;}
  case 'OBSTACLE':out.push(['Obstacle','-',n,'Warn','logged by hand',t]);break;
  case 'DRIFT_ABORT':out.push(['Heading hold','error < 25°',n,'Fail','run aborted',t]);break;
  case 'DRIVE_ABORT':out.push(['Path ahead','clear',n,'Fail','run aborted',t]);break;
  case 'WALL_LOST':out.push(['Side wall','held','lost',"Warn",n,t]);break;
  case 'WALL_REF':out.push(['Side wall','-',n,'Info','',t]);break;
  case 'PITCH_ZERO':case 'HEADING_REF':out.push([e.event==='PITCH_ZERO'?'Pitch zero':'Heading ref','-',n,'Info','',t]);break;
  case 'AUTO_STOP':out.push(['Zone time','full time',n,'Pass','',t]);break;
  case 'STOP':out.push(['Stop','-',n,'Info','',t]);break;
  case 'UNDO':case 'WP_CANCELLED':out.push([e.event==='UNDO'?'Undo':'WP cancelled','-',n,'Info','',t]);break;
  case 'ZONE_SUMMARY':{const m=/ramps (\d+) of (\d+).*fix (\d+)%/.exec(n);
   if(m){out.push(['Ramps found',m[2],m[1],m[1]===m[2]?'Pass':'Warn','',t]);out.push(['GPS fix','-',m[3]+' % of samples',+m[3]?'Info':'Warn','',t]);}break;}}}
 let notMarked=0;for(let w=z[3];w<=z[4];w++)if(!ok[w])notMarked++;
 return {out,dur,type,notMarked};}
// Files of the selected run, one row each, in the logger's file order.
const FORDER=['imu','lidar','us','gps','events','features'];
const fkey=n=>{const m=/_z(\d)_(\w+)\.csv$/.exec(n);return m?m[1]*10+FORDER.indexOf(m[2]):99;};
function drawFiles(run){
 const r=runs.find(x=>x.run===run),fs=r?r.files.slice().sort((a,b)=>fkey(a.n)-fkey(b.n)):[];
 $('files').innerHTML=fs.map(f=>`<div class="fr"><span class="fn">${esc(f.n)}<small>${f.s<1024?f.s+' B':Math.round(f.s/1024)+' KB'}</small></span>`+
  `<a class="dl" href="${link(run,f.n)}" download>Download</a></div>`).join('')||'<p>No saved runs yet.</p>';
 ['dlall','copyRow','delrun'].forEach(i=>$(i).disabled=!fs.length);}
// One tab-separated line per run: pastes into one spreadsheet row.
let sheetRow='';
$('copyRow').onclick=()=>{if(!sheetRow)return;const t=document.createElement('textarea');t.value=sheetRow;
 document.body.appendChild(t);t.select();let ok=false;try{ok=document.execCommand('copy');}catch(e){}t.remove();
 if(!ok&&navigator.clipboard)navigator.clipboard.writeText(sheetRow).then(()=>{},()=>{});
 const b=$('copyRow');b.textContent='Copied';setTimeout(()=>b.textContent='Copy row for sheet',1500);};
$('dlall').onclick=()=>dlAll(resRun);$('delrun').onclick=()=>del(resRun);
function showResult(run){
 resRun=run;drawFiles(run);sheetRow='';
 if(!run){$('resHead').textContent='no saved runs';$('resRows').innerHTML='';$('resChips').innerHTML='';$('resInfo').textContent='';return;}
 const r=runs.find(x=>x.run===run);if(!r)return;
 const ev=r.files.filter(f=>/_events\.csv$/.test(f.n)).sort((a,b)=>a.n.localeCompare(b.n));
 if(S&&S.rec&&S.run===run){$('resHead').textContent=run+' is still recording';$('resRows').innerHTML='';$('resChips').innerHTML='';$('resInfo').textContent='Results appear when the run is saved.';return;}
 Promise.all(ev.map(f=>fetch(link(run,f.n)).then(x=>x.text()))).then(txts=>{
  const rows=[].concat(...txts.map(parseCsv)),j=judge(rows,run),Z=ZONES[modeOf(run)]||ZONES[1];
  $('resHead').textContent=run+' · '+Z[0]+(j.type?' · '+j.type.split(',')[0]:'')+' · '+mmss(j.dur);
  const cnt=k=>j.out.filter(x=>x[3]===k).length;
  $('resChips').innerHTML=`<span class="chip g">${cnt('Pass')} pass</span><span class="chip a">${cnt('Warn')} warn</span><span class="chip r">${cnt('Fail')} fail</span><span class="chip">${j.notMarked} not marked</span>`;
  $('resRows').innerHTML=j.out.map(x=>`<tr><td>${esc(x[0])}<small>${mmss(x[5])}${x[4]?' · '+esc(x[4]):''}</small></td><td>${esc(x[1])}</td><td>${esc(x[2])}</td><td><span class="tag ${x[3]}">${x[3]}</span></td></tr>`).join('')||'<tr><td colspan="4">No events in this run.</td></tr>';
  $('resInfo').textContent='';
  // run, zone, type, duration, pass, warn, fail, not marked, obstacles, ramps found / expected
  const rf=j.out.find(x=>x[0]==='Ramps found');
  sheetRow=[run,Z[0],(j.type||'').split(',')[0],mmss(j.dur),cnt('Pass'),cnt('Warn'),cnt('Fail'),j.notMarked,
   j.out.filter(x=>x[0]==='Obstacle').length,rf?rf[2]+' of '+rf[1]:''].join('\t');}).catch(()=>{$('resHead').textContent='could not read '+run;});}
$('runSel').onchange=()=>showResult($('runSel').value);
function files(){fetch('/test/files').then(r=>r.json()).then(j=>{
 runs=j.runs.sort((a,b)=>a.run.localeCompare(b.run,undefined,{numeric:true}));let sz=0;
 runs.forEach(r=>r.files.forEach(f=>sz+=f.s));
 $('filesInfo').textContent='LittleFS · '+Math.round(sz/1024)+' KB'+(j.more?' · delete old runs':'');
 const saved=runs.filter(r=>r.run!==j.rec).map(r=>r.run);
 if(!saved.includes(resRun))resRun=S&&!S.rec&&saved.includes(S.run)?S.run:saved[saved.length-1]||'';
 $('runSel').innerHTML=saved.slice().reverse().map(r=>`<option${r===resRun?' selected':''}>${esc(r)}</option>`).join('');
 showResult(resRun);}).catch(()=>{});}
function dlAll(run){const r=runs.find(x=>x.run===run);if(!r)return;let i=0;
 (function next(){if(i>=r.files.length)return;const a=document.createElement('a');a.href=link(run,r.files[i++].n);
  a.download='';document.body.appendChild(a);a.click();a.remove();setTimeout(next,900);})();}
function del(run){if(confirm('Delete '+run+'?')){if(run===resRun)resRun='';post('delete?run='+encodeURIComponent(run)).then(()=>setTimeout(files,800));}}
$('delall').onclick=()=>{if(confirm('Delete ALL saved runs?')){resRun='';post('delete?run=*').then(()=>setTimeout(files,800));}};

// ---------- polling
function online(on){$('conn').className='pill '+(on?'on':'off');$('conn').lastChild.textContent=on?'Online':'Offline';}
function poll(){fetch('/test/state',{cache:'no-store'}).then(r=>r.json()).then(j=>{S=j;online(true);
 $('msg').textContent=S.fs?(S.note||'Ready.'):'No flash file system - nothing can be saved';
 if(wasRec===null&&S.rec&&tab==='setup')show('run');
 else if(wasRec!==null&&wasRec!==!!S.rec){if(S.rec)show('run');else{resRun=S.run;show('results');}}
 wasRec=!!S.rec;$('start').disabled=!!S.rec||!S.fs;
 drawRun();drawChips();if(tab==='setup')drawTypes();}).catch(()=>{online(false);$('msg').textContent='No connection to the box';});}
function pollValues(){fetch('/values',{cache:'no-store'}).then(r=>r.json()).then(j=>{V=j;drawChips();if(tab==='sensors')drawSensors();}).catch(()=>{});}
drawZones();drawTypes();poll();pollValues();
{const h=location.hash.slice(1);if(['run','sensors','results'].includes(h))show(h);}   // e.g. /test#results
setInterval(poll,1000);setInterval(pollValues,500);
</script></body></html>)HTML";

inline void addTestLogRoutes(AsyncWebServer& server, TestLogger& tl) {
  // specific routes first: "/test" alone would also match "/test/..."
  server.on("/test/status", HTTP_GET, [&tl](AsyncWebServerRequest* r) {
    char s[448];
    tl.copyStatus(s, sizeof s);
    r->send(200, "text/plain", s);
  });
  // Team B4: the same status as JSON, for the tabbed page.
  server.on("/test/state", HTTP_GET, [&tl](AsyncWebServerRequest* r) {
    char* buf = static_cast<char*>(malloc(1536));
    if (!buf) { r->send(500, "text/plain", "out of memory"); return; }
    tl.stateJson(buf, 1536);
    r->send(200, "application/json", buf);
    free(buf);
  });
  server.on("/test/files", HTTP_GET, [&tl](AsyncWebServerRequest* r) {
    const size_t n = 8192;
    char* buf = static_cast<char*>(malloc(n));
    if (!buf) { r->send(500, "text/plain", "out of memory"); return; }
    tl.listJson(buf, n);
    r->send(200, "application/json", buf);
    free(buf);
  });
  // zone = 0 full path, 1..4, 5 sensor bench. Team B4: optional type (0 sensors only, 1 manual,
  // 2 AUTO - the default, as before).
  server.on("/test/start", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    if (!r->hasParam("zone")) { r->send(400, "text/plain", "zone missing"); return; }
    const int z = r->getParam("zone")->value().toInt();
    if (z < 0 || z > TestLogger::kBench) { r->send(400, "text/plain", "zone must be 0 to 5"); return; }
    const int type = r->hasParam("type") ? r->getParam("type")->value().toInt() : (int)TestLogger::TYPE_AUTO;
    tl.requestStart((uint8_t)z, (uint8_t)constrain(type, 0, 2));
    r->send(200, "text/plain", "ok");
  });
  server.on("/test/stop", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    tl.requestStop(); r->send(200, "text/plain", "ok");
  });
  server.on("/test/mark", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    tl.requestMark(); r->send(200, "text/plain", "ok");
  });
  server.on("/test/undo", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    tl.requestUndo(); r->send(200, "text/plain", "ok");
  });
  server.on("/test/obstacle", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    tl.requestObstacle(); r->send(200, "text/plain", "ok");
  });
  server.on("/test/next", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    tl.requestNextZone(); r->send(200, "text/plain", "ok");
  });
  server.on("/test/delete", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    if (!r->hasParam("run")) { r->send(400, "text/plain", "run missing"); return; }
    tl.requestDelete(r->getParam("run")->value().c_str());
    r->send(200, "text/plain", "ok");
  });
  server.on("/test/file", HTTP_GET, [&tl](AsyncWebServerRequest* r) {
    if (!r->hasParam("path")) { r->send(400, "text/plain", "path missing"); return; }
    const String p = r->getParam("path")->value();
    const char* path = p.c_str();
    if (path[0] != '/' || strstr(path, "..") || !strstr(path, ".csv")) { r->send(400, "text/plain", "bad path"); return; }
    if (tl.isActivePath(path))  { r->send(409, "text/plain", "Stop the run first"); return; }
    if (!LittleFS.exists(path)) { r->send(404, "text/plain", "No such file"); return; }
    r->send(LittleFS, path, "text/csv", true);   // true = save as a file
  });
  server.on("/test", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(200, "text/html", kTestLogPage);
  });
}
