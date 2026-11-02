#include "WebHMI.h"

#include "TestLoggerWeb.h"      // at the top of WebHMI.cpp
extern TestLogger testLog;      // the object from cas_box.ino
#include "NavSurveyWeb.h"       // /survey: live LiDAR geometry for calibration
extern NavSurvey navSurvey;     // the object from main.cpp


// ---------------------------------------------------------------------------------------------
// The web page. %TITLE% and %MAXSPD% are filled in by _buildPage().
// ---------------------------------------------------------------------------------------------
static const char kPage[] PROGMEM = R"rawpage(<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'><title>%TITLE%</title>
<style>
:root{--navy:#1f3864;--bg:#eef1f6;--card:#fff;--line:#e3e7ee;--text:#1b2433;--muted:#6b7587;--red:#e53935;
 --redbg:#fde8e8;--green:#2e7d32;--greenbg:#e5f4e6;--amberbg:#fff3dc;--grey:#eef0f4;--blue:#3d6fd6;}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;}
.app{max-width:520px;margin:0 auto;min-height:100vh;padding-bottom:76px;}
header{background:var(--navy);color:#fff;padding:12px 14px;display:flex;align-items:center;gap:8px;position:sticky;top:0;z-index:5;}
header .t{flex:1;min-width:0}header h1{font-size:19px;margin:0}header small{opacity:.75;font-size:12px}
.pill{background:rgba(255,255,255,.14);border-radius:999px;padding:5px 11px;font-size:13px;white-space:nowrap}
.pill i{display:inline-block;width:8px;height:8px;border-radius:50%;background:#9aa4b2;margin-right:6px}
.pill.on i{background:#4cd964}.pill.off i{background:var(--red)}
.estop{background:var(--red);color:#fff;border:0;border-radius:10px;font-weight:800;padding:9px 12px;font-size:15px;letter-spacing:.5px}
.msg{background:#e2e7f0;color:#34405a;font-size:14px;padding:8px 14px;text-align:center}
main{padding:10px 12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:12px 14px;margin-bottom:12px}
.ch{display:flex;justify-content:space-between;align-items:baseline;gap:8px;margin-bottom:8px}
.ch b{font-size:13px;letter-spacing:.6px;color:#4a5468}.ch span{font-size:13px;color:var(--muted);text-align:right}
.hint{font-size:13px;color:var(--muted);margin-top:8px}
.hidden{display:none!important}
.pad{display:grid;grid-template-columns:repeat(3,84px);grid-template-rows:repeat(3,84px);gap:10px;
 justify-content:center;margin:8px 0 4px;touch-action:none;user-select:none;-webkit-user-select:none;}
.pad button{font-size:28px;touch-action:none;border:0;border-radius:16px;background:var(--navy);color:#fff;box-shadow:0 3px 0 #15284a}
.pad button.stop{background:var(--red);box-shadow:0 3px 0 #b71c1c}
.pad button.on{background:#7b3fc4;box-shadow:0 0 0 4px #d9c6f2}
.sl{margin:4px 0 12px}.sl label{display:flex;justify-content:space-between;font-size:15px;color:#4a5468}
.sl label b{color:var(--navy)}input[type=range]{width:100%;accent-color:var(--navy);margin-top:6px}
.times{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.times label{font-size:13px;color:var(--muted)}
.times input{width:100%;margin-top:4px;padding:10px;font-size:18px;font-weight:650;border-radius:10px;border:1px solid var(--line);color:var(--text)}
.tot{display:inline-block;background:var(--greenbg);color:var(--green);border-radius:999px;padding:3px 10px;font-weight:700}
.links{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.links a{display:block;text-align:center;border:2px solid var(--navy);color:var(--navy);border-radius:12px;padding:12px 6px;font-weight:700;text-decoration:none}
canvas{display:block;width:100%;max-width:360px;margin:0 auto;background:#f6f8fb;border-radius:10px}
.kv{display:grid;grid-template-columns:1fr 1fr;gap:4px 18px;margin-top:10px}.kv div{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid var(--line)}
.us{display:grid;grid-template-columns:96px 1fr 44px;align-items:center;gap:8px;margin:6px 0;font-size:15px}
.us .tr{height:16px;background:var(--grey);border-radius:4px;overflow:hidden}.us .tr i{display:block;height:100%}
.us b{text-align:right}
.two{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.bigv{font-size:26px;font-weight:800}.bigv small{font-size:14px;font-weight:400;color:var(--muted)}
table{width:100%;border-collapse:collapse;font-size:14px;table-layout:fixed;overflow-wrap:anywhere}
td{padding:8px 4px;border-bottom:1px solid var(--line)}.k{color:var(--muted)}
nav{position:fixed;bottom:0;left:0;right:0;background:#fff;border-top:1px solid var(--line);display:flex;justify-content:center;z-index:5}
nav div{display:flex;width:100%;max-width:520px}
nav button{flex:1;border:0;background:none;padding:10px 0 12px;font-size:15px;color:#8a93a3}
nav button:before{content:"";display:block;width:30px;height:4px;border-radius:4px;background:#d5dae3;margin:0 auto 6px}
nav button.sel{color:var(--navy);font-weight:700}nav button.sel:before{background:var(--navy)}
</style></head><body><div class='app'>
<header><div class='t'><h1>%TITLE%</h1><small id='host'></small></div>
 <span class='pill' id='conn'><i></i>...</span><button class='estop' id='estop'>STOP</button></header>
<div class='msg' id='state'>Connecting...</div>
<main>
<!-- ================= DRIVE ================= -->
<section id='t-drive'>
 <!-- Team B4: One press drives for the time selected below; the box measures it, not the phone. -->
 <div class='card hidden' id='driveCard'><div class='ch'><b>DRIVE</b><span>one press = one timed move · &#9632; stops</span></div>
  <div class='pad'>
   <span></span><button data-d='f'>&#9650;</button><span></span>
   <button data-d='l'>&#9664;</button><button class='stop' data-d='s'>&#9632;</button><button data-d='r'>&#9654;</button>
   <span></span><button data-d='b'>&#9660;</button><span></span>
  </div>
  <div class='hint' style='text-align:center'>Runs for <span class='tot' id='tTotal'>-</span> and stops by itself.</div></div>
 <div class='card hidden' id='speedCard'><div class='ch'><b>SPEED</b><span>0.10 to max, used on the next press</span></div>
  <!-- Team B4: Both speed controls allow 0.10 to 0.50, applied on the next direction press. -->
  <div class='sl'><label for='spd'>Driving / fast turning <b id='spdVal'></b></label><input type='range' id='spd' min='0.10' step='0.01'></div>
  <div class='sl'><label for='turnSpd'>Slow turning (inner wheel) <b id='turnSpdVal'></b></label><input type='range' id='turnSpd' min='0.10' step='0.01'></div></div>
 <!-- Team B4: Move time, selected as seconds + milliseconds and enforced by the box. -->
 <div class='card hidden' id='timeCard'><div class='ch'><b>MOVE TIME</b><span>timed on the box</span></div>
  <div class='times'>
   <label for='tSec'>Seconds (0 - 5)<input type='number' id='tSec' min='0' max='5' step='1' value='0' inputmode='numeric'></label>
   <label for='tMs'>Milliseconds (0 - 999)<input type='number' id='tMs' min='0' max='999' step='10' value='500' inputmode='numeric'></label>
  </div></div>
 <div class='card hidden' id='noPad'><b>Drive pad is off.</b><div class='hint'>enableDrivePad() is not called in main.cpp.</div></div>
 <div class='card'><div class='ch'><b>OTHER PAGES</b></div>
  <div class='links'><a href='/test'>Zone tests</a><a href='/survey'>LiDAR survey</a></div></div>
</section>
<!-- ================= SENSORS ================= -->
<section id='t-sensors' class='hidden'>
 <div class='card'><div class='ch'><b>LIDAR</b><span id='lidarInfo'>-</span></div>
  <canvas id='envCv' width='360' height='360'></canvas>
  <div class='kv'><div><span>Front</span><b id='lF'>-</b></div><div><span>Back</span><b id='lB'>-</b></div>
   <div><span>Left</span><b id='lL'>-</b></div><div><span>Right</span><b id='lR'>-</b></div></div>
  <div class='hint'>Top view, front = up, rings 1 m.</div></div>
 <div class='card'><div class='ch'><b>ULTRASONIC</b><span>cm · red under 40, amber under 70</span></div><div id='usBars'></div></div>
 <div class='two'>
  <div class='card'><div class='ch'><b>IMU</b></div><div id='imuBox'>-</div></div>
  <div class='card'><div class='ch'><b>GPS</b></div><div id='gpsBox'>-</div></div></div>
 <div class='hint' style='margin:0 4px 12px'>ISO 8855: roll + = right side down, pitch + = front down, yaw + = turning left (0 = north). Calibration 3 = good.</div>
</section>
<!-- ================= DEV ================= -->
<section id='t-dev' class='hidden'>
 <div class='card'><div class='ch'><b>VALUES</b></div><table id='valTable'></table></div>
 <div class='card'><div class='ch'><b>LOOP TIME (BOX)</b></div>
  <table><tr><td class='k'>min / avg / max</td><td id='cycle'>-</td></tr></table></div>
</section>
</main>
<nav><div><button data-tab='drive' class='sel'>Drive</button><button data-tab='sensors'>Sensors</button><button data-tab='dev'>Dev</button></div></nav>
</div>
<script>
const MAX_SPEED=%MAXSPD%, ENV_MAX_CM=300;
const $=id=>document.getElementById(id);
const esc=s=>(''+s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
$('host').textContent=location.host||'box';

// ---- tabs (Dev = the old "Dev" button: raw values and loop time)
let tab='drive';
function show(t){tab=t;['drive','sensors','dev'].forEach(x=>$('t-'+x).classList.toggle('hidden',x!==t));
  document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('sel',b.dataset.tab===t));window.scrollTo(0,0);}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>show(b.dataset.tab));
{const h=location.hash.slice(1);if(h==='sensors'||h==='dev')show(h);}   // e.g. /#sensors

// Team B4: Timed single-press drive. The wanted length travels to the box in the same request
// and the box measures it there, so a slow or lost packet no longer changes the distance driven.
// No heartbeat is needed: one press is one request.
const MIN_MOVE_MS=50,MAX_MOVE_MS=5000;
let drvBtn=null,drvOffTimer=null;
// Requests can overtake each other on Wi-Fi. Each command carries this page's random id and a
// rising number; the box ignores anything older than the last command it accepted.
const PAGE_ID=Math.floor(Math.random()*4e9)+1;let seq=0;
function drvSend(l,r,ms){seq++;
  fetch('/drive?l='+l.toFixed(2)+'&r='+r.toFixed(2)+'&id='+PAGE_ID+'&n='+seq+(ms>0?'&ms='+ms:'')).catch(()=>{});}
// Team B4: Seconds + milliseconds from the two inputs, clamped to what the box accepts.
function moveMs(){
  const num=(id,hi)=>Math.max(0,Math.min(hi,parseInt($(id).value,10)||0));
  return Math.max(MIN_MOVE_MS,Math.min(MAX_MOVE_MS,num('tSec',5)*1000+num('tMs',999)));}
// Team B4: The highlight only shows the time that was asked for; the box owns the real timing.
function drvLight(b,ms){
  clearTimeout(drvOffTimer);drvOffTimer=null;
  if(drvBtn)drvBtn.classList.remove('on');
  drvBtn=b;
  if(b){b.classList.add('on');
    drvOffTimer=setTimeout(()=>{b.classList.remove('on');if(drvBtn===b)drvBtn=null;},ms);}}
// Team B4: Stop sends a command without ms, which clears the box timer at once.
function drvStop(){drvLight(null,0);drvSend(0,0,0);}
$('estop').onclick=drvStop;
// Team B4: One press = one request = one timed move in the direction the user selected.
function drvStart(b){
  if(b.dataset.d==='s'){drvStop();return;}
  // Team B4: Accept slider speeds from 0.10 up to the configured maximum for every direction.
  const floor=Math.min(0.1,MAX_SPEED);
  const readSpeed=id=>Math.min(MAX_SPEED,Math.max(floor,parseFloat($(id).value)||floor));
  const s=readSpeed('spd');
  // Team B4: The reverse turning side cannot exceed the selected fast-side speed.
  const turnForward=s,turnBackward=Math.min(readSpeed('turnSpd'),s);
  // Team B4: Each pair is [left motor speed, right motor speed]; sign sets rotation direction.
  const m={f:[s,s],b:[-s,-s],l:[-turnBackward,turnForward],r:[turnForward,-turnBackward]}[b.dataset.d];
  const ms=moveMs();
  drvLight(b,ms);
  drvSend(m[0],m[1],ms);  // the box starts now and stops itself after ms
}
if(MAX_SPEED>0){
  ['driveCard','speedCard','timeCard'].forEach(id=>$(id).classList.remove('hidden'));
  // Team B4: Initialize the 0.10 minimum; keep the slower turning speed below the fast speed.
  const sp=$('spd'),turnSp=$('turnSpd'),floor=Math.min(0.1,MAX_SPEED);
  sp.min=floor;sp.max=MAX_SPEED;sp.value=MAX_SPEED;
  turnSp.min=floor;turnSp.max=sp.value;turnSp.value=floor;
  const showSpd=()=>{
    turnSp.max=sp.value;
    if(parseFloat(turnSp.value)>parseFloat(sp.value))turnSp.value=sp.value;
    $('spdVal').textContent=parseFloat(sp.value).toFixed(2);
    $('turnSpdVal').textContent=parseFloat(turnSp.value).toFixed(2);
  };
  sp.addEventListener('input',showSpd);turnSp.addEventListener('input',showSpd);showSpd();
  // Team B4: Show the selected move time; moveMs() clamps both fields to the allowed range.
  const showTime=()=>{const t=moveMs();$('tTotal').textContent=(t/1000).toFixed(2)+' s';};
  ['tSec','tMs'].forEach(id=>$(id).addEventListener('input',showTime));showTime();
  document.querySelectorAll('.pad button').forEach(b=>{
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);drvStart(b);});
    // Team B4: Releasing a normal press keeps the timed run; a cancelled gesture stops it.
    b.addEventListener('pointercancel',()=>{if(drvBtn===b)drvStop();});
    b.addEventListener('contextmenu',e=>e.preventDefault());
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&drvBtn)drvStop();});
  window.addEventListener('blur',()=>{if(drvBtn)drvStop();});
}else $('noPad').classList.remove('hidden');

// ---- LiDAR top view: line through neighbouring points, broken at "no return" and at jumps
function drawEnv(s){
  const cv=$('envCv'),g=cv.getContext('2d'),W=cv.width,c=W/2,k=(c-14)/ENV_MAX_CM;
  g.clearRect(0,0,W,W);
  g.strokeStyle='#d5dae3';g.lineWidth=1.5;
  for(let m=100;m<=ENV_MAX_CM;m+=100){g.beginPath();g.arc(c,c,m*k,0,2*Math.PI);g.stroke();}
  const d=(s.lidar&&s.lidar.d)||[],N=d.length,st=360/Math.max(N,1);
  const P=d.map((cm,i)=>{if(!cm||cm>ENV_MAX_CM)return null;const a=(i*st+st/2)*Math.PI/180;
    return [c+cm*k*Math.sin(a),c-cm*k*Math.cos(a),cm];});
  const near=(p,q)=>p&&q&&Math.abs(p[2]-q[2])<Math.max(30,0.25*Math.min(p[2],q[2]));
  g.strokeStyle='#3d6fd6';g.fillStyle='#3d6fd6';g.lineWidth=3;g.lineCap='round';
  for(let i=0;i<N;i++){const p=P[i];if(!p)continue;const q=P[(i+1)%N],o=P[(i+N-1)%N];
    if(near(p,q)){g.beginPath();g.moveTo(p[0],p[1]);g.lineTo(q[0],q[1]);g.stroke();}
    else if(!near(o,p)){g.beginPath();g.arc(p[0],p[1],2.5,0,2*Math.PI);g.fill();}}
  g.fillStyle='#7b3fc4';g.beginPath();g.moveTo(c,c-16);g.lineTo(c-10,c+10);g.lineTo(c+10,c+10);g.fill();  // box, front up
}
// nearest return in sectors a..b (4 deg each, 0 = front, clockwise)
function sector(d,a,b){let m=0;for(let i=a;i<=b;i++){const v=d[(i+90)%90];if(v&&(!m||v<m))m=v;}return m;}

// ---- sensor cards
function renderSensors(s){
  if(!s)return;
  drawEnv(s);
  const L=s.lidar||{},d=L.d||[];
  $('lidarInfo').textContent=L.ok?(L.pps+' packets/s · checksum errors '+L.crc+' %'):'no data';
  const f=v=>v?(v/100).toFixed(2)+' m':'-';
  $('lF').textContent=f(sector(d,-2,2));$('lR').textContent=f(sector(d,20,25));$('lB').textContent=f(sector(d,43,47));$('lL').textContent=f(sector(d,65,70));
  const us=s.us||[],names=['Front','Left','Right','Back','Front angled'],mx=s.usMax||300;
  $('usBars').innerHTML=names.map((n,i)=>{const v=us[i],far=v===undefined||v>=mx,col=far?'#3d6fd6':v<40?'#e53935':v<70?'#e69500':'#3d6fd6';
    return `<div class='us'><span>${n}</span><div class='tr'><i style='width:${far?100:Math.min(100,100*v/mx)}%;background:${col}'></i></div><b>${v===undefined?'-':far?mx+'+':v}</b></div>`;}).join('');
  const I=s.imu||{},G=s.gps||{};
  // ISO 8855: roll + = right side down, pitch + = front down, yaw + = turning left (0 = north)
  $('imuBox').innerHTML=I.ok?`<div class='bigv'>${I.yaw}&deg;<small> yaw</small></div><div class='bigv' style='font-size:20px'>${I.pitch}&deg;<small> pitch</small></div><div class='hint'>roll ${I.roll}&deg; · cal ${(I.cal||[]).join('/')}</div>`:`<div class='hint'>IMU not found</div>`;
  $('gpsBox').innerHTML=G.chars?`<div class='bigv'>${G.sats}<small> sats</small></div><div class='bigv' style='font-size:20px'>${G.hdop}<small> HDOP</small></div><div class='hint'>${G.fix?G.lat+', '+G.lon:'no fix'}</div>`:`<div class='hint'>no data from the module</div>`;
}

// ---- poll the box 4 times per second
function online(on){$('conn').className='pill '+(on?'on':'off');$('conn').lastChild.textContent=on?'Online':'Offline';}
async function poll(){
  try{
    const j=await (await fetch('/values',{cache:'no-store'})).json();
    online(true);
    $('state').textContent=j.state||'-';
    if(tab==='sensors')renderSensors(j.sensors);
    if(tab==='dev'){
      $('valTable').innerHTML=(j.values||[]).map(v=>`<tr><td class='k'>${esc(v.name)}</td><td>${esc(v.value)}</td></tr>`).join('');
      const c=j.cycle||{};$('cycle').textContent=`${c.min} / ${c.avg} / ${c.max} ms`;
    }
  }catch(e){online(false);$('state').textContent='No connection to the box';}
}
setInterval(poll,250);poll();
</script></body></html>)rawpage";

// Appends s to out as JSON string content (quotes and backslashes escaped, control chars dropped).
static void jsonEscape(String& out, const char* s) {
  for (; *s; ++s) {
    const char c = *s;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if ((uint8_t)c >= 0x20) out += c;
  }
}

// ---------------------------------------------------------------------------------------------

// Takes the mutex; creates it on first use (always in setup(), before the server task runs).
void WebHMI::_lock() {
  if (!_mtx) _mtx = xSemaphoreCreateMutex();
  xSemaphoreTake(_mtx, portMAX_DELAY);
}

void WebHMI::begin(const char* ssid, const char* pass, IPAddress ip, uint8_t channel) {
  _lock();  // make sure the mutex exists before the server starts
  xSemaphoreGive(_mtx);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                                  // no Wi-Fi power save on the box
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(ssid, pass, channel)) {
    Serial.printf("[WebHMI] AP %s, channel %d, IP %s\n", ssid, WiFi.channel(),
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WebHMI] WiFi.softAP failed");
  }

  _server = new AsyncWebServer(80);
  _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "text/html; charset=utf-8", _buildPage());
  });
  _server->on("/values", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", _buildJson());
  });
  _server->on("/drive", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("l") || !req->hasParam("r") || !req->hasParam("id") || !req->hasParam("n")) {
      req->send(400, "application/json", "{\"ok\":false}");
      return;
    }
    const float l = constrain(req->getParam("l")->value().toFloat(), -_driveMax, _driveMax);
    const float r = constrain(req->getParam("r")->value().toFloat(), -_driveMax, _driveMax);
    const uint32_t id = strtoul(req->getParam("id")->value().c_str(), nullptr, 10);
    const uint32_t n = strtoul(req->getParam("n")->value().c_str(), nullptr, 10);
    // ms > 0: timed move. The page sends the wanted length once and the box measures it, so the
    // distance driven no longer depends on a stop request arriving in time.
    uint32_t ms = 0;
    if (req->hasParam("ms")) ms = strtoul(req->getParam("ms")->value().c_str(), nullptr, 10);
    if (ms > kMaxMoveMs) ms = kMaxMoveMs;
    const uint32_t now = millis();
    bool fresh;
    portENTER_CRITICAL(&_driveMux);
    // a new page (id) starts over; within a page only a higher number counts
    fresh = (id != _drivePageId) || (n > _driveSeq);
    if (fresh) {
      _drivePageId = id;
      _driveSeq = n;
      _driveL = l;
      _driveR = r;
      _driveMs = now;
      _driveTimed = ms > 0;        // a command without ms (Stop, hold-to-drive) clears the timer
      _driveUntilMs = now + ms;
    }
    portEXIT_CRITICAL(&_driveMux);
    req->send(200, "application/json", fresh ? "{\"ok\":true}" : "{\"ok\":false,\"stale\":true}");
  });
  // next to your server.on("/drive", ...) lines:
  addTestLogRoutes(*_server, testLog);   // use WebHMI's real server variable name
  addSurveyRoutes(*_server, navSurvey);  // read only: the survey never commands the motors
  _server->begin();
  _cycleLastUs = micros();
  _cycleWinMs = millis();
}

void WebHMI::setTitle(const String& title) { _title = title; }

void WebHMI::enableDrivePad(float maxSpeed) {
  _drivePad = true;
  _driveMax = maxSpeed;
}

void WebHMI::setStateText(const String& text) {
  _lock();
  _state = text;
  xSemaphoreGive(_mtx);
}

void WebHMI::setSensorJson(const String& json) {
  _lock();
  _sensorJson = json;
  xSemaphoreGive(_mtx);
}

void WebHMI::showValue(const char* name, double value) { _setValue(name, String(value, 3)); }
void WebHMI::showValue(const char* name, const String& value) { _setValue(name, value); }

// Updates the value with this name or takes the first free slot (ignored if all are used).
void WebHMI::_setValue(const char* name, const String& text) {
  _lock();
  Value* slot = nullptr;
  for (Value& v : _values) {
    if (v.inUse && strncmp(v.name, name, sizeof(v.name)) == 0) { slot = &v; break; }
    if (!v.inUse && !slot) slot = &v;
  }
  if (slot) {
    if (!slot->inUse) {
      strlcpy(slot->name, name, sizeof(slot->name));
      slot->inUse = true;
    }
    strlcpy(slot->text, text.c_str(), sizeof(slot->text));
  }
  xSemaphoreGive(_mtx);
}

void WebHMI::getDrive(float& left, float& right, uint32_t& ageMs) const {
  portENTER_CRITICAL(&_driveMux);
  left = _driveL;
  right = _driveR;
  const uint32_t ms = _driveMs;
  const bool timed = _driveTimed;
  const uint32_t until = _driveUntilMs;
  portEXIT_CRITICAL(&_driveMux);
  const uint32_t now = millis();
  if (timed) {
    // While a timed move runs, the box itself is "holding the key": report age 0 so the 0.4 s
    // deadman in main.cpp cannot cut the move short. The move is still bounded - by its own end
    // time, at most kMaxMoveMs - and Stop clears it at once. RoverLink's 0.5 s deadman and the
    // rover's own 3 s timeout are untouched.
    if ((int32_t)(now - until) < 0) { ageMs = 0; return; }  // signed: survives the millis() wrap
    left = right = 0;
    ageMs = UINT32_MAX;
    return;
  }
  ageMs = ms ? now - ms : UINT32_MAX;  // millis() read after ms: never negative
}

// Loop time over 1 s windows: min / average / max time between two tick() calls.
void WebHMI::tick() {
  const uint32_t nowUs = micros();
  const uint32_t dt = nowUs - _cycleLastUs;
  _cycleLastUs = nowUs;
  if (_cycleCount == 0 || dt < _cycleMinUs) _cycleMinUs = dt;
  if (dt > _cycleMaxUs) _cycleMaxUs = dt;
  _cycleSumUs += dt;
  _cycleCount++;

  const uint32_t nowMs = millis();
  if (nowMs - _cycleWinMs < 1000) return;
  _cycleWinMs = nowMs;
  _lock();
  _cycleMinMs = _cycleMinUs / 1000.0f;
  _cycleMaxMs = _cycleMaxUs / 1000.0f;
  _cycleAvgMs = _cycleSumUs / 1000.0f / _cycleCount;
  xSemaphoreGive(_mtx);
  _cycleCount = 0;
  _cycleSumUs = 0;
  _cycleMaxUs = 0;
}

String WebHMI::_buildPage() const {
  String page = FPSTR(kPage);
  page.replace("%TITLE%", _title);
  page.replace("%MAXSPD%", _drivePad ? String(_driveMax, 2) : String("0"));
  return page;
}

// {"state":"..","values":[{"name":..,"value":..}],"cycle":{min,avg,max},"sensors":{..}|null}
String WebHMI::_buildJson() const {
  String out;
  out.reserve(1200);
  xSemaphoreTake(_mtx, portMAX_DELAY);
  out += "{\"state\":\"";
  jsonEscape(out, _state.c_str());
  out += "\",\"values\":[";
  bool first = true;
  for (const Value& v : _values) {
    if (!v.inUse) continue;
    if (!first) out += ',';
    first = false;
    out += "{\"name\":\"";
    jsonEscape(out, v.name);
    out += "\",\"value\":\"";
    jsonEscape(out, v.text);
    out += "\"}";
  }
  out += "],\"cycle\":{\"min\":";
  out += String(_cycleMinMs, 2);
  out += ",\"avg\":";
  out += String(_cycleAvgMs, 2);
  out += ",\"max\":";
  out += String(_cycleMaxMs, 2);
  out += "},\"sensors\":";
  out += _sensorJson.length() ? _sensorJson : String("null");
  out += '}';
  xSemaphoreGive(_mtx);
  return out;
}
