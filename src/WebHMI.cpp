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
:root{--bg:#0b0f17;--card:#121a27;--muted:#9aa4b2;--text:#e7edf5;--acc:#4ea1ff;--warn:#ffb020;}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;}
.wrap{max-width:980px;margin:0 auto;padding:16px;}
.top{display:flex;align-items:center;justify-content:space-between;gap:12px;}
.title{font-size:18px;font-weight:700;}
.pill{font-size:12px;color:var(--muted);padding:6px 10px;border:1px solid rgba(255,255,255,.08);border-radius:999px;}
.card{background:var(--card);border:1px solid rgba(255,255,255,.06);border-radius:16px;padding:14px;margin-top:12px;}
.label{font-size:12px;color:var(--muted);}
.value{font-size:16px;font-weight:650;}
.sectionTitle{font-size:13px;color:var(--muted);margin:0 0 8px;}
.hidden{display:none;}
button{appearance:none;border:0;border-radius:14px;font-weight:650;cursor:pointer;color:var(--text);
 background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.09);}
.btnTiny{padding:10px 12px;font-size:13px;}
.stop{background:linear-gradient(180deg,rgba(255,90,95,.95),rgba(255,90,95,.75));color:#190406;}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,72px);gap:10px;
 justify-content:center;margin:6px 0 10px;touch-action:none;user-select:none;-webkit-user-select:none;}
.pad button{font-size:24px;touch-action:none;}
.pad button.on{outline:3px solid var(--acc);}
input[type=range]{width:100%;}
.table{width:100%;border-collapse:collapse;font-size:13px;}
.table td{padding:8px 10px;border-bottom:1px solid rgba(255,255,255,.06);}
.k{color:var(--muted);}
canvas{display:block;margin:0 auto;max-width:100%;}
.times{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:6px 0 10px;}
.times input{width:100%;box-sizing:border-box;margin-top:4px;padding:10px;font-size:17px;font-weight:650;
 border-radius:10px;border:1px solid rgba(255,255,255,.16);background:rgba(255,255,255,.06);color:var(--text);}
.tot{color:var(--acc);font-weight:700;}
</style></head><body><div class='wrap'>
<div class='top'><div class='title'>%TITLE%</div><div class='pill' id='conn'>-</div>
 <button id='devBtn' class='btnTiny'>Dev</button></div>
<div class='card'><div class='label'>State</div><div class='value' id='state'>-</div></div>
<!-- Team B4: One press drives for the time selected below; the box measures it, not the phone. -->
<div class='card hidden' id='driveCard'><div class='sectionTitle'>Drive (one press = one timed move; Stop cancels)</div>
 <div class='pad'>
  <span></span><button data-d='f'>&#9650;</button><span></span>
  <button data-d='l'>&#9664;</button><button class='stop' data-d='s'>&#9632;</button><button data-d='r'>&#9654;</button>
  <span></span><button data-d='b'>&#9660;</button><span></span>
 </div>
 <!-- Team B4: Removed the automatic test button and sequence status; retain individual direction controls. -->
 <!-- Team B4: Both speed controls allow 0.10 to 0.50, applied on the next direction press. -->
 <label class='label' for='spd'>Driving / fast turning speed: <span id='spdVal'></span></label>
 <!-- Team B4: Lower the driving and fast turning slider minimum to 0.10. -->
 <input type='range' id='spd' min='0.10' step='0.01'>
 <label class='label' for='turnSpd'>Slow turning speed: <span id='turnSpdVal'></span></label>
 <!-- Team B4: Lower the slow turning slider minimum to 0.10. -->
 <input type='range' id='turnSpd' min='0.10' step='0.01'>
 <!-- Team B4: Move time, selected as seconds + milliseconds and enforced by the box. -->
 <div class='sectionTitle'>Move time (the box measures it, so Wi-Fi delay does not change it)</div>
 <div class='times'>
  <label class='label' for='tSec'>Seconds (0 - 5)
   <input type='number' id='tSec' min='0' max='5' step='1' value='0' inputmode='numeric'></label>
  <label class='label' for='tMs'>Milliseconds (0 - 999)
   <input type='number' id='tMs' min='0' max='999' step='10' value='500' inputmode='numeric'></label>
 </div>
 <div class='label'>Set the speeds and the time, then press an arrow: it runs for
  <span class='tot' id='tTotal'>-</span> and stops by itself.</div></div>
<div class='card'><div class='sectionTitle'>Surroundings (LiDAR top view, front = up, rings 1 m)</div>
 <canvas id='envCv' width='280' height='280'></canvas><div class='label' id='lidarInfo'>-</div>
 <table class='table' id='usTable'></table></div>
<div class='card'><div class='sectionTitle'>IMU / GPS</div><table class='table' id='navTable'></table></div>
<div id='devRoot' class='hidden'>
 <div class='card'><div class='sectionTitle'>Values</div><table class='table' id='valTable'></table></div>
 <div class='card'><div class='sectionTitle'>Loop time (box)</div>
  <table class='table'><tr><td class='k'>min / avg / max</td><td id='cycle'>-</td></tr></table></div>
</div></div>
<script>
const MAX_SPEED=%MAXSPD%, ENV_MAX_CM=300;
const $=id=>document.getElementById(id);
const esc=s=>(''+s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');

// ---- "Dev" view toggle
let dev=false;
$('devBtn').onclick=()=>{dev=!dev;$('devRoot').classList.toggle('hidden',!dev);$('devBtn').textContent=dev?'Hide':'Dev';};

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
// Team B4: Removed the sequence button handler; initialize individual controls only.
if(MAX_SPEED>0){
  $('driveCard').classList.remove('hidden');
  // Team B4: Initialize the 0.10 minimum; keep the slower turning speed below the fast speed.
  const sp=$('spd'),turnSp=$('turnSpd'),floor=Math.min(0.1,MAX_SPEED);
  sp.min=floor;sp.max=MAX_SPEED;sp.value=MAX_SPEED;
  turnSp.min=floor;turnSp.max=sp.value;turnSp.value=floor;
  const show=()=>{
    turnSp.max=sp.value;
    if(parseFloat(turnSp.value)>parseFloat(sp.value))turnSp.value=sp.value;
    $('spdVal').textContent=parseFloat(sp.value).toFixed(2);
    $('turnSpdVal').textContent=parseFloat(turnSp.value).toFixed(2);
  };
  sp.addEventListener('input',show);turnSp.addEventListener('input',show);show();
  // Team B4: Show the selected move time; moveMs() clamps both fields to the allowed range.
  const showTime=()=>{const t=moveMs();$('tTotal').textContent=(t/1000).toFixed(2)+' s ('+t+' ms)';};
  ['tSec','tMs'].forEach(id=>$(id).addEventListener('input',showTime));showTime();
  document.querySelectorAll('.pad button').forEach(b=>{
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);drvStart(b);});
    // Team B4: Releasing a normal press keeps the timed run; a cancelled gesture stops it.
    b.addEventListener('pointercancel',()=>{if(drvBtn===b)drvStop();});
    b.addEventListener('contextmenu',e=>e.preventDefault());
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&drvBtn)drvStop();});
  window.addEventListener('blur',()=>{if(drvBtn)drvStop();});
}

// ---- LiDAR top view: line through neighbouring points, broken at "no return" and at jumps
function drawEnv(s){
  const cv=$('envCv'),g=cv.getContext('2d'),W=cv.width,c=W/2,k=(c-12)/ENV_MAX_CM;
  g.clearRect(0,0,W,W);
  g.strokeStyle='rgba(255,255,255,.12)';g.lineWidth=1;
  for(let m=100;m<=ENV_MAX_CM;m+=100){g.beginPath();g.arc(c,c,m*k,0,2*Math.PI);g.stroke();}
  const d=(s.lidar&&s.lidar.d)||[],N=d.length,st=360/Math.max(N,1);
  const P=d.map((cm,i)=>{if(!cm||cm>ENV_MAX_CM)return null;const a=(i*st+st/2)*Math.PI/180;
    return [c+cm*k*Math.sin(a),c-cm*k*Math.cos(a),cm];});
  const near=(p,q)=>p&&q&&Math.abs(p[2]-q[2])<Math.max(30,0.25*Math.min(p[2],q[2]));
  g.strokeStyle='#4ea1ff';g.fillStyle='#4ea1ff';g.lineWidth=3;g.lineCap='round';
  for(let i=0;i<N;i++){const p=P[i];if(!p)continue;const q=P[(i+1)%N],o=P[(i+N-1)%N];
    if(near(p,q)){g.beginPath();g.moveTo(p[0],p[1]);g.lineTo(q[0],q[1]);g.stroke();}
    else if(!near(o,p)){g.beginPath();g.arc(p[0],p[1],2.5,0,2*Math.PI);g.fill();}}
  g.fillStyle='#e7edf5';g.fillRect(c-9,c-12,18,24);  // box
  g.fillStyle='#ffb020';g.beginPath();g.moveTo(c,c-20);g.lineTo(c-6,c-12);g.lineTo(c+6,c-12);g.fill();  // front
}

// ---- sensor cards
function renderSensors(s){
  if(!s)return;
  drawEnv(s);
  const L=s.lidar||{};
  $('lidarInfo').textContent=L.ok?('LiDAR '+L.pps+' packets/s, checksum errors '+L.crc+' %'):'LiDAR: no data';
  const us=s.us||[],names=['Front','Left','Right','Back','Front angled'];
  $('usTable').innerHTML=names.map((n,i)=>`<tr><td class='k'>Ultrasonic ${n}</td><td>${
    (us[i]===undefined||us[i]>=s.usMax)?'-':us[i]+' cm'}</td></tr>`).join('');
  const I=s.imu||{},G=s.gps||{};
  // ISO 8855: roll + = right side down, pitch + = front down, yaw + = turning left (0 = north)
  let t=I.ok?`<tr><td class='k'>Roll / pitch / yaw (ISO 8855)</td><td>${I.roll}&deg; / ${I.pitch}&deg; / ${I.yaw}&deg;</td></tr>`
    +`<tr><td class='k'>IMU calibration sys/gyr/acc/mag</td><td>${(I.cal||[]).join(' / ')} (3 = good)</td></tr>`
    :`<tr><td class='k'>IMU</td><td>not found</td></tr>`;
  t+=G.chars?`<tr><td class='k'>GPS</td><td>${G.fix?'fix':'no fix'}, ${G.sats} satellites, HDOP ${G.hdop}</td></tr>`
    :`<tr><td class='k'>GPS</td><td>no data from the module</td></tr>`;
  if(G.fix)t+=`<tr><td class='k'>Latitude / longitude</td><td>${G.lat}, ${G.lon}</td></tr>`;
  $('navTable').innerHTML=t;
}

// ---- poll the box 4 times per second
async function poll(){
  try{
    const j=await (await fetch('/values',{cache:'no-store'})).json();
    $('conn').textContent='Online';
    $('state').textContent=j.state||'-';
    renderSensors(j.sensors);
    if(dev){
      $('valTable').innerHTML=(j.values||[]).map(v=>`<tr><td class='k'>${esc(v.name)}</td><td>${esc(v.value)}</td></tr>`).join('');
      const c=j.cycle||{};$('cycle').textContent=`${c.min} / ${c.avg} / ${c.max} ms`;
    }
  }catch(e){$('conn').textContent='Offline';}
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
