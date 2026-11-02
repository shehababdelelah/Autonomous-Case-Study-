#include "WEBpage.hpp"
#include <LittleFS.h>
#include <math.h>
#include <stdlib.h>

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
.recordActions{display:flex;flex-wrap:wrap;gap:10px;margin:10px 0;}
.recordActions button,.recordActions a{min-height:44px;padding:10px 14px;font-size:14px;}
.recordActions a{display:inline-flex;align-items:center;border-radius:14px;background:rgba(78,161,255,.18);
 border:1px solid rgba(78,161,255,.4);color:var(--text);text-decoration:none;font-weight:650;}
.recordActions button:disabled,.recordActions a[aria-disabled='true']{opacity:.45;pointer-events:none;}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,72px);gap:10px;
 justify-content:center;margin:6px 0 10px;touch-action:none;user-select:none;-webkit-user-select:none;}
.pad button{font-size:24px;touch-action:none;}
.pad button.on{outline:3px solid var(--acc);}
input[type=range]{width:100%;}
.table{width:100%;border-collapse:collapse;font-size:13px;}
.table td{padding:8px 10px;border-bottom:1px solid rgba(255,255,255,.06);}
.k{color:var(--muted);}
canvas{display:block;margin:0 auto;max-width:100%;}
.lidar-summary{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin:12px 0;}
.lidar-metric{background:#0d1420;border:1px solid rgba(255,255,255,.08);border-radius:10px;padding:9px;}
.lidar-metric small{display:block;color:var(--muted);font-size:11px;}
.lidar-metric strong{font-size:17px;}
.lidar-layout{display:grid;grid-template-columns:minmax(260px,1fr) minmax(280px,1fr);gap:14px;align-items:start;}
.lidar-table-wrap{max-height:360px;overflow:auto;}
.lidar-table{width:100%;border-collapse:collapse;font-size:13px;}
.lidar-table th,.lidar-table td{text-align:left;padding:9px 7px;border-bottom:1px solid rgba(255,255,255,.07);white-space:nowrap;}
.lidar-table th{color:var(--muted);font-weight:550;position:sticky;top:0;background:var(--card);}
.return-id{color:#d9eaff;font-weight:700;}
@media(max-width:680px){.lidar-layout{grid-template-columns:1fr;}.lidar-summary{gap:6px;}.lidar-metric strong{font-size:15px;}}
</style></head><body><div class='wrap'>
<div class='top'><div class='title'>%TITLE%</div><div class='pill' id='conn'>-</div>
 <button id='devBtn' class='btnTiny'>Dev</button></div>
<div class='card'><div class='label'>State</div><div class='value' id='state'>-</div></div>
 <div class='card'><div class='sectionTitle'>Movement sensor CSV</div>
  <div class='value' id='recordStatus'>Checking logger...</div>
  <div class='recordActions'>
   <button id='recordStart' type='button'>Record + drive forward (60 s)</button>
   <button id='recordStop' class='stop' type='button'>Stop movement + recording</button>
   <a id='imuDownload' href='/imu.csv' download='imu_euler_data.csv' aria-disabled='true'>Download IMU CSV</a>
   <a id='lidarDownload' href='/lidar.csv' download='lidar_data.csv' aria-disabled='true'>Download LiDAR CSV</a>
   <button id='recordClear' type='button'>Delete saved movement CSVs</button>
  </div>
  <div class='label'>Moves forward at the selected speed and records IMU plus LiDAR every 250 ms for one minute.
  IMU readings do not steer the rover. Press Stop to end early.</div>
 </div>
  <div class='card'>
   <div class='sectionTitle'>LiDAR left-wall following</div>
   <div class='label'>Follows the left-side scan sector (bins 63-72; approximately 252-292 degrees).
    It stops if the LiDAR is stale, the wall is not detected, or the front/left distance is unsafe.</div>
   <label class='label' for='wallSpeed'>Wall-follow speed: <span id='wallSpeedVal'>0.15</span></label>
   <input type='range' id='wallSpeed' min='0.10' max='%MAXSPD%' step='0.01' value='0.15'>
   <div class='recordActions'>
    <button id='wallFollowStart' type='button'>Start wall following</button>
    <button id='wallFollowStop' class='stop' type='button'>Stop wall following</button>
   </div>
   <div class='value' id='wallFollowReason'>Stopped</div>
   <table class='table'>
    <tr><td class='k'>Mode / speed</td><td id='wallFollowMode'>Stopped</td></tr>
    <tr><td class='k'>Left wall (cm)</td><td id='wallFollowWall'>--</td></tr>
    <tr><td class='k'>Front (cm)</td><td id='wallFollowFront'>--</td></tr>
    <tr><td class='k'>Correction</td><td id='wallFollowCorrection'>0.000</td></tr>
    <tr><td class='k'>Left / right wheel</td><td id='wallFollowWheels'>0.000 / 0.000</td></tr>
   </table>
  </div>
  <div class='card'>
   <div class='sectionTitle'>Right-turn test</div>
   <div class='label'>Turns in place at maximum speed for 3 seconds. Test only in a clear area, keep the rover supervised, and use Stop if needed.</div>
   <div class='recordActions'>
    <button id='rightTurnStart' type='button'>Test right turn (3 s)</button>
    <button id='rightTurnStop' class='stop' type='button'>Stop turn test</button>
   </div>
   <div class='value' id='rightTurnStatus'>Ready</div>
  </div>
  <div class='card hidden' id='driveCard'><div class='sectionTitle'>Drive</div>
 <div class='pad'>
  <span></span><button data-d='f'>&#9650;</button><span></span>
  <button data-d='l'>&#9664;</button><button class='stop' data-d='s'>&#9632;</button><button data-d='r'>&#9654;</button>
  <span></span><button data-d='b'>&#9660;</button><span></span>
 </div>
 <label class='label' for='moveDuration'>Drive-button duration: <span id='moveDurationVal'>5 s</span></label>
 <input type='range' id='moveDuration' min='5' max='60' step='1' value='5'>
 <div class='label'>Tap a direction to move for the selected time. Releasing does not stop the rover; press Stop to cancel immediately.</div>
 <label class='label' for='spd'>Driving speed: <span id='spdVal'></span></label>
 <input type='range' id='spd' min='0.10' step='0.01'>
 <label class='label' for='turnSpd'>Turning speed: <span id='turnSpdVal'></span></label>
 <input type='range' id='turnSpd' min='0.10' step='0.01'>
 </div>
<div class='card'>
 <div class='sectionTitle'>Surroundings (LiDAR top view, front = up, rings 1 m)</div>
 <div class='label'>Adjacent scan bins are grouped by range continuity. These are measured returns, not wall/pillar/panel classifications.</div>
 <div class='lidar-summary'>
  <div class='lidar-metric'><small>Valid scan bins</small><strong id='lidarBinCount'>--</strong></div>
  <div class='lidar-metric'><small>Nearest return</small><strong id='lidarNearest'>--</strong></div>
  <div class='lidar-metric'><small>Scan age</small><strong id='lidarAge'>--</strong></div>
 </div>
 <div class='lidar-layout'>
  <div>
   <canvas id='envCv' width='320' height='320' role='img' aria-label='Live LiDAR top view with rover front pointing up'></canvas>
   <div class='label' id='lidarInfo'>-</div>
  </div>
  <div class='lidar-table-wrap'>
   <table class='lidar-table'>
    <thead><tr><th>Return</th><th>Side / angle</th><th>Distance</th><th>Bins</th></tr></thead>
    <tbody id='lidarReturns'><tr><td colspan='4'>Waiting for LiDAR scan...</td></tr></tbody>
   </table>
   <div class='label' id='lidarReturnCount'></div>
  </div>
 </div>
 <table class='table' id='usTable'></table></div>
<div class='card'><div class='sectionTitle'>IMU BNO055 readings / GPS</div><table class='table' id='navTable'></table>
 <div class='label' id='imuDiag'>Waiting for IMU data</div></div>
<div id='devRoot' class='hidden'>
 <div class='card'><div class='sectionTitle'>Values</div><table class='table' id='valTable'></table></div>
 <div class='card'><div class='sectionTitle'>UI task time (box)</div>
  <table class='table'><tr><td class='k'>min / avg / max</td><td id='cycle'>-</td></tr></table></div>
</div></div>
<script>
const MAX_SPEED=%MAXSPD%, ENV_MAX_CM=300;
const $=id=>document.getElementById(id);
const esc=s=>(''+s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');

// ---- "Dev" view toggle
let dev=false;
$('devBtn').onclick=()=>{dev=!dev;$('devRoot').classList.toggle('hidden',!dev);$('devBtn').textContent=dev?'Hide':'Dev';};

let drv=null,drvTimer=null,drvBtn=null,drvStopTimer=null,drvDeadline=0;
// Requests can overtake each other on Wi-Fi. Each command carries this page's random id and a
// rising number; the box ignores anything older than the last command it accepted.
const PAGE_ID=Math.floor(Math.random()*4e9)+1;let seq=0,movementRunActive=false,wallFollowActive=false,rightTurnTestActive=false;
function drvSend(l,r){seq++;return fetch('/drive?l='+l.toFixed(2)+'&r='+r.toFixed(2)+'&id='+PAGE_ID+'&n='+seq).catch(()=>null);}
function drvClear(){clearInterval(drvTimer);clearTimeout(drvStopTimer);drvTimer=null;drvStopTimer=null;drv=null;if(drvBtn){drvBtn.classList.remove('on');drvBtn=null;}}
function drvStop(){drvClear();drvSend(0,0);}
function drvStart(b){
  if(b.dataset.d==='s'){
    if(movementRunActive)stopMovementRun();
    else if(wallFollowActive)stopWallFollow();
    else if(rightTurnTestActive)stopRightTurnTest();
    else drvStop();
    return;
  }
  const floor=Math.min(0.1,MAX_SPEED);
  const readSpeed=id=>Math.min(MAX_SPEED,Math.max(floor,parseFloat($(id).value)||floor));
  const s=readSpeed('spd');
  const turnForward=s,turnBackward=Math.min(readSpeed('turnSpd'),s);
  const m={f:[s,s],b:[-s,-s],l:[-turnBackward,turnForward],r:[turnForward,-turnBackward]}[b.dataset.d];
  drvClear();  // switching keys: no stop in between, the new command follows directly
  drv=m;drvBtn=b;b.classList.add('on');drvSend(m[0],m[1]);
  const runMs=Math.round(parseFloat($('moveDuration').value)*1000);
  drvDeadline=performance.now()+runMs;
  drvTimer=setInterval(()=>{if(performance.now()>=drvDeadline){drvStop();return;}drvSend(drv[0],drv[1]);},150);
  drvStopTimer=setTimeout(drvStop,runMs);
}
if(MAX_SPEED>0){
  $('driveCard').classList.remove('hidden');
  const sp=$('spd'),turnSp=$('turnSpd'),moveDuration=$('moveDuration'),floor=Math.min(0.1,MAX_SPEED);
  sp.min=floor;sp.max=MAX_SPEED;sp.value=MAX_SPEED;
  turnSp.min=floor;turnSp.max=sp.value;turnSp.value=floor;
  const wallSpeed=$('wallSpeed');wallSpeed.max=MAX_SPEED;wallSpeed.value=Math.min(0.15,MAX_SPEED);
  $('wallSpeedVal').textContent=parseFloat(wallSpeed.value).toFixed(2);
  wallSpeed.addEventListener('input',()=>{$('wallSpeedVal').textContent=parseFloat(wallSpeed.value).toFixed(2);});
  moveDuration.addEventListener('input',()=>{$('moveDurationVal').textContent=moveDuration.value+' s';});
  const show=()=>{
    turnSp.max=sp.value;
    if(parseFloat(turnSp.value)>parseFloat(sp.value))turnSp.value=sp.value;
    $('spdVal').textContent=parseFloat(sp.value).toFixed(2);
    $('turnSpdVal').textContent=parseFloat(turnSp.value).toFixed(2);
  };
  sp.addEventListener('input',show);turnSp.addEventListener('input',show);show();
  document.querySelectorAll('.pad button').forEach(b=>{
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);drvStart(b);});
    b.addEventListener('pointercancel',()=>{if(drvBtn===b)drvStop();});
    b.addEventListener('contextmenu',e=>e.preventDefault());
  });
  document.addEventListener('visibilitychange',()=>{
    if(document.hidden&&movementRunActive)stopMovementRun();
    else if(document.hidden&&drvBtn)drvStop();
  });
  window.addEventListener('blur',()=>{
    if(movementRunActive)stopMovementRun();
    else if(drvBtn)drvStop();
  });
}

function lidarSegments(d){
  const N=d.length,step=360/Math.max(N,1);
  const points=d.map((cm,i)=>{
    if(!cm||cm>ENV_MAX_CM)return null;
    const angle=(i*step+step/2)%360,rad=angle*Math.PI/180;
    return {i,cm,angle,x:cm*Math.sin(rad),y:-cm*Math.cos(rad)};
  });
  const near=(a,b)=>a&&b&&Math.abs(a.cm-b.cm)<Math.max(30,0.25*Math.min(a.cm,b.cm));
  const groups=[];
  let group=[];
  for(let i=0;i<N;i++){
    if(!points[i]){
      if(group.length)groups.push(group);
      group=[];
      continue;
    }
    if(group.length&&!near(points[i-1],points[i])){
      groups.push(group);
      group=[];
    }
    group.push(points[i]);
  }
  if(group.length)groups.push(group);
  if(groups.length>1){
    const first=groups[0],last=groups[groups.length-1];
    if(near(last[last.length-1],first[0])){
      groups[0]=last.concat(first);
      groups.pop();
    }
  }
  const median=values=>{
    values.sort((a,b)=>a-b);
    const mid=Math.floor(values.length/2);
    return values.length%2?values[mid]:(values[mid-1]+values[mid])/2;
  };
  return groups.map((items,index)=>{
    const sin=items.reduce((sum,p)=>sum+Math.sin(p.angle*Math.PI/180),0);
    const cos=items.reduce((sum,p)=>sum+Math.cos(p.angle*Math.PI/180),0);
    const angle=(Math.atan2(sin,cos)*180/Math.PI+360)%360;
    const side=angle<45||angle>=315?'Front':angle<135?'Right':angle<225?'Back':'Left';
    return {
      id:index+1,items,angle,side,
      distance:median(items.map(p=>p.cm)),
      x:items.reduce((sum,p)=>sum+p.x,0)/items.length,
      y:items.reduce((sum,p)=>sum+p.y,0)/items.length
    };
  });
}

// ---- LiDAR top view: front is bin zero and points clockwise in the rover frame.
function drawEnv(s,segments){
  const cv=$('envCv'),g=cv.getContext('2d'),W=cv.width,c=W/2,k=(c-12)/ENV_MAX_CM;
  g.clearRect(0,0,W,W);
  g.strokeStyle='rgba(255,255,255,.12)';g.lineWidth=1;
  for(let m=100;m<=ENV_MAX_CM;m+=100){g.beginPath();g.arc(c,c,m*k,0,2*Math.PI);g.stroke();}
  const d=(s.lidar&&s.lidar.d)||[],N=d.length,st=360/Math.max(N,1);
  const P=d.map((cm,i)=>{if(!cm||cm>ENV_MAX_CM)return null;const a=(i*st+st/2)*Math.PI/180;
    return [c+cm*k*Math.sin(a),c-cm*k*Math.cos(a),cm];});
  const near=(p,q)=>p&&q&&Math.abs(p[2]-q[2])<Math.max(30,0.25*Math.min(p[2],q[2]));
  g.strokeStyle='#4ea1ff';g.fillStyle='#4ea1ff';g.lineWidth=2;g.lineCap='round';
  for(let i=0;i<N;i++){const p=P[i];if(!p)continue;const q=P[(i+1)%N];
    if(near(p,q)){g.beginPath();g.moveTo(p[0],p[1]);g.lineTo(q[0],q[1]);g.stroke();}
    g.beginPath();g.arc(p[0],p[1],3,0,2*Math.PI);g.fill();}
  g.font='bold 11px system-ui';g.textAlign='left';g.textBaseline='middle';
  for(const segment of segments.slice(0,24)){
    const x=c+segment.x*k,y=c+segment.y*k;
    g.lineWidth=3;g.strokeStyle='#0b0f17';g.strokeText('R'+segment.id,x+5,y-5);
    g.fillStyle='#d9eaff';g.fillText('R'+segment.id,x+5,y-5);
  }
  g.fillStyle='#e7edf5';g.fillRect(c-9,c-12,18,24);  // box
  g.fillStyle='#ffb020';g.beginPath();g.moveTo(c,c-20);g.lineTo(c-6,c-12);g.lineTo(c+6,c-12);g.fill();  // front
}

// ---- sensor cards
function renderSensors(s,lidarAgeMs){
  if(!s)return;
  const L=s.lidar||{};
  const d=L.d||[],valid=d.filter(cm=>cm>0&&cm<=ENV_MAX_CM);
  const segments=lidarSegments(d);
  drawEnv(s,segments);
  $('lidarBinCount').textContent=valid.length+' / '+d.length;
  $('lidarNearest').textContent=valid.length?Math.min(...valid)+' cm':'--';
  $('lidarAge').textContent=Number.isFinite(lidarAgeMs)?lidarAgeMs+' ms':'--';
  $('lidarInfo').textContent=L.ok?('LiDAR '+L.pps+' packets/s, checksum errors '+L.crc+' %'):'LiDAR: no data';
  const shown=segments.slice(0,24);
  $('lidarReturns').innerHTML=shown.length?shown.map(segment=>
    `<tr><td class='return-id'>R${segment.id}</td><td>${segment.side} · ${Math.round(segment.angle)}&deg;</td>`
    +`<td>${segment.distance.toFixed(0)} cm</td><td>${segment.items.length}</td></tr>`
  ).join(''):`<tr><td colspan='4'>No valid returns within ${ENV_MAX_CM} cm</td></tr>`;
  $('lidarReturnCount').textContent=segments.length>shown.length
    ?`Showing ${shown.length} of ${segments.length} returns`:`${segments.length} return segments`;
  const us=s.us||[],names=['Front','Left','Right','Back','Front angled'];
  $('usTable').innerHTML=names.map((n,i)=>`<tr><td class='k'>Ultrasonic ${n}</td><td>${
    (us[i]===undefined||us[i]>=s.usMax)?'-':us[i]+' cm'}</td></tr>`).join('');
  const I=s.imu||{},G=s.gps||{};
  const age=I.ageMs===null?'no sample':I.ageMs+' ms old';
  $('imuDiag').textContent=I.sample?'Latest IMU sample: '+age
    :(I.ok?'Waiting for the first IMU sample':'BNO055 not initialized');
  let t=I.ok&&I.sample
    ?`<tr><td class='k'>Heading / yaw (BNO055)</td><td>${I.heading}&deg;</td></tr>`
    +`<tr><td class='k'>Roll (BNO055)</td><td>${I.roll}&deg;</td></tr>`
    +`<tr><td class='k'>Pitch (BNO055)</td><td>${I.pitch}&deg;</td></tr>`
    +`<tr><td class='k'>Calibration (system / gyro / accel / mag)</td><td>${(I.cal||[]).join(' / ')} (0–3)</td></tr>`
    +`<tr><td class='k'>Sample time since boot</td><td>${I.timestampMs} ms</td></tr>`
    :`<tr><td class='k'>IMU</td><td>${I.ok?'Waiting for first sample':'not found'}</td></tr>`;
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
    const recording=j.movementLog||{}, drive=j.movementDrive||{}, pending=recording.queued||0;
    movementRunActive=!!drive.active;
    const turnTest=j.rightTurnTest||{};
    rightTurnTestActive=!!turnTest.active;
    $('rightTurnStatus').textContent=rightTurnTestActive
      ?'Turning right at maximum speed ('+Math.ceil(turnTest.remainingMs/1000)+' s remaining)'
      :turnTest.lastResult||'Ready';
    const wall=j.wallFollow||{};
    wallFollowActive=!!wall.active;
    $('wallFollowReason').textContent=wall.reason||'Stopped';
    $('wallFollowMode').textContent=wall.active?'Following at '+Number(wall.speed).toFixed(2):'Stopped';
    $('wallFollowWall').textContent=wall.wallCm?wall.wallCm+' cm':'No valid reading';
    $('wallFollowFront').textContent=wall.frontCm?wall.frontCm+' cm':'No valid reading';
    $('wallFollowCorrection').textContent=Number(wall.correction||0).toFixed(3);
    $('wallFollowWheels').textContent=Number(wall.left||0).toFixed(3)+' / '+Number(wall.right||0).toFixed(3);
    document.querySelectorAll('.pad button').forEach(b=>{
      b.disabled=(movementRunActive||wallFollowActive||rightTurnTestActive)&&b.dataset.d!=='s';
    });
    $('recordStatus').textContent=recording.error?'Recording error; check storage'
      :recording.full?'CSV storage full; download saved data'
      :drive.active?'Driving forward + recording ('+Math.ceil(drive.remainingMs/1000)+' s remaining)'
      :recording.recording?'Recording sensor snapshots'
      :(recording.writing||pending)?'Stopped; saving pending samples ('+pending+')'
      :recording.ready?'Ready; not recording':'Logger unavailable';
    $('recordStart').disabled=!recording.ready||recording.recording||drive.active||wallFollowActive||rightTurnTestActive||recording.error||recording.full;
    $('recordStop').disabled=!recording.recording&&!drive.active;
    $('recordClear').disabled=recording.recording||recording.writing||pending||drive.active||rightTurnTestActive;
    $('wallFollowStart').disabled=wallFollowActive||recording.recording||drive.active||rightTurnTestActive;
    $('wallFollowStop').disabled=!wallFollowActive;
    $('rightTurnStart').disabled=rightTurnTestActive||movementRunActive||recording.recording||wallFollowActive;
    $('rightTurnStop').disabled=!rightTurnTestActive;
    const downloadDisabled=recording.recording||recording.writing||pending?'true':'false';
    $('imuDownload').setAttribute('aria-disabled',downloadDisabled);
    $('lidarDownload').setAttribute('aria-disabled',downloadDisabled);
    renderSensors(j.sensors,j.lidarAgeMs);
    if(dev){
      $('valTable').innerHTML=(j.values||[]).map(v=>`<tr><td class='k'>${esc(v.name)}</td><td>${esc(v.value)}</td></tr>`).join('');
      const c=j.cycle||{};$('cycle').textContent=`${c.min} / ${c.avg} / ${c.max} ms`;
    }
  }catch(e){$('conn').textContent='Offline';}
}
setInterval(poll,250);poll();
setInterval(()=>{
  if(wallFollowActive)fetch('/wall-follow/heartbeat',{method:'POST'}).catch(()=>{});
},250);
async function startWallFollow(){
  const speed=parseFloat($('wallSpeed').value);
  if(!Number.isFinite(speed)||speed<0.1||speed>MAX_SPEED)return;
  if(!window.confirm('The rover will move autonomously while following the left LiDAR wall. '
    +'Test at low speed in a clear area, keep the stop button accessible, and supervise continuously. Start?'))return;
  $('wallFollowReason').textContent='Starting...';
  $('wallFollowStart').disabled=true;
  try{
    const response=await fetch('/wall-follow/start',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'speed='+encodeURIComponent(speed.toFixed(2))
    });
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Wall-follow start failed');
    poll();
  }catch(error){
    $('wallFollowReason').textContent=error.message||'Wall-follow start failed';
    $('wallFollowStart').disabled=false;
  }
}
async function stopWallFollow(){
  $('wallFollowReason').textContent='Stopping...';
  try{
    const response=await fetch('/wall-follow/stop',{method:'POST'});
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Wall-follow stop failed');
    wallFollowActive=false;
    poll();
  }catch(error){$('wallFollowReason').textContent=error.message||'Wall-follow stop failed';}
}
async function startRightTurnTest(){
  if(!window.confirm('The rover will rotate right at maximum speed for 3 seconds. '
    +'Make sure the area is clear, keep the rover supervised, and be ready to stop it. Start?'))return;
  drvClear();
  $('rightTurnStatus').textContent='Starting...';
  $('rightTurnStart').disabled=true;
  try{
    const response=await fetch('/turn/right/start',{method:'POST'});
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Right-turn test could not start');
    poll();
  }catch(error){
    $('rightTurnStatus').textContent=error.message||'Right-turn test could not start';
    $('rightTurnStart').disabled=false;
  }
}
async function stopRightTurnTest(){
  $('rightTurnStatus').textContent='Stopping...';
  try{
    const response=await fetch('/turn/right/stop',{method:'POST'});
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Could not stop right-turn test');
    rightTurnTestActive=false;
    poll();
  }catch(error){$('rightTurnStatus').textContent=error.message||'Could not stop right-turn test';}
}
async function startMovementRun(){
  const speed=parseFloat($('spd').value);
  if(!Number.isFinite(speed)||speed<0.1||speed>MAX_SPEED)return;
  if(!window.confirm('The rover will drive forward for 60 seconds at speed '
    +speed.toFixed(2)+' while saving IMU and LiDAR data. Ensure the entire path is clear. Start?'))return;
  $('recordStatus').textContent='Starting timed run...';
  $('recordStart').disabled=true;
  try{
    const response=await fetch('/movement/drive/start',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'speed='+encodeURIComponent(speed.toFixed(2))
    });
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Timed run request failed');
  }catch(error){
    $('recordStatus').textContent=error.message||'Timed run request failed';
    $('recordStart').disabled=false;
  }
}
async function stopMovementRun(){
  drvStop();
  $('recordStatus').textContent='Stopping run...';
  try{
    const response=await fetch('/movement/stop',{method:'POST'});
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Stop request failed');
    poll();
  }catch(error){$('recordStatus').textContent=error.message||'Stop request failed';}
}
$('recordStart').onclick=startMovementRun;
$('recordStop').onclick=stopMovementRun;
$('wallFollowStart').onclick=startWallFollow;
$('wallFollowStop').onclick=stopWallFollow;
$('rightTurnStart').onclick=startRightTurnTest;
$('rightTurnStop').onclick=stopRightTurnTest;
$('recordClear').onclick=async()=>{
  if(!window.confirm('Delete all saved movement IMU and LiDAR CSV data, including the previous combined movement file? The manually labeled LiDAR CSV will be kept.'))return;
  $('recordStatus').textContent='Deleting saved movement CSVs...';
  $('recordClear').disabled=true;
  try{
    const response=await fetch('/movement/clear',{method:'POST'});
    const result=await response.json();
    if(!response.ok)throw new Error(result.error||'Could not clear movement files');
    $('recordStatus').textContent='Saved movement CSV files cleared';
    poll();
  }catch(error){$('recordStatus').textContent=error.message||'Could not clear movement files';}
};
</script></body></html>)rawpage";

static const uint32_t kRightTurnTestDurationMs = 3000;

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
void WEBpage::_lock() {
  if (!_mtx) _mtx = xSemaphoreCreateMutex();
  xSemaphoreTake(_mtx, portMAX_DELAY);
}

void WEBpage::begin(const char* ssid, const char* pass, IPAddress ip, uint8_t channel) {
  _lock();  // make sure the mutex exists before the server starts
  xSemaphoreGive(_mtx);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                                  // no Wi-Fi power save on the box
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(ssid, pass, channel)) {
    Serial.printf("[WEBpage] AP %s, channel %d, IP %s\n", ssid, WiFi.channel(),
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WEBpage] WiFi.softAP failed");
  }

  _server = new AsyncWebServer(80);
  _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "text/html; charset=utf-8", _buildPage());
  });
  _lidarLogger.begin();
  _server->on("/movement/drive/start", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("speed", true)) {
      req->send(400, "application/json", "{\"ok\":false,\"error\":\"missing speed\"}");
      return;
    }
    const String speedText = req->getParam("speed", true)->value();
    char* end = nullptr;
    const float speed = strtof(speedText.c_str(), &end);
    if (end == speedText.c_str() || *end != '\0' || !isfinite(speed) ||
        speed < 0.1f || speed > _driveMax) {
      req->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid speed\"}");
      return;
    }
    if (startMovementDrive(speed)) {
      req->send(202, "application/json", "{\"ok\":true,\"durationMs\":60000}");
      return;
    }
    const MovementLogStatus status = _lidarLogger.movementStatus();
    float wallSpeed = 0;
    if (getWallFollow(wallSpeed)) {
      req->send(409, "application/json",
                "{\"ok\":false,\"error\":\"stop wall-follow before starting a timed run\"}");
      return;
    }
    const char* error = status.full ? "CSV storage is full" :
                        status.error ? "CSV logger has an error" :
                        status.recording ? "a movement recording is already active" :
                        "turn test active or CSV logger unavailable";
    req->send(status.full ? 507 : status.recording ? 409 : 503,
              "application/json", String("{\"ok\":false,\"error\":\"") + error + "\"}");
  });
  _server->on("/movement/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
    stopMovementDrive();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  _server->on("/wall-follow/start", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("speed", true)) {
      req->send(400, "application/json", "{\"ok\":false,\"error\":\"missing speed\"}");
      return;
    }
    const String speedText = req->getParam("speed", true)->value();
    char* end = nullptr;
    const float speed = strtof(speedText.c_str(), &end);
    if (end == speedText.c_str() || *end != '\0' || !isfinite(speed) ||
        speed < 0.1f || speed > _driveMax) {
      req->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid speed\"}");
      return;
    }
    if (startWallFollow(speed)) {
      req->send(202, "application/json", "{\"ok\":true}");
      return;
    }
    req->send(409, "application/json",
              "{\"ok\":false,\"error\":\"another movement mode or recording is active\"}");
  });
  _server->on("/wall-follow/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
    stopWallFollow();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  _server->on("/wall-follow/heartbeat", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (!heartbeatWallFollow()) {
      req->send(409, "application/json", "{\"ok\":false,\"error\":\"wall-follow mode is stopped\"}");
      return;
    }
    req->send(200, "application/json", "{\"ok\":true}");
  });
  _server->on("/turn/right/start", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (startRightTurnTest()) {
      req->send(202, "application/json",
                "{\"ok\":true,\"durationMs\":3000,\"speed\":\"maximum\"}");
      return;
    }
    req->send(409, "application/json",
              "{\"ok\":false,\"error\":\"another movement mode or recording is active\"}");
  });
  _server->on("/turn/right/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
    stopRightTurnTest();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  _server->on("/movement/clear", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (_lidarLogger.clearMovementFiles()) {
      req->send(200, "application/json", "{\"ok\":true}");
      return;
    }
    const MovementLogStatus status = _lidarLogger.movementStatus();
    bool driveActive;
    float driveSpeed;
    uint32_t remainingMs;
    _lidarLogger.getMovementDriveStatus(driveActive, driveSpeed, remainingMs);
    if (status.recording || status.writing || status.queued || driveActive) {
      req->send(409, "application/json",
                "{\"ok\":false,\"error\":\"stop the run and wait for queued samples first\"}");
      return;
    }
    req->send(500, "application/json",
              "{\"ok\":false,\"error\":\"could not clear or recreate the sensor CSV files\"}");
  });
  const auto serveMovementFile = [this](AsyncWebServerRequest* req, const char* path) {
    const MovementLogStatus status = _lidarLogger.movementStatus();
    if (status.recording || status.writing || status.queued) {
      req->send(409, "text/plain", "Stop recording and wait for pending samples to be saved");
      return;
    }
    if (!LittleFS.exists(path)) {
      req->send(404, "text/plain", "Sensor CSV is not available");
      return;
    }
    req->send(LittleFS, path, "text/csv", true);
  };
  _server->on("/imu.csv", HTTP_GET, [serveMovementFile](AsyncWebServerRequest* req) {
    serveMovementFile(req, "/imu_euler_data.csv");
  });
  _server->on("/imu-previous.csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!LittleFS.exists("/imu_data.csv")) {
      req->send(404, "text/plain", "Previous IMU CSV is not available");
      return;
    }
    req->send(LittleFS, "/imu_data.csv", "text/csv", true);
  });
  _server->on("/lidar.csv", HTTP_GET, [serveMovementFile](AsyncWebServerRequest* req) {
    serveMovementFile(req, "/lidar_data.csv");
  });
  _server->on("/test/lidar/live", HTTP_GET, [this](AsyncWebServerRequest* req) {
    uint16_t frontCm, rightCm, leftCm;
    float packetsPerSecond;
    uint32_t updatedAt;
    xSemaphoreTake(_mtx, portMAX_DELAY);
    frontCm = _lidarFrontCm;
    rightCm = _lidarRightCm;
    leftCm = _lidarLeftCm;
    packetsPerSecond = _lidarPacketsPerSecond;
    updatedAt = _lidarSnapshotAtMs;
    xSemaphoreGive(_mtx);

    char json[192];
    snprintf(json, sizeof(json),
             "{\"frontCm\":%u,\"rightCm\":%u,\"leftCm\":%u,\"pps\":%.0f,"
             "\"ageMs\":%lu,\"loggerReady\":%s,\"loggerError\":%s,\"loggerFull\":%s}",
             frontCm, rightCm, leftCm, packetsPerSecond,
             (unsigned long)(updatedAt ? millis() - updatedAt : UINT32_MAX),
             _lidarLogger.ready() ? "true" : "false",
             _lidarLogger.hasError() ? "true" : "false",
             _lidarLogger.isFull() ? "true" : "false");
    req->send(200, "application/json", json);
  });
  _server->on("/test/lidar/records", HTTP_GET, [this](AsyncWebServerRequest* req) {
    LidarTestRecord records[LidarTestLogger::kRecentRecordCount];
    const size_t count = _lidarLogger.copyRecent(records, LidarTestLogger::kRecentRecordCount);
    char json[4096];
    size_t used = 0;
    int written = snprintf(json, sizeof(json), "{\"records\":[");
    if (written < 0 || (size_t)written >= sizeof(json)) {
      req->send(500, "application/json", "{\"error\":\"response buffer error\"}");
      return;
    }
    used = (size_t)written;
    for (size_t i = 0; i < count; ++i) {
      written = snprintf(json + used, sizeof(json) - used,
                         "%s{\"uptimeMs\":%lu,\"direction\":\"%s\","
                         "\"type\":\"%s\",\"distanceCm\":%u}",
                         i ? "," : "", (unsigned long)records[i].timestampMs,
                         records[i].direction, records[i].objectType, records[i].distanceCm);
      if (written < 0 || (size_t)written >= sizeof(json) - used) {
        req->send(500, "application/json", "{\"error\":\"records exceed response buffer\"}");
        return;
      }
      used += (size_t)written;
    }
    snprintf(json + used, sizeof(json) - used, "],\"loggerReady\":%s,\"loggerError\":%s,"
             "\"loggerFull\":%s}",
             _lidarLogger.ready() ? "true" : "false",
             _lidarLogger.hasError() ? "true" : "false",
             _lidarLogger.isFull() ? "true" : "false");
    req->send(200, "application/json", json);
  });
  _server->on("/test/lidar/save", HTTP_POST, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("direction", true) || !req->hasParam("type", true)) {
      req->send(400, "application/json", "{\"ok\":false,\"error\":\"missing direction or type\"}");
      return;
    }
    const String direction = req->getParam("direction", true)->value();
    const String type = req->getParam("type", true)->value();
    if ((direction != "Front" && direction != "Right" && direction != "Left") ||
        (type != "Wall" && type != "Pillar" && type != "Panel")) {
      req->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid direction or type\"}");
      return;
    }

    uint16_t distanceCm = 0;
    xSemaphoreTake(_mtx, portMAX_DELAY);
    if (_lidarPacketsPerSecond > 1.0f && millis() - _lidarSnapshotAtMs <= 1000) {
      distanceCm = direction == "Front" ? _lidarFrontCm :
                   direction == "Right" ? _lidarRightCm : _lidarLeftCm;
    }
    xSemaphoreGive(_mtx);
    if (!distanceCm) {
      req->send(409, "application/json", "{\"ok\":false,\"error\":\"no fresh LiDAR return in selected direction\"}");
      return;
    }
    if (!_lidarLogger.ready()) {
      req->send(503, "application/json", "{\"ok\":false,\"error\":\"CSV logger unavailable\"}");
      return;
    }
    if (_lidarLogger.isFull() || _lidarLogger.hasError()) {
      req->send(507, "application/json", "{\"ok\":false,\"error\":\"CSV storage is full or has an error\"}");
      return;
    }
    if (!_lidarLogger.enqueue(direction.c_str(), type.c_str(), distanceCm)) {
      req->send(503, "application/json", "{\"ok\":false,\"error\":\"CSV queue is full; retry\"}");
      return;
    }
    req->send(202, "application/json", "{\"ok\":true,\"queued\":true}");
  });
  _server->on("/test/lidar.csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!LittleFS.exists("/lidar_test.csv")) {
      req->send(404, "text/plain", "LiDAR CSV file is not available");
      return;
    }
    req->send(LittleFS, "/lidar_test.csv", "text/csv", true);
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
    float wallSpeed = 0;
    getWallFollow(wallSpeed);

    _lock();
    bool movementActive;
    float movementSpeed;
    uint32_t movementRemainingMs;
    _lidarLogger.getMovementDriveStatus(movementActive, movementSpeed, movementRemainingMs);
    portENTER_CRITICAL(&_wallFollowMux);
    const bool wallActive = _wallFollowActive;
    portENTER_CRITICAL(&_rightTurnTestMux);
    const bool turnTestActive = _rightTurnTestActive;
    bool fresh;
    portENTER_CRITICAL(&_driveMux);
    // a new page (id) starts over; within a page only a higher number counts
    fresh = (id != _drivePageId) || (n > _driveSeq);
    if (fresh) {
      _drivePageId = id;
      _driveSeq = n;
      if (!movementActive && !wallActive && !turnTestActive) {
        _driveL = l;
        _driveR = r;
        _driveMs = millis();
      } else if (turnTestActive && l == 0 && r == 0) {
        _rightTurnTestActive = false;
        _rightTurnTestStartedMs = 0;
        _driveL = 0;
        _driveR = 0;
        _driveMs = 0;
        strlcpy(_rightTurnTestResult, "Stopped by user", sizeof(_rightTurnTestResult));
      }
    }
    portEXIT_CRITICAL(&_driveMux);
    portEXIT_CRITICAL(&_rightTurnTestMux);
    portEXIT_CRITICAL(&_wallFollowMux);
    xSemaphoreGive(_mtx);
    if (fresh && (l != 0 || r != 0) &&
        (movementActive || wallActive || turnTestActive)) {
      req->send(409, "application/json", "{\"ok\":false,\"error\":\"another movement mode active\"}");
      return;
    }
    req->send(200, "application/json", fresh ? "{\"ok\":true}" : "{\"ok\":false,\"stale\":true}");
  });
  _server->begin();
  _cycleLastUs = micros();
  _cycleWinMs = millis();
}

void WEBpage::setTitle(const String& title) { _title = title; }

void WEBpage::enableDrivePad(float maxSpeed) {
  _drivePad = true;
  _driveMax = maxSpeed;
}

void WEBpage::setStateText(const String& text) {
  _lock();
  _state = text;
  xSemaphoreGive(_mtx);
}

void WEBpage::setSensorJson(const String& json) {
  _lock();
  _sensorJson = json;
  xSemaphoreGive(_mtx);
}

void WEBpage::recordSensorSnapshot(const SensorLogSnapshot& snapshot) {
  _lidarLogger.enqueueMovementSnapshot(snapshot);
}

bool WEBpage::startMovementDrive(float speed) {
  float wallSpeed = 0;
  getWallFollow(wallSpeed);
  _lock();
  portENTER_CRITICAL(&_wallFollowMux);
  const bool wallActive = _wallFollowActive;
  portENTER_CRITICAL(&_rightTurnTestMux);
  const bool turnTestActive = _rightTurnTestActive;
  portEXIT_CRITICAL(&_rightTurnTestMux);
  portEXIT_CRITICAL(&_wallFollowMux);
  if (wallActive || turnTestActive || !_lidarLogger.startMovementDrive(speed)) {
    xSemaphoreGive(_mtx);
    return false;
  }

  portENTER_CRITICAL(&_driveMux);
  _driveL = 0;
  _driveR = 0;
  _driveMs = 0;
  portEXIT_CRITICAL(&_driveMux);
  xSemaphoreGive(_mtx);
  return true;
}

void WEBpage::stopMovementDrive() {
  _lock();
  _lidarLogger.stopMovementDrive();
  portENTER_CRITICAL(&_driveMux);
  _driveL = 0;
  _driveR = 0;
  _driveMs = 0;
  portEXIT_CRITICAL(&_driveMux);
  xSemaphoreGive(_mtx);
}

bool WEBpage::getMovementDrive(float& speed) {
  return _lidarLogger.getMovementDrive(speed);
}

bool WEBpage::startWallFollow(float speed) {
  float currentSpeed = 0;
  getWallFollow(currentSpeed);
  _lock();
  bool movementActive;
  float movementSpeed;
  uint32_t movementRemainingMs;
  _lidarLogger.getMovementDriveStatus(movementActive, movementSpeed, movementRemainingMs);
  const MovementLogStatus recording = _lidarLogger.movementStatus();
  portENTER_CRITICAL(&_wallFollowMux);
  const bool alreadyActive = _wallFollowActive;
  portENTER_CRITICAL(&_rightTurnTestMux);
  const bool turnTestActive = _rightTurnTestActive;
  if (!alreadyActive && !turnTestActive && !movementActive && !recording.recording) {
    _wallFollowActive = true;
    _wallFollowSpeed = speed;
    _wallFollowHeartbeatMs = millis();
    portENTER_CRITICAL(&_driveMux);
    _driveL = 0;
    _driveR = 0;
    _driveMs = 0;
    portEXIT_CRITICAL(&_driveMux);
  }
  portEXIT_CRITICAL(&_rightTurnTestMux);
  portEXIT_CRITICAL(&_wallFollowMux);
  if (alreadyActive || turnTestActive || movementActive || recording.recording) {
    xSemaphoreGive(_mtx);
    return false;
  }

  _wallFollowReportedSpeed = speed;
  _wallFollowLeftSpeed = 0;
  _wallFollowRightSpeed = 0;
  strlcpy(_wallFollowReason, "Starting; waiting for LiDAR", sizeof(_wallFollowReason));
  xSemaphoreGive(_mtx);
  return true;
}

void WEBpage::stopWallFollow(const char* reason) {
  _lock();
  portENTER_CRITICAL(&_wallFollowMux);
  _wallFollowActive = false;
  _wallFollowSpeed = 0;
  _wallFollowHeartbeatMs = 0;
  portENTER_CRITICAL(&_driveMux);
  _driveL = 0;
  _driveR = 0;
  _driveMs = 0;
  portEXIT_CRITICAL(&_driveMux);
  portEXIT_CRITICAL(&_wallFollowMux);

  _wallFollowLeftSpeed = 0;
  _wallFollowRightSpeed = 0;
  strlcpy(_wallFollowReason, reason ? reason : "Stopped", sizeof(_wallFollowReason));
  xSemaphoreGive(_mtx);
}

bool WEBpage::getWallFollow(float& speed) {
  bool expired = false;
  bool active;
  portENTER_CRITICAL(&_wallFollowMux);
  if (_wallFollowActive && millis() - _wallFollowHeartbeatMs > 750) {
    _wallFollowActive = false;
    _wallFollowSpeed = 0;
    expired = true;
  }
  active = _wallFollowActive;
  speed = active ? _wallFollowSpeed : 0;
  portEXIT_CRITICAL(&_wallFollowMux);
  if (expired) stopWallFollow("Webpage heartbeat lost; stopped");
  return active;
}

bool WEBpage::heartbeatWallFollow() {
  float speed = 0;
  if (!getWallFollow(speed)) return false;
  portENTER_CRITICAL(&_wallFollowMux);
  const bool active = _wallFollowActive;
  if (active) _wallFollowHeartbeatMs = millis();
  portEXIT_CRITICAL(&_wallFollowMux);
  return active;
}

bool WEBpage::startRightTurnTest() {
  float wallSpeed = 0;
  if (getWallFollow(wallSpeed)) return false;

  _lock();
  bool movementActive;
  float movementSpeed;
  uint32_t movementRemainingMs;
  _lidarLogger.getMovementDriveStatus(movementActive, movementSpeed, movementRemainingMs);
  const MovementLogStatus recording = _lidarLogger.movementStatus();
  portENTER_CRITICAL(&_wallFollowMux);
  const bool wallActive = _wallFollowActive;
  portENTER_CRITICAL(&_rightTurnTestMux);
  const bool alreadyActive = _rightTurnTestActive;
  if (!wallActive && !alreadyActive && !movementActive && !recording.recording) {
    _rightTurnTestActive = true;
    _rightTurnTestStartedMs = millis();
    portENTER_CRITICAL(&_driveMux);
    _driveL = 0;
    _driveR = 0;
    _driveMs = 0;
    portEXIT_CRITICAL(&_driveMux);
    strlcpy(_rightTurnTestResult, "Turning right", sizeof(_rightTurnTestResult));
  }
  portEXIT_CRITICAL(&_rightTurnTestMux);
  portEXIT_CRITICAL(&_wallFollowMux);
  xSemaphoreGive(_mtx);
  return !wallActive && !alreadyActive && !movementActive && !recording.recording;
}

void WEBpage::stopRightTurnTest() {
  _lock();
  portENTER_CRITICAL(&_rightTurnTestMux);
  const bool wasActive = _rightTurnTestActive;
  _rightTurnTestActive = false;
  _rightTurnTestStartedMs = 0;
  portENTER_CRITICAL(&_driveMux);
  _driveL = 0;
  _driveR = 0;
  _driveMs = 0;
  portEXIT_CRITICAL(&_driveMux);
  if (wasActive) strlcpy(_rightTurnTestResult, "Stopped by user", sizeof(_rightTurnTestResult));
  portEXIT_CRITICAL(&_rightTurnTestMux);
  xSemaphoreGive(_mtx);
}

bool WEBpage::getRightTurnTest(float& speed, uint32_t& remainingMs) {
  bool active;
  _lock();
  portENTER_CRITICAL(&_rightTurnTestMux);
  const uint32_t elapsedMs = millis() - _rightTurnTestStartedMs;
  if (_rightTurnTestActive && elapsedMs >= kRightTurnTestDurationMs) {
    _rightTurnTestActive = false;
    _rightTurnTestStartedMs = 0;
    portENTER_CRITICAL(&_driveMux);
    _driveL = 0;
    _driveR = 0;
    _driveMs = 0;
    portEXIT_CRITICAL(&_driveMux);
    strlcpy(_rightTurnTestResult, "Turn test complete", sizeof(_rightTurnTestResult));
  }
  active = _rightTurnTestActive;
  speed = active ? _driveMax : 0;
  remainingMs = active ? kRightTurnTestDurationMs - elapsedMs : 0;
  portEXIT_CRITICAL(&_rightTurnTestMux);
  xSemaphoreGive(_mtx);
  return active;
}

void WEBpage::setWallFollowDiagnostics(uint16_t wallCm, uint16_t frontCm, float correction,
                                       float leftSpeed, float rightSpeed, float speed,
                                       const char* reason) {
  _lock();
  _wallFollowWallCm = wallCm;
  _wallFollowFrontCm = frontCm;
  _wallFollowCorrection = correction;
  _wallFollowLeftSpeed = leftSpeed;
  _wallFollowRightSpeed = rightSpeed;
  _wallFollowReportedSpeed = speed;
  strlcpy(_wallFollowReason, reason ? reason : "", sizeof(_wallFollowReason));
  xSemaphoreGive(_mtx);
}

void WEBpage::setLidarTestSnapshot(uint16_t frontCm, uint16_t rightCm,
                                  uint16_t leftCm, float packetsPerSecond) {
  _lock();
  _lidarFrontCm = frontCm;
  _lidarRightCm = rightCm;
  _lidarLeftCm = leftCm;
  _lidarPacketsPerSecond = packetsPerSecond;
  _lidarSnapshotAtMs = millis();
  xSemaphoreGive(_mtx);
}

void WEBpage::showValue(const char* name, double value) { _setValue(name, String(value, 3)); }
void WEBpage::showValue(const char* name, const String& value) { _setValue(name, value); }

// Updates the value with this name or takes the first free slot (ignored if all are used).
void WEBpage::_setValue(const char* name, const String& text) {
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

void WEBpage::getDrive(float& left, float& right, uint32_t& ageMs) const {
  portENTER_CRITICAL(&_driveMux);
  left = _driveL;
  right = _driveR;
  const uint32_t ms = _driveMs;
  portEXIT_CRITICAL(&_driveMux);
  ageMs = ms ? millis() - ms : UINT32_MAX;  // millis() read after ms: never negative
}

// UI task timing over 1 s windows: min / average / max time between tick() calls.
void WEBpage::tick() {
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

String WEBpage::_buildPage() const {
  String page = FPSTR(kPage);
  page.replace("%TITLE%", _title);
  page.replace("%MAXSPD%", _drivePad ? String(_driveMax, 2) : String("0"));
  return page;
}

// {"state":"..","values":[{"name":..,"value":..}],"cycle":{min,avg,max},
//  "sensors":{..}|null}
String WEBpage::_buildJson() {
  String out;
  out.reserve(1900);
  float wallFollowSpeed = 0;
  const bool wallFollowActive = getWallFollow(wallFollowSpeed);
  float rightTurnSpeed = 0;
  uint32_t rightTurnRemainingMs = 0;
  const bool rightTurnActive = getRightTurnTest(rightTurnSpeed, rightTurnRemainingMs);
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
  out += ",\"lidarAgeMs\":";
  out += _lidarSnapshotAtMs
      ? String((unsigned long)(millis() - _lidarSnapshotAtMs))
      : String("null");
  const MovementLogStatus movement = _lidarLogger.movementStatus();
  bool movementDriveActive;
  float movementDriveSpeed;
  uint32_t movementRemainingMs;
  _lidarLogger.getMovementDriveStatus(movementDriveActive, movementDriveSpeed,
                                      movementRemainingMs);
  out += ",\"movementLog\":{\"ready\":";
  out += movement.ready ? "true" : "false";
  out += ",\"recording\":";
  out += movement.recording ? "true" : "false";
  out += ",\"error\":";
  out += movement.error ? "true" : "false";
  out += ",\"full\":";
  out += movement.full ? "true" : "false";
  out += ",\"writing\":";
  out += movement.writing ? "true" : "false";
  out += ",\"queued\":";
  out += String((unsigned long)movement.queued);
  out += "}";
  out += ",\"movementDrive\":{\"active\":";
  out += movementDriveActive ? "true" : "false";
  out += ",\"speed\":";
  out += String(movementDriveSpeed, 2);
  out += ",\"remainingMs\":";
  out += String((unsigned long)movementRemainingMs);
  out += '}';
  out += ",\"rightTurnTest\":{\"active\":";
  out += rightTurnActive ? "true" : "false";
  out += ",\"remainingMs\":";
  out += String((unsigned long)rightTurnRemainingMs);
  out += ",\"speed\":";
  out += String(rightTurnActive ? rightTurnSpeed : 0, 2);
  out += ",\"lastResult\":\"";
  portENTER_CRITICAL(&_rightTurnTestMux);
  jsonEscape(out, _rightTurnTestResult);
  portEXIT_CRITICAL(&_rightTurnTestMux);
  out += "\"}";
  out += ",\"wallFollow\":{\"active\":";
  out += wallFollowActive ? "true" : "false";
  out += ",\"speed\":";
  out += String(wallFollowActive ? wallFollowSpeed : _wallFollowReportedSpeed, 2);
  out += ",\"wallCm\":";
  out += String(_wallFollowWallCm);
  out += ",\"frontCm\":";
  out += String(_wallFollowFrontCm);
  out += ",\"correction\":";
  out += String(_wallFollowCorrection, 3);
  out += ",\"left\":";
  out += String(_wallFollowLeftSpeed, 3);
  out += ",\"right\":";
  out += String(_wallFollowRightSpeed, 3);
  out += ",\"reason\":\"";
  jsonEscape(out, _wallFollowReason);
  out += "\"}";
  out += '}';
  xSemaphoreGive(_mtx);
  return out;
}
