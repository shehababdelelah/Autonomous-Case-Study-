#pragma once
#include <Arduino.h>
// Simple navigation page. It reads /api/status and sends Start and Stop to /api/test.
static const char kNavigationPage[] PROGMEM = R"RV8NAV(
<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover 8</title>
<style>
:root{--bg:#f4f6f8;--card:#fff;--ink:#17202a;--muted:#5b6876;--line:#d9e0e6;
 --ok:#17803d;--okbg:#e4f5ea;--wait:#935f00;--waitbg:#fdf1d7;--bad:#b3261e;--badbg:#fce8e6;
 --idle:#34506b;--idlebg:#e6edf4}
@media(prefers-color-scheme:dark){:root{--bg:#10161c;--card:#1a232c;--ink:#eef3f7;--muted:#a3b1be;
 --line:#2e3b47;--ok:#6fdc97;--okbg:#17382a;--wait:#f5c469;--waitbg:#3a2e12;--bad:#ff9d94;
 --badbg:#43201d;--idle:#a9c7e3;--idlebg:#22313f}}
*{box-sizing:border-box}
[hidden]{display:none!important}
body{margin:0;background:var(--bg);color:var(--ink);font:17px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
main{max-width:480px;margin:0 auto;padding:16px}
header{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}
h1{font-size:22px;margin:0}
h2{font-size:15px;margin:0 0 8px;color:var(--muted);font-weight:600}
.tag{font-size:14px;color:var(--muted)}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-top:12px}
#status{border-width:2px;margin-top:0}
#title{font-size:30px;line-height:1.15;font-weight:800;margin:0 0 6px}
#why{margin:0;font-size:18px}
#todo{margin:6px 0 0;color:var(--muted);font-size:16px}
#todo:empty{display:none}
.ok{background:var(--okbg);border-color:var(--ok)}.ok #title{color:var(--ok)}
.wait{background:var(--waitbg);border-color:var(--wait)}.wait #title{color:var(--wait)}
.bad{background:var(--badbg);border-color:var(--bad)}.bad #title{color:var(--bad)}
.idle{background:var(--idlebg);border-color:var(--idle)}.idle #title{color:var(--idle)}
.buttons{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
button{font:inherit;font-weight:800;min-height:68px;border:0;border-radius:12px;color:#fff;cursor:pointer}
#start{background:#17803d}#stop{background:#c62828}
#start:disabled{background:#8a96a3;cursor:not-allowed}
#msg{margin:8px 0 0;font-size:15px;color:var(--muted)}
#msg:empty{display:none}
ul{list-style:none;margin:0;padding:0}
li,.row{display:flex;justify-content:space-between;gap:12px;padding:7px 0;border-top:1px solid var(--line)}
li:first-child,.row:first-of-type{border-top:0}
li b{font-weight:700;text-align:right}
.yes{color:var(--ok)}.no{color:var(--bad)}
.row span:first-child{color:var(--muted)}
.row span:last-child{font-weight:700;text-align:right}
.bar{height:10px;background:var(--line);border-radius:5px;overflow:hidden;margin:2px 0 8px}
.bar i{display:block;height:100%;width:0;background:var(--idle)}
details{margin-top:12px;color:var(--muted);font-size:15px}
pre{white-space:pre-wrap;word-break:break-all;font-size:13px}
</style></head><body><main>
<header><h1>Rover 8</h1><span class="tag" id="link">Connecting</span></header>
<section class="card idle" id="status">
 <p id="title">Connecting</p><p id="why">Waiting for the box.</p><p id="todo"></p>
</section>
<div class="buttons"><button id="start" disabled>Start</button><button id="stop">STOP</button></div>
<p id="msg"></p>
<section class="card"><h2>Checks before driving</h2><ul id="checks"></ul></section>
<section class="card"><h2>Wheel power asked by the program</h2>
 <div class="row"><span>Left</span><span id="left">-</span></div><div class="bar"><i id="leftBar"></i></div>
 <div class="row"><span>Right</span><span id="right">-</span></div><div class="bar"><i id="rightBar"></i></div>
 <div class="row" id="batRow" hidden><span>Battery</span><span id="bat">-</span></div>
</section>
<section class="card"><h2>This run</h2>
 <div class="row"><span>Step</span><span id="step">-</span></div>
 <div class="row"><span>Time</span><span id="time">-</span></div>
 <div class="row"><span>Distance (estimate only)</span><span id="dist">-</span></div>
 <div class="row"><span>Pillars</span><span id="pillars">-</span></div>
</section>
<section class="card"><h2>Distance sensors</h2>
 <div class="row"><span>Front</span><span id="front">-</span></div>
 <div class="row"><span>Left</span><span id="sideL">-</span></div>
 <div class="row"><span>Right</span><span id="sideR">-</span></div>
</section>
<details><summary>Raw status for debugging</summary><pre id="raw"></pre></details>
<script>
'use strict';
const $=id=>document.getElementById(id);
const say=(id,text)=>{$(id).textContent=text;};
const STEPS={START_FORWARD:'Driving forward',TURN_LEFT:'Turning left',COUNT_PILLARS:'Counting pillars',
 CLEAR_FIFTH:'Passing the last pillar',TURN_RIGHT:'Turning right',IDLE:'Not started',WAITING:'Waiting',
 DONE:'Finished',STOPPED:'Stopped',FAULT:'Stopped by safety'};
let busy=false;

async function call(path,options){
 const stop=new AbortController(),timer=setTimeout(()=>stop.abort(),1500);
 try{
  const r=await fetch(path,{...(options||{}),signal:stop.signal,cache:'no-store'});
  const data=await r.json();
  if(!r.ok||data.ok===false)throw Error(data.error||('HTTP '+r.status));
  return data;
 }finally{clearTimeout(timer);}
}
function cm(v){return Number.isFinite(v)&&v>=0?Math.round(v)+' cm':'No echo';}
function checks(s){
 const n=s.navigation||{},u=s.ultrasonics||{},i=s.imu||{},limit=n.frontStopCm||35;
 const sonar=Number.isFinite(n.sonarAgeMs)&&n.sonarAgeMs<=1500;
 const still=!!i.ok&&Math.abs(i.rollDeg)<=5&&Math.abs(i.pitchDeg)<=5&&Math.abs(i.yawRateDps)<=3;
 return [
  {name:'Rover link',ok:!!s.robot.connected,text:s.robot.connected?'Connected':'Not connected',
   todo:'Start the box first, then restart the rover.'},
  {name:'IMU',ok:!!i.ok,text:i.ok?'Working':'No data',todo:'Check the IMU wiring and wait for calibration.'},
  {name:'Sonar',ok:sonar,text:sonar?'Working':'No fresh data',todo:'Check the ultrasonic sensors.'},
  {name:'Front clear',ok:sonar&&u.frontCm>=limit,text:sonar?cm(u.frontCm):'Unknown',
   todo:u.frontCm<0?'No echo: put a flat wall 1 to 3 m ahead for this test.':'Move the rover or the object in front.'},
  {name:'Level and still',ok:still,text:still?'Yes':'No',todo:'Put the rover on flat ground and do not touch it.'}
 ];
}
function faultText(note,limit){
 const list=[
  ['front obstacle','Something is closer than '+limit+' cm in front.','Clear the path, then press Start.'],
  ['front range unknown','The front sensor got no echo.','Nothing is within 3 m, or the echo was missed. Put a flat wall 1 to 3 m ahead.'],
  ['tilt','The rover tilted more than 18 degrees.','Put it on flatter ground.'],
  ['unavailable','IMU, sonar or rover link was lost while driving.','Check which item below is not OK.'],
  ['timed out','The rover was not ready 30 seconds after Start.','Fix the items below, then press Start.'],
  ['CampusNavTest','This route step took too long.','The rover may not be moving enough. Check wheel power and battery.'],
  ['disabled','Navigation is turned off in the code.','Set kEnableNavigationTest to true.']];
 const hit=list.find(item=>note.includes(item[0]));
 return hit?[hit[1],hit[2]]:[note,''];
}
function explain(s,list){
 const n=s.navigation||{},phase=(s.test||{}).state,state=s.robot.state;
 const missing=list.find(item=>!item.ok);
 if(phase==='running'){
  if(state==='WAITING'){
   if(missing)return ['wait','WAITING',missing.name+': '+missing.text,missing.todo];
   return ['wait','WAITING','Keep the rover still.','Driving starts in about 1.5 seconds.'];
  }
  return ['ok','DRIVING',STEPS[state]||state,'If the wheels do not turn, the power is too low or the battery is weak.'];
 }
 if(phase==='completed')return ['ok','FINISHED','The program finished this step.',
  'It has no wheel sensor, so it says this even if the rover did not move. Check the rover with your eyes.'];
 if(phase==='stopped')return ['idle','STOPPED','You pressed Stop.','Press Start to run again.'];
 if(phase==='fault'){const t=faultText(s.robot.mode||'',n.frontStopCm||35);return ['bad','STOPPED BY SAFETY',t[0],t[1]];}
 if(!s.robot.movementEnabled)return ['bad','TURNED OFF','Navigation is turned off in the code.','Set kEnableNavigationTest to true.'];
 if(missing)return ['wait','NOT READY',missing.name+': '+missing.text,missing.todo];
 return ['idle','READY','Press Start to drive.','Keep your hand near the power switch.'];
}
function wheel(id,value){
 const ok=Number.isFinite(value),percent=ok?Math.min(100,Math.abs(value)/0.5*100):0;
 say(id,ok?value.toFixed(2)+' ('+Math.round(percent)+'%)':'-');
 $(id+'Bar').style.width=percent+'%';
}
function show(s){
 const n=s.navigation||{},t=s.test||{},u=s.ultrasonics||{},list=checks(s),view=explain(s,list);
 const running=t.state==='running',seconds=Math.floor((t.elapsedMs||0)/1000);
 $('status').className='card '+view[0];
 say('title',view[1]);say('why',view[2]);say('todo',view[3]||'');say('link','Connected to box');
 $('start').disabled=busy||running||!s.robot.movementEnabled||list.some(item=>!item.ok);
 const box=$('checks');box.textContent='';
 for(const item of list){
  const row=document.createElement('li'),name=document.createElement('span'),value=document.createElement('b');
  name.textContent=item.name;value.textContent=(item.ok?'OK - ':'NOT OK - ')+item.text;
  value.className=item.ok?'yes':'no';row.append(name,value);box.append(row);
 }
 wheel('left',s.robot.leftCommand);wheel('right',s.robot.rightCommand);
 const volts=s.robot.voltage;$('batRow').hidden=!(volts>0);if(volts>0)say('bat',volts.toFixed(1)+' V');
 say('step',STEPS[s.robot.state]||s.robot.state||'-');
 say('time',Math.floor(seconds/60)+':'+String(seconds%60).padStart(2,'0'));
 say('dist',Number.isFinite(n.estimatedM)?n.estimatedM.toFixed(2)+' m':'-');
 say('pillars',(n.pillars||0)+' of 5');
 say('front',cm(u.frontCm));say('sideL',cm(u.leftCm));say('sideR',cm(u.rightCm));
 say('raw',JSON.stringify(s,null,1));
}
function offline(){
 $('status').className='card bad';
 say('title','NO CONNECTION');say('why','This phone cannot reach the box.');
 say('todo','Join Wi-Fi CAS-Box-08. A running test can continue, so stay near the rover.');
 say('link','Not connected to box');$('start').disabled=true;
 $('checks').textContent='';$('batRow').hidden=true;
 for(const id of ['step','time','dist','pillars','front','sideL','sideR'])say(id,'-');
 wheel('left',NaN);wheel('right',NaN);
}
async function poll(){
 try{show(await call('/api/status'));}catch(e){offline();}
 finally{setTimeout(poll,500);}
}
async function send(query,text){
 busy=true;$('start').disabled=true;say('msg',text);
 try{await call('/api/test?'+query,{method:'POST'});say('msg','');}
 catch(e){say('msg','Failed: '+e.message);}
 finally{busy=false;}
}
$('start').onclick=()=>send('action=start&id=autonomous','Starting...');
$('stop').onclick=()=>send('action=stop','Stopping...');
poll();
</script></main></body></html>
)RV8NAV";
