#pragma once
#include <Arduino.h>
// Simple route page. It reads /api/status and sends /api/start and /api/stop.
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
.ok{background:var(--okbg);border-color:var(--ok)}.ok #title{color:var(--ok)}
.wait{background:var(--waitbg);border-color:var(--wait)}.wait #title{color:var(--wait)}
.bad{background:var(--badbg);border-color:var(--bad)}.bad #title{color:var(--bad)}
.idle{background:var(--idlebg);border-color:var(--idle)}.idle #title{color:var(--idle)}
.buttons{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
button{font:inherit;font-weight:800;min-height:68px;border:0;border-radius:12px;color:#fff;cursor:pointer}
#start{background:#17803d}#stop{background:#c62828}
#start:disabled{background:#8a96a3;cursor:not-allowed}
#restart{background:#5b6876;width:100%;min-height:52px;margin-top:16px}
.check{display:flex;align-items:center;gap:10px;margin-top:12px}
.check input{width:24px;height:24px}
.row{display:flex;justify-content:space-between;gap:12px;padding:7px 0;border-top:1px solid var(--line)}
.row:first-of-type{border-top:0}
.row span:first-child{color:var(--muted)}
.row span:last-child{font-weight:700;text-align:right}
.yes{color:var(--ok)}.no{color:var(--bad)}
</style></head><body><main>
<header><h1>Rover 8</h1><span class="tag" id="link">Connecting</span></header>
<section class="card idle" id="status"><p id="title">Connecting</p><p id="why">Waiting for the box.</p></section>
<label class="check"><input type="checkbox" id="record" checked>Save this run as CSV</label>
<div class="buttons"><button id="start" disabled>Start</button><button id="stop">STOP</button></div>
<p><a href="/log.csv">Download the last run (CSV)</a> &middot; <a href="/mission">Mission mode</a></p>
<section class="card"><h2>Checks</h2>
 <div class="row"><span>Rover link</span><span id="roverOk">-</span></div>
 <div class="row"><span>IMU</span><span id="imuOk">-</span></div>
 <div class="row"><span>LiDAR</span><span id="lidarOk">-</span></div>
</section>
<section class="card"><h2>Route</h2>
 <div class="row"><span>Step</span><span id="step">-</span></div>
 <div class="row"><span>Pillars</span><span id="pillars">-</span></div>
 <div class="row"><span>Walls</span><span id="walls">-</span></div>
 <div class="row"><span>Panels</span><span id="panels">-</span></div>
 <div class="row"><span>Distance (estimate)</span><span id="distance">-</span></div>
</section>
<section class="card"><h2>LiDAR distance</h2>
 <div class="row"><span>Front</span><span id="front">-</span></div>
 <div class="row"><span>Right</span><span id="right">-</span></div>
 <div class="row"><span>Back</span><span id="back">-</span></div>
 <div class="row"><span>Left</span><span id="left">-</span></div>
</section>
<section class="card"><h2>Rover</h2>
 <div class="row"><span>Wheels left / right</span><span id="wheels">-</span></div>
 <div class="row"><span>Heading / target</span><span id="heading">-</span></div>
 <div class="row"><span>Battery</span><span id="battery">-</span></div>
 <div class="row"><span>CSV of this run</span><span id="logging">-</span></div>
</section>
<button id="restart">Restart box</button>
<script>
'use strict';
const $=id=>document.getElementById(id);
const say=(id,text)=>{$(id).textContent=text;};
const cm=v=>v>0?v+' cm':'No reading';
function check(id,ok){say(id,ok?'OK':'NOT OK');$(id).className=ok?'yes':'no';}
function view(s){
 if(s.blocked)return ['wait','OBSTACLE','Waiting until the path is clear.'];
 if(s.running)return ['ok','DRIVING',s.step];
 if(s.stepNumber===13)return ['ok','FINISHED','Check the real position with your eyes.'];
 if(s.stepNumber===14)return ['bad','STOPPED BY SAFETY',s.note];
 if(!s.roverOk)return ['wait','NOT READY','Start the box first, then restart the rover.'];
 if(!s.imuOk)return ['wait','NOT READY','Waiting for the IMU.'];
 if(!s.lidarOk)return ['wait','NOT READY','No LiDAR data. Check the wiring.'];
 return ['idle','READY',s.note];
}
function show(s){
 const v=view(s);
 $('status').className='card '+v[0];say('title',v[1]);say('why',v[2]);say('link','Connected to box');
 $('start').disabled=s.running||!s.roverOk||!s.imuOk||!s.lidarOk;
 $('record').disabled=s.running;
 say('logging',s.logging?'Saved':'Not saved (last saved CSV kept)');
 check('roverOk',s.roverOk);check('imuOk',s.imuOk);check('lidarOk',s.lidarOk);
 say('step',s.step);say('pillars',s.pillars+' of 5');say('walls',s.walls+' of 4');say('panels',s.panels);
 say('distance',s.distanceM.toFixed(2)+' m');
 say('front',cm(s.frontCm));say('right',cm(s.rightCm));say('back',cm(s.backCm));say('left',cm(s.leftCm));
 say('wheels',s.left.toFixed(2)+' / '+s.right.toFixed(2));
 say('heading',s.headingDeg.toFixed(1)+' / '+s.targetDeg.toFixed(1));
 say('battery',s.voltage>0?s.voltage.toFixed(1)+' V':'-');
}
function offline(){
 $('status').className='card bad';say('title','NO CONNECTION');
 say('why','Join Wi-Fi CAS-Box-08. A running test can continue.');
 say('link','Not connected');$('start').disabled=true;
}
async function poll(){
 const stop=new AbortController(),timer=setTimeout(()=>stop.abort(),1500);
 try{show(await (await fetch('/api/status',{signal:stop.signal,cache:'no-store'})).json());}
 catch(e){offline();}
 finally{clearTimeout(timer);setTimeout(poll,500);}
}
try{$('record').checked=localStorage.getItem('record')!=='0';}catch(e){}
$('record').onchange=()=>{try{localStorage.setItem('record',$('record').checked?'1':'0');}catch(e){}};
$('start').onclick=()=>fetch('/api/start?log='+($('record').checked?1:0),{method:'POST'});
$('stop').onclick=()=>fetch('/api/stop',{method:'POST'});
$('restart').onclick=()=>{
 if(confirm('Restart the box? The rover stops and the run log (CSV) is deleted. Download it first.'))
  fetch('/api/restart',{method:'POST'});
};
poll();
</script></main></body></html>
)RV8NAV";
