#pragma once
#include <Arduino.h>
static const char kNavigationPage[] PROGMEM = R"RV8NAV(
<!doctype html>
<html lang="en" data-demo="false" data-page="navigation"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><title>Rover 8 · Navigation mode</title><style>:root{color-scheme:dark;--bg:#0b1218;--panel:#131e27;--ink:#edf5f5;--muted:#96aab9;--line:#2b3d49;--teal:#73e1c1;--amber:#f1be72;--red:#ff9393}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}button,input{font:inherit}button,a,input{ -webkit-tap-highlight-color:transparent}a{color:var(--teal)}button{cursor:pointer;min-height:44px;border:1px solid var(--line);border-radius:9px;padding:10px 16px;background:#1c2c38;color:var(--ink);font-weight:650}button:disabled{opacity:.42;cursor:not-allowed}.primary{background:var(--teal);color:#09241f;border-color:var(--teal)}.danger{color:#ffbaba;background:#352129;border-color:#75404b}.ghost{background:transparent}.shell{max-width:1320px;margin:auto;padding:24px max(20px,env(safe-area-inset-right)) 28px max(20px,env(safe-area-inset-left))}.top{display:flex;justify-content:space-between;align-items:center;gap:20px}.brand{font-size:12px;font-weight:800;letter-spacing:.15em}.brand span{color:var(--muted);font-weight:500}.badge{font-size:12px;border:1px solid var(--line);border-radius:20px;padding:5px 11px;white-space:nowrap}.good{color:var(--teal)}.warn{color:var(--amber)}.bad{color:var(--red)}nav{display:flex;gap:6px;padding:7px;background:#111c25;border:1px solid var(--line);border-radius:13px;margin:24px 0}nav a{flex:1;text-align:center;text-decoration:none;color:var(--muted);padding:12px 9px;border-radius:8px;font-weight:650}nav a[aria-current=page]{background:#29483f;color:#aaf7df}.pagehead{display:flex;justify-content:space-between;align-items:center;gap:20px;margin:28px 0}h1{font-size:clamp(28px,4vw,39px);line-height:1.2;letter-spacing:-.04em;margin:0 0 9px}h2{font-size:17px;margin:0}h3{font-size:14px;margin:0}p{margin:8px 0}.muted,.sub{color:var(--muted)}.sub{font-size:13px}.eyebrow{font-size:11px;letter-spacing:.12em;color:var(--muted);font-weight:750}.offline{display:none;padding:12px 16px;margin-bottom:18px;background:#382a22;color:#ffd7a3;border:1px solid #725337;border-radius:9px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}.full{grid-column:1/-1}.panel{background:var(--panel);border:1px solid var(--line);border-radius:13px;min-width:0;overflow:hidden}.panelhead{padding:17px 20px;border-bottom:1px solid var(--line);display:flex;align-items:center;justify-content:space-between;gap:14px}.body{padding:20px}.stats{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin:18px 0}.stat{border:1px solid var(--line);background:var(--panel);border-radius:11px;padding:17px;min-width:0}.stat span{display:block;color:var(--muted);font-size:12px}.stat strong{display:block;font-size:25px;margin-top:5px;overflow-wrap:anywhere}.actions{display:flex;align-items:center;gap:10px;flex-wrap:wrap}.field{display:flex;gap:12px;justify-content:space-between;align-items:center;margin:13px 0}input[type=number],select{font-size:16px;border:1px solid #48616e;background:#0f1921;color:var(--ink);border-radius:8px;min-height:44px;padding:8px;width:110px}input[type=range]{width:100%;accent-color:var(--teal);min-height:44px}input[type=checkbox]{accent-color:var(--teal);width:18px;height:18px;vertical-align:middle}.rangeends{display:flex;justify-content:space-between;color:var(--muted);font-size:11px}.motorvalue{font-size:38px;font-weight:750}.motorvalue small{font-size:14px;color:var(--muted)}.pad{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:9px;max-width:410px;margin:22px auto}.pad button{min-height:72px;touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}.pad .pressed{background:#275349;border-color:var(--teal)}.forward{grid-column:2}.left{grid-column:1}.backward{grid-column:2}.hint{font-size:12px;color:var(--muted);margin-top:14px}.route{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:9px;list-style:none;padding:0;counter-reset:stage}.route li{counter-increment:stage;border:1px solid var(--line);border-radius:8px;padding:12px;font-size:12px;color:var(--muted)}.route li:before{content:counter(stage);display:block;font-size:21px;margin-bottom:7px}.route li.active{border-color:var(--teal);color:var(--teal);background:#1a352e}.route li.gated{opacity:.4;border-style:dashed}.metricgrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:18px}.metricgrid span{display:block;color:var(--muted);font-size:12px}.metricgrid strong{font-size:21px;display:block;margin-top:5px}.callout{background:#29271f;border-left:3px solid var(--amber);padding:12px 15px;border-radius:4px;margin-top:18px;font-size:13px}.scroller{overflow:auto;max-height:350px}table{width:100%;border-collapse:collapse;font-size:13px}th{text-align:left;color:var(--muted);font-size:11px;font-weight:600;position:sticky;top:0;background:var(--panel)}td,th{padding:11px 13px;border-bottom:1px solid var(--line);white-space:nowrap}canvas{display:block;width:100%;height:330px}.calrow{display:grid;grid-template-columns:100px 1fr 40px;gap:12px;align-items:center;margin:13px 0;font-size:12px}.track{background:#283a45;height:7px;border-radius:10px;overflow:hidden}.track div{height:100%;background:var(--teal)}.events{margin:0;padding:0;list-style:none;max-height:230px;overflow:auto}.events li{padding:9px 0;font-size:13px;border-bottom:1px solid var(--line)}.events time{color:var(--muted);margin-right:12px;font-size:11px}.recordnum{font-size:39px;font-weight:750}.foot{display:flex;justify-content:space-between;gap:12px;color:var(--muted);font-size:11px;margin-top:26px;padding-bottom:env(safe-area-inset-bottom)}.toast{display:none;position:fixed;bottom:24px;left:50%;transform:translateX(-50%);max-width:min(92vw,600px);padding:13px 19px;border:1px solid #4a6a71;background:#23343f;color:white;border-radius:10px;z-index:9}.check{display:flex;align-items:center;gap:9px;min-height:44px}button:focus-visible,a:focus-visible,input:focus-visible{outline:3px solid var(--teal);outline-offset:3px}@media(max-width:900px){.stats{grid-template-columns:repeat(2,minmax(0,1fr))}.route{grid-template-columns:repeat(3,minmax(0,1fr))}}@media(max-width:650px){.shell{padding-top:18px}.grid{grid-template-columns:1fr}.pagehead{align-items:flex-start}.pagehead .danger{font-size:12px;padding:10px}.metricgrid{grid-template-columns:repeat(2,minmax(0,1fr))}nav a{font-size:12px;padding:11px 5px}.route{grid-template-columns:repeat(2,minmax(0,1fr))}.top{gap:8px}.brand{font-size:10px}.badge{font-size:10px}.foot{flex-direction:column}.panelhead,.body{padding:16px}canvas{height:300px}}</style></head><body><div class="shell"><header class="top"><div class="brand">ROVER 8 <span>/ TEAM 4</span></div><span class="badge" id="connection">Connecting…</span></header><div class="pagehead"><div><h1>Navigation mode</h1><p class="sub">Test the campus route and follow its state transitions.</p></div><button class="danger" id="stopAll">■ Stop robot</button></div><div class="offline" id="offline" role="status"></div><div class="stats"><div class="stat"><span>Current route state</span><strong id="navState">—</strong></div><div class="stat"><span>Pillar counter</span><strong id="pillars">— / 5</strong></div><div class="stat"><span>Estimated distance</span><strong id="distance">—</strong></div><div class="stat"><span>Run time</span><strong id="elapsed">—</strong></div></div>
<section class="panel"><div class="panelhead"><h2>Algorithm test bench</h2><span class="badge" id="runState">Idle</span></div><div class="body"><label class="field" for="algorithm">Choose test<select id="algorithm" style="width:min(100%,300px)"><option value="autonomous">CampusNavTest</option></select></label><div class="actions"><button class="primary" id="start">▶ Start selected test</button><button class="danger" id="stop">■ Stop test</button></div><label class="check" id="demoOption"><input type="checkbox" id="fullDemo"> Preview full route (simulation only)</label><p class="hint" id="runNote">Waiting for controller telemetry.</p><div class="callout">Starts only after a Start request. Closing this page does not stop an active navigation run. Use Stop to end it.</div></div></section>
<section class="panel" style="margin-top:18px"><div class="panelhead"><h2>Campus route</h2><span class="sub" id="stage">Configured stage: —</span></div><div class="body"><ol class="route" id="route"><li>Forward 0.50 m</li><li>Turn left 90°</li><li>Count five pillars</li><li>Clear 0.80 m</li><li>Turn right 90°</li></ol><div class="metricgrid"><div><span>Target heading</span><strong id="target">—</strong></div><div><span>Heading error</span><strong id="error">—</strong></div><div><span>Segment estimate</span><strong id="segment">—</strong></div><div><span>Counter gate</span><strong id="gate">—</strong></div><div><span>Near / far samples</span><strong id="streak">—</strong></div><div><span>IMU heading</span><strong id="actualHeading">—</strong></div><div><span>L / R requested command</span><strong id="wheelCommands">—</strong></div><div><span>Sonar age</span><strong id="sonarAge">—</strong></div><div><span>Front / obstacle</span><strong id="navFront">—</strong></div><div><span>Right / pillar sensor</span><strong id="navRight">—</strong></div><div><span>Left clearance</span><strong id="navLeft">—</strong></div></div><div class="callout" id="wall">Wall classification is not implemented. Front ranges indicate generic obstacles; the pillar counter uses right ultrasonic events.</div><p class="hint">Distance is estimated from commands/time. LiDAR groups do not increment this pillar counter. New algorithms must be registered in firmware before they can be selected here.</p></div></section>
<section class="panel" style="margin-top:18px"><div class="panelhead"><h2>Run events</h2></div><div class="body"><ul class="events" id="events"></ul></div></section><footer class="foot"><span id="source">Waiting for telemetry</span><span>ROVER 8 · TEAM 4 · <span id="uptime">—</span></span></footer></div><div class="toast" id="toast" role="status"></div><script>
'use strict';
(function(){
 const $=id=>document.getElementById(id),DEMO=document.documentElement.dataset.demo==='true'||location.protocol==='file:';
 const listeners=[];
 let latest=null,online=false,timer;
 const number=(v,d=1)=>Number.isFinite(v)?v.toFixed(d):'—';
 const range=v=>Number.isFinite(v)&&v>=0?Math.round(v)+' cm':'Unknown';
 const duration=ms=>{const s=Math.floor(Math.max(0,ms)/1000);return String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');};
 const text=(id,v)=>{if($(id))$(id).textContent=v;};
 function toast(message){text('toast',message);$('toast').style.display='block';clearTimeout(timer);timer=setTimeout(()=>$('toast').style.display='none',4000);}
 const boot=Date.now();
 let demo={phase:'idle',started:0,elapsed:0,full:false};
 function snapshot(){
   const now=Date.now(),running=demo.phase==='running',elapsed=running?now-demo.started:demo.elapsed,limit=demo.full?54000:8000,t=Math.min(elapsed/1000,limit/1000);
   if(running&&elapsed>=limit){demo.phase='completed';demo.elapsed=limit;}
   let state='IDLE',count=0,distance=0,segment=0,target=36.4,last=0,right=180,l=0,r=0;
   if(demo.started){
     if(t<8){state='START_FORWARD';segment=distance=.5*t/8;l=r=.12;}
     else if(!demo.full){state='DONE';segment=distance=.5;}
     else if(t<14){state='TURN_LEFT';distance=.5;target=306.4;l=-.07;r=.07;last=1;}
     else if(t<39){state='COUNT_PILLARS';count=Math.min(5,Math.floor((t-14)/5)+1);segment=(t-14)*.4;distance=.5+segment;target=306.4;right=(t-14)%5<1.5?75:180;l=r=.12;last=2;}
     else if(t<47){state='CLEAR_FIFTH';count=5;segment=.8*(t-39)/8;distance=10.5+segment;target=306.4;l=r=.10;last=3;}
     else if(t<54){state='TURN_RIGHT';count=5;distance=11.3;l=.07;r=-.07;last=4;}
     else{state='DONE';count=5;distance=11.3;last=4;}
   }
   if(demo.phase==='stopped')state='STOPPED';
   if(demo.phase!=='running')l=r=0;
   return {uptimeMs:now-boot,robot:{connected:true,state,mode:'Scripted preview — no robot connection',movementEnabled:true,leftCommand:l,rightCommand:r},imu:{ok:true,headingDeg:target,rollDeg:0,pitchDeg:0,yawRateDps:0,ageMs:20},ultrasonics:{frontCm:220,rightCm:right,leftCm:130},test:{id:'autonomous',state:demo.phase,elapsedMs:demo.phase==='completed'?limit:elapsed,durationMs:0},capabilities:{autonomous:true},navigation:{fresh:true,ageMs:0,enabled:true,state,note:'Simulation only',pillars:count,estimatedM:distance,segmentM:segment,targetHeadingDeg:target,headingErrorDeg:0,pillarArmed:right>=145,nearSamples:right<=100?2:0,farSamples:right>=145?2:0,sonarAgeMs:35,sonarSequence:Math.floor((now-boot)/500),lastStageIndex:last,frontStopCm:35,config:{stopAfterInitial:!demo.full,stopAfterLeft:!demo.full,stopAfterFifth:!demo.full}}};
 }
 async function request(route,options={}){
   if(DEMO){
     const p=new URL(route,'http://preview.local').searchParams;
     if(route.startsWith('/api/status'))return snapshot();
     if(p.get('action')==='stop'){demo.elapsed=demo.started?Date.now()-demo.started:0;demo.phase='stopped';return {ok:true};}
     if(p.get('action')==='start'){
       if(demo.phase==='running')throw Error('A run is already active');
       demo={phase:'running',started:Date.now(),elapsed:0,full:p.get('full')==='1'};return {ok:true};
     }
     throw Error('Unsupported preview request');
   }
   const c=new AbortController(),timeout=setTimeout(()=>c.abort(),1500);
   try{const response=await fetch(route,{...options,signal:c.signal,cache:'no-store'}),data=await response.json();if(!response.ok||data.ok===false)throw Error(data.error||'HTTP '+response.status);return data;}finally{clearTimeout(timeout);}
 }
 async function stop(){await request('/api/test?action=stop',{method:'POST'});toast(DEMO?'Preview stopped.':'Stop accepted; verify the reported state.');}
 async function poll(){
   try{const s=await request('/api/status');if(!s.robot||!s.navigation||!Number.isFinite(s.uptimeMs))throw Error('Unexpected status');
     latest=s;online=true;text('connection',DEMO?'SIMULATED PREVIEW':'ESP32 · navigation');$('connection').className='badge good';$('offline').style.display='none';text('source',DEMO?'SIMULATED DATA · No robot connection':'Live ESP32 navigation');text('uptime',duration(s.uptimeMs)+' uptime');listeners.forEach(fn=>fn(s,true));
   }catch(e){online=false;text('connection','Telemetry unavailable');$('connection').className='badge bad';$('offline').style.display='block';text('offline','No fresh telemetry. An active navigation run may continue; Stop requires a working connection.');listeners.forEach(fn=>fn(null,false));}
   finally{setTimeout(poll,500);}
 }
 window.Rover={$,DEMO,text,number,range,duration,toast,request,stop,subscribe:fn=>listeners.push(fn),get latest(){return latest;},get online(){return online;}};
 window.addEventListener('DOMContentLoaded',()=>{$('stopAll').onclick=()=>stop().catch(e=>toast('Stop failed: '+e.message));poll();});
})();
'use strict';
(function(){
 const R=window.Rover,$=R.$,states=['START_FORWARD','TURN_LEFT','COUNT_PILLARS','CLEAR_FIFTH','TURN_RIGHT'];
 let pending=false,lastState='',lastCount=null;
 function event(message){const li=document.createElement('li'),time=document.createElement('time'),body=document.createElement('span');time.textContent=new Date().toLocaleTimeString();body.textContent=message;li.append(time,body);$('events').prepend(li);while($('events').children.length>30)$('events').lastChild.remove();}
 function update(s,online){
   if(!online){for(const id of ['navState','pillars','distance','elapsed','target','error','segment','gate','streak','sonarAge','navFront','navRight','navLeft'])R.text(id,'—');$('start').disabled=true;R.text('runState','Unavailable');R.text('actualHeading','—');R.text('wheelCommands','—');R.text('stage','Unavailable');R.text('runNote','No fresh controller telemetry');R.text('wall','Front clearance unavailable');document.querySelectorAll('#route li').forEach(li=>li.classList.remove('active'));return;}
   const n=s.navigation||{},t=s.test||{},u=s.ultrasonics||{},fresh=n.fresh&&Number.isFinite(n.ageMs)&&n.ageMs<=500,running=t.state==='running';
   R.text('navState',fresh?n.state:'Unavailable');R.text('pillars',fresh?n.pillars+' / 5':'— / 5');R.text('distance',fresh?R.number(n.estimatedM,2)+' m':'—');R.text('elapsed',R.duration(t.elapsedMs||0));R.text('runState',t.state||'idle');
   R.text('runNote',s.robot.mode);R.text('actualHeading',s.imu?.ok?R.number(s.imu.headingDeg)+'°':'—');R.text('wheelCommands',R.number(s.robot.leftCommand,3)+' / '+R.number(s.robot.rightCommand,3));
   for(const [id,key,unit]of [['target','targetHeadingDeg','°'],['error','headingErrorDeg','°'],['segment','segmentM',' m']])R.text(id,fresh?R.number(n[key],id==='segment'?2:1)+unit:'—');
   R.text('gate',fresh&&n.state==='COUNT_PILLARS'?(n.pillarArmed?'Armed':'Waiting for gap'):'Inactive');
   R.text('streak',fresh?(n.nearSamples||0)+' / '+(n.farSamples||0):'—');
   const sonarOk=fresh&&Number.isFinite(n.sonarAgeMs)&&n.sonarAgeMs<=1500;
   R.text('sonarAge',sonarOk?n.sonarAgeMs+' ms':'Unavailable');
   for(const [id,key]of [['navFront','frontCm'],['navRight','rightCm'],['navLeft','leftCm']])R.text(id,sonarOk?R.range(u[key]):'Unknown');
   R.text('wall',!sonarOk||u.frontCm<0?'Front range unknown: campus motion is blocked.':u.frontCm<(n.frontStopCm||35)?'Front obstruction detected: type unclassified.':'Front above stop threshold. Wall classification is not implemented; right near surfaces can trigger the ultrasonic counter.');
   const c=n.config||{},label=c.stopAfterInitial?'Initial 0.50 m only':c.stopAfterLeft?'Through left turn':c.stopAfterFifth?'Through five pillars + clearance':'Full route';
   R.text('stage',fresh?label:'Unavailable');
   document.querySelectorAll('#route li').forEach((li,index)=>{li.classList.toggle('active',fresh&&states.indexOf(n.state)===index);li.classList.toggle('gated',fresh&&((c.stopAfterInitial&&index>0)||(!c.stopAfterInitial&&c.stopAfterLeft&&index>1)||(!c.stopAfterInitial&&!c.stopAfterLeft&&c.stopAfterFifth&&index>3)));});
   $('start').disabled=pending||running||!s.capabilities?.[$('algorithm').value];$('algorithm').disabled=pending||running;$('fullDemo').disabled=running;
   if(fresh){if(n.state!==lastState){event('Campus: '+n.state);lastState=n.state;}if(Number.isInteger(n.pillars)&&n.pillars!==lastCount){if(lastCount!==null&&n.pillars>lastCount)event('Pillar counter: '+n.pillars+' / 5');lastCount=n.pillars;}}
 }
 R.subscribe(update);
 window.addEventListener('DOMContentLoaded',()=>{
   $('demoOption').style.display=R.DEMO?'':'none';
   $('algorithm').onchange=()=>{if(R.latest)update(R.latest,R.online);};
   $('start').onclick=async()=>{if(pending||!R.online||R.latest?.test?.state==='running')return;const id=$('algorithm').value;if(!R.latest?.capabilities?.[id])return;
     pending=true;$('start').disabled=true;
     try{await R.request('/api/test?action=start&id='+encodeURIComponent(id)+(R.DEMO?'&full='+($('fullDemo').checked?'1':'0'):''),{method:'POST'});event('Requested: '+id);}
     catch(e){R.toast(e.message);event('Start rejected: '+e.message);}finally{pending=false;}
   };
   $('stop').onclick=()=>R.stop().catch(e=>R.toast(e.message));
 });
})();
</script></body></html>
)RV8NAV";
