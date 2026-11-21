#pragma once
#include <Arduino.h>
// Mission page: one Start button. The run is not saved and the page never asks for data,
// so the phone can leave the Wi-Fi after Start.
static const char kMissionPage[] PROGMEM = R"RV8MIS(
<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover 8 mission</title>
<style>
:root{--bg:#f4f6f8;--ink:#17202a;--muted:#5b6876;--ok:#17803d;--bad:#b3261e}
@media(prefers-color-scheme:dark){:root{--bg:#10161c;--ink:#eef3f7;--muted:#a3b1be;--ok:#6fdc97;--bad:#ff9d94}}
body{margin:0;background:var(--bg);color:var(--ink);font:18px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
main{max-width:480px;margin:0 auto;padding:16px;text-align:center}
h1{font-size:24px;margin:8px 0 4px}
p{color:var(--muted);margin:0 0 20px}
button{font:inherit;font-size:28px;font-weight:800;width:100%;min-height:120px;border:0;border-radius:16px;color:#fff;background:#17803d;cursor:pointer}
#stop{background:#c62828;margin-top:14px}
button:disabled{background:#8a96a3;cursor:not-allowed}
#msg{min-height:3em;margin-top:20px;font-weight:700}
.ok{color:var(--ok)}.bad{color:var(--bad)}
a{color:var(--muted)}
</style></head><body><main>
<h1>Rover 8 mission</h1>
<p>Runs the route alone. Nothing is saved or sent. After Start you can leave the Wi-Fi.</p>
<button id="start">Start mission</button>
<button id="stop">EMERGENCY STOP</button>
<p id="msg"></p>
<a href="/">Test page</a>
<script>
'use strict';
const msg=document.getElementById('msg'),start=document.getElementById('start');
start.onclick=async()=>{
 start.disabled=true;msg.className='';msg.textContent='Sending...';
 try{
  const r=await (await fetch('/api/mission',{method:'POST'})).json();
  msg.className=r.ok?'ok':'bad';msg.textContent=r.text;
  if(!r.ok)start.disabled=false;
 }catch(e){msg.className='bad';msg.textContent='No connection to the box. Join Wi-Fi CAS-Box-08.';start.disabled=false;}
};
document.getElementById('stop').onclick=()=>fetch('/api/stop',{method:'POST'});
</script></main></body></html>
)RV8MIS";
