// TestLoggerWeb.h - phone page for the zone test logger (v3).
//
// Open http://192.168.4.1/test on a phone connected to the box's Wi-Fi
// (best on the Logger's phone; the Operator keeps the driving page).
//
//   Zone 1..4               choose before Start; one zone is one test. Each button shows how
//                           long that zone records: 1:30, 3:00, 2:00 and 1:00.
//   Start test              new run folder, e.g. Z1-R2. Drives the rover straight forward at
//                           full speed and records for that zone's time, then stops and saves by
//                           itself, so one press is the whole test for that zone. It also stops
//                           early if anything comes within kStopCm in front. Test the four zones
//                           one after the other, pressing Start each time.
//   Mark / Undo             waypoint averaging (stand still 20 s)
//   Stop and save           ends a run before its 30 s are up
//   Saved runs              tap a file to download it, "Download all" for the 5 or 20 files
//                           of a run, "Delete" to free flash
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
body{font-family:sans-serif;margin:14px;max-width:520px}
pre{background:#f0f0f0;padding:10px;border-radius:8px;white-space:pre-wrap;font-size:15px;min-height:7em}
button{font-size:18px;padding:12px;margin:6px 0;border-radius:10px;border:1px solid #888;background:#fff}
.big{display:block;width:100%}
.zones{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.z small{display:block;font-size:12px;color:#555}
.z.sel{background:#1f3864;color:#fff}.z.sel small{color:#dde}
#mark{background:#2e7d32;color:#fff;border:0}#stop{background:#c62828;color:#fff;border:0}
button:disabled{opacity:.4}
.run{border:1px solid #ccc;border-radius:8px;padding:8px;margin:8px 0}
.run button{font-size:14px;padding:6px 10px;margin:2px}
ul{margin:6px 0;padding-left:18px}li{margin:4px 0}
</style></head><body>
<h2>Zone tests</h2>
<pre id="st">Connecting...</pre>
<div class="zones">
<button class="z sel" data-z="1">Zone 1<small>B to C, R1<br>1:30</small></button>
<button class="z" data-z="2">Zone 2<small>F to E, R2<br>3:00</small></button>
<button class="z" data-z="3">Zone 3<small>D/E to K, R3<br>2:00</small></button>
<button class="z" data-z="4">Zone 4<small>K to H<br>1:00</small></button>
</div>
<button id="start" class="big">Start test (drives forward and records)</button>
<button id="mark" class="big">Mark waypoint (stand still 20 s)</button>
<button id="undo" class="big">Undo last waypoint</button>
<button id="stop" class="big">Stop and save now</button>
<h3>Saved runs</h3>
<div id="files">Loading...</div>
<button id="delall" class="big">Delete all saved runs</button>
<script>
const $=i=>document.getElementById(i);
let zone=1,rec=false,lastRec=null,runs=[];
document.querySelectorAll('.z').forEach(b=>b.onclick=()=>{zone=+b.dataset.z;
 document.querySelectorAll('.z').forEach(x=>x.classList.toggle('sel',x===b));});
function post(u){return fetch('/test/'+u,{method:'POST'}).then(()=>setTimeout(poll,300));}
$('start').onclick=()=>post('start?zone='+zone).then(()=>setTimeout(files,800));
$('mark').onclick=()=>post('mark');
$('undo').onclick=()=>post('undo');
$('stop').onclick=()=>{if(confirm('Stop and save this run?'))post('stop').then(()=>setTimeout(files,800));};
$('delall').onclick=()=>{if(confirm('Delete ALL saved runs?'))post('delete?run=*').then(()=>setTimeout(files,800));};
function poll(){fetch('/test/status').then(r=>r.text()).then(t=>{$('st').textContent=t;
 rec=t.startsWith('RECORDING');$('start').disabled=rec;
 ['mark','undo','stop'].forEach(i=>$(i).disabled=!rec);
 document.querySelectorAll('.z').forEach(b=>b.disabled=rec);
 if(rec!==lastRec){lastRec=rec;files();}}).catch(()=>{$('st').textContent='No connection to the box';});}
function kb(s){return (s/1024).toFixed(1)+' KB';}
function link(run,n){return '/test/file?path='+encodeURIComponent('/'+run+'/'+n);}
function files(){fetch('/test/files').then(r=>r.json()).then(j=>{
 runs=j.runs.sort((a,b)=>a.run.localeCompare(b.run,undefined,{numeric:true}));let h='';
 for(const r of runs){const act=r.run===j.rec;
  h+='<div class="run"><b>'+r.run+'</b>'+(act?' (recording)':
   ' <button onclick="dlAll(\''+r.run+'\')">Download all</button> <button onclick="del(\''+r.run+'\')">Delete</button>')+'<ul>';
  for(const f of r.files.sort((a,b)=>a.n.localeCompare(b.n)))
   h+='<li>'+(act?f.n:'<a href="'+link(r.run,f.n)+'">'+f.n+'</a>')+' <small>'+kb(f.s)+'</small></li>';
  h+='</ul></div>';}
 if(j.more)h+='<p>More runs are saved than fit here. Delete old runs.</p>';
 $('files').innerHTML=h||'<p>No saved runs yet.</p>';}).catch(()=>{});}
function dlAll(run){const r=runs.find(x=>x.run===run);if(!r)return;let i=0;
 (function next(){if(i>=r.files.length)return;const a=document.createElement('a');a.href=link(run,r.files[i++].n);
  a.download='';document.body.appendChild(a);a.click();a.remove();setTimeout(next,900);})();}
function del(run){if(confirm('Delete '+run+'?'))post('delete?run='+encodeURIComponent(run)).then(()=>setTimeout(files,800));}
setInterval(poll,1000);setInterval(()=>{if(rec)files();},10000);poll();files();
</script></body></html>)HTML";

inline void addTestLogRoutes(AsyncWebServer& server, TestLogger& tl) {
  // specific routes first: "/test" alone would also match "/test/..."
  server.on("/test/status", HTTP_GET, [&tl](AsyncWebServerRequest* r) {
    char s[448];
    tl.copyStatus(s, sizeof s);
    r->send(200, "text/plain", s);
  });
  server.on("/test/files", HTTP_GET, [&tl](AsyncWebServerRequest* r) {
    const size_t n = 8192;
    char* buf = static_cast<char*>(malloc(n));
    if (!buf) { r->send(500, "text/plain", "out of memory"); return; }
    tl.listJson(buf, n);
    r->send(200, "application/json", buf);
    free(buf);
  });
  server.on("/test/start", HTTP_POST, [&tl](AsyncWebServerRequest* r) {
    if (!r->hasParam("zone")) { r->send(400, "text/plain", "zone missing"); return; }
    const int z = r->getParam("zone")->value().toInt();
    if (z < 0 || z > 4) { r->send(400, "text/plain", "zone must be 0 to 4"); return; }
    tl.requestStart((uint8_t)z);
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
