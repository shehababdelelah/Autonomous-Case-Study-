#pragma once
// NavSurveyWeb.h - phone page for the LiDAR calibration survey.
//
// Open http://192.168.<10+BOX_ID>.1/survey  (for kit 8: http://192.168.18.1/survey)
//
// How to use it to fill in NavCal:
//   1. Drive the box to the pillar before the first right turn. Read the run whose chord is
//      small and whose range is a couple of metres: its chord is the pillar's apparent width
//      and its nearest range is how far away you can still trust it. Those give
//      pillarMinMm / pillarMaxMm / pillarMaxRangeMm.
//   2. Face the zone 1 end wall from where you want the robot to stop. The "end wall: YES at
//      N mm over M deg" line gives endWallMm and endWallMinSpanDeg.
//   3. Stand beside the opening between B and C. The left opening width is openingMinWidthMm
//      (take a little less than the measured value).
//   4. Face the rectangular panel squarely. Its run has a long chord and a low tilt: the chord
//      is panelMinLenMm (again take a little less) and the tilt is panelMaxTiltDeg.
//   5. In the corridor, the left and right nearest values give the corridor width, and
//      bodyHalfWidthMm is measured with a tape, not from the LiDAR.
//
// Add the route where the AsyncWebServer is set up, BEFORE server.begin():
//   #include "NavSurveyWeb.h"
//   addSurveyRoutes(server, navSurvey);
#pragma once
#include <ESPAsyncWebServer.h>
#include "NavSurvey.h"

// The zone selector, Start test, Stop and saved-runs controls below are the same controls as on
// /test, in the same markup and with the same labels, and they call the same /test/* endpoints.
// There is therefore one TestLogger, one zone selection and one recording session: pressing
// Start here is identical to pressing it on the test page. Only the presentation is repeated -
// no test or recording logic is duplicated.
static const char kSurveyPage[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LiDAR survey</title>
<style>
body{font-family:sans-serif;margin:12px;max-width:620px}
pre{background:#101820;color:#d8e6f2;padding:10px;border-radius:8px;white-space:pre;
 overflow-x:auto;font-size:13px;line-height:1.45;min-height:18em}
pre#st{background:#f0f0f0;color:#000;white-space:pre-wrap;font-size:15px;min-height:7em}
h2{margin:0 0 4px}h3{margin:16px 0 4px}p{color:#555;font-size:13px;margin:4px 0 10px}
button{font-size:18px;padding:12px;margin:6px 0;border-radius:10px;border:1px solid #888;background:#fff}
.big{display:block;width:100%}
.zones{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.z small{display:block;font-size:12px;color:#555}
.z.sel{background:#1f3864;color:#fff}.z.sel small{color:#dde}
#start{background:#2e7d32;color:#fff;border:0}#stop{background:#c62828;color:#fff;border:0}
button:disabled{opacity:.4}
.run{border:1px solid #ccc;border-radius:8px;padding:8px;margin:8px 0}
.run button{font-size:14px;padding:6px 10px;margin:2px}
ul{margin:6px 0;padding-left:18px}li{margin:4px 0}
</style></head><body>
<h2>LiDAR survey</h2>
<p>Live geometry, twice a second. Drive to a landmark and read the numbers into NavCal.
The survey itself never moves the robot.</p>
<pre id="r">Connecting...</pre>

<h3>Record a run</h3>
<pre id="st">Connecting...</pre>
<div class="zones">
<button class="z sel" data-z="1">Zone 1<small>B to C, R1<br>1:30</small></button>
<button class="z" data-z="2">Zone 2<small>F to E, R2<br>3:00</small></button>
<button class="z" data-z="3">Zone 3<small>D/E to K, R3<br>2:00</small></button>
<button class="z" data-z="4">Zone 4<small>K to H<br>1:00</small></button>
</div>
<button id="start" class="big">Start test (drives forward and records)</button>
<button id="mark" class="big">Mark waypoint (stand still 20 s)</button>
<button id="stop" class="big">Stop and save now</button>
<h3>Saved runs</h3>
<div id="files">Loading...</div>

<script>
const $=i=>document.getElementById(i);
let zone=1,rec=false,lastRec=null,runs=[],busy=false;
// Zone selection: same state and same values the test page sends, 1..4.
document.querySelectorAll('.z').forEach(b=>b.onclick=()=>{zone=+b.dataset.z;
 document.querySelectorAll('.z').forEach(x=>x.classList.toggle('sel',x===b));});
// Every control posts to the SAME /test/* endpoint the test page uses. busy blocks a second
// press from starting an overlapping run before the status has caught up.
function post(u){if(busy)return Promise.resolve();busy=true;
 return fetch('/test/'+u,{method:'POST'}).then(()=>setTimeout(()=>{busy=false;poll();},300))
  .catch(()=>{busy=false;});}
$('start').onclick=()=>post('start?zone='+zone).then(()=>setTimeout(files,800));
$('mark').onclick=()=>post('mark');
$('stop').onclick=()=>{if(confirm('Stop and save this run?'))post('stop').then(()=>setTimeout(files,800));};
function survey(){fetch('/survey/data').then(r=>r.text()).then(t=>{$('r').textContent=t;})
 .catch(()=>{$('r').textContent='No connection to the box';});}
function poll(){fetch('/test/status').then(r=>r.text()).then(t=>{$('st').textContent=t;
 rec=t.startsWith('RECORDING');$('start').disabled=rec||busy;
 ['mark','stop'].forEach(i=>$(i).disabled=!rec);
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
setInterval(survey,500);setInterval(poll,1000);setInterval(()=>{if(rec)files();},10000);
survey();poll();files();
</script></body></html>)HTML";

/**
 * @brief Registers the survey page and its data endpoint on an existing AsyncWebServer.
 * @param server The server WebHMI already created; routes are added before begin().
 * @param survey The NavSurvey whose report is published; updated from loop(), read here.
 * @return Nothing.
 * @note Route order matters. A plain "/survey" string is matched in
 *       AsyncURIMatcher's BackwardCompatible mode, which also matches "/survey/...", so the
 *       data endpoint MUST be registered first or it would never be reached. Same trap as
 *       addTestLogRoutes(). Read only: no handler here commands the motors.
 */
inline void addSurveyRoutes(AsyncWebServer& server, const NavSurvey& survey) {
  // specific route first: "/survey" alone would also match "/survey/data"
  server.on("/survey/data", HTTP_GET, [&survey](AsyncWebServerRequest* r) {
    char* buf = static_cast<char*>(malloc(NavSurvey::kReportSize));
    if (!buf) { r->send(500, "text/plain", "out of memory"); return; }
    survey.copyReport(buf, NavSurvey::kReportSize);
    r->send(200, "text/plain", buf);
    free(buf);   // send() copies into the response, so this is safe here
  });
  server.on("/survey", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(200, "text/html", kSurveyPage);
  });
}
