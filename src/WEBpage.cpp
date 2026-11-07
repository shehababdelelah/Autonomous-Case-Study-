// #include "WEBpage.hpp"
// #include "NavigationPage.hpp"
// #include "CampusNavTest.hpp"
// #include <string.h>

// namespace {
// String num(double v, unsigned int digits = 2) {
//   return isfinite(v) ? String(v, digits) : String("null");
// }
// String yes(bool v) { return v ? "true" : "false"; }
// String quote(const char* s) {
//   String out;out+='"';
//   for (; *s; ++s) {
//     if (static_cast<uint8_t>(*s)==92 || *s=='"') { out+=static_cast<char>(92); out+=*s; }
//     else if (static_cast<uint8_t>(*s)>=32) out+=*s;
//   }
//   out+='"';return out;
// }
// }
// bool WEBpage::lock() { return mutex_ && xSemaphoreTake(mutex_,pdMS_TO_TICKS(10))==pdTRUE; }
// void WEBpage::unlock() { xSemaphoreGive(mutex_); }
// bool WEBpage::begin() {
//   if(mutex_) return false;
//   mutex_=xSemaphoreCreateMutex(); if(!mutex_) return false;
//   server_.on("/",HTTP_GET,[](AsyncWebServerRequest* r){r->redirect("/navigation");});
//   server_.on("/navigation",HTTP_GET,[](AsyncWebServerRequest* r){
//     auto* response=r->beginResponse_P(200,"text/html; charset=utf-8",kNavigationPage);
//     response->addHeader("Cache-Control","no-store");r->send(response);
//   });
//   server_.on("/api/status",HTTP_GET,[this](AsyncWebServerRequest* r){
//     String json;
//     if(!statusJson(json)){r->send(503,"application/json","{\"ok\":false,\"error\":\"Telemetry busy\"}");return;}
//     auto* response=r->beginResponse(200,"application/json",json);
//     response->addHeader("Cache-Control","no-store");r->send(response);
//   });
//   server_.on("/api/test",HTTP_POST,[this](AsyncWebServerRequest* r){handleCommand(r);});
//   server_.onNotFound([](AsyncWebServerRequest* r){r->send(404,"application/json","{\"ok\":false,\"error\":\"Route not found\"}");});
//   server_.begin();return true;
// }
// void WEBpage::publish(const Status& s) {
//   if(!lock())return;
//   status_=s;publishedMs_=millis();seen_=true;
//   busy_=strcmp(s.phase,"running")==0;
//   unlock();
// }
// WEBpage::Command WEBpage::takeCommand() {
//   if(!lock())return Command::None;
//   Command command=Command::None;
//   if(stopPending_){stopPending_=false;startPending_=false;command=Command::Stop;}
//   else if(startPending_){startPending_=false;busy_=true;command=Command::Start;}
//   unlock();return command;
// }
// void WEBpage::handleCommand(AsyncWebServerRequest* r) {
//   const String action=r->hasParam("action")?r->getParam("action")->value():"";
//   if(action!="start" && action!="stop"){
//     r->send(400,"application/json","{\"ok\":false,\"error\":\"Use action=start or stop\"}");return;
//   }
//   if(action=="start" && (!r->hasParam("id") || r->getParam("id")->value()!="autonomous")){
//     r->send(400,"application/json","{\"ok\":false,\"error\":\"Only CampusNavTest is implemented\"}");return;
//   }
//   if(!lock()){r->send(503,"application/json","{\"ok\":false,\"error\":\"Controller busy\"}");return;}
//   int code=202;const char* reply="{\"ok\":true}";
//   if(action=="stop"){stopPending_=true;startPending_=false;}
//   else if(!seen_ || millis()-publishedMs_>500){code=503;reply="{\"ok\":false,\"error\":\"Controller unavailable\"}";}
//   else if(!status_.enabled){code=403;reply="{\"ok\":false,\"error\":\"kEnableNavigationTest is false\"}";}
//   else if(busy_ || startPending_ || stopPending_){code=409;reply="{\"ok\":false,\"error\":\"A run or stop is already active/pending\"}";}
//   else startPending_=true;
//   unlock();r->send(code,"application/json",reply);
// }
// bool WEBpage::statusJson(String& j) {
//   Status s;bool seen;uint32_t published;
//   if(!lock()) return false;
//   s=status_; seen=seen_; published=publishedMs_;
//   unlock();
//   const uint32_t now=millis();
//   const bool fresh=seen && now-published<=500;
//   const bool imu=fresh && s.imuOk && now-s.imu.sampleMs<=200;
//   const bool sonar=fresh && s.sonarSequence>0 && now-s.sonarMs<=1500;
//   j.reserve(2400);
//   j="{\"uptimeMs\":"+String(now);
//   j+=",\"robot\":{\"connected\":"+yes(fresh&&s.connected);
//   j+=",\"state\":"+quote(fresh?s.state:"UNAVAILABLE")+",\"mode\":"+quote(fresh?s.note:"Controller unavailable");
//   j+=",\"movementEnabled\":"+yes(s.enabled);
//   j+=",\"leftCommand\":"+(fresh?num(s.leftCommand,3):String("null"));
//   j+=",\"rightCommand\":"+(fresh?num(s.rightCommand,3):String("null"))+"}";
//   j+=",\"imu\":{\"ok\":"+yes(imu)+",\"ageMs\":"+(s.imuOk?String(now-s.imu.sampleMs):String("null"));
//   j+=",\"headingDeg\":"+(imu?num(s.imu.headingDeg):String("null"));
//   j+=",\"rollDeg\":"+(imu?num(s.imu.rollDeg):String("null"));
//   j+=",\"pitchDeg\":"+(imu?num(s.imu.pitchDeg):String("null"));
//   j+=",\"yawRateDps\":"+(imu?num(s.imu.yawRateDps):String("null"))+"}";
//   j+=",\"ultrasonics\":{\"frontCm\":"+String(sonar?s.frontCm:-1);
//   j+=",\"rightCm\":"+String(sonar?s.rightCm:-1)+",\"leftCm\":"+String(sonar?s.leftCm:-1)+"}";
//   j+=",\"test\":{\"id\":\"autonomous\",\"state\":"+quote(fresh?s.phase:"fault")+",\"elapsedMs\":"+String(s.elapsedMs)+",\"durationMs\":0}";
//   j+=",\"capabilities\":{\"autonomous\":"+yes(fresh&&s.enabled)+"}";
//   j+=",\"navigation\":{\"fresh\":"+yes(fresh)+",\"ageMs\":"+(seen?String(now-published):String("null"));
//   j+=",\"enabled\":"+yes(s.enabled)+",\"state\":"+quote(fresh?s.state:"UNAVAILABLE")+",\"note\":"+quote(s.note);
//   j+=",\"pillars\":"+String(s.pillars)+",\"estimatedM\":"+num(s.estimatedM,3)+",\"segmentM\":"+num(s.segmentM,3);
//   j+=",\"targetHeadingDeg\":"+num(s.targetHeadingDeg)+",\"headingErrorDeg\":"+num(s.headingErrorDeg);
//   j+=",\"pillarArmed\":"+yes(s.pillarArmed)+",\"nearSamples\":"+String(s.nearSamples)+",\"farSamples\":"+String(s.farSamples);
//   j+=",\"lastStageIndex\":"+String(s.lastStageIndex);
//   j+=",\"sonarAgeMs\":"+(s.sonarSequence?String(now-s.sonarMs):String("null"))+",\"sonarSequence\":"+String(s.sonarSequence);
//   j+=",\"frontStopCm\":"+String(CampusNavTest::kFrontStopCm)+",\"enterCm\":"+String(CampusNavTest::kPillarEnterCm)+",\"exitCm\":"+String(CampusNavTest::kPillarExitCm);
//   j+=",\"minSpacingM\":"+num(CampusNavTest::kMinPillarSpacingM);
//   j+=",\"config\":{\"stopAfterInitial\":"+yes(CampusNavTest::kStopAfterInitialForward)+",\"stopAfterLeft\":"+yes(CampusNavTest::kStopAfterLeftTurn);
//   j+=",\"stopAfterFifth\":"+yes(CampusNavTest::kStopAfterFifthPillar)+"}}}";
//   return true;
// }

#include "WEBpage.hpp"
#include "NavigationPage.hpp"
#include "CampusNavTest.hpp"
#include <string.h>

namespace {
String num(double v, unsigned int digits=2) { return isfinite(v) ? String(v,digits) : String("null"); }
String yes(bool v) { return v ? "true" : "false"; }
String quote(const char* s) {
  String out;out+='"';
  for (; *s; ++s) {
    if (static_cast<uint8_t>(*s)==92 || *s=='"') { out+=static_cast<char>(92); out+=*s; }
    else if (static_cast<uint8_t>(*s)>=32) out+=*s;
  }
  out+='"';return out;
}
}
bool WEBpage::lock() { return mutex_ && xSemaphoreTake(mutex_,pdMS_TO_TICKS(10))==pdTRUE; }
void WEBpage::unlock() { xSemaphoreGive(mutex_); }
bool WEBpage::begin() {
  if(mutex_) return false;
  mutex_=xSemaphoreCreateMutex(); if(!mutex_) return false;
  server_.on("/",HTTP_GET,[](AsyncWebServerRequest* r){r->redirect("/navigation");});
  server_.on("/navigation",HTTP_GET,[](AsyncWebServerRequest* r){
    auto* response=r->beginResponse_P(200,"text/html; charset=utf-8",kNavigationPage);
    response->addHeader("Cache-Control","no-store");r->send(response);
  });
  server_.on("/api/status",HTTP_GET,[this](AsyncWebServerRequest* r){
    String json;
    if(!statusJson(json)){r->send(503,"application/json","{\"ok\":false,\"error\":\"Telemetry busy\"}");return;}
    auto* response=r->beginResponse(200,"application/json",json);
    response->addHeader("Cache-Control","no-store");r->send(response);
  });
  server_.on("/api/test",HTTP_POST,[this](AsyncWebServerRequest* r){handleCommand(r);});
  server_.onNotFound([](AsyncWebServerRequest* r){r->send(404,"application/json","{\"ok\":false,\"error\":\"Route not found\"}");});
  server_.begin();return true;
}
void WEBpage::publish(const Status& s) {
  if(!lock())return;
  status_=s;publishedMs_=millis();seen_=true;
  busy_=strcmp(s.phase,"running")==0;
  unlock();
}
WEBpage::Command WEBpage::takeCommand() {
  if(!lock())return Command::None;
  Command command=Command::None;
  if(stopPending_){stopPending_=false;startPending_=false;command=Command::Stop;}
  else if(startPending_){startPending_=false;busy_=true;command=Command::Start;}
  unlock();return command;
}
void WEBpage::handleCommand(AsyncWebServerRequest* r) {
  const String action=r->hasParam("action")?r->getParam("action")->value():"";
  if(action!="start" && action!="stop"){
    r->send(400,"application/json","{\"ok\":false,\"error\":\"Use action=start or stop\"}");return;
  }
  if(action=="start" && (!r->hasParam("id") || r->getParam("id")->value()!="autonomous")){
    r->send(400,"application/json","{\"ok\":false,\"error\":\"Only CampusNavTest is implemented\"}");return;
  }
  if(!lock()){r->send(503,"application/json","{\"ok\":false,\"error\":\"Controller busy\"}");return;}
  int code=202;const char* reply="{\"ok\":true}";
  if(action=="stop"){stopPending_=true;startPending_=false;}
  else if(!seen_ || millis()-publishedMs_>500){code=503;reply="{\"ok\":false,\"error\":\"Controller unavailable\"}";}
  else if(!status_.enabled){code=403;reply="{\"ok\":false,\"error\":\"kEnableNavigationTest is false\"}";}
  else if(busy_ || startPending_ || stopPending_){code=409;reply="{\"ok\":false,\"error\":\"A run or stop is already active/pending\"}";}
  else startPending_=true;
  unlock();r->send(code,"application/json",reply);
}
bool WEBpage::statusJson(String& j) {
  Status s;bool seen;uint32_t published;
  if(!lock()) return false;
  s=status_; seen=seen_; published=publishedMs_;
  unlock();
  const uint32_t now=millis();
  const bool fresh=seen && now-published<=500;
  const bool imu=fresh && s.imuOk && now-s.imu.sampleMs<=200;
  const bool sonar=fresh && s.sonarSequence>0 && now-s.sonarMs<=1500;
  j.reserve(2400);
  j="{\"uptimeMs\":"+String(now);
  j+=",\"robot\":{\"connected\":"+yes(fresh&&s.connected);
  j+=",\"state\":"+quote(fresh?s.state:"UNAVAILABLE")+",\"mode\":"+quote(fresh?s.note:"Controller unavailable");
  j+=",\"movementEnabled\":"+yes(s.enabled);
  j+=",\"leftCommand\":"+(fresh?num(s.leftCommand,3):String("null"));
  j+=",\"rightCommand\":"+(fresh?num(s.rightCommand,3):String("null"))+"}";
  j+=",\"imu\":{\"ok\":"+yes(imu)+",\"ageMs\":"+(s.imuOk?String(now-s.imu.sampleMs):String("null"));
  j+=",\"headingDeg\":"+(imu?num(s.imu.headingDeg):String("null"));
  j+=",\"rollDeg\":"+(imu?num(s.imu.rollDeg):String("null"));
  j+=",\"pitchDeg\":"+(imu?num(s.imu.pitchDeg):String("null"));
  j+=",\"yawRateDps\":"+(imu?num(s.imu.yawRateDps):String("null"))+"}";
  j+=",\"ultrasonics\":{\"frontCm\":"+String(sonar?s.frontCm:-1);
  j+=",\"rightCm\":"+String(sonar?s.rightCm:-1)+",\"leftCm\":"+String(sonar?s.leftCm:-1)+"}";
  j+=",\"test\":{\"id\":\"autonomous\",\"state\":"+quote(fresh?s.phase:"fault")+",\"elapsedMs\":"+String(s.elapsedMs)+",\"durationMs\":0}";
  j+=",\"capabilities\":{\"autonomous\":"+yes(fresh&&s.enabled)+"}";
  j+=",\"navigation\":{\"fresh\":"+yes(fresh)+",\"ageMs\":"+(seen?String(now-published):String("null"));
  j+=",\"enabled\":"+yes(s.enabled)+",\"state\":"+quote(fresh?s.state:"UNAVAILABLE")+",\"note\":"+quote(s.note);
  j+=",\"pillars\":"+String(s.pillars)+",\"estimatedM\":"+num(s.estimatedM,3)+",\"segmentM\":"+num(s.segmentM,3);
  j+=",\"targetHeadingDeg\":"+num(s.targetHeadingDeg)+",\"headingErrorDeg\":"+num(s.headingErrorDeg);
  j+=",\"pillarArmed\":"+yes(s.pillarArmed)+",\"nearSamples\":"+String(s.nearSamples)+",\"farSamples\":"+String(s.farSamples);
  j+=",\"lastStageIndex\":"+String(s.lastStageIndex);
  j+=",\"sonarAgeMs\":"+(s.sonarSequence?String(now-s.sonarMs):String("null"))+",\"sonarSequence\":"+String(s.sonarSequence);
  j+=",\"frontStopCm\":"+String(CampusNavTest::kFrontStopCm)+",\"enterCm\":"+String(CampusNavTest::kPillarEnterCm)+",\"exitCm\":"+String(CampusNavTest::kPillarExitCm);
  j+=",\"minSpacingM\":"+num(CampusNavTest::kMinPillarSpacingM);
  j+=",\"config\":{\"stopAfterInitial\":"+yes(CampusNavTest::kStopAfterInitialForward)+",\"stopAfterLeft\":"+yes(CampusNavTest::kStopAfterLeftTurn);
  j+=",\"stopAfterFifth\":"+yes(CampusNavTest::kStopAfterFifthPillar)+"}}}";
  return true;
}
