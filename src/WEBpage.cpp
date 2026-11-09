#include "WEBpage.hpp"
#include "NavigationPage.hpp"
#include "RunLog.hpp"

void WEBpage::begin() {
  mutex_ = xSemaphoreCreateMutex();

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(200, "text/html; charset=utf-8", kNavigationPage);
  });
  server_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* r) {
    r->send(200, "application/json", statusJson());
  });
  server_.on("/api/start", HTTP_POST, [this](AsyncWebServerRequest* r) {
    // /api/start?log=0 means: do not save this run. Without "log", it is saved as before.
    logWanted_ = !(r->hasParam("log") && r->getParam("log")->value() == "0");
    startPending_ = true;
    r->send(200, "application/json", "{\"ok\":true}");
  });
  server_.on("/api/stop", HTTP_POST, [this](AsyncWebServerRequest* r) {
    stopPending_ = true;
    r->send(200, "application/json", "{\"ok\":true}");
  });
  server_.on("/api/restart", HTTP_POST, [this](AsyncWebServerRequest* r) {
    restartPending_ = true;
    r->send(200, "application/json", "{\"ok\":true}");
  });
  server_.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest* r) { runLog.send(r); });
  server_.begin();
}

void WEBpage::publish(const Status& status) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_ = status;
  xSemaphoreGive(mutex_);
}

WEBpage::Command WEBpage::takeCommand() {
  if (restartPending_) {
    restartPending_ = false;
    stopPending_ = false;
    startPending_ = false;
    return Command::Restart;
  }
  if (stopPending_) {
    stopPending_ = false;
    startPending_ = false;
    return Command::Stop;
  }
  if (startPending_) {
    startPending_ = false;
    return Command::Start;
  }
  return Command::None;
}

String WEBpage::statusJson() {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  const Status s = status_;
  xSemaphoreGive(mutex_);

  char json[420];
  snprintf(json, sizeof(json),
           "{\"running\":%d,\"blocked\":%d,\"logging\":%d,\"roverOk\":%d,\"imuOk\":%d,\"lidarOk\":%d,"   // line 62
           "\"stepNumber\":%d,\"step\":\"%s\",\"note\":\"%s\",\"left\":%.2f,\"right\":%.2f,"
           "\"distanceM\":%.2f,\"voltage\":%.1f,\"pillars\":%d,\"walls\":%d,\"panels\":%d,"
           "\"headingDeg\":%.1f,\"targetDeg\":%.1f,\"frontCm\":%d,\"rightCm\":%d,"
           "\"backCm\":%d,\"leftCm\":%d}",
           s.running, s.blocked, s.logging, s.roverOk, s.imuOk, s.lidarOk, s.stepNumber, s.step, s.note,   // line 67
           s.left, s.right, s.distanceM, s.voltage, s.pillars, s.walls, s.panels,
           s.headingDeg, s.targetDeg, s.frontCm, s.rightCm, s.backCm, s.leftCm);
  return String(json);
}
