#include "WEBpage.hpp"
#include "NavigationPage.hpp"

void WEBpage::begin() {
  mutex_ = xSemaphoreCreateMutex();

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(200, "text/html; charset=utf-8", kNavigationPage);
  });
  server_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* r) {
    r->send(200, "application/json", statusJson());
  });
  server_.on("/api/start", HTTP_POST, [this](AsyncWebServerRequest* r) {
    startPending_ = true;
    r->send(200, "application/json", "{\"ok\":true}");
  });
  server_.on("/api/stop", HTTP_POST, [this](AsyncWebServerRequest* r) {
    stopPending_ = true;
    r->send(200, "application/json", "{\"ok\":true}");
  });
  server_.begin();
}

void WEBpage::publish(const Status& status) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  status_ = status;
  xSemaphoreGive(mutex_);
}

WEBpage::Command WEBpage::takeCommand() {
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
           "{\"running\":%d,\"blocked\":%d,\"roverOk\":%d,\"imuOk\":%d,\"lidarOk\":%d,"
           "\"stepNumber\":%d,\"step\":\"%s\",\"note\":\"%s\",\"left\":%.2f,\"right\":%.2f,"
           "\"distanceM\":%.2f,\"voltage\":%.1f,\"pillars\":%d,\"walls\":%d,\"panels\":%d,"
           "\"headingDeg\":%.1f,\"targetDeg\":%.1f,\"frontCm\":%d,\"rightCm\":%d,"
           "\"backCm\":%d,\"leftCm\":%d}",
           s.running, s.blocked, s.roverOk, s.imuOk, s.lidarOk, s.stepNumber, s.step, s.note,
           s.left, s.right, s.distanceM, s.voltage, s.pillars, s.walls, s.panels,
           s.headingDeg, s.targetDeg, s.frontCm, s.rightCm, s.backCm, s.leftCm);
  return String(json);
}
