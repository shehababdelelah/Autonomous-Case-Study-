#include <Arduino.h>
#include <WiFi.h>
#include "BNO055Sensor.hpp"
#include "LidarScan.h"
#include "Route.hpp"
#include "RoverLink.hpp"
#include "Settings.hpp"
#include "WEBpage.hpp"
#include <Wire.h>
#include "LCDDisplay.hpp"

static constexpr uint32_t kRoutePeriodMs = 50;
static constexpr uint32_t kPagePeriodMs = 200;
static constexpr uint32_t kOledPeriodMs = 1000;
static constexpr uint32_t kMaxImuAgeMs = 200;
static constexpr uint32_t kMaxLidarAgeMs = 500;

LidarScan lidar;
RoverLink rover;
Route route(lidar);
WEBpage web;
LCDDisplay oled;
bool oledReady = false;
// Reads the IMU in the background, because its calibration wait can take a long time.
void imuTask(void* parameter) {
  if (!initBNO055(21, 22)) {
    Serial.println("BNO055 initialization failed");
    vTaskDelete(nullptr);
  }
  TickType_t lastWakeTime = xTaskGetTickCount();
  for (;;) {
    updateBNO055();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
  }
}

// Sends the wheel commands to the rover over Wi-Fi.
void roverLinkTask(void* parameter) {
  constexpr uint32_t kPeriodMs = 100;
  for (;;) {
    const uint32_t startMs = millis();
    rover.update();
    const uint32_t usedMs = millis() - startMs;
    vTaskDelay(pdMS_TO_TICKS(usedMs < kPeriodMs ? kPeriodMs - usedMs : 1));
  }
}

// True if a device answers at this I2C address.
static bool i2cPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}


static void startWiFi() {
  const IPAddress ip(192, 168, 18, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP("CAS-Box-08", "CASbox08", 11);
  Serial.print("Access point IP: ");
  Serial.println(WiFi.softAPIP());
}

// Line 1: rover name, line 2: web page address, line 3: rover battery.
static void updateOled() {
  char battery[21];
  oledReady = i2cPresent(0x3C) && oled.begin();
  if (!rover.connected()) {
    snprintf(battery, sizeof(battery), "Rover: searching");
  } else if (rover.voltage() > 0) {
    snprintf(battery, sizeof(battery), "Battery %.1f V", rover.voltage());
  } else {
    snprintf(battery, sizeof(battery), "Battery: waiting");
  }
  oled.show("Rover 8", WiFi.softAPIP().toString().c_str(), battery);
}

void setup() {
  Serial.begin(115200);
  startWiFi();   // first, because the rover only looks for the box while it boots
  web.begin();
  Wire.begin(21, 22);   // the IMU and the OLED share these pins
  oledReady = oled.begin();
  if (!oledReady) Serial.println("OLED not found");

  LidarScan::Config config;
  config.angleShiftDeg = kLidarAngleShift;
  lidar.begin(Serial1, config);

  rover.begin();
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(roverLinkTask, "roverLink", 6144, nullptr, 1, nullptr, 0);
}

static void publishStatus(const Route::Input& in) {
  WEBpage::Status s;
  s.running = route.running();
  s.blocked = route.blocked();
  s.roverOk = in.roverOk;
  s.imuOk = in.imuOk;
  s.lidarOk = in.lidarOk;
  s.stepNumber = route.step();
  snprintf(s.step, sizeof(s.step), "%s", Route::name(route.step()));
  snprintf(s.note, sizeof(s.note), "%s", route.note());
  s.left = route.left();
  s.right = route.right();
  s.pillars = route.pillars();
  s.walls = route.walls();
  s.panels = route.panels();
  s.distanceM = route.totalM();
  s.headingDeg = in.headingDeg;
  s.targetDeg = route.targetHeading();
  s.frontCm = lidar.distanceMm(0) / 10;
  s.rightCm = lidar.distanceMm(90) / 10;
  s.backCm = lidar.distanceMm(180) / 10;
  s.leftCm = lidar.distanceMm(270) / 10;
  s.voltage = rover.voltage();
  web.publish(s);
}

void loop() {
  static uint32_t lastRouteMs = 0, lastPageMs = 0;
  static uint32_t lastPackets = 0, lastPacketMs = 0;

  lidar.update();   // read every LiDAR byte that is waiting

  ImuData imu{};
  const bool hasImu = getBNO055Data(imu);
  const uint32_t nowMs = millis();

  if (lidar.packets() != lastPackets) {
    lastPackets = lidar.packets();
    lastPacketMs = nowMs;
  }

  if (nowMs - lastRouteMs >= kRoutePeriodMs) {
    lastRouteMs = nowMs;

    Route::Input in;
    in.nowMs = nowMs;
    in.headingDeg = imu.headingDeg;
    in.imuOk = hasImu && nowMs - imu.sampleMs <= kMaxImuAgeMs && isfinite(imu.headingDeg);
    in.roverOk = rover.connected();
    in.lidarOk = nowMs - lastPacketMs <= kMaxLidarAgeMs;

    const auto command = web.takeCommand();
    if (command == WEBpage::Command::Stop) route.stopNow("You pressed Stop");
    if (command == WEBpage::Command::Start && !route.running()) route.start(in);

    route.update(in);
    rover.setDrive(route.left(), route.right());

    if (nowMs - lastPageMs >= kPagePeriodMs) {
      lastPageMs = nowMs;
      publishStatus(in);
    }
  }
    static uint32_t lastOledMs = 0;
  if (oledReady && nowMs - lastOledMs >= kOledPeriodMs) {
    lastOledMs = nowMs;
    updateOled();
  }

  delay(1);
}
