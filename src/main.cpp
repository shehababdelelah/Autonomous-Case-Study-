// =============================================================================================
//  CAS Box - starter program for the sensor box on the WAVE ROVER
//  Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15)
// =============================================================================================
//
//  WHAT IT DOES
//    The box ESP32 is the "brain" of your robot. It opens its own Wi-Fi, serves a web page for
//    your phone, reads all box sensors and drives the WAVE ROVER underneath it over Wi-Fi.
//
//      phone ──Wi-Fi──► box (this program): web page, sensors ──Wi-Fi──► rover (motors)
//
//    Web page: tap a direction to move for the selected duration; Stop cancels immediately.
//      Includes speed/duration sliders, LiDAR top view, ultrasonic distances, IMU
//      (roll/pitch/yaw), GPS. "Dev" (top right) shows raw values.
//
//  BEFORE YOU FLASH: set BOX_ID below to the number of your kit (sticker on box and rover).
//    Everything else follows from it - do not change the naming scheme:
//      Wi-Fi name  CAS-Box-<NN>      e.g. kit 7 -> CAS-Box-07
//      password    CASbox<NN>        e.g. kit 7 -> CASbox07
//      web page    http://192.168.<10+N>.1   e.g. kit 7 -> http://192.168.17.1
//      channel     odd kits 1, even kits 11
//    Your rover is already set up to join exactly this Wi-Fi. A wrong BOX_ID means: your rover
//    does not find your box, or your box takes over another team's network.
//
//  START ORDER: switch on the box first, then the rover (the rover looks for its box only
//    while it boots). Phone: join CAS-Box-<NN>, open the web page.
//
//  BUILD: open the project root in VS Code with PlatformIO; configuration/dependencies are in
//    platformio.ini. The serial monitor runs at 115200 baud.
//
//  FILES
//    main.cpp           settings, setup(), and FreeRTOS application tasks
//    RoverLink.*        box -> rover link (sends drive commands, reads battery voltage)
//    LidarSceneor.*     LiDAR reader, filtered scan, and CSV logger
//    IMUSensor.*        BNO055 orientation and calibration readings
//    Ultrasonics.*      five HC-SR04 sensors and their background task
//    RoverLink.*        Wi-Fi connection and commands to the rover
//    GPSsesnor.*        NEO-6M GPS module
//    LCDDisplay.*       small OLED
//    WEBpage.*          access point, web server, and page
//
//  SAFETY / BEHAVIOUR
//    Timed drive-pad motion stops at its selected duration; Stop, page loss of focus, or no
//    update from the phone for 0.4 s also stops it. RoverLink stops after 0.5 s without a
//    setpoint, and the rover itself stops after 3 s without a command. Do not remove these stops.
//    The rover firmware must not be changed.
// =============================================================================================

#include "WEBpage.hpp"
#include "RoverLink.hpp"
#include "LidarSensor.hpp"
#include "IMUSensor.hpp"
#include "Ultrasonics.hpp"
#include "GPSsesnor.hpp"
#include "LCDDisplay.hpp"
#include <math.h>
#include <Wire.h>
// ---- YOUR KIT ------------------------------------------------------------------------------
const int BOX_ID = 8;                     // 1..10 = kit number on the box

// ---- settings --------------------------------------------------------------------------------
constexpr float MAX_SPEED = 0.5f;         // drive pad maximum; 0.5 = full rover speed (100 % PWM)
const uint32_t PAD_TIMEOUT_MS = 400;      // no update from the phone for this long -> stop
constexpr uint16_t TARGET_WALL_CM = 130;
constexpr uint16_t MAX_WALL_DISTANCE_CM = 245;
constexpr uint16_t STOP_DISTANCE_CM = 30;
constexpr uint16_t WALL_DEADBAND_CM = 5;
constexpr float CORRECTION_PER_CM = 0.0025f;
constexpr float MAX_CORRECTION = 0.08f;
constexpr uint8_t WALL_FIRST_BIN = 63;
constexpr uint8_t WALL_BIN_COUNT = 10;
constexpr uint8_t FRONT_FIRST_BIN = 86;
constexpr uint8_t FRONT_BIN_COUNT = 8;
constexpr uint32_t LIDAR_MAX_AGE_MS = 500;
constexpr uint32_t WALL_DIAGNOSTIC_PERIOD_MS = 250;
const int LIDAR_OFFSET_DEG = 90;          // rotates the LiDAR view so that "front" is up
const uint32_t SENSOR_TASK_PERIOD_MS = 2;
const uint32_t UI_TASK_PERIOD_MS = 10;
const uint32_t DRIVE_TASK_PERIOD_MS = 20;

// ---- objects ---------------------------------------------------------------------------------
WEBpage hmi;           // web server + page
RoverLink rover;      // connection to the rover
LidarSceneor lidar;
IMUSensor imu;
Ultrasonics ultrasonics;
GPSsensor gps;
LCDDisplay disp;  // small OLED on the box
bool oledReady = false;
char boxSsid[16], boxPassword[16];

static uint16_t medianBins(const uint16_t* distances, uint8_t first, uint8_t count,
                           uint8_t minimumSamples) {
  uint16_t values[LidarSceneor::kPointCount];
  uint8_t valid = 0;
  for (uint8_t offset = 0; offset < count; ++offset) {
    const uint16_t distance = distances[(first + offset) % LidarSceneor::kPointCount];
    if (!distance) continue;
    uint8_t insertAt = valid;
    while (insertAt && values[insertAt - 1] > distance) {
      values[insertAt] = values[insertAt - 1];
      --insertAt;
    }
    values[insertAt] = distance;
    ++valid;
  }
  if (valid < minimumSamples) return 0;
  return (values[(valid - 1) / 2] + values[valid / 2]) / 2;
}

static uint16_t nearestBins(const uint16_t* distances, uint8_t first, uint8_t count,
                            uint8_t minimumSamples) {
  uint16_t nearest = UINT16_MAX;
  uint8_t valid = 0;
  for (uint8_t offset = 0; offset < count; ++offset) {
    const uint16_t distance = distances[(first + offset) % LidarSceneor::kPointCount];
    if (!distance) continue;
    if (distance < nearest) nearest = distance;
    ++valid;
  }
  return valid >= minimumSamples ? nearest : 0;
}

static String sensorJson() {
  const IMUReading reading = imu.read();
  String json;
  json.reserve(1000);
  json = "{\"us\":";
  json += ultrasonics.toJson();
  json += ",\"usMax\":"; json += 300;
  json += ",\"lidar\":"; json += lidar.toJson();
  json += ",\"imu\":{\"ok\":";
  json += reading.initialized ? "true" : "false";
  json += ",\"sample\":"; json += reading.hasSample ? "true" : "false";
  json += ",\"timestampMs\":";
  if (reading.hasSample) json += reading.timestampMs;
  else json += "null";
  json += ",\"ageMs\":";
  if (reading.hasSample) json += millis() - reading.timestampMs;
  else json += "null";
  json += ",\"heading\":"; json += String(reading.headingDeg, 1);
  json += ",\"roll\":"; json += String(reading.rollDeg, 1);
  json += ",\"pitch\":"; json += String(reading.pitchDeg, 1);
  json += ",\"cal\":[";
  json += reading.systemCalibration; json += ',';
  json += reading.gyroCalibration; json += ',';
  json += reading.accelCalibration; json += ',';
  json += reading.magCalibration;
  json += "]}";
  json += ",\"gps\":"; json += gps.toJson();
  json += '}';
  return json;
}

static bool i2cPresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// OLED shares the I2C bus with the IMU and is therefore updated by sensorTask only.
void updateOled() {
  static uint32_t lastTry = 0;
  if (!oledReady) {
    if (millis() - lastTry < 5000) return;
    lastTry = millis();
    if (!i2cPresent(0x3C)) return;
    oledReady = disp.begin();
  }
  String ip = WiFi.softAPIP().toString();
  String r = rover.connected() ? "Rover " + String(rover.voltage(), 1) + "V" : "Rover: search";
  disp.show(boxSsid, ip.c_str(), r.c_str());
}

// Sensor/UI data task (core 1).
// Polls each sensor module directly. Every 250 ms it
// publishes the sensor JSON for the web page. Every second it refreshes the OLED, sharing
// I2C access with the IMU in this task. Every 10 seconds it prints a sensor/heap status report.
// The 2 ms delay gives other FreeRTOS tasks time to run; individual sensor managers enforce
// their own slower sampling intervals.
static void sensorTask(void*) {
  uint32_t lastSnapshot = 0;
  uint32_t lastOled = 0;
  uint32_t lastReport = 0;

  for (;;) {
    lidar.update();
    gps.poll();
    imu.update();
    const uint32_t now = millis();

    if (now - lastSnapshot >= 250) {
      lastSnapshot = now;
      hmi.setSensorJson(sensorJson());
      SensorLogSnapshot logSnapshot;
      const IMUReading reading = imu.read();
      logSnapshot.timestampMs = now;
      logSnapshot.imuOk = reading.initialized && reading.hasSample &&
                          isfinite(reading.headingDeg) && isfinite(reading.rollDeg) &&
                          isfinite(reading.pitchDeg);
      logSnapshot.headingDeg = reading.headingDeg;
      logSnapshot.rollDeg = reading.rollDeg;
      logSnapshot.pitchDeg = reading.pitchDeg;
      logSnapshot.calibration[0] = reading.systemCalibration;
      logSnapshot.calibration[1] = reading.gyroCalibration;
      logSnapshot.calibration[2] = reading.accelCalibration;
      logSnapshot.calibration[3] = reading.magCalibration;
      logSnapshot.lidarOk = lidar.ok();
      logSnapshot.lidarPacketsPerSecond = lidar.packetsPerSecond();
      logSnapshot.lidarFrontCm = lidar.medianSector(0);
      logSnapshot.lidarRightCm = lidar.medianSector(22);
      logSnapshot.lidarLeftCm = lidar.medianSector(67);
      lidar.copyDistances(logSnapshot.lidarCm, kSensorLogLidarPoints);
      hmi.recordSensorSnapshot(logSnapshot);
      hmi.setLidarTestSnapshot(logSnapshot.lidarFrontCm, logSnapshot.lidarRightCm,
                               logSnapshot.lidarLeftCm, logSnapshot.lidarPacketsPerSecond);
    }
    if (now - lastOled >= 1000) {
      lastOled = now;
      updateOled();
    }
    if (now - lastReport >= 10000) {
      Serial.printf("[sys] sensor task, heap %lu kB, rover %s\n",
                    (unsigned long)(ESP.getFreeHeap() / 1024),
                    rover.connected() ? "ok" : "searching");
      Serial.printf("[sys] %s\n", sensorJson().c_str());
      lastReport = now;
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
  }
}

// Web status task (core 1).
// Calls hmi.tick() every 10 ms to measure this task's timing. Every 500 ms it updates the web
// page's rover status, connection statistics, battery voltage, and free-heap value. It does
// not handle HTTP requests itself: ESPAsyncWebServer runs those asynchronously.
static void uiTask(void*) {
  uint32_t lastUi = 0;
  TickType_t wakeTime = xTaskGetTickCount();

  for (;;) {
    hmi.tick();
    const uint32_t now = millis();
    if (now - lastUi >= 500) {
      lastUi = now;
      hmi.setStateText(rover.statusText());
      hmi.showValue("rover.ip", rover.ip().toString());
      hmi.showValue("rover.voltage_V", rover.voltage());
      hmi.showValue("rover.rtt_ms", rover.rttMs());
      hmi.showValue("rover.ok", (double)rover.okCount());
      hmi.showValue("rover.fail", (double)rover.failCount());
      hmi.showValue("sys.heap_kB", ESP.getFreeHeap() / 1024.0);
    }

    vTaskDelayUntil(&wakeTime, pdMS_TO_TICKS(UI_TASK_PERIOD_MS));
  }
}

// Motor-command owner (core 1). IMU readings do not affect motor commands.
static void driveTask(void*) {
  uint32_t lastWallDiagnostic = 0;
  for (;;) {
    float left = 0, right = 0;
    float wallSpeed = 0;
    bool safetyStopped = false;
    if (hmi.getWallFollow(wallSpeed)) {
      uint16_t distances[LidarSceneor::kPointCount] = {};
      lidar.copyDistances(distances, LidarSceneor::kPointCount);
      const uint16_t wallCm = medianBins(distances, WALL_FIRST_BIN, WALL_BIN_COUNT, 4);
      const uint16_t frontCm = nearestBins(distances, FRONT_FIRST_BIN, FRONT_BIN_COUNT, 2);
      const uint32_t scanAgeMs = lidar.scanAgeMs();
      const char* stopReason = nullptr;
      float correction = 0;

      if (!lidar.ok() || scanAgeMs > LIDAR_MAX_AGE_MS) {
        stopReason = "LiDAR unavailable or scan stale";
      } else if (!wallCm) {
        stopReason = "Left wall not detected";
      } else if (wallCm <= STOP_DISTANCE_CM) {
        stopReason = "Left wall inside stop distance";
      } else if (wallCm > MAX_WALL_DISTANCE_CM) {
        stopReason = "Left wall too far";
      } else if (!frontCm) {
        stopReason = "Insufficient front LiDAR returns";
      } else if (frontCm <= STOP_DISTANCE_CM) {
        stopReason = "Front obstacle inside stop distance";
      }

      if (stopReason) {
        hmi.stopWallFollow(stopReason);
        rover.setDrive(0, 0);
        hmi.setWallFollowDiagnostics(wallCm, frontCm, 0, 0, 0, wallSpeed, stopReason);
        safetyStopped = true;
      } else {
        const int16_t errorCm = (int16_t)TARGET_WALL_CM - (int16_t)wallCm;
        if (abs(errorCm) > WALL_DEADBAND_CM) {
          correction = constrain(errorCm * CORRECTION_PER_CM,
                                 -MAX_CORRECTION, MAX_CORRECTION);
        }
        left = constrain(wallSpeed + correction, 0.0f, MAX_SPEED);
        right = constrain(wallSpeed - correction, 0.0f, MAX_SPEED);
        const uint32_t now = millis();
        if (now - lastWallDiagnostic >= WALL_DIAGNOSTIC_PERIOD_MS) {
          lastWallDiagnostic = now;
          hmi.setWallFollowDiagnostics(wallCm, frontCm, correction, left, right,
                                       wallSpeed, "Following left wall");
        }
      }
    } else {
      float turnSpeed = 0;
      uint32_t turnRemainingMs = 0;
      if (hmi.getRightTurnTest(turnSpeed, turnRemainingMs)) {
        left = turnSpeed;
        right = -turnSpeed;
      } else {
        float movementSpeed = 0;
        if (hmi.getMovementDrive(movementSpeed)) {
          left = right = constrain(movementSpeed, 0.0f, MAX_SPEED);
        } else {
          uint32_t ageMs;
          hmi.getDrive(left, right, ageMs);
          if (ageMs > PAD_TIMEOUT_MS) left = right = 0;
        }
      }
    }
    if (!safetyStopped) rover.setDrive(left, right);

    vTaskDelay(pdMS_TO_TICKS(DRIVE_TASK_PERIOD_MS));
  }
}

static bool startApplicationTask(TaskFunction_t entry, const char* name,
                                 uint32_t stackBytes, UBaseType_t priority) {
  if (xTaskCreatePinnedToCore(entry, name, stackBytes, nullptr, priority, nullptr, 1) == pdPASS) {
    return true;
  }
  Serial.printf("[startup] failed to create %s task\n", name);
  return false;
}


void setup() {
  Serial.begin(115200);
  delay(200);
  // naming scheme, derived from BOX_ID only (see header)
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  uint8_t channel = (BOX_ID % 2) ? 1 : 11;

  hmi.setTitle(String("CAS Box ") + BOX_ID);
  hmi.enableDrivePad(MAX_SPEED);
  hmi.setStateText("Starting ...");
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);

  imu.begin();
  ultrasonics.begin(300, 17500);
  lidar.begin(LIDAR_OFFSET_DEG);
  gps.begin();
  const bool tasksStarted =
      // Sensor and UI tasks run on core 1; driveTask owns application-level motor commands.
      startApplicationTask(sensorTask, "sensor", 6144, 2) &&
      startApplicationTask(uiTask, "ui", 4096, 1) &&
      startApplicationTask(driveTask, "drive", 3072, 2);

  // Start the rover link even if task allocation failed: its default setpoint is stop.
  rover.begin(100);  // sends the current drive command every 100 ms
  if (!tasksStarted) Serial.println("[startup] application tasks incomplete; rover remains stopped");
}



void loop() {
  // Application work is performed by FreeRTOS tasks; Arduino's loop only yields.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
