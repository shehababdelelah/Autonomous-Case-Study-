#pragma once
#include <Arduino.h>
#include <math.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "BNO055Sensor.hpp"

class WEBpage {
 public:
  enum class Command : uint8_t { None, Start, Stop };
  struct Status {
    bool enabled = false, connected = false, imuOk = false;
    ImuData imu{};
    int frontCm = -1, rightCm = -1, leftCm = -1;
    uint32_t sonarMs = 0, sonarSequence = 0;
    char state[24] = "IDLE", phase[16] = "idle";
    char note[128] = "Waiting for a webpage Start command";
    uint8_t pillars = 0, nearSamples = 0, farSamples = 0, lastStageIndex = 0;
    bool pillarArmed = false;
    float estimatedM = 0, segmentM = 0;
    float targetHeadingDeg = NAN, headingErrorDeg = NAN;
    float leftCommand = 0, rightCommand = 0;
    uint32_t elapsedMs = 0;
  };
  bool begin();
  void publish(const Status& status);
  Command takeCommand();
 private:
  AsyncWebServer server_{80};
  SemaphoreHandle_t mutex_ = nullptr;
  Status status_{};
  uint32_t publishedMs_ = 0;
  bool seen_ = false, busy_ = false, startPending_ = false, stopPending_ = false;
  bool lock();
  void unlock();
  bool statusJson(String& json);
  void handleCommand(AsyncWebServerRequest* request);
};
