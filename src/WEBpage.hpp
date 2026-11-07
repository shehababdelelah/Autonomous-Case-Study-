#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class WEBpage {
 public:
  enum class Command : uint8_t { None, Start, Stop };

  struct Status {
    bool running = false, blocked = false;
    bool roverOk = false, imuOk = false, lidarOk = false;
    int stepNumber = 0;
    char step[32] = "Not started";
    char note[48] = "Press Start";
    float left = 0, right = 0, distanceM = 0, voltage = 0;
    int pillars = 0, walls = 0, panels = 0;
    float headingDeg = 0, targetDeg = 0;
    int frontCm = 0, rightCm = 0, backCm = 0, leftCm = 0;
  };

  void begin();
  void publish(const Status& status);
  Command takeCommand();

 private:
  String statusJson();

  AsyncWebServer server_{80};
  SemaphoreHandle_t mutex_ = nullptr;
  Status status_{};
  volatile bool startPending_ = false, stopPending_ = false;
};
