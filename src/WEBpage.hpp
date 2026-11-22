#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class WEBpage {
 public:
  enum class Command : uint8_t { None, Start, Stop, Restart };

  struct Status {
    bool running = false, blocked = false;
    bool logging = true;   // this run is saved as CSV
    bool roverOk = false, imuOk = false, lidarOk = false;
    int stepNumber = 0;
    int group = 0;   // 0 = the whole route, 1-4 = a test group
    bool newWay = false;   // this run uses the new way from the small pillar to wall K
    char step[32] = "Not started";
    char note[48] = "Press Start";
    float left = 0, right = 0, distanceM = 0, voltage = 0;
    int pillars = 0, walls = 0, panels = 0;
    float headingDeg = 0, targetDeg = 0;
    int frontCm = 0, rightCm = 0, backCm = 0, leftCm = 0;
    // GPS and the clock
    bool gpsOn = false, gpsFix = false;
    int satellites = 0, fixQuality = 0;
    double latDeg = 0, lonDeg = 0;
    char clock[24] = "no time yet";   // from the RTC once the GPS has set it
    int freeKb = 0;                   // free memory in the box, in kB
  };

  void begin();
  void publish(const Status& status);
  Command takeCommand();
  bool logWanted() const { return logWanted_; }   // the CSV choice sent with the last Start
  int groupWanted() const { return group_; }      // 0 = the whole route, 1-4 = a test group
  bool gpsWanted() const { return gpsOn_; }       // the GPS On / Off button
  bool newWayWanted() const { return newWay_; }   // the Start button that was pressed

 private:
 String statusJson();
 const char* missionStart();
 
  AsyncWebServer server_{80};
  SemaphoreHandle_t mutex_ = nullptr;
  Status status_{};
  volatile bool startPending_ = false, stopPending_ = false, restartPending_ = false;
  volatile bool logWanted_ = true;
  volatile int group_ = 0;
  volatile bool gpsOn_ = false;
  volatile bool newWay_ = false;
};
