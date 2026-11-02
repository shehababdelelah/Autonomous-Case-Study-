#pragma once
// WEBpage - Wi-Fi access point, web server and web page of the box.
//
// Page (phone): state line, timed drive pad, speed sliders,
// LiDAR top view + ultrasonic, IMU/GPS, and a "Dev" view with named values and UI task timing.
// The page polls /values every 250 ms; timed drive commands refresh /drive every 150 ms.
//
// Routes:  /                 the page
//          /values           JSON: state, dev values, UI task timing, sensor snapshot
//          /test/lidar/live   current Front/Right/Left sector ranges and logger health
//          /test/lidar/save   POST operator-selected direction/type to the CSV queue
//          /test/lidar/records JSON: recent saved CSV rows
//          /test/lidar.csv    download the LittleFS CSV file
//          /movement/drive/start start forward motion and IMU/LiDAR recording for 60 s
//          /movement/stop     stop the timed movement and recording
//          /movement/clear    clear movement IMU/LiDAR CSV files
//          /wall-follow/start start LiDAR left-wall following
//          /wall-follow/stop  stop LiDAR left-wall following
//          /wall-follow/heartbeat keep the webpage-controlled wall-follow mode active
//          /turn/right/start start a 3 s maximum-speed right-turn test
//          /turn/right/stop  stop the right-turn test
//          /imu.csv           download recorded IMU samples
//          /lidar.csv         download recorded LiDAR samples
//          /drive?l=..&r=..&id=..&n=..  wheel speeds from the drive pad; id = page instance,
//                            n = rising number (older commands that arrive late are ignored)
//
// Threads: the web server runs in its own task. Everything the page reads is guarded by a mutex,
// the drive setpoint by a spinlock, so application tasks and the server never see half-written data.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "LidarSensor.hpp"

class WEBpage {
public:
  static const uint8_t kMaxValues = 16;  // named values in the "Dev" view

  // Starts the access point (password min. 8 characters) and the web server.
  void begin(const char* ssid, const char* pass, IPAddress ip, uint8_t channel);

  void setTitle(const String& title);           // page title, set before the first page load
  void enableDrivePad(float maxSpeed);          // shows the drive pad, slider max = maxSpeed
  void setStateText(const String& text);        // state line at the top of the page
  void setSensorJson(const String& json);       // combined sensor snapshot built in main.cpp
  void recordSensorSnapshot(const SensorLogSnapshot& snapshot);
  bool startMovementDrive(float speed);
  void stopMovementDrive();
  bool getMovementDrive(float& speed);
  bool startWallFollow(float speed);
  void stopWallFollow(const char* reason = "Stopped by user");
  bool getWallFollow(float& speed);
  bool heartbeatWallFollow();
  bool startRightTurnTest();
  void stopRightTurnTest();
  bool getRightTurnTest(float& speed, uint32_t& remainingMs);
  void setWallFollowDiagnostics(uint16_t wallCm, uint16_t frontCm, float correction,
                                float leftSpeed, float rightSpeed, float speed,
                                const char* reason);
  void setLidarTestSnapshot(uint16_t frontCm, uint16_t rightCm,
                            uint16_t leftCm, float packetsPerSecond);
  void showValue(const char* name, double value);         // named value in the "Dev" view
  void showValue(const char* name, const String& value);

  // Latest drive command from the page; ageMs = time since the page last sent it.
  void getDrive(float& left, float& right, uint32_t& ageMs) const;

  void tick();  // call from uiTask (measures its period for the "Dev" view)

private:
  struct Value {
    char name[24];
    char text[32];
    bool inUse;
  };

  void _lock();  // takes _mtx (release with xSemaphoreGive)
  void _setValue(const char* name, const String& text);
  String _buildPage() const;
  String _buildJson();

  AsyncWebServer* _server = nullptr;
  String _title = "CAS Box";
  bool _drivePad = false;
  float _driveMax = 0.5f;

  SemaphoreHandle_t _mtx = nullptr;   // guards _state, _sensorJson, _values, _cycle*
  String _state = "Starting ...";
  String _sensorJson;
  uint16_t _lidarFrontCm = 0, _lidarRightCm = 0, _lidarLeftCm = 0;
  float _lidarPacketsPerSecond = 0;
  uint32_t _lidarSnapshotAtMs = 0;
  uint16_t _wallFollowWallCm = 0, _wallFollowFrontCm = 0;
  float _wallFollowCorrection = 0, _wallFollowLeftSpeed = 0, _wallFollowRightSpeed = 0;
  float _wallFollowReportedSpeed = 0;
  char _wallFollowReason[64] = "Stopped";
  LidarTestLogger _lidarLogger;
  Value _values[kMaxValues] = {};
  float _cycleMinMs = 0, _cycleMaxMs = 0, _cycleAvgMs = 0;

  mutable portMUX_TYPE _driveMux = portMUX_INITIALIZER_UNLOCKED;  // guards the drive set point
  float _driveL = 0, _driveR = 0;
  uint32_t _driveMs = 0;            // 0 = never received
  uint32_t _drivePageId = 0, _driveSeq = 0;  // newest accepted command (page id, number)

  mutable portMUX_TYPE _wallFollowMux = portMUX_INITIALIZER_UNLOCKED;
  bool _wallFollowActive = false;
  float _wallFollowSpeed = 0;
  uint32_t _wallFollowHeartbeatMs = 0;

  mutable portMUX_TYPE _rightTurnTestMux = portMUX_INITIALIZER_UNLOCKED;
  bool _rightTurnTestActive = false;
  uint32_t _rightTurnTestStartedMs = 0;
  char _rightTurnTestResult[32] = "Ready";

  // UI task timing statistics, uiTask only
  uint32_t _cycleLastUs = 0, _cycleWinMs = 0, _cycleMinUs = 0, _cycleMaxUs = 0, _cycleCount = 0;
  uint64_t _cycleSumUs = 0;
};
