#pragma once
#include <Arduino.h>
#include <WiFi.h>

class RoverLink {
public:
  // Starts the background link task. Call once from setup().
  // Initialize the link state. Call once before starting its task.
  void begin();

  // Perform one networking cycle. Called only by roverLinkTask().
  void update();

  // Sets left/right wheel speed: -0.5 to 0.5.
  // The background task ramps toward the requested speed.
  void setDrive(float left, float right);

  // Convenience movement commands. Speed should be 0 to 0.5.
  void forward(float speed);
  void backward(float speed);
  void turnLeft(float speed);
  void turnRight(float speed);
  void stop();

  bool connected() const { return _ipRaw != 0; }
  IPAddress ip() const { return IPAddress(_ipRaw); }
  float voltage() const { return _voltage; }
  float rttMs() const { return _rttMs; }
  uint32_t okCount() const { return _ok; }
  uint32_t failCount() const { return _fail; }

  String statusText() const;
  
private:
  uint32_t lastFindMs = 0;
uint32_t lastIdleMs = 0;
uint32_t lastRampMs = 0;

uint8_t stopsLeft = 0;
bool idleStatus = false;

float currentLeft = 0;
float currentRight = 0;

  bool _find();
  int _cmd(uint32_t ip, const String& json, String* reply);

  // Protects the requested wheel speeds and their update time.
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  float _setL = 0;
  float _setR = 0;
  uint32_t _setMs = 0;

  // Written by the link task; read by the application.
  volatile uint32_t _ipRaw = 0;
  volatile float _voltage = 0;
  volatile float _rttMs = 0;
  volatile uint32_t _ok = 0;
  volatile uint32_t _fail = 0;

  uint8_t _failRow = 0;  // Used only by the link task.
};