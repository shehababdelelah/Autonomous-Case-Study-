#pragma once
#include <Arduino.h>
#include <WiFi.h>

class RoverLink {
public:
  // Call once before starting the link task.
  void begin();

  // One networking cycle. Called only by roverLinkTask().
  void update();

  // Wheel speeds: -0.5 to 0.5. Call this all the time; the rover stops if you do not.
  void setDrive(float left, float right);
  void stop() { setDrive(0, 0); }

  bool connected() const { return _ipRaw != 0; }
  IPAddress ip() const { return IPAddress(_ipRaw); }
  float voltage() const { return _voltage; }

private:
  uint32_t lastFindMs = 0;
uint32_t lastIdleMs = 0;
uint32_t lastVoltsMs = 0;   // last time the voltage was asked for while driving

uint8_t stopsLeft = 0;
bool idleStatus = false;

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