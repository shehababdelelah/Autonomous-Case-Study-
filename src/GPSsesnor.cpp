#include "GPSsesnor.hpp"


static const int kRxPin = 16;  // ESP32 RX2 <- GPS TX
static const int kTxPin = 17;  // ESP32 TX2 -> GPS RX
static const uint32_t kBaud = 9600;
static const uint32_t kMaxFixAgeMs = 3000;

void GPSsensor::begin() { Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin); }

void GPSsensor::poll() {
  while (Serial2.available()) _gps.encode(Serial2.read());
}

bool GPSsensor::hasFix() { return _gps.location.isValid() && _gps.location.age() < kMaxFixAgeMs; }
double GPSsensor::latDeg() { return _gps.location.lat(); }
double GPSsensor::lonDeg() { return _gps.location.lng(); }
int GPSsensor::satellites() {
  if (!_gps.satellites.isValid() ||
      _gps.satellites.age() >= kMaxFixAgeMs) {
    return 0;
  }

  return static_cast<int>(_gps.satellites.value());
}
float GPSsensor::hdop() {
  if (!_gps.hdop.isValid() ||
      _gps.hdop.age() >= kMaxFixAgeMs) {
    return -1.0f;
  }

  return static_cast<float>(_gps.hdop.hdop());
}
uint32_t GPSsensor::charsReceived() { return _gps.charsProcessed(); }

