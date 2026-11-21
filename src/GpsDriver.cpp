#include "GpsDriver.hpp"

namespace {
// UBX-CFG-SBAS (class 0x06, id 0x16): SBAS on, use it for ranging and corrections,
// up to 3 SBAS satellites, search all of them (in Europe that is EGNOS).
const uint8_t kCfgClass = 0x06;
const uint8_t kCfgSbasId = 0x16;
const uint8_t kCfgSbas[8] = {
    0x01,                     // mode: enabled
    0x03,                     // usage: ranging + differential corrections
    0x03,                     // max number of SBAS satellites
    0x00,                     // scanmode2
    0x00, 0x00, 0x00, 0x00};  // scanmode1: 0 = all SBAS satellites
const uint8_t kMaxTries = 3;
const uint32_t kAnswerTimeoutMs = 1000;
// Answers of the GPS: B5 62 05 01 (ACK) or 05 00 (NAK), length 2, then the class and id we sent.
const uint8_t kAck[8] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, kCfgClass, kCfgSbasId};
const uint8_t kNak[8] = {0xB5, 0x62, 0x05, 0x00, 0x02, 0x00, kCfgClass, kCfgSbasId};

uint8_t step(uint8_t pos, uint8_t c, const uint8_t* pattern) {
  if (c == pattern[pos]) return pos + 1;
  return c == pattern[0] ? 1 : 0;
}
}  // namespace

void GpsDriver::begin(bool enableEgnos) {
  Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
  if (!enableEgnos) {
    _egnos = EgnosState::Off;
    return;
  }
  _tries = 0;
  _sendEgnosCommand();
}

void GpsDriver::_sendEgnosCommand() {
  uint8_t ckA = 0, ckB = 0;
  const uint8_t head[4] = {kCfgClass, kCfgSbasId, sizeof(kCfgSbas), 0x00};
  Serial2.write(0xB5);
  Serial2.write(0x62);
  for (uint8_t b : head) { Serial2.write(b); ckA += b; ckB += ckA; }
  for (uint8_t b : kCfgSbas) { Serial2.write(b); ckA += b; ckB += ckA; }
  Serial2.write(ckA);
  Serial2.write(ckB);
  ++_tries;
  _sentMs = millis();
  _ackPos = _nakPos = 0;
  _egnos = EgnosState::Waiting;
}

void GpsDriver::_checkAck(uint8_t c) {
  _ackPos = step(_ackPos, c, kAck);
  _nakPos = step(_nakPos, c, kNak);
  if (_ackPos == sizeof(kAck)) {
    _egnos = EgnosState::On;
  } else if (_nakPos == sizeof(kNak)) {
    _egnos = EgnosState::Failed;   // the GPS refused the command
  }
}

void GpsDriver::poll() {
  while (Serial2.available() > 0) {
    const uint8_t c = Serial2.read();
    _gps.encode(c);
    if (_egnos == EgnosState::Waiting) _checkAck(c);
  }
  if (_egnos == EgnosState::Waiting && millis() - _sentMs > kAnswerTimeoutMs) {
    if (_tries < kMaxTries) _sendEgnosCommand();   // the GPS may still be starting: try again
    else _egnos = EgnosState::Failed;
  }
}

GpsData GpsDriver::data() {
  GpsData d{};
  d.fixAgeMs = _gps.location.isValid() ? _gps.location.age() : UINT32_MAX;
  d.fix = _gps.location.isValid() && d.fixAgeMs < kMaxFixAgeMs;
  if (d.fix) {
    d.latDeg = _gps.location.lat();
    d.lonDeg = _gps.location.lng();
  }
  if (_gps.satellites.isValid() && _gps.satellites.age() < kMaxFixAgeMs) d.satellites = _gps.satellites.value();
  if (_gps.hdop.isValid() && _gps.hdop.age() < kMaxFixAgeMs) d.hdop = _gps.hdop.hdop();
  if (_ggaQuality.isValid() && _ggaQuality.age() < kMaxFixAgeMs) d.fixQuality = (uint8_t)atoi(_ggaQuality.value());
  if (_gps.date.isValid()) d.utcDate = _gps.date.value();
  if (_gps.time.isValid()) d.utcTime = _gps.time.value();
  if (d.fix && _gps.altitude.isValid()) d.altitudeM = _gps.altitude.meters();
  if (d.fix && _gps.speed.isValid()) d.speedMps = _gps.speed.mps();
  if (d.fix && _gps.course.isValid()) d.courseDeg = _gps.course.deg();
  return d;
}

const char* GpsDriver::egnosText() const {
  switch (_egnos) {
    case EgnosState::On: return "ON";
    case EgnosState::Failed: return "FAILED";
    case EgnosState::Waiting: return "waiting";
    default: return "off";
  }
}

const char* GpsDriver::fixQualityText(uint8_t q) {
  return q == 2 ? "GPS + EGNOS" : q == 1 ? "GPS only" : "no fix";
}
