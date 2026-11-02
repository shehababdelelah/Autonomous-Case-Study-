#include "IMUSensor.hpp"

#include <Wire.h>
#include <Adafruit_BNO055.h>

namespace {
constexpr uint8_t kI2cAddress = 0x28;
constexpr uint32_t kSamplePeriodMs = 100;
constexpr uint32_t kRetryPeriodMs = 5000;
constexpr int kSdaPin = 21;
constexpr int kSclPin = 22;

Adafruit_BNO055 bno(55, kI2cAddress, &Wire);
}

bool IMUSensor::begin() {
  Wire.begin(kSdaPin, kSclPin);
  return _startIfPresent();
}

bool IMUSensor::_startIfPresent() {
  const uint32_t now = millis();
  if (_hasTriedBegin && now - _lastBeginTryMs < kRetryPeriodMs) return false;
  _hasTriedBegin = true;
  _lastBeginTryMs = now;

  Wire.beginTransmission(kI2cAddress);
  if (Wire.endTransmission() != 0 || !bno.begin()) {
    Serial.println("[IMU] BNO055 not found at I2C address 0x28; will retry");
    return false;
  }

  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  portENTER_CRITICAL(&_dataMux);
  _reading.initialized = true;
  portEXIT_CRITICAL(&_dataMux);

  Serial.println("[IMU] BNO055 initialized in NDOF mode on I2C GPIO 21/22");
  return true;
}

void IMUSensor::update() {
  if (!read().initialized && !_startIfPresent()) return;

  const uint32_t now = millis();
  if (now - _lastSampleMs < kSamplePeriodMs) return;

  float headingDeg, rollDeg, pitchDeg;
  uint8_t systemCalibration, gyroCalibration, accelCalibration, magCalibration;
  _readEuler(headingDeg, rollDeg, pitchDeg);
  _readCalibration(systemCalibration, gyroCalibration, accelCalibration, magCalibration);

  const uint32_t timestampMs = millis();
  portENTER_CRITICAL(&_dataMux);
  _reading.hasSample = true;
  _reading.timestampMs = timestampMs;
  _reading.headingDeg = headingDeg;
  _reading.rollDeg = rollDeg;
  _reading.pitchDeg = pitchDeg;
  _reading.systemCalibration = systemCalibration;
  _reading.gyroCalibration = gyroCalibration;
  _reading.accelCalibration = accelCalibration;
  _reading.magCalibration = magCalibration;
  portEXIT_CRITICAL(&_dataMux);
  _lastSampleMs = timestampMs;

  Serial.printf("[IMU] t=%lu ms | heading/yaw=%.1f deg | roll=%.1f deg | pitch=%.1f deg"
                " | cal sys/gyro/accel/mag=%u/%u/%u/%u\n",
                (unsigned long)timestampMs, headingDeg, rollDeg, pitchDeg,
                systemCalibration, gyroCalibration, accelCalibration, magCalibration);
}

void IMUSensor::_readEuler(float& headingDeg, float& rollDeg, float& pitchDeg) {
  const imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  headingDeg = euler.x();
  rollDeg = euler.y();
  pitchDeg = euler.z();
}

void IMUSensor::_readCalibration(uint8_t& system, uint8_t& gyro,
                                 uint8_t& accel, uint8_t& mag) {
  bno.getCalibration(&system, &gyro, &accel, &mag);
}

IMUReading IMUSensor::read() const {
  portENTER_CRITICAL(&_dataMux);
  const IMUReading reading = _reading;
  portEXIT_CRITICAL(&_dataMux);
  return reading;
}
