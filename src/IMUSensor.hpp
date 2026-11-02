#pragma once

#include <Arduino.h>

// One BNO055 sample. Euler angles are the sensor's direct values in degrees:
// heading/yaw, roll, and pitch. Calibration values range from 0 to 3.
struct IMUReading {
  bool initialized = false;
  bool hasSample = false;
  uint32_t timestampMs = 0;
  float headingDeg = 0;
  float rollDeg = 0;
  float pitchDeg = 0;
  uint8_t systemCalibration = 0;
  uint8_t gyroCalibration = 0;
  uint8_t accelCalibration = 0;
  uint8_t magCalibration = 0;
};

class IMUSensor {
public:
  // Starts I2C on GPIO 21/22 and tries to initialize the BNO055.
  bool begin();

  // Read Euler angles and calibration, print them to Serial, and retry initialization if needed.
  void update();

  // Return one thread-safe copy of the latest sample.
  IMUReading read() const;

private:
  bool _startIfPresent();
  void _readEuler(float& headingDeg, float& rollDeg, float& pitchDeg);
  void _readCalibration(uint8_t& system, uint8_t& gyro,
                        uint8_t& accel, uint8_t& mag);

  mutable portMUX_TYPE _dataMux = portMUX_INITIALIZER_UNLOCKED;
  IMUReading _reading;
  uint32_t _lastSampleMs = 0;
  uint32_t _lastBeginTryMs = 0;
  bool _hasTriedBegin = false;
};
