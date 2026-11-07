#pragma once

#include <Arduino.h>


// One complete IMU reading. Other tasks read a copy of this.
struct ImuData {
  float headingDeg;   // 0 to 360, increases when the rover turns clockwise
  float rollDeg;
  float pitchDeg;
  float yawRateDps;   // degrees per second, positive = clockwise
  uint32_t sampleMs;  // millis() when this reading was taken
};

// Copies the newest reading. Returns false until the first reading exists.
bool getBNO055Data(ImuData& data);
// Initialize the BNO055 and restore or collect calibration data.
// The calibration process waits until the sensor is fully calibrated.
bool initBNO055(int sdaPin = 21, int sclPin = 22);
// Print orientation and calibration values when the sample interval is due.
// Call this repeatedly from loop().
void updateBNO055();
