#pragma once
#include <Arduino.h>

class Ultrasonics {
public:
  void begin(int maxDistanceCm, uint32_t timeoutUs);
  // Read all five sensors once. Call from loop() or your own task in main.cpp.
  // A scan can take about 113 ms with the default timeout and sensor spacing.
  void update();

  // Return the readings saved by update(); these getters do not trigger a measurement.
  int readFrontCm() const { return _distanceCm[0]; }
  int readLeftCm() const { return _distanceCm[1]; }
  int readRightCm() const { return _distanceCm[2]; }
  int readBackCm() const { return _distanceCm[3]; }
  int readFrontAngledCm() const { return _distanceCm[4]; }
  int maxDistanceCm() const { return _maxDistanceCm; }

private:
  int _measureCm(int trigPin, int echoPin);

  // Order: front, left, right, back, front angled.
  const int _trigPins[5] = {32, 4, 13, 5, 23};
  const int _echoPins[5] = {25, 18, 19, 14, 26};
  int _distanceCm[5] = {-1, -1, -1, -1, -1};
  int _maxDistanceCm = 300;
  uint32_t _timeoutUs = 17500;
};
