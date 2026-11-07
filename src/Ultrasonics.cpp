#include "Ultrasonics.hpp"

void Ultrasonics::begin(int maxDistanceCm, uint32_t timeoutUs) {
  _maxDistanceCm = maxDistanceCm;
  _timeoutUs = timeoutUs;

  for (int i = 0; i < 5; ++i) {
    pinMode(_trigPins[i], OUTPUT);
    digitalWrite(_trigPins[i], LOW);
    pinMode(_echoPins[i], INPUT);
    _distanceCm[i] = -1;
  }
}

int Ultrasonics::_measureCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Echo time is the round trip: approximately 58 microseconds per centimetre.
  const unsigned long durationUs =
    pulseIn(echoPin, HIGH, _timeoutUs);
    if (durationUs == 0) {
    return -1;
    }
    const int cm = durationUs / 58;
    if (cm <= 0) {
    return -1;
    }
    return cm > _maxDistanceCm ? _maxDistanceCm : cm;
}

void Ultrasonics::update() {
  for (int i = 0; i < 5; ++i) {
    _distanceCm[i] = _measureCm(_trigPins[i], _echoPins[i]);
    delay(5);  // Space the pings to reduce interference between sensors.
  }
}
