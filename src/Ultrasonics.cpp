#include "Ultrasonics.hpp"

static const uint32_t kPauseBetweenSensorsMs = 5;  // lets the previous echo die out

void Ultrasonics::begin(int maxDistanceCm, uint32_t timeoutUs) {
  if (_task) return;  // already running
  _maxDistanceCm = maxDistanceCm;
  _timeoutUs = timeoutUs;
  _frontCm = _leftCm = _rightCm = _backCm = _frontAngledCm = maxDistanceCm;

  const int pins[5][2] = {{_pins.trigFront, _pins.echoFront}, {_pins.trigLeft, _pins.echoLeft},
                          {_pins.trigRight, _pins.echoRight}, {_pins.trigBack, _pins.echoBack},
                          {_pins.trigFrontAngled, _pins.echoFrontAngled}};
  for (auto& p : pins) {
    pinMode(p[0], OUTPUT);
    digitalWrite(p[0], LOW);
    pinMode(p[1], INPUT);
  }
  // Low priority on core 0; the application tasks run on core 1.
  xTaskCreatePinnedToCore(_taskEntry, "ultrasonic", 4096, this, 1, &_task, 0);
}

// 10 us trigger pulse, then time the echo pulse. 58 us of echo = 1 cm distance.
int Ultrasonics::_measureCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  const uint32_t us = pulseIn(echoPin, HIGH, _timeoutUs);  // blocks, but only in this task
  if (us == 0) return _maxDistanceCm;                        // no echo
  const int cm = us / 58;
  return (cm <= 0 || cm > _maxDistanceCm) ? _maxDistanceCm : cm;
}

void Ultrasonics::_taskEntry(void* self) { static_cast<Ultrasonics*>(self)->_taskLoop(); }

// Endless sweep over all five sensors, one at a time.
void Ultrasonics::_taskLoop() {
  for (;;) {
    _frontCm = _measureCm(_pins.trigFront, _pins.echoFront);
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    _leftCm = _measureCm(_pins.trigLeft, _pins.echoLeft);
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    _rightCm = _measureCm(_pins.trigRight, _pins.echoRight);
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    _backCm = _measureCm(_pins.trigBack, _pins.echoBack);
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
    _frontAngledCm = _measureCm(_pins.trigFrontAngled, _pins.echoFrontAngled);
    vTaskDelay(pdMS_TO_TICKS(kPauseBetweenSensorsMs));
  }
}

String Ultrasonics::toJson() const {
  String json = "[";
  json += _frontCm; json += ',';
  json += _leftCm; json += ',';
  json += _rightCm; json += ',';
  json += _backCm; json += ',';
  json += _frontAngledCm;
  json += ']';
  return json;
}
