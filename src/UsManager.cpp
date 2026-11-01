#include "UsManager.h"

// Pause between two sensors. This is what stops one sensor hearing the burst another just sent:
// in 5 ms sound travels only 1.7 m, so the previous burst was still crossing the room and the
// next sensor timed it as a near object - which is why readings came back short with nothing in
// front. 30 ms gives it about 10 m of travel to spread out and fade, and still leaves every
// sensor a cycle of roughly 5 x 30 ms = 150 ms, comfortably over the 60 ms the HC-SR04 datasheet
// asks for. If short readings still appear, raise this towards 60 ms before changing anything
// else - the cost is only the refresh rate.
static const uint32_t kPauseBetweenSensorsMs = 30;

// Readings below this are not real: the HC-SR04 cannot measure under about 2 cm, so anything
// shorter is the module timing its own transmit burst ringing in the horn.
static const int kMinPlausibleCm = 3;

// The module holds echo HIGH for up to ~38 ms when nothing answers at all. A trigger sent while
// it is still busy is ignored, and pulseIn() would then time the tail of the previous echo and
// report a short distance. So the line has to be idle before triggering.
static const uint32_t kEchoIdleTimeoutUs = 45000;

void UsManager::begin(int maxDistanceCm, uint32_t timeoutUs) {
  if (_task) return;  // already running
  _maxDistanceCm = maxDistanceCm;
  _timeoutUs = timeoutUs;
  // assigned one by one: chaining them reads back a volatile, which C++20 deprecates
  _frontCm = maxDistanceCm;
  _leftCm = maxDistanceCm;
  _rightCm = maxDistanceCm;
  _backCm = maxDistanceCm;
  _frontAngledCm = maxDistanceCm;

  const int pins[5][2] = {{_pins.trigFront, _pins.echoFront}, {_pins.trigLeft, _pins.echoLeft},
                          {_pins.trigRight, _pins.echoRight}, {_pins.trigBack, _pins.echoBack},
                          {_pins.trigFrontAngled, _pins.echoFrontAngled}};
  for (auto& p : pins) {
    pinMode(p[0], OUTPUT);
    digitalWrite(p[0], LOW);
    pinMode(p[1], INPUT);
  }
  // low priority on core 0; loop() runs on core 1
  xTaskCreatePinnedToCore(_taskEntry, "ultrasonic", 4096, this, 1, &_task, 0);
}

// 10 us trigger pulse, then time the echo pulse. 58 us of echo = 1 cm distance.
int UsManager::_measureCm(int trigPin, int echoPin) {
  // Wait out a previous echo still on the line, or we would measure its tail as a near object.
  const uint32_t idleStart = micros();
  while (digitalRead(echoPin) == HIGH) {
    if (micros() - idleStart > kEchoIdleTimeoutUs) return _maxDistanceCm;  // stuck: give up
    vTaskDelay(1);   // yields: RoverLink shares core 0 at the same priority
  }
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  const uint32_t us = pulseIn(echoPin, HIGH, _timeoutUs);  // blocks, but only in this task
  if (us == 0) return _maxDistanceCm;                        // no echo
  const int cm = us / 58;
  // Out of the module's usable band either way -> "nothing in range", the same as no echo.
  return (cm < kMinPlausibleCm || cm > _maxDistanceCm) ? _maxDistanceCm : cm;
}

void UsManager::_taskEntry(void* self) { static_cast<UsManager*>(self)->_taskLoop(); }

// Endless sweep over all five sensors, one at a time.
void UsManager::_taskLoop() {
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
