// #pragma once

// #include <Arduino.h>
// #include <math.h>

// // First campus route test. No wheel encoders: distance is only an estimate
// // derived from calibrated commanded speed and elapsed time.
// // All constants below are TEST VALUES, not validated motor/gearbox limits.
// class CampusNavTest {
//  public:
//   struct Input {
//     float headingDeg;
//     float yawRateDps;
//     float pitchDeg;
//     float rollDeg;
//     int frontCm;            // -1 means no valid echo; not proof of free space
//     int rightCm;            // -1 means no valid echo
//     int leftCm;
//     uint32_t sonarSequence; // increments once per full ultrasonic update
//     uint32_t sonarTimeMs;
//     uint32_t nowMs;
//   };

//   struct Output {
//     float left = 0.0f;
//     float right = 0.0f;
//     bool hardStop = false;
//   };

//   enum class State : uint8_t {
//     START_FORWARD,
//     TURN_LEFT,
//     COUNT_PILLARS,
//     CLEAR_FIFTH,
//     TURN_RIGHT,
//     DONE,
//     FAULT
//   };

//   // IMPORTANT: change only after physical speed calibration and bench tests.
//   // Ratio = actual m/s when straight wheel command is +1.0 (extrapolated).
//   // E.g. if command 0.10 produces measured 0.08 m/s, set to 0.8.
//   static constexpr float kMetersPerSecondPerCommand = 1.0f; // UNCALIBRATED

//   // Tune conservatively to rover + gearbox. Units are RoverLink command/s.
//   static constexpr float kCruiseCommand = 0.12f;
//   static constexpr float kMaxTurnCommand = 0.085f;
//   static constexpr float kMaxCommandSlewPerSecond = 0.06f;
//   static constexpr float kEstimatedBrakeMps2 = 0.06f;
//   static constexpr float kInitialForwardM = 0.50f;
//   static constexpr float kAfterFifthPillarM = 0.80f; // measure chassis clearance!

//   // Enable stages only after testing the previous stage successfully.
//   // Default behavior (once enabled in main): move ~0.5 m, stop, do NOT turn.
//   static constexpr bool kStopAfterInitialForward = true;
//   static constexpr bool kStopAfterLeftTurn = true;
//   static constexpr bool kStopAfterFifthPillar = true;

//   // Right ultrasonic side-looking sensor thresholds; verify using logs.
//   static constexpr int kPillarEnterCm = 100;
//   static constexpr int kPillarExitCm = 145;
//   static constexpr float kMinPillarSpacingM = 2.0f; // false-count protection
//   static constexpr int kFrontStopCm = 35;

//   explicit CampusNavTest(float startHeadingDeg, uint32_t nowMs)
//     : state_(State::START_FORWARD), startHeading_(startHeadingDeg),
//       targetHeading_(startHeadingDeg), stateStartMs_(nowMs),
//       lastTickMs_(nowMs) {}

//   Output update(const Input& in) {
//     Output out;
//     const uint32_t elapsedMs = in.nowMs - lastTickMs_;
//     lastTickMs_ = in.nowMs;
//     const float dt = constrain(elapsedMs * 0.001f, 0.0f, 0.10f);

//     // Conservative fail-stop on known hazards, stale sonar task or bad IMU.
//     // A -1 front echo is UNKNOWN, not a verified clear path: see README.
//     if (!isfinite(in.headingDeg) || !isfinite(in.yawRateDps) ||
//         !isfinite(in.pitchDeg) || !isfinite(in.rollDeg) ||
//         fabsf(in.pitchDeg) > 18.0f || fabsf(in.rollDeg) > 18.0f ||
//         (in.nowMs - in.sonarTimeMs) > 1500 ||
//         (in.frontCm >= 0 && in.frontCm < kFrontStopCm)) {
//       fault();
//     }

//     if (state_ == State::FAULT) {
//       appliedLeft_ = appliedRight_ = 0.0f;
//       out.hardStop = true;
//       return out;
//     }

//     if (state_ == State::DONE) {
//       appliedLeft_ = appliedRight_ = 0.0f;
//       return out;
//     }

//     // Integrate the *previous applied wheel commands*; this is model-based
//     // odometry, not a measurement of actual movement. Turns give zero average.
//     if (dt > 0.0f) {
//       const float estimatedForwardCmd = 0.5f * (appliedLeft_ + appliedRight_);
//       if (estimatedForwardCmd > 0.0f) {
//         totalEstimatedM_ += estimatedForwardCmd *
//                             kMetersPerSecondPerCommand * dt;
//       }
//     }

//     // Track a pillar only on NEW sonar samples (the driver updates ~500 ms).
//     if (in.sonarSequence != lastSonarSequence_) {
//       lastSonarSequence_ = in.sonarSequence;
//       if (state_ == State::COUNT_PILLARS) observePillar(in.rightCm);
//     }

//     // Fail closed if any phase takes unexpectedly long.
//     const uint32_t inStateMs = in.nowMs - stateStartMs_;
//     if ((state_ == State::START_FORWARD && inStateMs > 20000) ||
//         (state_ == State::TURN_LEFT && inStateMs > 25000) ||
//         (state_ == State::COUNT_PILLARS && inStateMs > 300000) ||
//         (state_ == State::CLEAR_FIFTH && inStateMs > 20000) ||
//         (state_ == State::TURN_RIGHT && inStateMs > 25000)) {
//       fault();
//       out.hardStop = true;
//       appliedLeft_ = appliedRight_ = 0.0f;
//       return out;
//     }

//     float targetLeft = 0.0f;
//     float targetRight = 0.0f;
//     const float headingErr = wrap180(targetHeading_ - in.headingDeg);

//     switch (state_) {
//       case State::START_FORWARD: {
//         const float traveled = totalEstimatedM_ - segmentStartM_;
//         const float remaining = kInitialForwardM - traveled;
//         const float forward = forwardTarget(remaining);
//         holdHeading(forward, headingErr, in.yawRateDps,
//                     targetLeft, targetRight);
//         if (remaining <= 0.0f && nearlyStopped()) {
//           if (kStopAfterInitialForward) {
//             transition(State::DONE, in.nowMs);
//           } else {
//             targetHeading_ = wrap360(startHeading_ - 90.0f);
//             transition(State::TURN_LEFT, in.nowMs);
//           }
//         }
//         break;
//       }
//       case State::TURN_LEFT:
//         turnToward(headingErr, in.yawRateDps, targetLeft, targetRight);
//         if (fabsf(headingErr) < 2.5f &&
//             fabsf(in.yawRateDps) < 5.0f && nearlyStopped()) {
//           if (kStopAfterLeftTurn) {
//             transition(State::DONE, in.nowMs);
//           } else {
//             transition(State::COUNT_PILLARS, in.nowMs);
//             segmentStartM_ = totalEstimatedM_;
//             nearStreak_ = farStreak_ = 0;
//             pillarArmed_ = true;
//           }
//         }
//         break;
//       case State::COUNT_PILLARS:
//         holdHeading(kCruiseCommand, headingErr, in.yawRateDps,
//                     targetLeft, targetRight);
//         if (pillarCount_ >= 5) {
//           transition(State::CLEAR_FIFTH, in.nowMs);
//           segmentStartM_ = totalEstimatedM_;
//         }
//         break;
//       case State::CLEAR_FIFTH: {
//         const float traveled = totalEstimatedM_ - segmentStartM_;
//         const float remaining = kAfterFifthPillarM - traveled;
//         const float forward = forwardTarget(remaining);
//         holdHeading(forward, headingErr, in.yawRateDps,
//                     targetLeft, targetRight);
//         if (remaining <= 0.0f && nearlyStopped()) {
//           if (kStopAfterFifthPillar) {
//             transition(State::DONE, in.nowMs);
//           } else {
//             targetHeading_ = wrap360(startHeading_);
//             transition(State::TURN_RIGHT, in.nowMs);
//           }
//         }
//         break;
//       }
//       case State::TURN_RIGHT:
//         turnToward(headingErr, in.yawRateDps, targetLeft, targetRight);
//         if (fabsf(headingErr) < 2.5f &&
//             fabsf(in.yawRateDps) < 5.0f && nearlyStopped()) {
//           transition(State::DONE, in.nowMs);
//         }
//         break;
//       case State::DONE:
//       case State::FAULT:
//         break;
//     }

//     // Sole command smoother. Both wheels are independently rate-limited.
//     // A reversal must therefore go through zero rather than switching at once.
//     const float step = kMaxCommandSlewPerSecond * dt;
//     appliedLeft_ += constrain(targetLeft - appliedLeft_, -step, step);
//     appliedRight_ += constrain(targetRight - appliedRight_, -step, step);
//     out.left = appliedLeft_;
//     out.right = appliedRight_;
//     return out;
//   }

//   State state() const { return state_; }
//   uint8_t pillarCount() const { return pillarCount_; }
//   float estimatedMeters() const { return totalEstimatedM_; }
//   static const char* name(State s) {
//     switch (s) {
//       case State::START_FORWARD: return "START_FORWARD";
//       case State::TURN_LEFT: return "TURN_LEFT";
//       case State::COUNT_PILLARS: return "COUNT_PILLARS";
//       case State::CLEAR_FIFTH: return "CLEAR_FIFTH";
//       case State::TURN_RIGHT: return "TURN_RIGHT";
//       case State::DONE: return "DONE";
//       case State::FAULT: return "FAULT";
//     }
//     return "UNKNOWN";
//   }

//  private:
//   static float wrap180(float a) {
//     while (a > 180.0f) a -= 360.0f;
//     while (a <= -180.0f) a += 360.0f;
//     return a;
//   }
//   static float wrap360(float a) {
//     while (a >= 360.0f) a -= 360.0f;
//     while (a < 0.0f) a += 360.0f;
//     return a;
//   }
//   bool nearlyStopped() const {
//     return fabsf(appliedLeft_) < 0.004f &&
//            fabsf(appliedRight_) < 0.004f;
//   }
//   void transition(State next, uint32_t nowMs) {
//     state_ = next;
//     stateStartMs_ = nowMs;
//     Serial.printf("[NAV] %s; pillars=%u; estimated=%.2fm\n",
//                   name(next), static_cast<unsigned>(pillarCount_),
//                   totalEstimatedM_);
//   }
//   void fault() {
//     if (state_ != State::FAULT) Serial.println("[NAV] FAULT: emergency stop");
//     state_ = State::FAULT;
//   }
//   float forwardTarget(float remainingM) const {
//     if (remainingM <= 0.0f) return 0.0f;
//     const float brakingSpeedMps = sqrtf(2.0f * kEstimatedBrakeMps2 * remainingM);
//     const float command = brakingSpeedMps / kMetersPerSecondPerCommand;
//     return constrain(command, 0.0f, kCruiseCommand);
//   }
//   void holdHeading(float forward, float errorDeg, float yawRateDps,
//                    float& left, float& right) {
//     // Convention from user's original code: +heading error -> clockwise turn
//     // -> faster left wheel. Verify actual IMU/motor directions on stands.
//     float turn = constrain(0.003f * errorDeg - 0.00025f * yawRateDps,
//                            -0.025f, 0.025f);
//     if (forward <= 0.0f) turn = 0.0f; // stop fully before turning state
//     left = constrain(forward + turn, -0.18f, 0.18f);
//     right = constrain(forward - turn, -0.18f, 0.18f);
//   }
//   void turnToward(float errorDeg, float yawRateDps,
//                   float& left, float& right) {
//     float turning = constrain(0.003f * errorDeg - 0.0004f * yawRateDps,
//                               -kMaxTurnCommand, kMaxTurnCommand);
//     if (fabsf(errorDeg) < 2.5f && fabsf(yawRateDps) < 5.0f) turning = 0.0f;
//     left = turning;
//     right = -turning;
//   }
//   void observePillar(int rightCm) {
//     const bool near = rightCm >= 0 && rightCm <= kPillarEnterCm;
//     // No echo is treated as "not near" for re-arming only, never as a pillar.
//     const bool far = rightCm < 0 || rightCm >= kPillarExitCm;
//     if (near) {
//       nearStreak_ = (nearStreak_ < 2) ? nearStreak_ + 1 : 2;
//       farStreak_ = 0;
//     } else if (far) {
//       farStreak_ = (farStreak_ < 2) ? farStreak_ + 1 : 2;
//       nearStreak_ = 0;
//     } else {
//       nearStreak_ = farStreak_ = 0;
//     }
//     if (!pillarArmed_ && farStreak_ >= 2) pillarArmed_ = true;
//     if (pillarArmed_ && nearStreak_ >= 2 &&
//         (pillarCount_ == 0 ||
//          totalEstimatedM_ - lastPillarM_ >= kMinPillarSpacingM)) {
//       ++pillarCount_;
//       lastPillarM_ = totalEstimatedM_;
//       pillarArmed_ = false;
//       nearStreak_ = farStreak_ = 0;
//       Serial.printf("[NAV] Pillar %u detected, est %.2f m\n",
//                     static_cast<unsigned>(pillarCount_), totalEstimatedM_);
//     }
//   }

//   State state_;
//   float startHeading_ = 0;
//   float targetHeading_ = 0;
//   float appliedLeft_ = 0;
//   float appliedRight_ = 0;
//   float totalEstimatedM_ = 0;
//   float segmentStartM_ = 0;
//   float lastPillarM_ = 0;
//   uint32_t stateStartMs_ = 0;
//   uint32_t lastTickMs_ = 0;
//   uint32_t lastSonarSequence_ = 0;
//   uint8_t pillarCount_ = 0;
//   uint8_t nearStreak_ = 0;
//   uint8_t farStreak_ = 0;
//   bool pillarArmed_ = true;
// };

// #pragma once

// #include <Arduino.h>
// #include <math.h>

// // First campus route test. No wheel encoders: distance is only an estimate
// // derived from calibrated commanded speed and elapsed time.
// // All constants below are TEST VALUES, not validated motor/gearbox limits.
// class CampusNavTest {
//  public:
//   struct Input {
//     float headingDeg;
//     float yawRateDps;
//     float pitchDeg;
//     float rollDeg;
//     int frontCm;            // -1 means no valid echo; not proof of free space
//     int rightCm;            // -1 means no valid echo
//     int leftCm;
//     uint32_t sonarSequence; // increments once per full ultrasonic update
//     uint32_t sonarTimeMs;
//     uint32_t nowMs;
//   };

//   struct Output {
//     float left = 0.0f;
//     float right = 0.0f;
//     bool hardStop = false;
//   };

//   enum class State : uint8_t {
//     START_FORWARD,
//     TURN_LEFT,
//     COUNT_PILLARS,
//     CLEAR_FIFTH,
//     TURN_RIGHT,
//     DONE,
//     FAULT
//   };

//   // IMPORTANT: change only after physical speed calibration and bench tests.
//   // Ratio = actual m/s when straight wheel command is +1.0 (extrapolated).
//   // E.g. if command 0.10 produces measured 0.08 m/s, set to 0.8.
//   static constexpr float kMetersPerSecondPerCommand = 1.0f; // UNCALIBRATED

//   // Tune conservatively to rover + gearbox. Units are RoverLink command/s.
//   static constexpr float kCruiseCommand = 0.12f;
//   static constexpr float kMaxTurnCommand = 0.085f;
//   static constexpr float kMaxCommandSlewPerSecond = 0.06f;
//   static constexpr float kEstimatedBrakeMps2 = 0.06f;
//   static constexpr float kInitialForwardM = 0.50f;
//   static constexpr float kAfterFifthPillarM = 0.80f; // measure chassis clearance!

//   // Enable stages only after testing the previous stage successfully.
//   // Default behavior (once enabled in main): move ~0.5 m, stop, do NOT turn.
//   static constexpr bool kStopAfterInitialForward = true;
//   static constexpr bool kStopAfterLeftTurn = true;
//   static constexpr bool kStopAfterFifthPillar = true;

//   // Right ultrasonic side-looking sensor thresholds; verify using logs.
//   static constexpr int kPillarEnterCm = 100;
//   static constexpr int kPillarExitCm = 145;
//   static constexpr float kMinPillarSpacingM = 2.0f; // false-count protection
//   static constexpr int kFrontStopCm = 35;

//   explicit CampusNavTest(float startHeadingDeg, uint32_t nowMs)
//     : state_(State::START_FORWARD), startHeading_(startHeadingDeg),
//       targetHeading_(startHeadingDeg), stateStartMs_(nowMs),
//       lastTickMs_(nowMs) {}

//   Output update(const Input& in) {
//     Output out;
//     const uint32_t elapsedMs = in.nowMs - lastTickMs_;
//     lastTickMs_ = in.nowMs;
//     const float dt = constrain(elapsedMs * 0.001f, 0.0f, 0.10f);

//     // Conservative fail-stop on known hazards, stale sonar task or bad IMU.
//     // A -1 front echo is UNKNOWN, not a verified clear path: see README.
//     if (!isfinite(in.headingDeg) || !isfinite(in.yawRateDps) ||
//         !isfinite(in.pitchDeg) || !isfinite(in.rollDeg) ||
//         fabsf(in.pitchDeg) > 18.0f || fabsf(in.rollDeg) > 18.0f ||
//         (in.nowMs - in.sonarTimeMs) > 1500 ||
//         (in.frontCm >= 0 && in.frontCm < kFrontStopCm)) {
//       fault();
//     }

//     if (state_ == State::FAULT) {
//       appliedLeft_ = appliedRight_ = 0.0f;
//       out.hardStop = true;
//       return out;
//     }

//     if (state_ == State::DONE) {
//       appliedLeft_ = appliedRight_ = 0.0f;
//       return out;
//     }

//     // Integrate the *previous applied wheel commands*; this is model-based
//     // odometry, not a measurement of actual movement. Turns give zero average.
//     if (dt > 0.0f) {
//       const float estimatedForwardCmd = 0.5f * (appliedLeft_ + appliedRight_);
//       if (estimatedForwardCmd > 0.0f) {
//         totalEstimatedM_ += estimatedForwardCmd *
//                             kMetersPerSecondPerCommand * dt;
//       }
//     }

//     // Track a pillar only on NEW sonar samples (the driver updates ~500 ms).
//     if (in.sonarSequence != lastSonarSequence_) {
//       lastSonarSequence_ = in.sonarSequence;
//       if (state_ == State::COUNT_PILLARS) observePillar(in.rightCm);
//     }

//     // Fail closed if any phase takes unexpectedly long.
//     const uint32_t inStateMs = in.nowMs - stateStartMs_;
//     if ((state_ == State::START_FORWARD && inStateMs > 20000) ||
//         (state_ == State::TURN_LEFT && inStateMs > 25000) ||
//         (state_ == State::COUNT_PILLARS && inStateMs > 300000) ||
//         (state_ == State::CLEAR_FIFTH && inStateMs > 20000) ||
//         (state_ == State::TURN_RIGHT && inStateMs > 25000)) {
//       fault();
//       out.hardStop = true;
//       appliedLeft_ = appliedRight_ = 0.0f;
//       return out;
//     }

//     float targetLeft = 0.0f;
//     float targetRight = 0.0f;
//     const float headingErr = wrap180(targetHeading_ - in.headingDeg);

//     switch (state_) {
//       case State::START_FORWARD: {
//         const float traveled = totalEstimatedM_ - segmentStartM_;
//         const float remaining = kInitialForwardM - traveled;
//         const float forward = forwardTarget(remaining);
//         holdHeading(forward, headingErr, in.yawRateDps,
//                     targetLeft, targetRight);
//         if (remaining <= 0.0f && nearlyStopped()) {
//           if (kStopAfterInitialForward) {
//             transition(State::DONE, in.nowMs);
//           } else {
//             targetHeading_ = wrap360(startHeading_ - 90.0f);
//             transition(State::TURN_LEFT, in.nowMs);
//           }
//         }
//         break;
//       }
//       case State::TURN_LEFT:
//         turnToward(headingErr, in.yawRateDps, targetLeft, targetRight);
//         if (fabsf(headingErr) < 2.5f &&
//             fabsf(in.yawRateDps) < 5.0f && nearlyStopped()) {
//           if (kStopAfterLeftTurn) {
//             transition(State::DONE, in.nowMs);
//           } else {
//             transition(State::COUNT_PILLARS, in.nowMs);
//             segmentStartM_ = totalEstimatedM_;
//             nearStreak_ = farStreak_ = 0;
//             pillarArmed_ = true;
//           }
//         }
//         break;
//       case State::COUNT_PILLARS:
//         holdHeading(kCruiseCommand, headingErr, in.yawRateDps,
//                     targetLeft, targetRight);
//         if (pillarCount_ >= 5) {
//           transition(State::CLEAR_FIFTH, in.nowMs);
//           segmentStartM_ = totalEstimatedM_;
//         }
//         break;
//       case State::CLEAR_FIFTH: {
//         const float traveled = totalEstimatedM_ - segmentStartM_;
//         const float remaining = kAfterFifthPillarM - traveled;
//         const float forward = forwardTarget(remaining);
//         holdHeading(forward, headingErr, in.yawRateDps,
//                     targetLeft, targetRight);
//         if (remaining <= 0.0f && nearlyStopped()) {
//           if (kStopAfterFifthPillar) {
//             transition(State::DONE, in.nowMs);
//           } else {
//             targetHeading_ = wrap360(startHeading_);
//             transition(State::TURN_RIGHT, in.nowMs);
//           }
//         }
//         break;
//       }
//       case State::TURN_RIGHT:
//         turnToward(headingErr, in.yawRateDps, targetLeft, targetRight);
//         if (fabsf(headingErr) < 2.5f &&
//             fabsf(in.yawRateDps) < 5.0f && nearlyStopped()) {
//           transition(State::DONE, in.nowMs);
//         }
//         break;
//       case State::DONE:
//       case State::FAULT:
//         break;
//     }

//     // Sole command smoother. Both wheels are independently rate-limited.
//     // A reversal must therefore go through zero rather than switching at once.
//     const float step = kMaxCommandSlewPerSecond * dt;
//     appliedLeft_ += constrain(targetLeft - appliedLeft_, -step, step);
//     appliedRight_ += constrain(targetRight - appliedRight_, -step, step);
//     out.left = appliedLeft_;
//     out.right = appliedRight_;
//     return out;
//   }

//   // Read-only diagnostics for navigation webpage.
//   float targetHeadingDeg() const { return targetHeading_; }
//   float segmentMeters() const { return totalEstimatedM_ - segmentStartM_; }
//   bool pillarArmed() const { return pillarArmed_; }
//   uint8_t nearSamples() const { return nearStreak_; }
//   uint8_t farSamples() const { return farStreak_; }
//   State state() const { return state_; }
//   uint8_t pillarCount() const { return pillarCount_; }
//   float estimatedMeters() const { return totalEstimatedM_; }
//   static const char* name(State s) {
//     switch (s) {
//       case State::START_FORWARD: return "START_FORWARD";
//       case State::TURN_LEFT: return "TURN_LEFT";
//       case State::COUNT_PILLARS: return "COUNT_PILLARS";
//       case State::CLEAR_FIFTH: return "CLEAR_FIFTH";
//       case State::TURN_RIGHT: return "TURN_RIGHT";
//       case State::DONE: return "DONE";
//       case State::FAULT: return "FAULT";
//     }
//     return "UNKNOWN";
//   }

//  private:
//   static float wrap180(float a) {
//     while (a > 180.0f) a -= 360.0f;
//     while (a <= -180.0f) a += 360.0f;
//     return a;
//   }
//   static float wrap360(float a) {
//     while (a >= 360.0f) a -= 360.0f;
//     while (a < 0.0f) a += 360.0f;
//     return a;
//   }
//   bool nearlyStopped() const {
//     return fabsf(appliedLeft_) < 0.004f &&
//            fabsf(appliedRight_) < 0.004f;
//   }
//   void transition(State next, uint32_t nowMs) {
//     state_ = next;
//     stateStartMs_ = nowMs;
//     Serial.printf("[NAV] %s; pillars=%u; estimated=%.2fm\n",
//                   name(next), static_cast<unsigned>(pillarCount_),
//                   totalEstimatedM_);
//   }
//   void fault() {
//     if (state_ != State::FAULT) Serial.println("[NAV] FAULT: emergency stop");
//     state_ = State::FAULT;
//   }
//   float forwardTarget(float remainingM) const {
//     if (remainingM <= 0.0f) return 0.0f;
//     const float brakingSpeedMps = sqrtf(2.0f * kEstimatedBrakeMps2 * remainingM);
//     const float command = brakingSpeedMps / kMetersPerSecondPerCommand;
//     return constrain(command, 0.0f, kCruiseCommand);
//   }
//   void holdHeading(float forward, float errorDeg, float yawRateDps,
//                    float& left, float& right) {
//     // Convention from user's original code: +heading error -> clockwise turn
//     // -> faster left wheel. Verify actual IMU/motor directions on stands.
//     float turn = constrain(0.003f * errorDeg - 0.00025f * yawRateDps,
//                            -0.025f, 0.025f);
//     if (forward <= 0.0f) turn = 0.0f; // stop fully before turning state
//     left = constrain(forward + turn, -0.18f, 0.18f);
//     right = constrain(forward - turn, -0.18f, 0.18f);
//   }
//   void turnToward(float errorDeg, float yawRateDps,
//                   float& left, float& right) {
//     float turning = constrain(0.003f * errorDeg - 0.0004f * yawRateDps,
//                               -kMaxTurnCommand, kMaxTurnCommand);
//     if (fabsf(errorDeg) < 2.5f && fabsf(yawRateDps) < 5.0f) turning = 0.0f;
//     left = turning;
//     right = -turning;
//   }
//   void observePillar(int rightCm) {
//     const bool near = rightCm >= 0 && rightCm <= kPillarEnterCm;
//     // No echo is treated as "not near" for re-arming only, never as a pillar.
//     const bool far = rightCm < 0 || rightCm >= kPillarExitCm;
//     if (near) {
//       nearStreak_ = (nearStreak_ < 2) ? nearStreak_ + 1 : 2;
//       farStreak_ = 0;
//     } else if (far) {
//       farStreak_ = (farStreak_ < 2) ? farStreak_ + 1 : 2;
//       nearStreak_ = 0;
//     } else {
//       nearStreak_ = farStreak_ = 0;
//     }
//     if (!pillarArmed_ && farStreak_ >= 2) pillarArmed_ = true;
//     if (pillarArmed_ && nearStreak_ >= 2 &&
//         (pillarCount_ == 0 ||
//          totalEstimatedM_ - lastPillarM_ >= kMinPillarSpacingM)) {
//       ++pillarCount_;
//       lastPillarM_ = totalEstimatedM_;
//       pillarArmed_ = false;
//       nearStreak_ = farStreak_ = 0;
//       Serial.printf("[NAV] Pillar %u detected, est %.2f m\n",
//                     static_cast<unsigned>(pillarCount_), totalEstimatedM_);
//     }
//   }

//   State state_;
//   float startHeading_ = 0;
//   float targetHeading_ = 0;
//   float appliedLeft_ = 0;
//   float appliedRight_ = 0;
//   float totalEstimatedM_ = 0;
//   float segmentStartM_ = 0;
//   float lastPillarM_ = 0;
//   uint32_t stateStartMs_ = 0;
//   uint32_t lastTickMs_ = 0;
//   uint32_t lastSonarSequence_ = 0;
//   uint8_t pillarCount_ = 0;
//   uint8_t nearStreak_ = 0;
//   uint8_t farStreak_ = 0;
//   bool pillarArmed_ = true;
// };



#pragma once

#include <Arduino.h>
#include <math.h>

// First campus route test. No wheel encoders: distance is only an estimate
// derived from calibrated commanded speed and elapsed time.
// All constants below are TEST VALUES, not validated motor/gearbox limits.
class CampusNavTest {
 public:
  struct Input {
    float headingDeg;
    float yawRateDps;
    float pitchDeg;
    float rollDeg;
    int frontCm;            // -1 means no valid echo; not proof of free space
    int rightCm;            // -1 means no valid echo
    int leftCm;
    uint32_t sonarSequence; // increments once per full ultrasonic update
    uint32_t sonarTimeMs;
    uint32_t nowMs;
  };

  struct Output {
    float left = 0.0f;
    float right = 0.0f;
    bool hardStop = false;
  };

  enum class State : uint8_t {
    START_FORWARD,
    TURN_LEFT,
    COUNT_PILLARS,
    CLEAR_FIFTH,
    TURN_RIGHT,
    DONE,
    FAULT
  };

  // IMPORTANT: change only after physical speed calibration and bench tests.
  // Ratio = actual m/s when straight wheel command is +1.0 (extrapolated).
  // E.g. if command 0.10 produces measured 0.08 m/s, set to 0.8.
  static constexpr float kMetersPerSecondPerCommand = 1.0f; // UNCALIBRATED

  // Tune conservatively to rover + gearbox. Units are RoverLink command/s.
  static constexpr float kCruiseCommand = 0.30f;
  static constexpr float kMaxTurnCommand = 0.085f;
  static constexpr float kMaxCommandSlewPerSecond = 0.20f;
  static constexpr float kEstimatedBrakeMps2 = 0.50f;
  static constexpr float kInitialForwardM = 0.50f;
  static constexpr float kAfterFifthPillarM = 0.80f; // measure chassis clearance!

  // Enable stages only after testing the previous stage successfully.
  // Default behavior (once enabled in main): move ~0.5 m, stop, do NOT turn.
  static constexpr bool kStopAfterInitialForward = true;
  static constexpr bool kStopAfterLeftTurn = true;
  static constexpr bool kStopAfterFifthPillar = true;

  // Right ultrasonic side-looking sensor thresholds; verify using logs.
  static constexpr int kPillarEnterCm = 100;
  static constexpr int kPillarExitCm = 145;
  static constexpr float kMinPillarSpacingM = 2.0f; // false-count protection
  static constexpr int kFrontStopCm = 35;

  explicit CampusNavTest(float startHeadingDeg, uint32_t nowMs)
    : state_(State::START_FORWARD), startHeading_(startHeadingDeg),
      targetHeading_(startHeadingDeg), stateStartMs_(nowMs),
      lastTickMs_(nowMs) {}

  Output update(const Input& in) {
    Output out;
    const uint32_t elapsedMs = in.nowMs - lastTickMs_;
    lastTickMs_ = in.nowMs;
    const float dt = constrain(elapsedMs * 0.001f, 0.0f, 0.10f);

    // Conservative fail-stop on known hazards, stale sonar task or bad IMU.
    // A -1 front echo is UNKNOWN, not a verified clear path: see README.
    if (!isfinite(in.headingDeg) || !isfinite(in.yawRateDps) ||
        !isfinite(in.pitchDeg) || !isfinite(in.rollDeg) ||
        fabsf(in.pitchDeg) > 18.0f || fabsf(in.rollDeg) > 18.0f ||
        (in.nowMs - in.sonarTimeMs) > 1500 ||
        (in.frontCm >= 0 && in.frontCm < kFrontStopCm)) {
      fault();
    }

    if (state_ == State::FAULT) {
      appliedLeft_ = appliedRight_ = 0.0f;
      out.hardStop = true;
      return out;
    }

    if (state_ == State::DONE) {
      appliedLeft_ = appliedRight_ = 0.0f;
      return out;
    }

    // Integrate the *previous applied wheel commands*; this is model-based
    // odometry, not a measurement of actual movement. Turns give zero average.
    if (dt > 0.0f) {
      const float estimatedForwardCmd = 0.5f * (appliedLeft_ + appliedRight_);
      if (estimatedForwardCmd > 0.0f) {
        totalEstimatedM_ += estimatedForwardCmd *
                            kMetersPerSecondPerCommand * dt;
      }
    }

    // Track a pillar only on NEW sonar samples (the driver updates ~500 ms).
    if (in.sonarSequence != lastSonarSequence_) {
      lastSonarSequence_ = in.sonarSequence;
      if (state_ == State::COUNT_PILLARS) observePillar(in.rightCm);
    }

    // Fail closed if any phase takes unexpectedly long.
    const uint32_t inStateMs = in.nowMs - stateStartMs_;
    if ((state_ == State::START_FORWARD && inStateMs > 20000) ||
        (state_ == State::TURN_LEFT && inStateMs > 25000) ||
        (state_ == State::COUNT_PILLARS && inStateMs > 300000) ||
        (state_ == State::CLEAR_FIFTH && inStateMs > 20000) ||
        (state_ == State::TURN_RIGHT && inStateMs > 25000)) {
      fault();
      out.hardStop = true;
      appliedLeft_ = appliedRight_ = 0.0f;
      return out;
    }

    float targetLeft = 0.0f;
    float targetRight = 0.0f;
    const float headingErr = wrap180(targetHeading_ - in.headingDeg);

    switch (state_) {
      case State::START_FORWARD: {
        const float traveled = totalEstimatedM_ - segmentStartM_;
        const float remaining = kInitialForwardM - traveled;
        const float forward = forwardTarget(remaining);
        holdHeading(forward, headingErr, in.yawRateDps,
                    targetLeft, targetRight);
        if (remaining <= 0.0f && nearlyStopped()) {
          if (kStopAfterInitialForward) {
            transition(State::DONE, in.nowMs);
          } else {
            targetHeading_ = wrap360(startHeading_ - 90.0f);
            transition(State::TURN_LEFT, in.nowMs);
          }
        }
        break;
      }
      case State::TURN_LEFT:
        turnToward(headingErr, in.yawRateDps, targetLeft, targetRight);
        if (fabsf(headingErr) < 2.5f &&
            fabsf(in.yawRateDps) < 5.0f && nearlyStopped()) {
          if (kStopAfterLeftTurn) {
            transition(State::DONE, in.nowMs);
          } else {
            transition(State::COUNT_PILLARS, in.nowMs);
            segmentStartM_ = totalEstimatedM_;
            nearStreak_ = farStreak_ = 0;
            pillarArmed_ = true;
          }
        }
        break;
      case State::COUNT_PILLARS:
        holdHeading(kCruiseCommand, headingErr, in.yawRateDps,
                    targetLeft, targetRight);
        if (pillarCount_ >= 5) {
          transition(State::CLEAR_FIFTH, in.nowMs);
          segmentStartM_ = totalEstimatedM_;
        }
        break;
      case State::CLEAR_FIFTH: {
        const float traveled = totalEstimatedM_ - segmentStartM_;
        const float remaining = kAfterFifthPillarM - traveled;
        const float forward = forwardTarget(remaining);
        holdHeading(forward, headingErr, in.yawRateDps,
                    targetLeft, targetRight);
        if (remaining <= 0.0f && nearlyStopped()) {
          if (kStopAfterFifthPillar) {
            transition(State::DONE, in.nowMs);
          } else {
            targetHeading_ = wrap360(startHeading_);
            transition(State::TURN_RIGHT, in.nowMs);
          }
        }
        break;
      }
      case State::TURN_RIGHT:
        turnToward(headingErr, in.yawRateDps, targetLeft, targetRight);
        if (fabsf(headingErr) < 2.5f &&
            fabsf(in.yawRateDps) < 5.0f && nearlyStopped()) {
          transition(State::DONE, in.nowMs);
        }
        break;
      case State::DONE:
      case State::FAULT:
        break;
    }

    // Sole command smoother. Both wheels are independently rate-limited.
    // A reversal must therefore go through zero rather than switching at once.
    const float step = kMaxCommandSlewPerSecond * dt;
    appliedLeft_ += constrain(targetLeft - appliedLeft_, -step, step);
    appliedRight_ += constrain(targetRight - appliedRight_, -step, step);
    out.left = appliedLeft_;
    out.right = appliedRight_;
    return out;
  }

  // Read-only diagnostics for navigation webpage.
  float targetHeadingDeg() const { return targetHeading_; }
  float segmentMeters() const { return totalEstimatedM_ - segmentStartM_; }
  bool pillarArmed() const { return pillarArmed_; }
  uint8_t nearSamples() const { return nearStreak_; }
  uint8_t farSamples() const { return farStreak_; }
  State state() const { return state_; }
  uint8_t pillarCount() const { return pillarCount_; }
  float estimatedMeters() const { return totalEstimatedM_; }
  static const char* name(State s) {
    switch (s) {
      case State::START_FORWARD: return "START_FORWARD";
      case State::TURN_LEFT: return "TURN_LEFT";
      case State::COUNT_PILLARS: return "COUNT_PILLARS";
      case State::CLEAR_FIFTH: return "CLEAR_FIFTH";
      case State::TURN_RIGHT: return "TURN_RIGHT";
      case State::DONE: return "DONE";
      case State::FAULT: return "FAULT";
    }
    return "UNKNOWN";
  }

 private:
  static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
  }
  static float wrap360(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
  }
  bool nearlyStopped() const {
    return fabsf(appliedLeft_) < 0.004f &&
           fabsf(appliedRight_) < 0.004f;
  }
  void transition(State next, uint32_t nowMs) {
    state_ = next;
    stateStartMs_ = nowMs;
    Serial.printf("[NAV] %s; pillars=%u; estimated=%.2fm\n",
                  name(next), static_cast<unsigned>(pillarCount_),
                  totalEstimatedM_);
  }
  void fault() {
    if (state_ != State::FAULT) Serial.println("[NAV] FAULT: emergency stop");
    state_ = State::FAULT;
  }
  float forwardTarget(float remainingM) const {
    if (remainingM <= 0.0f) return 0.0f;
    const float brakingSpeedMps = sqrtf(2.0f * kEstimatedBrakeMps2 * remainingM);
    const float command = brakingSpeedMps / kMetersPerSecondPerCommand;
    return constrain(command, 0.0f, kCruiseCommand);
  }
  void holdHeading(float forward, float errorDeg, float yawRateDps,
                   float& left, float& right) {
    // Convention from user's original code: +heading error -> clockwise turn
    // -> faster left wheel. Verify actual IMU/motor directions on stands.
    float turn = constrain(0.003f * errorDeg - 0.00025f * yawRateDps,
                           -0.025f, 0.025f);
    if (forward <= 0.0f) turn = 0.0f; // stop fully before turning state
    left = constrain(forward + turn, -0.45f, 0.45f);
    right = constrain(forward - turn, -0.45f, 0.45f);
  }
  void turnToward(float errorDeg, float yawRateDps,
                  float& left, float& right) {
    float turning = constrain(0.003f * errorDeg - 0.0004f * yawRateDps,
                              -kMaxTurnCommand, kMaxTurnCommand);
    if (fabsf(errorDeg) < 2.5f && fabsf(yawRateDps) < 5.0f) turning = 0.0f;
    left = turning;
    right = -turning;
  }
  void observePillar(int rightCm) {
    const bool near = rightCm >= 0 && rightCm <= kPillarEnterCm;
    // No echo is treated as "not near" for re-arming only, never as a pillar.
    const bool far = rightCm < 0 || rightCm >= kPillarExitCm;
    if (near) {
      nearStreak_ = (nearStreak_ < 2) ? nearStreak_ + 1 : 2;
      farStreak_ = 0;
    } else if (far) {
      farStreak_ = (farStreak_ < 2) ? farStreak_ + 1 : 2;
      nearStreak_ = 0;
    } else {
      nearStreak_ = farStreak_ = 0;
    }
    if (!pillarArmed_ && farStreak_ >= 2) pillarArmed_ = true;
    if (pillarArmed_ && nearStreak_ >= 2 &&
        (pillarCount_ == 0 ||
         totalEstimatedM_ - lastPillarM_ >= kMinPillarSpacingM)) {
      ++pillarCount_;
      lastPillarM_ = totalEstimatedM_;
      pillarArmed_ = false;
      nearStreak_ = farStreak_ = 0;
      Serial.printf("[NAV] Pillar %u detected, est %.2f m\n",
                    static_cast<unsigned>(pillarCount_), totalEstimatedM_);
    }
  }

  State state_;
  float startHeading_ = 0;
  float targetHeading_ = 0;
  float appliedLeft_ = 0;
  float appliedRight_ = 0;
  float totalEstimatedM_ = 0;
  float segmentStartM_ = 0;
  float lastPillarM_ = 0;
  uint32_t stateStartMs_ = 0;
  uint32_t lastTickMs_ = 0;
  uint32_t lastSonarSequence_ = 0;
  uint8_t pillarCount_ = 0;
  uint8_t nearStreak_ = 0;
  uint8_t farStreak_ = 0;
  bool pillarArmed_ = true;
};
