#pragma once
#include <Arduino.h>
#include <math.h>
#include "LidarScan.h"
#include "Settings.hpp"

// Returns true once, at the moment something appears. It must stay seen for onMs,
// and stay gone for offMs before it can count again.
class AppearFilter {
 public:
  AppearFilter(uint32_t onMs, uint32_t offMs) : onMs_(onMs), offMs_(offMs) {}

  void reset(uint32_t nowMs) {
    present_ = false;
    changeMs_ = nowMs;
  }

  bool update(bool seen, uint32_t nowMs) {
    if (seen == present_) {
      changeMs_ = nowMs;
      return false;
    }
    if (nowMs - changeMs_ < (seen ? onMs_ : offMs_)) return false;
    present_ = seen;
    changeMs_ = nowMs;
    return present_;
  }

 private:
  uint32_t onMs_, offMs_, changeMs_ = 0;
  bool present_ = false;
};

class Route {
 public:
  // The numbers match kLastStep in Settings.hpp. The page uses DONE = 13 and FAULT = 14.
  enum Step : uint8_t {
    IDLE, FORWARD_OUT, TURN_LEFT, FORWARD_NO_LIDAR, COUNT_PILLARS,
    FORWARD_AFTER_C, TURN_RIGHT, DRIVE_TO_END,
    TURN_RIGHT_AT_E, FORWARD_ALONG_E, TURN_LEFT_AT_E, FIND_E_SIDE, ALONG_E_SIDE,
    DONE, FAULT
  };


  struct Input {
    uint32_t nowMs;
    float headingDeg;   // 0 to 360, clockwise is positive
    bool imuOk, roverOk, lidarOk;
  };

  explicit Route(const LidarScan& lidar) : lidar_(lidar) {}

  void start(const Input& in) {
    startHeading_ = targetHeading_ = in.headingDeg;
    pillars_ = walls_ = panels_ = 0;
    speed_ = pull_ = totalM_ = 0;
    blocked_ = pausing_ = false;
    lastMs_ = in.nowMs;
    enterStep(FORWARD_OUT, in.nowMs);
    note_ = "Driving";
  }

  void stopNow(const char* why) {
    step_ = IDLE;
    note_ = why;
    speed_ = left_ = right_ = 0;
  }

  // Call this every 50 ms.
  void update(const Input& in) {
    const float dt = (in.nowMs - lastMs_) / 1000.0f;
    lastMs_ = in.nowMs;

    if (step_ == IDLE || step_ == DONE || step_ == FAULT) {
      left_ = right_ = 0;
      return;
    }
    if (!in.roverOk) return fail("Rover link lost");
    if (!in.imuOk) return fail("IMU data lost");
    if (!in.lidarOk) return fail("LiDAR data lost");
    if (pausing_) return pause(in.nowMs, dt);

    switch (step_) {
      case FORWARD_OUT:
        driveStraight(in, dt);
        if (reached(kFirstForwardM)) finishStep(in.nowMs);
        break;

      case TURN_LEFT:
        if (spinFinished(in)) finishStep(in.nowMs);   // always on the spot: little room at the door
        break;

      case FORWARD_NO_LIDAR:
        driveStraight(in, dt);
        if (stepM_ >= kNoLidarForwardM) nextStepNoStop(in.nowMs);
        break;

      case COUNT_PILLARS:
        keepWallDistance(in, wrap360(startHeading_ - 90.0f));
        driveStraight(in, dt);
        if (pillarFilter_.update(pillarOnRight(), in.nowMs) &&
            (pillars_ == 0 || stepM_ - lastPillarM_ >= kPillarMinGapM)) {
          ++pillars_;               // a new pillar, far enough from the last one
          lastPillarM_ = stepM_;
        }
        if (wallFilter_.update(wallOnLeft(), in.nowMs)) ++walls_;
        if (pillars_ >= kPillarsNeeded && walls_ >= kWallsNeeded) nextStepNoStop(in.nowMs);
        else if (stepM_ >= kCountMaxM) fail("Pillars or walls not found");
        break;

      case FORWARD_AFTER_C:
        keepWallDistance(in, wrap360(startHeading_ - 90.0f));
        driveStraight(in, dt);
        if (reached(kAfterPillarsM)) finishStep(in.nowMs);
        break;

      case TURN_RIGHT:
        if (spinFinished(in)) finishStep(in.nowMs);   // always on the spot: little room at the door        break;
        break; 
        
      case DRIVE_TO_END:
        driveStraight(in, dt);
        if (panelFilter_.update(panelOnRight(), in.nowMs)) ++panels_;   // for the page only
        if (endWallFilter_.update(stepM_ >= kToEndMinM && frontWallClose(), in.nowMs)) {
          ++walls_;                      // wall E counts as wall 3
          speed_ = left_ = right_ = 0;   // stop at once, the wall is close
          finishStep(in.nowMs);
        } else if (stepM_ >= kToEndMaxM) {
          fail("Wall E not found");
        }
        break;

      case TURN_RIGHT_AT_E:
        if (spinFinished(in)) finishStep(in.nowMs);   // always on the spot: wall E is close in front
        break;

      case FORWARD_ALONG_E:
        driveStraight(in, dt);
        if (reached(kAlongEM)) finishStep(in.nowMs);
        break;

      case TURN_LEFT_AT_E:
        if (turnFinished(in, dt)) finishStep(in.nowMs);
        break;

      case FIND_E_SIDE:
        driveStraight(in, dt);
        if (wallFilter_.update(wallOnLeft(), in.nowMs)) {
          ++walls_;                      // the side wall of E counts as wall 4
          nextStepNoStop(in.nowMs);
        } else if (stepM_ >= kFindESideMaxM) {
          fail("Side wall of E not found");
        }
        break;

      case ALONG_E_SIDE:
        driveStraight(in, dt);
        if (wallOnLeft()) lastWallM_ = stepM_;
        if (stepM_ - lastWallM_ >= kESideGoneM) finishStep(in.nowMs);   // the wall has ended
        else if (stepM_ >= kESideMaxM) fail("Side wall of E did not end");
        break;

      default:
        break;
    }
  }

  // Everything the page needs to show.
  Step step() const { return step_; }
  bool running() const { return step_ != IDLE && step_ != DONE && step_ != FAULT; }
  bool blocked() const { return blocked_; }
  float left() const { return left_; }
  float right() const { return right_; }
  int pillars() const { return pillars_; }
  int walls() const { return walls_; }
  int panels() const { return panels_; }
  float totalM() const { return totalM_; }
  float targetHeading() const { return targetHeading_; }
  const char* note() const { return note_; }
  // How many LiDAR directions see a pillar or a wall right now (for the run log).
  int pillarHits() const { return hits(90, kPillarFanDeg, kPillarMinCm, kPillarMaxCm); }
  int wallHits() const { return hits(270, kWallFanDeg, kLidarMinCm, kWallMaxCm); }

  static const char* name(Step step) {
    switch (step) {
      case IDLE: return "Not started";
      case FORWARD_OUT: return "Forward out of the door";
      case TURN_LEFT: return "Turning left";
      case FORWARD_NO_LIDAR: return "Forward, no LiDAR decisions";
      case COUNT_PILLARS: return "Counting pillars and walls";
      case FORWARD_AFTER_C: return "Forward after wall C";
      case TURN_RIGHT: return "Turning right";
      case DRIVE_TO_END: return "Driving to wall E";
      case TURN_RIGHT_AT_E: return "Turning right at wall E";
      case FORWARD_ALONG_E: return "Forward along wall E";
      case TURN_LEFT_AT_E: return "Turning left after wall E";
      case FIND_E_SIDE: return "Looking for the side wall of E";
      case ALONG_E_SIDE: return "Along the side wall of E";
      case DONE: return "Finished";
      case FAULT: return "Stopped by safety";
    }
    return "Unknown";
  }

 private:
  static float wrap180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
  }

  static float wrap360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
  }

  // ---- What the LiDAR sees ----
  // Counts the 1-degree directions in a fan that see something between minCm and maxCm.
  int hits(int centerDeg, int halfDeg, int minCm, int maxCm) const {
    int count = 0;
    for (int angle = centerDeg - halfDeg; angle <= centerDeg + halfDeg; ++angle) {
      const int cm = lidar_.distanceMm(angle) / 10;
      if (cm >= minCm && cm <= maxCm) ++count;
    }
    return count;
  }

  bool obstacleInFront() const {
    return hits(0, kFrontConeDeg, kLidarMinCm, kObstacleStopCm + kLidarToFrontCm) >= kObstacleMinHits;
  }
  bool wallOnLeft() const { return wallHits() >= kWallMinHits; }
  bool pillarOnRight() const { return pillarHits() >= kPillarMinHits; }

  bool panelOnRight() const {
    return hits(90, kPanelFanDeg, kPanelMinCm, kPanelMaxCm) >= kPanelMinHits;
  }
  // Wall E: something wide in front, closer than kEndWallStopCm to the rover's front.
  bool frontWallClose() const {
    return hits(0, kEndWallFanDeg, kLidarMinCm, kEndWallStopCm + kLidarToFrontCm) >= kEndWallMinHits;
  }

  
  // ---- Steps 4 and 5: drive parallel to the wall on the left ----
  // Average distance in cm around one direction, or 0 if fewer than 3 good readings.
  float rayCm(int centerDeg) const {
    float sum = 0;
    int count = 0;
    for (int angle = centerDeg - 2; angle <= centerDeg + 2; ++angle) {
      const int cm = lidar_.distanceMm(angle) / 10;
      if (cm >= kLidarMinCm && cm <= kWallRayMaxCm) {
        sum += cm;
        ++count;
      }
    }
    return count >= 3 ? sum / count : 0;
  }

  // Two rays, kWallRayDeg in front of and behind the left side, give the wall's direction and
  // distance. The rover aims along the wall: a little toward it when farther than the middle of
  // kWallNearCm..kWallFarCm, a little away when closer. With no clear wall (a door, the gap
  // between B and C) it keeps the last heading.
  void keepWallDistance(const Input& in, float pathHeading) {
    const float front = rayCm(270 + kWallRayDeg);
    const float back = rayCm(270 - kWallRayDeg);
    if (front <= 0 || back <= 0) return;
    const float ray = kWallRayDeg * DEG_TO_RAD;
    const float angle = atanf((front - back) / ((front + back) * tanf(ray)));   // + = pointing away from the wall
    const float wallHeading = wrap360(in.headingDeg - angle * RAD_TO_DEG);
    if (fabsf(wrap180(wallHeading - pathHeading)) > kWallMaxAngleDeg) return;   // a door or a corner, not the wall
    const float wallCm = 2 * cosf(ray) * cosf(angle) * front * back / (front + back);   // like "Left" on the page
    const float middleCm = (kWallNearCm + kWallFarCm) / 2.0f;
    const float steer = constrain((wallCm - middleCm) * kWallGain, -kWallMaxSteerDeg, kWallMaxSteerDeg);
    targetHeading_ = wrap360(wallHeading - steer);   // - = to the left, toward the wall
  }

  // ---- Moving ----
  // Stops at once for an obstacle. It stays stopped until the front is clear for kClearMs.
  bool waitForObstacle(uint32_t nowMs) {
    if (obstacleInFront()) {
      blocked_ = true;
      clearSinceMs_ = nowMs;
    } else if (blocked_ && nowMs - clearSinceMs_ >= kClearMs) {
      blocked_ = false;
    }
    return blocked_;
  }

  void driveStraight(const Input& in, float dt) {
    if (waitForObstacle(in.nowMs)) {
      speed_ = left_ = right_ = 0;   // no distance is counted while waiting
      return;
    }

    speed_ = min(speed_ + kRampPerSec * dt, kDriveSpeed);
    const float metres = kRealSpeedMps * (speed_ / kDriveSpeed) * dt;
    stepM_ += metres;
    totalM_ += metres;

    // Positive error means the rover must turn right, so the left wheel goes faster.
    const float error = wrap180(targetHeading_ - in.headingDeg);
    if (speed_ >= kDriveSpeed) {
      pull_ = constrain(pull_ + kPullGain * error * dt, -kMaxPull, kMaxPull);
    }
    float turn = pull_;
    if (fabsf(error) > kHeadingDeadDeg) {
      turn += kHeadingGain * (error - copysignf(kHeadingDeadDeg, error));
    }
    turn = constrain(turn, -kMaxCorrection, kMaxCorrection);
    turn = constrain(turn, -speed_, speed_);
    left_ = speed_ + turn;
    right_ = (speed_ - turn) * kRightTrim;

    if (fabsf(error) < kMaxHeadingErrorDeg) {
      headingBadSinceMs_ = in.nowMs;
    } else if (in.nowMs - headingBadSinceMs_ >= kHeadingErrorMs) {
      fail("Lost the straight line");
    }
  }

  // True when the soft stop will end at this distance, so slowing down starts early.
  bool reached(float targetM) const {
    const float stopSeconds = speed_ / kRampPerSec;
    const float rollM = 0.5f * kRealSpeedMps * (speed_ / kDriveSpeed) * stopSeconds;
    return stepM_ + rollM >= targetM;
  }

  // Chooses the kind of turn: a curve, or the old turn on the spot (kCurveTurns in Settings.hpp).
  bool turnFinished(const Input& in, float dt) {
    return kCurveTurns ? curveFinished(in, dt) : spinFinished(in);
  }

  // Turns in a curve: both sides go forward, the outer side faster. The power ramps up
  // like straight driving. In the last kTurnSlowDeg the curve opens up, so it does not overshoot.
  bool curveFinished(const Input& in, float dt) {
    const float error = wrap180(targetHeading_ - in.headingDeg);   // + = turn right
    if (in.nowMs - stepStartMs_ > kTurnTimeoutMs) {
      fail("Turn took too long");
      return false;
    }
    if (fabsf(error) > kTurnWrongWayDeg) {
      fail("Turn went the wrong way");
      return false;
    }
    if (fabsf(error) <= kTurnToleranceDeg) return true;   // the soft stop comes next
    if (waitForObstacle(in.nowMs)) {
      speed_ = left_ = right_ = 0;   // the rover moves in a curve, so it must stop for obstacles
      return false;
    }

    speed_ = min(speed_ + kRampPerSec * dt, kTurnSpeed);
    const float metres = kRealSpeedMps * (speed_ / kDriveSpeed) * dt;
    stepM_ += metres;
    totalM_ += metres;

    const float sharpness = kTurnSharpness * min(1.0f, fabsf(error) / kTurnSlowDeg);
    float turn = copysignf(speed_ * sharpness, error) + pull_;   // + = left side faster
    turn = constrain(turn, -speed_, speed_);   // the inner side never goes backwards
    left_ = speed_ + turn;
    right_ = (speed_ - turn) * kRightTrim;
    return false;
  }

  // The old turn: spins on the spot at full power, then uses short pulses near the target.
  bool spinFinished(const Input& in) {
    const uint32_t nowMs = in.nowMs;
    const float error = wrap180(targetHeading_ - in.headingDeg);   // + = turn right

    if (nowMs - stepStartMs_ > kTurnTimeoutMs) {
      fail("Turn took too long");
      return false;
    }
    if (fabsf(error) > kTurnWrongWayDeg) {
      fail("Turn went the wrong way");
      return false;
    }
    if (fabsf(error) <= kTurnToleranceDeg) {
      left_ = right_ = 0;
      return nowMs - inToleranceSinceMs_ >= kTurnHoldMs;
    }
    inToleranceSinceMs_ = nowMs;
    float power = kTurnPower;
    if (fabsf(error) <= kTurnPulseDeg) {
      const uint32_t phase = (nowMs - stepStartMs_) % (kTurnPulseOnMs + kTurnPulseOffMs);
      if (phase >= kTurnPulseOnMs) power = 0;
    }
    left_ = error > 0 ? power : -power;
    right_ = -left_;
    return false;
  }

  // ---- Changing steps ----
  void finishStep(uint32_t nowMs) {
    nextStep_ = step_ >= kLastStep ? DONE : static_cast<Step>(step_ + 1);
    pausing_ = true;
    pauseStartMs_ = nowMs;
    blocked_ = false;   // a step can end next to a wall; the next step checks again
  }

  // Goes to the next step while still driving, unless this is the last test step.
  void nextStepNoStop(uint32_t nowMs) {
    if (step_ >= kLastStep) finishStep(nowMs);
    else enterStep(static_cast<Step>(step_ + 1), nowMs);
  }

  // Slows to zero, waits kPauseMs, then starts the next step.
  void pause(uint32_t nowMs, float dt) {
    speed_ = max(speed_ - kRampPerSec * dt, 0.0f);
    left_ = speed_;
    right_ = speed_ * kRightTrim;
    if (speed_ > 0 || nowMs - pauseStartMs_ < kPauseMs) return;
    pausing_ = false;
    if (nextStep_ == DONE) note_ = "Route step finished";
    enterStep(nextStep_, nowMs);
  }

  void enterStep(Step next, uint32_t nowMs) {
    step_ = next;
    stepStartMs_ = inToleranceSinceMs_ = headingBadSinceMs_ = nowMs;
    stepM_ = 0;
    if (next == TURN_LEFT) targetHeading_ = wrap360(startHeading_ - 90.0f); // + 90 Test turns right
    if (next == TURN_RIGHT) targetHeading_ = startHeading_;
    if (next == COUNT_PILLARS) {
      pillarFilter_.reset(nowMs);
      wallFilter_.reset(nowMs);
    }
    if (next == DRIVE_TO_END) {
      panelFilter_.reset(nowMs);
      endWallFilter_.reset(nowMs);
    }
    if (next == TURN_RIGHT_AT_E) targetHeading_ = wrap360(startHeading_ + 90.0f);
    if (next == TURN_LEFT_AT_E) targetHeading_ = startHeading_;
    if (next == FIND_E_SIDE) wallFilter_.reset(nowMs);
    if (next == ALONG_E_SIDE) lastWallM_ = 0;
  }

  void fail(const char* why) {
    step_ = FAULT;
    note_ = why;
    speed_ = left_ = right_ = 0;
    blocked_ = pausing_ = false;
  }

  const LidarScan& lidar_;
  AppearFilter pillarFilter_{kPillarSeenMs, kPillarGoneMs};
  AppearFilter wallFilter_{kWallSeenMs, kWallGoneMs};
  AppearFilter panelFilter_{kPanelSeenMs, kPanelGoneMs};
  AppearFilter endWallFilter_{kEndWallSeenMs, kEndWallSeenMs};

  Step step_ = IDLE, nextStep_ = DONE;
  const char* note_ = "Press Start";
  float startHeading_ = 0, targetHeading_ = 0;
  float speed_ = 0, pull_ = 0, left_ = 0, right_ = 0;
  float stepM_ = 0, totalM_ = 0;
  float lastPillarM_ = 0;   // where the last pillar was counted, in metres of step 4
  float lastWallM_ = 0;     // where the side wall of E was last seen, in metres of step 12
  int pillars_ = 0, walls_ = 0, panels_ = 0;
  bool blocked_ = false, pausing_ = false;
  uint32_t lastMs_ = 0, stepStartMs_ = 0, pauseStartMs_ = 0;
  uint32_t clearSinceMs_ = 0, inToleranceSinceMs_ = 0, headingBadSinceMs_ = 0;
};
