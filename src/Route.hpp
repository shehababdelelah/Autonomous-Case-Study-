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
  // The numbers match kLastStep in Settings.hpp. The page uses DONE = 25 and FAULT = 26.
  enum Step : uint8_t {
    IDLE, FORWARD_OUT, TURN_LEFT, FORWARD_NO_LIDAR, COUNT_PILLARS,                       //  0- 4
    FORWARD_AFTER_C, TURN_RIGHT, DRIVE_TO_END, TURN_RIGHT_AT_E, FORWARD_ALONG_E,        //  5- 9
    TURN_LEFT_AT_E, FIND_E_SIDE, ALONG_E_SIDE, FIND_BENCH_PANEL, FIND_I_WALL,            // 10-14
    FIND_SMALL_PILLAR, TURN_RIGHT_AT_PILLAR, FORWARD_AFTER_PILLAR, TURN_LEFT_TO_G,       // 15-18
    DRIVE_TO_G, TURN_RIGHT_AT_G, ALONG_K, PAST_K, TURN_RIGHT_LAST, TO_LAST_PILLAR,       // 19-24
    DONE, FAULT                                                                          // 25-26
  };


  struct Input {
    uint32_t nowMs;
    float headingDeg;   // 0 to 360, clockwise is positive
    bool imuOk, roverOk, lidarOk;
  };

  explicit Route(const LidarScan& lidar) : lidar_(lidar) {}

  // Starts at firstStep and stops after lastStep (see the test groups in Settings.hpp).
  // facingDeg: the way the rover faces now, compared with the door direction (0 for step 1).
  // Every turn is counted from the door direction, so the turns stay right in a test group.
  void start(const Input& in, int firstStep = FORWARD_OUT, int lastStep = kLastStep, float facingDeg = 0) {
    startHeading_ = wrap360(in.headingDeg - facingDeg);
    targetHeading_ = in.headingDeg;
    lastStep_ = lastStep;
    pillars_ = walls_ = panels_ = 0;
    speed_ = pull_ = totalM_ = 0;
    blocked_ = pausing_ = stepEnded_ = false;
    lastMs_ = in.nowMs;
    enterStep(static_cast<Step>(firstStep), in.nowMs);
    note_ = "Driving";
  }

  void stopNow(const char* why) {
    if (running() && !pausing_) noteStepEnd(lastMs_, why);
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
        if (spinFinished(in, dt)) finishStep(in.nowMs);   // always on the spot: little room at the door
        break;

      case FORWARD_NO_LIDAR:
        driveStraight(in, dt);
        if (stepM_ >= kNoLidarForwardM) nextStepNoStop(in.nowMs);
        break;

      case COUNT_PILLARS:
        keepWallDistance(in, wrap360(startHeading_ - 90.0f), kWallNearCm, kWallFarCm);
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
        keepWallDistance(in, wrap360(startHeading_ - 90.0f), kWallNearCm, kWallFarCm);
        driveStraight(in, dt);
        if (reached(kAfterPillarsM)) finishStep(in.nowMs);
        break;

      case TURN_RIGHT:
        if (spinFinished(in, dt)) finishStep(in.nowMs);   // always on the spot
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
        if (spinFinished(in, dt)) finishStep(in.nowMs);   // always on the spot: wall E is close in front
        break;

      case FORWARD_ALONG_E: {   // 9: along wall E until a pillar on the right, then kAfterEPillarM more
        driveStraight(in, dt);
        const bool pillar = pillarFilter_.update(stepM_ >= kAlongEMinM && farPillarRight(kEPillarMinCm, kEPillarMaxCm), in.nowMs);
        if (pillar && foundAtM_ < 0) foundAtM_ = stepM_;   // remember where the pillar was
        if (foundAtM_ >= 0 && reached(foundAtM_ + kAfterEPillarM)) finishStep(in.nowMs);
        else if (foundAtM_ < 0 && stepM_ >= kAlongEMaxM) fail("Pillar along E not found");
        break;
      }

      case TURN_LEFT_AT_E:
        if (turnFinished(in, dt)) finishStep(in.nowMs);
        break;

      case FIND_E_SIDE:   // 11: the side wall of E on the left, closer than kESideFindMaxCm
        driveStraight(in, dt);
        if (wallFilter_.update(wallLeft(kLidarMinCm, kESideFindMaxCm), in.nowMs)) {
          ++walls_;                      // the side wall of E counts as wall 4
          nextStepNoStop(in.nowMs);
        } else if (stepM_ >= kFindESideMaxM) {
          fail("Side wall of E not found");
        }
        break;

      case ALONG_E_SIDE: {   // 12: along the side wall of E; after its end, on to the fire hydrant
        const bool wallThere = wallLeft(kLidarMinCm, kFollowWallMaxCm);
        if (wallThere) {
          keepWallDistance(in, startHeading_, kESideNearCm, kESideFarCm);
          lastWallM_ = stepM_;
        }
        driveStraight(in, dt);
        const bool wallEnded = stepM_ - lastWallM_ >= kESideGoneM;
        if (smallFilter_.update(wallEnded && smallLeft(kHydrantMinCm, kHydrantMaxCm), in.nowMs)) {
          nextStepNoStop(in.nowMs);      // the fire hydrant
        } else if (wallEnded && stepM_ - lastWallM_ >= kHydrantMaxM) {
          fail("Fire hydrant not found");
        } else if (stepM_ >= kESideMaxM) {
          fail("Side wall of E did not end");
        }
        break;
      }

      case FIND_BENCH_PANEL:   // 13: two bench legs on the left, then a panel on the left
        driveStraight(in, dt);
        if (found_ < kLegsNeeded && smallFilter_.update(smallLeft(kLegMinCm, kLegMaxCm), in.nowMs)) ++found_;
        if (panelFilter_.update(found_ >= kLegsNeeded && panelLeft(kBenchPanelMinCm, kBenchPanelMaxCm), in.nowMs)) {
          ++panels_;
          nextStepNoStop(in.nowMs);
        } else if (stepM_ >= kBenchMaxM) {
          fail("Bench legs or panel not found");
        }
        break;

      case FIND_I_WALL:   // 14: the wall of building I on the left
        driveStraight(in, dt);
        if (wallFilter_.update(stepM_ >= kIWallMinM && wallLeft(kIWallMinCm, kIWallMaxCm), in.nowMs)) {
          ++walls_;
          nextStepNoStop(in.nowMs);
        } else if (stepM_ >= kIWallMaxM) {
          fail("Wall of I not found");
        }
        break;

      case FIND_SMALL_PILLAR:   // 15: the small pillar in front, then stop
        driveStraight(in, dt);
        if (smallFilter_.update(somethingFront(kSmallPillarStopCm), in.nowMs)) {
          speed_ = left_ = right_ = 0;   // stop at once, it is close
          finishStep(in.nowMs);
        } else if (stepM_ >= kSmallPillarMaxM) {
          fail("Small pillar not found");
        }
        break;

      case TURN_RIGHT_AT_PILLAR:   // 16
      case TURN_LEFT_TO_G:         // 18
      case TURN_RIGHT_AT_G:        // 20
      case TURN_RIGHT_LAST:        // 23
        if (turnFinished(in, dt)) finishStep(in.nowMs);   // the direction is set in enterStep()
        break;

      case FORWARD_AFTER_PILLAR:   // 17: a short way between two turns
        driveStraight(in, dt);
        if (reached(kAfterSmallPillarM)) finishStep(in.nowMs);
        break;

      case DRIVE_TO_G:   // 19: wall G in front, then stop
        driveStraight(in, dt);
        if (endWallFilter_.update(wallFront(kGStopCm), in.nowMs)) {
          speed_ = left_ = right_ = 0;   // stop at once
          finishStep(in.nowMs);
        } else if (stepM_ >= kToGMaxM) {
          fail("Wall G not found");
        }
        break;

      case ALONG_K:   // 21: along wall K on the left, count two pillars on the right
        if (wallLeft(kLidarMinCm, kFollowWallMaxCm)) keepWallDistance(in, wrap360(startHeading_ + 90.0f), kKWallNearCm, kKWallFarCm);
        driveStraight(in, dt);
        if (wallFilter_.update(wallLeft(kLidarMinCm, kFollowWallMaxCm), in.nowMs)) ++walls_;   // wall K
        if (pillarFilter_.update(farPillarRight(kKPillarMinCm, kKPillarMaxCm), in.nowMs)) ++found_;
        if (found_ >= kKPillarsNeeded) nextStepNoStop(in.nowMs);
        else if (stepM_ >= kAlongKMaxM) fail("Pillars along K not found");
        break;

      case PAST_K: {   // 22: past the end of wall K (wall G on the left), then stop
        const bool kThere = wallLeft(kLidarMinCm, kFollowWallMaxCm);
        if (kThere) {
          keepWallDistance(in, wrap360(startHeading_ + 90.0f), kKWallNearCm, kKWallFarCm);
          lastWallM_ = stepM_;           // wall K is still there
        }
        driveStraight(in, dt);
        if (wallFilter_.update(!kThere && wallLeft(kGLeftMinCm, kGLeftMaxCm), in.nowMs)) found_ = 1;   // wall G
        if (reached(lastWallM_ + kAfterKWallM)) {
          if (found_) finishStep(in.nowMs);
          else fail("Wall G not seen on the left");
        } else if (stepM_ >= kPastKMaxM) {
          fail("End of wall K not found");
        }
        break;
      }

      case TO_LAST_PILLAR:   // 24: the last pillar in front, then stop: mission finished
        driveStraight(in, dt);
        if (smallFilter_.update(somethingFront(kLastPillarStopCm), in.nowMs)) {
          speed_ = left_ = right_ = 0;   // stop at once, it is close
          finishStep(in.nowMs);
        } else if (stepM_ >= kLastPillarMaxM) {
          fail("Last pillar not found");
        }
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

  // True once after a step has ended: which step, how long it worked (the pause after it is
  // not counted) and "done" or why the run stopped. main.cpp saves it in the step-times CSV.
  bool takeEndedStep(Step& step, uint32_t& ms, const char*& result) {
    if (!stepEnded_) return false;
    stepEnded_ = false;
    step = endedStep_;
    ms = endedMs_;
    result = endedResult_;
    return true;
  }
  // How many LiDAR directions see a pillar or a wall right now (for the run log).
  int pillarHits() const { return hits(90, kPillarFanDeg, kPillarMinCm, kPillarMaxCm); }
  int wallHits() const { return hits(270, kWallFanDeg, kLidarMinCm, kWallMaxCm); }

  // Names for the page: at most 31 characters (WEBpage.hpp keeps 32).
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
      case FORWARD_ALONG_E: return "Along wall E, to the pillar";
      case TURN_LEFT_AT_E: return "Turning left after wall E";
      case FIND_E_SIDE: return "Looking for the side wall of E";
      case ALONG_E_SIDE: return "Along E side, to the hydrant";
      case FIND_BENCH_PANEL: return "Looking for bench and panel";
      case FIND_I_WALL: return "Looking for the wall of I";
      case FIND_SMALL_PILLAR: return "Looking for the small pillar";
      case TURN_RIGHT_AT_PILLAR: return "Turning right at small pillar";
      case FORWARD_AFTER_PILLAR: return "Forward after small pillar";
      case TURN_LEFT_TO_G: return "Turning left to G";
      case DRIVE_TO_G: return "Driving to wall G";
      case TURN_RIGHT_AT_G: return "Turning right at wall G";
      case ALONG_K: return "Along wall K, 2 pillars right";
      case PAST_K: return "Past the end of wall K";
      case TURN_RIGHT_LAST: return "Last turn right";
      case TO_LAST_PILLAR: return "To the last pillar";
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

  // ---- Checks for steps 9 to 24. All distances are from the LiDAR centre, like the page. ----
  // A wall on the left: many directions see it.
  bool wallLeft(int minCm, int maxCm) const { return hits(270, kWallFanDeg, minCm, maxCm) >= kWallMinHits; }
  // Something small on the left (fire hydrant, bench leg): a few directions see it.
  bool smallLeft(int minCm, int maxCm) const { return hits(270, kSmallFanDeg, minCm, maxCm) >= kSmallMinHits; }
  bool panelLeft(int minCm, int maxCm) const { return hits(270, kPanelFanDeg, minCm, maxCm) >= kPanelMinHits; }
  // A pillar 2-2.5 m away on the right.
  bool farPillarRight(int minCm, int maxCm) const { return hits(90, kPillarFanDeg, minCm, maxCm) >= kFarPillarMinHits; }
  // A wall in front, closer than maxCm.
  bool wallFront(int maxCm) const { return hits(0, kEndWallFanDeg, kLidarMinCm, maxCm) >= kEndWallMinHits; }
  // Anything in front (a pillar), closer than maxCm. Looks as wide as the obstacle check.
  bool somethingFront(int maxCm) const { return hits(0, kFrontConeDeg, kLidarMinCm, maxCm) >= kSmallMinHits; }

  
  // ---- Steps 4, 5, 12, 21, 22: drive parallel to the wall on the left ----
  // Average distance in cm around one direction. 0 if fewer than 3 good readings, or if they
  // do not agree: then the ray is at the end of a wall and half sees what is behind it.
  float rayCm(int centerDeg) const {
    float sum = 0;
    int count = 0, lowest = 10000, highest = 0;
    for (int angle = centerDeg - 2; angle <= centerDeg + 2; ++angle) {
      const int cm = lidar_.distanceMm(angle) / 10;
      if (cm >= kLidarMinCm && cm <= kWallRayMaxCm) {
        sum += cm;
        ++count;
        lowest = min(lowest, cm);
        highest = max(highest, cm);
      }
    }
    if (count < 3 || highest - lowest > kWallRaySpreadCm) return 0;
    return sum / count;
  }

  // Two rays, kWallRayDeg in front of and behind the left side, give the wall's direction and
  // distance. The rover aims along the wall: a little toward it when farther than the middle of
  // nearCm..farCm, a little away when closer. With no clear wall (a door, the gap
  // between B and C) it keeps the last heading.
  void keepWallDistance(const Input& in, float pathHeading, int nearCm, int farCm) {
    const float front = rayCm(270 + kWallRayDeg);
    const float back = rayCm(270 - kWallRayDeg);
    if (front <= 0 || back <= 0) return;
    const float ray = kWallRayDeg * DEG_TO_RAD;
    const float angle = atanf((front - back) / ((front + back) * tanf(ray)));   // + = pointing away from the wall
    const float wallHeading = wrap360(in.headingDeg - angle * RAD_TO_DEG);
    if (fabsf(wrap180(wallHeading - pathHeading)) > kWallMaxAngleDeg) return;   // a door or a corner, not the wall
    const float wallCm = 2 * cosf(ray) * cosf(angle) * front * back / (front + back);   // like "Left" on the page
    const float middleCm = (nearCm + farCm) / 2.0f;
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
    return kCurveTurns ? curveFinished(in, dt) : spinFinished(in, dt);
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
  // The power moves to the wanted power in a straight line, kTurnRampPerSec per second, so the
  // gearbox gets no hard kick when it starts, stops or changes direction.
  bool spinFinished(const Input& in, float dt) {
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
    const bool inside = fabsf(error) <= kTurnToleranceDeg;
    float wanted = 0;                                              // + = turn right
    if (!inside) {
      inToleranceSinceMs_ = nowMs;
      wanted = kTurnPower;
      if (fabsf(error) <= kTurnPulseDeg) {
        const uint32_t phase = (nowMs - stepStartMs_) % (kTurnPulseOnMs + kTurnPulseOffMs);
        if (phase >= kTurnPulseOnMs) wanted = 0;
      }
      if (error < 0) wanted = -wanted;
    }
    const float change = kTurnRampPerSec * dt;
    spin_ = constrain(wanted, spin_ - change, spin_ + change);   // one small step toward wanted
    left_ = spin_;
    right_ = -spin_;
    return inside && spin_ == 0 && nowMs - inToleranceSinceMs_ >= kTurnHoldMs;
  }

  // ---- Changing steps ----
  void finishStep(uint32_t nowMs) {
    noteStepEnd(nowMs, "done");
    nextStep_ = step_ >= lastStep_ ? DONE : static_cast<Step>(step_ + 1);
    pausing_ = true;
    pauseStartMs_ = nowMs;
    blocked_ = false;   // a step can end next to a wall; the next step checks again
  }

  // Goes to the next step while still driving, unless this is the last test step.
  void nextStepNoStop(uint32_t nowMs) {
    if (step_ >= lastStep_) {
      finishStep(nowMs);
    } else {
      noteStepEnd(nowMs, "done");
      enterStep(static_cast<Step>(step_ + 1), nowMs);
    }
  }

  // Remembers the step that just ended, for takeEndedStep().
  void noteStepEnd(uint32_t nowMs, const char* result) {
    endedStep_ = step_;
    endedMs_ = nowMs - stepStartMs_;
    endedResult_ = result;
    stepEnded_ = true;
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
    spin_ = 0;   // every turn starts from standing still

    // Every step starts fresh: nothing seen yet.
    found_ = 0;
    foundAtM_ = -1;
    lastWallM_ = 0;
    pillarFilter_.reset(nowMs);
    wallFilter_.reset(nowMs);
    panelFilter_.reset(nowMs);
    endWallFilter_.reset(nowMs);
    smallFilter_.reset(nowMs);

    // Turns: the new direction, counted from startHeading_ (the way the rover faced at the door).
    if (next == TURN_LEFT) targetHeading_ = wrap360(startHeading_ - 90.0f); // + 90 Test turns right
    if (next == TURN_RIGHT || next == TURN_LEFT_AT_E || next == TURN_LEFT_TO_G) {
      targetHeading_ = startHeading_;                       // the same way as at the door
    }
    if (next == TURN_RIGHT_AT_E || next == TURN_RIGHT_AT_PILLAR || next == TURN_RIGHT_AT_G) {
      targetHeading_ = wrap360(startHeading_ + 90.0f);      // 90 to the right of that
    }
    if (next == TURN_RIGHT_LAST) targetHeading_ = wrap360(startHeading_ + 180.0f);   // back the other way
  }

  void fail(const char* why) {
    if (!pausing_) noteStepEnd(lastMs_, why);
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
  AppearFilter smallFilter_{kSmallSeenMs, kSmallGoneMs};

  Step step_ = IDLE, nextStep_ = DONE;
  int lastStep_ = kLastStep;   // the run stops after this step (kLastStep, or a test group's last)
  Step endedStep_ = IDLE;      // the step that just ended, for takeEndedStep()
  uint32_t endedMs_ = 0;
  const char* endedResult_ = "";
  bool stepEnded_ = false;
  const char* note_ = "Press Start";
  float startHeading_ = 0, targetHeading_ = 0;
  float speed_ = 0, pull_ = 0, left_ = 0, right_ = 0;
  float spin_ = 0;   // turn power now, + = turn right; it ramps (spinFinished)
  float stepM_ = 0, totalM_ = 0;
  float lastPillarM_ = 0;   // where the last pillar was counted, in metres of step 4
  float lastWallM_ = 0;     // where the wall we follow was last seen, in metres of this step (12, 22)
  float foundAtM_ = -1;     // step 9: where the pillar was seen, -1 = not yet
  int found_ = 0;           // things found in this step: bench legs (13), pillars (21), wall G (22)
  int pillars_ = 0, walls_ = 0, panels_ = 0;
  bool blocked_ = false, pausing_ = false;
  uint32_t lastMs_ = 0, stepStartMs_ = 0, pauseStartMs_ = 0;
  uint32_t clearSinceMs_ = 0, inToleranceSinceMs_ = 0, headingBadSinceMs_ = 0;
};
