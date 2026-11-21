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

// The direction of a flat thing beside the rover, from how the side distance changes while the
// rover drives past it. It fits a straight line through all the readings.
class SideLine {
 public:
  void reset() { n_ = 0; sumM_ = sumCm_ = sumMM_ = sumMCm_ = sumHeading_ = 0; }

  // sideCm: the side distance now. stepM: how far the step has driven. headingDeg: the heading
  // now, counted from the door direction.
  void add(float sideCm, float stepM, float headingDeg) {
    if (n_ == 0) firstM_ = stepM;
    const float m = stepM - firstM_;
    lastM_ = stepM;
    sumM_ += m;
    sumCm_ += sideCm;
    sumMM_ += m * m;
    sumMCm_ += m * sideCm;
    sumHeading_ += headingDeg;
    ++n_;
  }

  bool seen() const { return n_ > 0; }
  float lengthM() const { return n_ > 0 ? lastM_ - firstM_ : 0; }
  float lastM() const { return lastM_; }
  float headingDeg() const { return sumHeading_ / n_; }   // average while it was seen
  float avgCm() const { return n_ > 0 ? sumCm_ / n_ : 0; }   // average side distance

  // How fast the side distance opens, as an angle. + = the distance grows.
  float openDeg() const {
    const float spread = n_ * sumMM_ - sumM_ * sumM_;
    if (n_ < 3 || spread <= 0) return 0;
    const float cmPerM = (n_ * sumMCm_ - sumM_ * sumCm_) / spread;
    return atanf(cmPerM / 100.0f) * RAD_TO_DEG;
  }

 private:
  int n_ = 0;
  float firstM_ = 0, lastM_ = 0;
  float sumM_ = 0, sumCm_ = 0, sumMM_ = 0, sumMCm_ = 0, sumHeading_ = 0;
};

class Route {
 public:
  // The numbers match kLastStep in Settings.hpp. The page uses DONE = 25 and FAULT = 26.
  enum Step : uint8_t {
    IDLE, FORWARD_OUT, TURN_LEFT, FORWARD_NO_LIDAR, COUNT_PILLARS,                       //  0- 4
    FORWARD_AFTER_C, TURN_RIGHT, DRIVE_TO_END, TURN_RIGHT_AT_E, FORWARD_ALONG_E,         //  5- 9
    TURN_LEFT_AT_E, FIND_E_SIDE, ALONG_E_SIDE, FIND_PANEL, TO_RAMP,                      // 10-14
    CROSS_RAMP, TURN_RIGHT_AT_PILLAR, FORWARD_AFTER_PILLAR, TURN_LEFT_TO_G,              // 15-18
    DRIVE_TO_G, TURN_RIGHT_AT_G, ALONG_K, PAST_K, TURN_RIGHT_LAST, TO_LAST_PILLAR,       // 19-24
    DONE, FAULT                                                                          // 25-26
  };

  // Steps 14 and 15 do several things one after the other. Each one is a phase: it has its own
  // target heading, its own start time and its own start distance. The page shows which one runs.
  enum Phase : uint8_t { P_DRIVE, P_WATCH, P_SIDESTEP, P_TURN_BACK, P_SETTLE, P_PILLAR };


  struct Input {
    uint32_t nowMs;
    float headingDeg;   // 0 to 360, clockwise is positive
    float pitchDeg;     // nose up or down
    float rollDeg;      // leaning left or right
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
    beamDone_ = false;
    iWallDoorDeg_ = -1000;
    blocked_ = pausing_ = stepEnded_ = hardStop_ = lifted_ = false;
    lastMs_ = in.nowMs;
    enterStep(static_cast<Step>(firstStep), in.nowMs);
    note_ = "Driving";
  }

  // The STOP button. The wheels are not cut to 0 here: update() brakes them with eStop(),
  // which takes about half a second. Press STOP again while it is braking and it stops at once.
  void stopNow(const char* why) {
    if (kEStopHardOnSecondStop && step_ == IDLE && speed_ > 0) hardStop_ = true;
    if (running() && !pausing_) noteStepEnd(lastMs_, why);
    step_ = IDLE;
    note_ = why;
  }

  // Call this every 50 ms.
  void update(const Input& in) {
    pitchDeg_ = in.pitchDeg;
    const float dt = (in.nowMs - lastMs_) / 1000.0f;
    lastMs_ = in.nowMs;

    if (step_ == IDLE || step_ == DONE || step_ == FAULT) {
      eStop(dt, hardStop_);   // STOP, a fault or the end: brake, do not cut the power
      return;
    }
    if (liftedOffGround(in, dt)) return;   // in somebody's hands: wait, keep the step as it is
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
        if (turnFinished(in, dt)) finishStep(in.nowMs);   // a short curve: there is little room at the door
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
        // With the wall C reference the rover keeps one heading here: a straight line measures best.
        if (!kUseWallCRef) keepWallDistance(in, wrap360(startHeading_ - 90.0f), kWallNearCm, kWallFarCm);
        driveStraight(in, dt);
        watchSide(in, 270);   // wall C
        if (reached(kAfterPillarsM)) {
          // The door direction is 90 deg to the right of wall C.
          if (kUseWallCRef) resetDoorDirection(lineHeading(true) + 90.0f, kWallRefMaxDeg);
          finishStep(in.nowMs);
        }
        break;

      case TURN_RIGHT:
        if (turnFinished(in, dt)) finishStep(in.nowMs);
        break;

      case DRIVE_TO_END:
        driveStraight(in, dt);
        if (panelFilter_.update(panelOnRight(), in.nowMs)) ++panels_;   // for the page only
        if (kUsePanelRef && !refDone_) {
          const float cm = rayCm(90);
          if (stepM_ <= kPanelRefEndM && cm >= kPanelRefMinCm && cm <= kPanelRefMaxCm) {
            line_.add(cm, stepM_, wrap180(in.headingDeg - startHeading_));
          } else if (stepM_ > kPanelRefEndM || (line_.seen() && stepM_ - line_.lastM() > 0.30f)) {
            // The panel has passed. The door direction is along the panel.
            resetDoorDirection(lineHeading(false), kPanelRefMaxDeg);
            refDone_ = true;
          }
        }
        if (endWallFilter_.update(stepM_ >= kToEndMinM && frontWallClose(), in.nowMs)) {
          ++walls_;                      // wall E counts as wall 3
          speed_ = left_ = right_ = 0;   // stop at once, the wall is close
          finishStep(in.nowMs);
        } else if (stepM_ >= kToEndMaxM) {
          fail("Wall E not found");
        }
        break;

      case TURN_RIGHT_AT_E:
        if (turnFinished(in, dt)) finishStep(in.nowMs);   // the curve rolls forward: wall E is 80 cm away
        break;

       case FORWARD_ALONG_E: {   // 9: along wall E until a pillar on the right, then kAfterEPillarM more
        // Keep the distance to wall E on the left, like steps 4, 12, 21 and 22 do with their walls.
        if (wallLeft(kLidarMinCm, kFollowWallMaxCm)) {
          keepWallDistance(in, startHeading_, kEWallNearCm, kEWallFarCm);
        }
        driveStraight(in, dt);
        if (wallLeft(kLidarMinCm, kFollowWallMaxCm)) lastWallM_ = stepM_;   // wall E is still beside the rover
        if (kUseWallERef && !refDone_) {
          if (stepM_ < kWallERefM) {
            watchSide(in, 270);   // wall E
          } else {
            // The door direction is 90 deg to the left of wall E. The rover then drives parallel to it.
            resetDoorDirection(lineHeading(true) - 90.0f, kWallRefMaxDeg);
            refDone_ = true;
          }
        }
        // The same pillar check as in step 4: enough directions, seen long enough, and far enough
        // from the pillar before. The step ends kAfterEPillarM after pillar number kEPillarsNeeded.
        const bool seen = stepM_ >= kAlongEMinM && hits(90, kPillarFanDeg, kEPillarMinCm, kEPillarMaxCm) >= kPillarMinHits;
        if (pillarFilter_.update(seen, in.nowMs) && (found_ == 0 || stepM_ - lastPillarM_ >= kPillarMinGapM)) {
          ++found_;
          lastPillarM_ = stepM_;
          if (found_ == kEPillarsNeeded) foundAtM_ = stepM_;   // remember where the last pillar was
        }
        // Spare rule: the pillar was missed, but wall E has ended on the left.
        const bool wallEnded = foundAtM_ < 0 && lastWallM_ > 0 && lastWallM_ >= kAlongEMinM && reached(lastWallM_ + kAfterEWallEndM);
        if (foundAtM_ >= 0 && reached(foundAtM_ + kAfterEPillarM)) finishStep(in.nowMs);
        else if (wallEnded) finishStep(in.nowMs);
        else if (stepM_ >= kAlongEMaxM) fail("Pillar along E not found");
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

      case FIND_PANEL:   // 13: beside the wall of E, and the panel on the left says the ramp comes next
        // The same wall keeping as step 12, a little further out. If the wall is not there any
        // more, nothing happens here and the rover simply drives straight on.
        if (wallLeft(kLidarMinCm, kFollowWallMaxCm)) {
          keepWallDistance(in, startHeading_, kPanelWallNearCm, kPanelWallFarCm);
        }
        driveStraight(in, dt);
        if (panelFilter_.update(panelLeft(kPanelLeftMinCm, kPanelLeftMaxCm), in.nowMs)) {
          ++panels_;
          nextStepNoStop(in.nowMs);
        } else if (stepM_ >= kPanelMaxM) {
          fail("Panel not found");
        }
        break;

      case TO_RAMP:   // 14: straight on. The wall of I on the left says the ramp is reached.
        switch (phase_) {
          case P_WATCH:   // standing still in front of something: is it the beam?
            if (watchFront(in, dt, kFrontConeDeg, kBeamStopCm, kBeamHoldMs)) {
              beamDone_ = true;            // the beam is passed: it is never looked for again
              note_ = "Beam: a little to the left";
              startPhase(P_SIDESTEP, in.nowMs, wrap360(startHeading_ - kBeamAvoidDeg));
            } else if (watchGone_) {
              note_ = "Forward to the ramp";
              startPhase(P_DRIVE, in.nowMs, startHeading_);   // it moved away, or it is not the beam
            }
            break;

          case P_SIDESTEP: {
            // Aiming kBeamAvoidDeg to the left. The rover only has to move kBeamSideStepM
            // sideways, so it drives that much divided by the sine of the angle.
            driveStraight(in, dt);
            const float diagonalM = kBeamSideStepM / sinf(kBeamAvoidDeg * DEG_TO_RAD);
            if (stepM_ - phaseM_ >= diagonalM) {
              note_ = "Beam: straightening up";
              startPhase(P_TURN_BACK, in.nowMs, startHeading_);
            }
            break;
          }

          case P_TURN_BACK:
            // Back to the old direction. It keeps rolling: no turn on the spot, no stop.
            driveStraight(in, dt);
            if (fabsf(wrap180(startHeading_ - in.headingDeg)) <= kTurnToleranceDeg) {
              note_ = "Forward over the ramp";
              startPhase(P_DRIVE, in.nowMs, startHeading_);
            }
            break;

          default: {   // P_DRIVE: over the ramp, beside the wall of I, watching ahead for the beam
            // Keep the distance to the wall of I on the left, the same as steps 9, 12 and 21 do
            // with their walls. It is caught from further away: the wall of I is about 2 m off.
            // Not in the first kIWallFromM: there is no wall of I there yet, but the panel from
            // step 13 is still on the left at about the same distance and would be followed.
            if (stepM_ >= kIWallFromM && wallLeft(kLidarMinCm, kIWallCatchCm)) {
              keepWallDistance(in, startHeading_, kIWallNearCm, kIWallFarCm);
            }
            driveStraight(in, dt);

            if (rampFinished(in)) {
              finishStep(in.nowMs);                    // the nose went up and came back level
            } else if (!beamDone_ && smallFilter_.update(somethingFront(kBeamStopCm, kFrontConeDeg), in.nowMs)) {
              note_ = "Checking the beam";
              startPhase(P_WATCH, in.nowMs, targetHeading_);
            } else if (stepM_ >= kToRampMaxM) {
              note_ = "Ramp not felt, going on";
              finishStep(in.nowMs);                    // the ramp is only an indication, not a fault
            }
            break;
          }
        }
        break;

      case CROSS_RAMP:   // 15: straight beside the wall of I, stand still, then on to the small pillar
        switch (phase_) {
          case P_SETTLE:   // the gyro settles best when the rover does not move
            speed_ = left_ = right_ = 0;
            if (in.nowMs - phaseMs_ < kSettleMs) break;
            // The rover did not move here, so any heading change is the IMU settling. It is added to
            // the wall of I measurement, which was taken while driving beside it.
            if (kUseIWallRef) {
              resetDoorDirection(iWallDoorDeg_ + wrap180(in.headingDeg - settleHeading_), kWallRefMaxDeg);
            }
            note_ = "Looking for the small pillar";
            startPhase(P_PILLAR, in.nowMs, startHeading_);
            break;

          case P_WATCH:   // standing still in front of something: is it the small pillar?
            if (watchFront(in, dt, kPillarFrontDeg, kSmallPillarSeenCm, kSmallPillarHoldMs)) {
              finishStep(in.nowMs);        // step 16 turns right
            } else if (watchGone_) {
              startPhase(P_PILLAR, in.nowMs, startHeading_);   // it walked away, or it is not the pillar
            }
            break;

          case P_PILLAR:
            driveStraight(in, dt);
            if (smallFilter_.update(somethingFront(kSmallPillarSeenCm, kPillarFrontDeg), in.nowMs)) {
              note_ = "Checking the small pillar";
              startPhase(P_WATCH, in.nowMs, targetHeading_);
            } else if (stepM_ >= kRampClearM + kSmallPillarMaxM) {
              fail("Small pillar not found");
            }
            break;

          default:   // P_DRIVE: straight over the ramp, measuring the wall of I on the left
            driveStraight(in, dt);
            if (wallLeft(kIWallMinCm, kIWallMaxCm)) watchSide(in, 270, kIWallRayMaxCm);
            if (reached(kRampClearM)) {
              iWallDoorDeg_ = lineHeading(true);   // the rover drives along this wall
              settleHeading_ = in.headingDeg;
              note_ = "Standing still, IMU settling";
              startPhase(P_SETTLE, in.nowMs, targetHeading_);
            }
            break;
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
        // Wall K is caught from much further out than the other walls: after the turn in step 20
        // the rover can be 1.5-2.5 m away from it, and the usual kFollowWallMaxCm would never see it.
        if (wallLeft(kLidarMinCm, kKWallCatchCm)) keepWallDistance(in, wrap360(startHeading_ + 90.0f), kKWallNearCm, kKWallFarCm);
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
        if (smallFilter_.update(somethingFront(kLastPillarStopCm, kPillarFrontDeg), in.nowMs)) {
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
  float lastResetDeg() const { return lastResetDeg_; }   // last change of the door direction
  float pitchDeg() const { return pitchDeg_; }           // newest IMU pitch, for the CSV
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
      case FIND_PANEL: return "Looking for the panel";
      case TO_RAMP: return "To the ramp, beam check";
      case CROSS_RAMP: return "Over the ramp to the pillar";
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
  // So close to the rover's front that it must not roll any further.
  bool veryCloseInFront() const {
    return hits(0, kFrontConeDeg, kLidarMinCm, kObstacleHardStopCm + kLidarToFrontCm) >= kObstacleMinHits;
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
  // fanDeg: how far to each side it looks. Wide for the beam, narrow for the small pillar, because
  // a wide fan at 230 cm also sees things more than a metre to the side.
  bool somethingFront(int maxCm, int fanDeg) const { return hits(0, fanDeg, kLidarMinCm, maxCm) >= kSmallMinHits; }

  
  // ---- Steps 4, 5, 12, 21, 22: drive parallel to the wall on the left ----
  // Average distance in cm around one direction. 0 if fewer than 3 good readings, or if they
  // do not agree: then the ray is at the end of a wall and half sees what is behind it.
  float rayCm(int centerDeg, int maxCm = kWallRayMaxCm) const {
    float sum = 0;
    int count = 0, lowest = 10000, highest = 0;
    for (int angle = centerDeg - 2; angle <= centerDeg + 2; ++angle) {
      const int cm = lidar_.distanceMm(angle) / 10;
      if (cm >= kLidarMinCm && cm <= maxCm) {
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

  // ---- Reset the door direction from a flat thing beside the rover ----
  // Adds the side distance now (270 = left, 90 = right) to the line, if the reading is good.
  void watchSide(const Input& in, int sideDeg, int maxCm = kWallRayMaxCm) {
    const float cm = rayCm(sideDeg, maxCm);
    if (cm > 0) line_.add(cm, stepM_, wrap180(in.headingDeg - startHeading_));
  }

  // The compass direction of the thing the rover drove past. A thing on the left that gets
  // farther away means the rover points to the right of it; on the right it is the other way.
  float lineHeading(bool onLeft) const {
    if (line_.lengthM() < kRefMinLenM) return -1000;   // not seen long enough
    const float open = line_.openDeg();
    return startHeading_ + line_.headingDeg() + (onLeft ? -open : open);
  }

  // Sets a new door direction, unless the change is too big to believe. Every later turn and
  // the straight line of this step use it.
  void resetDoorDirection(float doorDeg, float maxChangeDeg) {
    if (doorDeg < -500) return;   // lineHeading() had no answer
    const float change = wrap180(doorDeg - startHeading_);
    if (fabsf(change) > maxChangeDeg) return;
    startHeading_ = wrap360(startHeading_ + change);
    targetHeading_ = wrap360(targetHeading_ + change);
    lastResetDeg_ = change;
  }

  // ---- Looking at something in front while standing still ----
  // The closest reading in the front fan, or 0 when nothing is seen there.
  int frontMinCm(int halfDeg) const {
    int best = 0;
    for (int angle = -halfDeg; angle <= halfDeg; ++angle) {
      const int cm = lidar_.distanceMm(angle) / 10;
      if (cm >= kLidarMinCm && (best == 0 || cm < best)) best = cm;
    }
    return best;
  }

  // Slows to a stop and then looks at the thing in front. It is only believed when it stays at the
  // same distance and the same width for holdMs while the rover stands still. That is what tells a
  // beam or a pillar from a person: a person moves, or walks away (then watchGone_ becomes true).
  bool watchFront(const Input& in, float dt, int fanDeg, int nearCm, uint32_t holdMs) {
    speed_ = max(speed_ - kRampPerSec * dt, 0.0f);   // soft stop, easy on the gearbox
    left_ = speed_;
    right_ = speed_ * kRightTrim;
    const int cm = frontMinCm(fanDeg);
    const int count = hits(0, fanDeg, kLidarMinCm, nearCm);
    if (cm == 0 || cm > nearCm || count < kSmallMinHits) {  // not in this reading
      if (goneSinceMs_ == 0) goneSinceMs_ = in.nowMs;
      // One dropped reading happens often, so it only counts as gone after kFrontGoneMs.
      watchGone_ = in.nowMs - goneSinceMs_ >= kFrontGoneMs;
      return false;
    }
    goneSinceMs_ = 0;
    if (speed_ > 0) {                                // still rolling: start again when it stands
      holdSinceMs_ = in.nowMs;
      holdCm_ = cm;
      return false;
    }
    if (abs(cm - holdCm_) > kFrontMoveCm) {          // it moved: start the time again
      holdSinceMs_ = in.nowMs;
      holdCm_ = cm;
    }
    // If it never stands still, the rover gives up instead of waiting for ever.
    if (in.nowMs - phaseMs_ >= kWatchMaxMs) {
      watchGone_ = true;
      return false;
    }
    return in.nowMs - holdSinceMs_ >= holdMs;
  }

  // ---- Moving ----
  // Stops at once for an obstacle. It stays stopped until the front is clear for kClearMs.
  // ---- The ramp, felt with the IMU ----
  // flatPitchDeg_ is the pitch on the flat ground where step 14 started. The nose first tilts by
  // kRampPitchDeg (up or down), and the ramp counts as finished once the pitch is back inside
  // kRampFlatDeg for kRampFlatMs. Nothing fails if it never happens: the step ends on distance.
  bool rampFinished(const Input& in) {
    const float tilt = fabsf(in.pitchDeg - flatPitchDeg_);
    if (tilt >= kRampPitchDeg) {
      onRamp_ = true;
      levelSinceMs_ = in.nowMs;
      note_ = "On the ramp";
      return false;
    }
    if (!onRamp_) return false;
    if (tilt > kRampFlatDeg) {
      levelSinceMs_ = in.nowMs;      // still tilted
      return false;
    }
    return in.nowMs - levelSinceMs_ >= kRampFlatMs;
  }

  // ---- Picked up off the ground ----
  // On the ground the rover never leans more than a few degrees, even on the ramp (measured: 8).
  // More than kLiftedDeg means somebody is holding it, so the wheels are cut at once and nothing
  // else happens until it is back down and flat for kBackDownMs. The step, the distance and the
  // direction are all kept, and every clock is pushed forward by the time it spent in the air,
  // so a long hold cannot run a turn or a step into its timeout.
  bool liftedOffGround(const Input& in, float dt) {
    const bool tilted = fabsf(in.pitchDeg) > kLiftedDeg || fabsf(in.rollDeg) > kLiftedDeg;
    if (tilted) {
      lifted_ = true;
      flatSinceMs_ = in.nowMs;
    } else if (lifted_ && in.nowMs - flatSinceMs_ >= kBackDownMs) {
      lifted_ = false;
      note_ = "Back on the ground";
    }
    if (!lifted_) return false;

    note_ = "Lifted: put me down";
    eStop(dt, true);                          // the wheels must not spin in the air
    const uint32_t held = lroundf(dt * 1000);   // this turn of the loop did not happen for the step
    stepStartMs_ += held;
    turnStartMs_ += held;
    phaseMs_ += held;
    clearSinceMs_ += held;
    headingBadSinceMs_ += held;
    holdSinceMs_ += held;
    pauseStartMs_ += held;
    return true;
  }

  // ---- Braking ----
  // Driving and turning come down together, keeping the same share, so a turn does not end as
  // a turn on the spot. change is how much power is taken away in this step.
  // True when the rover stands still.
  bool brake(float change) {
    const float now = fabsf(speed_) + fabsf(spin_);
    const float scale = now > change ? (now - change) / now : 0.0f;
    speed_ *= scale;
    spin_ *= scale;
    left_ = speed_ + spin_;
    right_ = (speed_ - spin_) * kRightTrim;
    return speed_ == 0 && spin_ == 0;
  }

  // The emergency stop: STOP pressed, a fault, or something in the way. Slower and gentler
  // than the end of a turn. hard = true cuts the power at once, when there is no room to roll.
  bool eStop(float dt, bool hard) { return brake(hard ? 1000.0f : kEStopRampPerSec * dt); }

  // Something in the way: brake to a stop and wait until the way is free for kClearMs.
  // Closer than kObstacleHardStopCm there is no room to roll, so the power is cut at once.
  bool waitForObstacle(const Input& in, float dt) {
    if (obstacleInFront()) {
      blocked_ = true;
      clearSinceMs_ = in.nowMs;
    } else if (blocked_ && in.nowMs - clearSinceMs_ >= kClearMs) {
      blocked_ = false;
    }
    if (blocked_) eStop(dt, veryCloseInFront());   // no distance is counted while it waits
    return blocked_;
  }

  void driveStraight(const Input& in, float dt) {
    if (waitForObstacle(in, dt)) return;

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

  // ---- One turn ----
  // A turn always uses the same total power, kTurnTotal. Only the share changes, and the share
  // is the curve:
  //     forward part  drives the rover ahead      inner wheels = forward - turn
  //     turn part     turns it                    outer wheels = forward + turn
  // kTurnForward = 0 turns on the spot. Bigger = a longer, rounder curve.
  bool turnFinished(const Input& in, float dt) {
    const float error = wrap180(targetHeading_ - in.headingDeg);   // + = turn right

    if (in.nowMs - turnStartMs_ > kTurnTimeoutMs) {
      fail("Turn took too long");
      return false;
    }
    if (fabsf(error) > kTurnWrongWayDeg) {
      fail("Turn went the wrong way");
      return false;
    }
    // On the new direction. The stop has its own rate: the rover turns fast right up to
    // here, so a slow stop would carry it past the target and it would have to come back.
    if (fabsf(error) <= kTurnToleranceDeg) return brake(kTurnBrakePerSec * dt);

    // Easing off. Over the last kTurnSlowDeg the power comes down with the angle that is left,
    // so the rover arrives slowly and the stop is soft. It never goes under kTurnSlowPart of
    // the power, or the wheels would be too weak to turn at all.
    const float ease = max(min(fabsf(error) / kTurnSlowDeg, 1.0f), kTurnSlowPart);
    const float total = kTurnTotal * ease;

    // How much of the power drives forward. Three cases, nothing else.
    float forward = kTurnForward * ease;
    if (in.nowMs - turnStartMs_ < kTurnRollMs) forward = total / 2;   // rolling start: the
                            // widest curve, so the inner wheels only stop and never go backwards
    if (obstacleInFront()) forward = 0;        // no room ahead: finish the turn on the spot

    // Both parts move in small steps, so the gearbox never gets a kick.
    const float step = kTurnRampPerSec * dt;
    const float turn = total - forward;                        // what is left does the turning
    speed_ = constrain(forward, speed_ - step, speed_ + step);
    spin_ = constrain(error > 0 ? turn : -turn, spin_ - step, spin_ + step);

    const float metres = kRealSpeedMps * (speed_ / kDriveSpeed) * dt;
    stepM_ += metres;
    totalM_ += metres;
    left_ = speed_ + spin_;
    right_ = (speed_ - spin_) * kRightTrim;
    return false;
  }

  // ---- Phases inside one step (steps 14 and 15) ----
  // A phase gets its own target heading, start time and start distance. The turn timeout and
  // the turn pulses count from the start of the phase, not from the start of the step.
  void startPhase(uint8_t next, uint32_t nowMs, float targetDeg) {
    phase_ = next;
    phaseM_ = stepM_;
    phaseMs_ = nowMs;
    turnStartMs_ = nowMs;
    targetHeading_ = targetDeg;
    headingBadSinceMs_ = nowMs;   // a new direction on purpose is not "lost the straight line"
    spin_ = 0;              // every turn starts from standing still
    watchGone_ = false;
    goneSinceMs_ = 0;
  }

  // ---- Changing steps ----
  void finishStep(uint32_t nowMs) {
    noteStepEnd(nowMs, "done");
    nextStep_ = step_ >= lastStep_ ? DONE : static_cast<Step>(step_ + 1);
    pausing_ = true;
    pauseStartMs_ = nowMs;
    blocked_ = false;   // a step can end next to a wall; the next step checks again
  }

  // Ends the step and then goes to a chosen step, not the next one (the beam detour).
  void finishStepTo(uint32_t nowMs, Step next) {
    noteStepEnd(nowMs, "done");
    nextStep_ = (step_ >= lastStep_ || next > lastStep_) ? DONE : next;
    pausing_ = true;
    pauseStartMs_ = nowMs;
    blocked_ = false;
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
    stepStartMs_ = headingBadSinceMs_ = nowMs;
    stepM_ = 0;
    spin_ = 0;   // every turn starts from standing still

    // Every step starts fresh: nothing seen yet.
    found_ = 0;
    foundAtM_ = -1;
    line_.reset();
    refDone_ = false;
    watchGone_ = false;
    goneSinceMs_ = 0;
    holdSinceMs_ = nowMs;
    holdCm_ = 0;
    phase_ = P_DRIVE;
    phaseM_ = 0;
    phaseMs_ = nowMs;
    turnStartMs_ = nowMs;
    detourM_ = 0;
    note_ = "Driving";
    settleHeading_ = -1000;
    lastWallM_ = 0;
    pillarFilter_.reset(nowMs);
    wallFilter_.reset(nowMs);
    panelFilter_.reset(nowMs);
    endWallFilter_.reset(nowMs);
    smallFilter_.reset(nowMs);

    // Turns: the new direction, counted from startHeading_ (the way the rover faced at the door).
    if (next == TURN_LEFT) targetHeading_ = wrap360(startHeading_ - 90.0f);
    // Steps 13-15 drive straight in the door direction, so the wall following of step 12
    // cannot leave a crooked heading behind.
    if (next == FIND_PANEL || next == TO_RAMP || next == CROSS_RAMP) targetHeading_ = startHeading_;
    if (next == TO_RAMP) {
      flatPitchDeg_ = pitchDeg_;   // the ground here is the flat the ramp is measured against
      onRamp_ = false;
    }
    if (next == TURN_RIGHT || next == TURN_LEFT_AT_E || next == TURN_LEFT_TO_G) {
      targetHeading_ = startHeading_;                       // the same way as at the door
    }
    if (next == TURN_RIGHT_AT_E || next == TURN_RIGHT_AT_PILLAR || next == TURN_RIGHT_AT_G) {
      targetHeading_ = wrap360(startHeading_ + 90.0f);      // 90 to the right of that
    }
    if (next == TURN_RIGHT_LAST) targetHeading_ = wrap360(startHeading_ + 180.0f);   // back the other way
  }

  // Something went wrong: the run stops here. The wheels are not cut to 0: update() brakes
  // them with eStop() on the next turn of the loop.
  void fail(const char* why) {
    if (!pausing_) noteStepEnd(lastMs_, why);
    step_ = FAULT;
    note_ = why;
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
  bool hardStop_ = false;    // STOP pressed again while braking: cut the power at once
  bool lifted_ = false;      // the rover is off the ground, in somebody's hands
  bool onRamp_ = false;      // step 14: the nose has tilted, the rover is on the ramp
  float flatPitchDeg_ = 0;   // the pitch on the flat ground before the ramp
  uint32_t levelSinceMs_ = 0;  // since when the nose has been level again
  uint32_t flatSinceMs_ = 0; // since when it has been flat again
  const char* note_ = "Press Start";
  float startHeading_ = 0, targetHeading_ = 0;
  float speed_ = 0, pull_ = 0, left_ = 0, right_ = 0;
  float spin_ = 0;   // turn power now, + = turn right; it ramps (turnFinished, eStop)
  SideLine line_;            // the wall or panel beside the rover in this step (steps 5, 7, 9)
  bool refDone_ = false;     // the door direction was already reset in this step
  float lastResetDeg_ = 0;   // the last change of the door direction, for testing
  float pitchDeg_ = 0;       // newest IMU pitch, kept only so the CSV can show it
  float stepM_ = 0, totalM_ = 0;
  float lastPillarM_ = 0;   // where the last pillar was counted, in metres of step 4
  float lastWallM_ = 0;     // where the wall we follow was last seen, in metres of this step (12, 22)
  float foundAtM_ = -1;     // step 9: where the pillar was seen, -1 = not yet
  int found_ = 0;           // things found in this step: bench legs (13), pillars (21), wall G (22)
  int pillars_ = 0, walls_ = 0, panels_ = 0;
  bool blocked_ = false, pausing_ = false;
  // Steps 13 to 20
  bool beamDone_ = false;    // the beam is behind the rover: it is never looked for again
  bool watchGone_ = false;   // the thing in front went away by itself
  uint8_t phase_ = P_DRIVE;  // which part of step 14 or 15 runs now
  float phaseM_ = 0;         // step metres when this phase began
  float detourM_ = 0;        // sideways metres driven around the beam
  uint32_t phaseMs_ = 0, turnStartMs_ = 0;
  uint32_t goneSinceMs_ = 0; // since when the thing in front was not in the reading
  uint32_t holdSinceMs_ = 0; // since when it stands at the same distance
  int holdCm_ = 0;           // how far it stood then
  float iWallDoorDeg_ = -1000;   // door direction measured from the wall of I (step 18)
  float settleHeading_ = -1000;  // heading when the rover stopped in step 19
  uint32_t lastMs_ = 0, stepStartMs_ = 0, pauseStartMs_ = 0;
  uint32_t clearSinceMs_ = 0, headingBadSinceMs_ = 0;
};
