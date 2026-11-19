#pragma once
#include <Arduino.h>

// All the numbers you may want to change are in this file.
// MEASURE means the value is only a first guess until you test it on the rover.

// ---- Test one step at a time: the rover stops after this step ----
// 1 forward 0.90 m       2 turn left 90
// 3 forward, no LiDAR    4 count 5 pillars and 2 walls
// 5 forward after C      6 turn right 90
// 7 drive to wall E      8 turn right 90 at wall E
// 9 along E to the pillar, +0.8 m      10 turn left 90
// 11 find the side wall of E           12 along it, then on to the fire hydrant
// 13 the panel on the left                14 straight on, the wall of I says the ramp is here
//                                            (the beam, if it blocks: left, 50 cm, right)
// 15 straight beside the wall of I, stand still, then stop at the small pillar
// 16 turn right 90                     17 forward 1.3 m
// 18 turn left 90                      19 stop at wall G in front
// 20 turn right 90                     21 along wall K, 2 pillars right
// 22 past the end of K, wall G left    23 turn right 90
// 24 stop at the last pillar: mission finished

constexpr int kLastStep = 24;              // the whole route; a test group can stop earlier

// ---- Driving ----
constexpr float kDriveSpeed = 0.30f;    // was 0.30f: the weak left side needs more power
constexpr float kRightTrim  = 1.0f;    // 0.18f; was 0.20f: outdoors it pulls left
constexpr float kRampPerSec = 0.20f;    //0.3 was 0.20f: start and stop still take 1.5 s
constexpr float kRealSpeedMps = 0.27f;          // MEASURE: real speed at kDriveSpeed
constexpr uint32_t kPauseMs = 1000;             // stop time between two steps

// ---- Route distances, in metres ----
constexpr float kFirstForwardM =  0.90f;         // or 0.98f, see above was 0.90f, only for this test out of the door
constexpr float kNoLidarForwardM = 0.10f;        // was 0.50f: pillar 1 is right after the turn
constexpr float kAfterPillarsM = 1.20f;         // MEASURE: your map label says 6.66 m
constexpr float kCountMaxM = 40.0f;             // stop if the pillars are not found
constexpr float kToEndMinM = 20.0f;             // MEASURE: right turn to wall E, minus 3 m (0 = off).
                                                // 20 for now: in run openSee, step 7 drove 25 m and wall E was not reached yet
constexpr float kToEndMaxM = 50.0f;             // stop if wall E is not found

// ---- Hold a straight line, values from Robot_8 ----
constexpr float kHeadingGain = 0.05f;           // wheel command per degree of error
constexpr float kHeadingDeadDeg = 1.0f;         // ignore errors smaller than this
constexpr float kMaxCorrection = 0.18f;         // largest steering command
constexpr float kPullGain = 0.01f;              // was 0.004f: learns a steady pull to one side faster
constexpr float kMaxPull = 0.15f;               // was 0.10f: the ramp needed about 0.13 (run-group2)
constexpr float kMaxHeadingErrorDeg = 25.0f;    // lost the line: stop the run
constexpr uint32_t kHeadingErrorMs = 1500;      // ...if it lasts this long

// ---- Reset the door direction from a flat thing beside the rover (SideLine in Route.hpp) ----
// The rover drives past it and watches the side distance. If the distance changes, the rover
// is not parallel, and the door direction is corrected by that angle.
constexpr bool kUseWallCRef = true;             // step 5: wall C on the left, before the right turn
constexpr bool kUsePanelRef = false;            // step 7: the panel on the right (short, so less exact: test it first)
constexpr bool kUseWallERef = true;             // step 9: wall E on the left
constexpr float kRefMinLenM = 0.5f;             // the thing must be seen for at least this far
constexpr float kWallRefMaxDeg = 12.0f;         // a bigger correction from a wall is not believed
constexpr float kPanelRefMaxDeg = 6.0f;         // ...from the short panel
constexpr int kPanelRefMinCm = 80;              // the panel is between these two on the right
constexpr int kPanelRefMaxCm = 220;             // (your measure: about 150)
constexpr float kPanelRefEndM = 3.0f;           // look for the panel only this far into step 7
constexpr float kWallERefM = 2.5f;              // step 9: measure wall E over this far, then correct

// ---- Turning: a short curve, not a turn on the spot ----
constexpr float kTurnPower = 0.42f;             // how hard the two sides push against each other
// THE KNOB for the turn: how much forward driving is mixed into it. 0 = turn on the spot,
// bigger = a longer curve. The circle the rover drives is about
//     1.3 x kTurnForward / kTurnPower  metres wide (measured from run 17):
//   0.00 with power 0.50 -> on the spot, like before
//   0.08 with power 0.42 -> short curve, about 25 cm      <-- start here
//   0.15 with power 0.35 -> about 55 cm
//   0.25 with power 0.25 -> about 130 cm
// Keep kTurnPower + kTurnForward at 0.50 or below: that is all the rover gives.
constexpr float kTurnForward = 0.08f;
constexpr float kTurnPulseDeg = 10.0f;          // 20 optional, was 35.0f: faster turn outdoors
constexpr uint32_t kTurnPulseOnMs = 300;        // was 100: a longer push breaks the friction
constexpr float kTurnRampPerSec = 2.5f;         // turn power 0 to 0.5 in 0.2 s (0.5 / 2.5)
constexpr uint32_t kTurnPulseOffMs = 200;       // was 300: shorter wait between pushes
constexpr float kTurnToleranceDeg = 4.0f;       // turn is finished inside this error
constexpr uint32_t kTurnHoldMs = 300;           // ...for this long
constexpr uint32_t kTurnTimeoutMs = 15000;      // a curve turn on a tired battery needs about 10 s
constexpr float kTurnWrongWayDeg = 115.0f;      // error got bigger: wrong direction

// ---- LiDAR directions: 0 = front, 90 = right, 180 = back, 270 = left ----
constexpr int kLidarAngleShift = 180;           // raw LD20 angle + this = rover angle
constexpr int kLidarMinCm = 20;                 // LidarScan ignores anything closer

// Obstacle in front: stop and wait until it moves away
constexpr int kObstacleStopCm = 25;             // measured from the front of the rover
constexpr int kLidarToFrontCm = 15;             // MEASURE: LiDAR centre to rover front
constexpr int kFrontConeDeg = 30;               // look this far to each side
constexpr int kObstacleMinHits = 3;             // directions that must see it
constexpr uint32_t kClearMs = 1000;             // front must stay clear this long
// Emergency stop: STOP pressed, a fault, or something in the way. The wheels go down to 0
// at this rate instead of being cut off, so the gearbox gets no hard kick. 1.0 = 0.5 to 0
// in half a second. Smaller = softer but it rolls further.
constexpr float kEStopRampPerSec = 1.0f;        // wheel command per second while braking
constexpr bool kEStopHardOnSecondStop = true;   // STOP pressed again while braking: 0 at once
constexpr int kObstacleHardStopCm = 12;         // obstacle closer than this to the rover front: 0 at once

// Wall on the left (buildings B and C)
constexpr int kWallMaxCm = 200;                 // MEASURE
constexpr int kWallFanDeg = 10;
constexpr int kWallMinHits = 10;
constexpr uint32_t kWallSeenMs = 500;
constexpr uint32_t kWallGoneMs = 1000;

// Steps 4-5: drive parallel to wall B/C. Distances are LiDAR centre to wall, like "Left" on the page.
constexpr int kWallNearCm = 110;                // never closer than this
constexpr int kWallFarCm = 140;                 // never farther; the rover aims at the middle (125)
constexpr float kWallGain = 0.3f;               // degrees toward the wall for each cm too far
constexpr float kWallMaxSteerDeg = 15.0f;       // largest angle toward or away from the wall
constexpr int kWallRayDeg = 30;                 // two rays, 30 deg in front of and behind the left side
constexpr int kWallRayMaxCm = 300;              // farther readings are not used
constexpr int kWallRaySpreadCm = 15;            // one ray's readings must agree this well (not a wall end)
constexpr float kWallMaxAngleDeg = 20.0f;       // wall more than this off the path: a door or a corner

// Pillar on the right
constexpr int kPillarMinCm = 90;           // was 50: a person at 87 cm gave 17 hits
constexpr int kPillarMaxCm = 180;          // was 250: pillar 1 is at about 120 cm
constexpr int kEPillarsNeeded = 2; 
            // count this many pillars along wall E (they are about 4 m apart)
constexpr int kPillarFanDeg = 8;
constexpr int kPillarMinHits = 6;          // was 3: one leg of a person gives about 4-5
constexpr uint32_t kPillarSeenMs = 800;     // was 150: a person walking past is seen for less
constexpr uint32_t kPillarGoneMs = 2000;    // was 1000: a person close to a pillar joins its hill
constexpr float kPillarMinGapM = 3.0f;      // NEW: pillars are about 4.6-5.2 m apart
constexpr int kPillarsNeeded = 5;
constexpr int kWallsNeeded = 2;

// End of the first route: wall E in front, no panel needed
constexpr int kEndWallStopCm = 80;              // stop when wall E is this close to the rover front
constexpr int kEndWallFanDeg = 20;              // look 20 deg to each side: a wall is wide
constexpr int kEndWallMinHits = 30;             // 30 of the 41 directions
constexpr uint32_t kEndWallSeenMs = 300;        // must stay seen this long
constexpr float kAfterEWallEndM = 0.40f;        // spare rule: stop this far after the end of wall E

// Panel on the right: counted for the page only
constexpr int kPanelMinCm = 40;
constexpr int kPanelMaxCm = 150;
constexpr int kPanelFanDeg = 15;
constexpr int kPanelMinHits = 5;
constexpr uint32_t kPanelSeenMs = 300;
constexpr uint32_t kPanelGoneMs = 1000;

// ---- Steps 9 to 24 ----
// All distances in cm are from the LiDAR centre: the Front / Left / Right numbers on the page
// (front_cm, left_cm, right_cm in the CSV). Your measures are in the comments; most ranges
// have 15-20 cm margin on each side, because the rover is never in exactly the same place.

// How a small thing (fire hydrant, bench leg, small pillar) is seen
constexpr int kSmallFanDeg = 8;                 // look 8 deg to each side
constexpr int kSmallMinHits = 2;                // a bench leg fills only 2-3 directions
constexpr uint32_t kSmallSeenMs = 200;          // must stay seen this long
constexpr uint32_t kSmallGoneMs = 500;          // must be gone this long before the next one counts
constexpr int kFarPillarMinHits = 4;            // steps 9, 21: a pillar 2-2.5 m away fills fewer directions
constexpr int kFollowWallMaxCm = 120;           // steps 12, 21, 22: the wall we follow counts as there

// Step 9: along wall E until a pillar on the right, then a little more
constexpr int kEPillarMinCm = 165;              // your measure: about 220
constexpr int kEPillarMaxCm = 205;
constexpr float kAlongEMinM = 0.0f;             // look for the pillar only after this far (0 = at once)
constexpr float kAfterEPillarM = 0.80f;         // then drive this far and stop
constexpr float kAlongEMaxM = 15.0f;            // safety: stop the run if the pillar is not found

// Step 11: the side wall of E on the left
constexpr int kESideFindMaxCm = 130;            // your measure: 80, but after step 9 the wall
                                                // can be 95 cm away
constexpr float kFindESideMaxM = 5.0f;          // safety: stop the run if the wall is not found

// Step 12: along the side wall of E; after its end, on to the fire hydrant
constexpr int kESideNearCm = 60;                // follow the wall between these two
constexpr int kESideFarCm = 80;
constexpr float kESideGoneM = 0.5f;             // the wall must be gone this far to count as ended
constexpr int kHydrantMinCm = 115;              // your measure: 130-140
constexpr int kHydrantMaxCm = 170;
constexpr float kHydrantMaxM = 6.0f;            // safety: after the wall end (your measure: about 3.5 m)
constexpr float kESideMaxM = 40.0f;             // safety: stop the run if the wall never ends

// ---- Steps 13 to 20: panel, beam, ramp, settle, small pillar ----
// Step 13: the panel on the left. It only says that the ramp comes next.
constexpr int kPanelLeftMinCm = 185;            // your measure: 190-250
constexpr int kPanelLeftMaxCm = 260;
constexpr float kPanelMaxM = 15.0f;             // safety

// Step 14: straight on until the wall of I appears on the left: then the rover is at the ramp.
constexpr float kIWallSeenM = 2.00f;            // the wall of I must stay in sight this far, so the
                                                // short panel is not taken for it
constexpr float kToRampMaxM = 8.0f;             // safety: no wall of I within this far

// The beam at the ramp entrance (your measure: about 40 cm wide, on the right).
// The rover stops in front of it and looks at it while standing still, so a person is not taken
// for the beam: the beam keeps the same distance, a person moves or walks away.
constexpr int kBeamStopCm = 100;                // stop when something in front is this close
constexpr uint32_t kBeamHoldMs = 1500;          // it must stand at the same distance this long
constexpr float kBeamSideStepM = 0.50f;         // the detour: turn left, this far forward, turn right

// Both front checks (the beam and the small pillar)
constexpr uint32_t kFrontGoneMs = 500;          // the thing only counts as gone after this long
constexpr int kFrontMoveCm = 15;                // it may move this much and still count as standing
constexpr uint32_t kWatchMaxMs = 5000;          // if it never stands still, the rover drives on
constexpr int kPillarFrontDeg = 10;             // narrow fan for the small pillar: at 230 cm it reaches
                                                // 40 cm to each side, so a rail beside the rover is ignored

// Step 15: straight beside the wall of building I
constexpr int kIWallMinCm = 190;                // your measure: 260-300. Wide enough to still find the
constexpr int kIWallMaxCm = 340;                // wall after a 50 cm step around the beam.
constexpr int kIWallRayMaxCm = 400;             // the wall of I is farther than kWallRayMaxCm
constexpr float kRampClearM = 5.00f;            // MEASURE: from the wall of I until fully off the ramp
constexpr bool kUseIWallRef = true;             // take the straight line for the next steps from this wall

// Step 15, after the ramp: stand still so the IMU settles, then correct from the wall of I
constexpr uint32_t kSettleMs = 3000;

// Step 15, last part: the small pillar in front, then stop
constexpr int kSmallPillarSeenCm = 230;         // your measure: stop about this far in front of it
constexpr uint32_t kSmallPillarHoldMs = 1500;   // it must stand still this long (a person walks away)
constexpr float kSmallPillarMaxM = 15.0f;       // safety

// Step 17: forward between the turns at the small pillar
constexpr float kAfterSmallPillarM = 1.30f;     // your measure: 120-140 cm

// Step 19: wall G in front, then stop
constexpr int kGStopCm = 210;                   // your measure: 180-190; it stops about 20 cm closer than this
constexpr float kToGMaxM = 30.0f;               // safety

// Step 21: along wall K on the left, count two pillars on the right
constexpr int kKWallNearCm = 80;                // follow wall K between these two (your measure: 80-90)
constexpr int kKWallFarCm = 90;
constexpr int kKPillarMinCm = 225;              // your measure: 240-250
constexpr int kKPillarMaxCm = 265;
constexpr int kKPillarsNeeded = 2;
constexpr float kAlongKMaxM = 30.0f;            // safety

// Step 22: past the end of wall K, wall G on the left, then stop
constexpr int kGLeftMinCm = 160;                // your measure: about 180
constexpr int kGLeftMaxCm = 200;
constexpr float kAfterKWallM = 2.30f;           // stop this far after the end of wall K
constexpr float kPastKMaxM = 15.0f;             // safety

// Step 24: the last pillar in front, then stop: mission finished
constexpr int kLastPillarStopCm = 55;           // your measure: about 30 from the rover front = 45 on the page;
                                                // it stops about 10 cm closer than this
constexpr float kLastPillarMaxM = 15.0f;        // safety

// ---- Test groups: one button each on the test page ----
// The rover drives only these steps and then stops. Put it where the first step starts,
// facing the way that step drives. facingDeg = that way, compared with the door direction:
// 0 = the same way as at the door, 90 = 90 deg to the right of it.
struct TestGroup {
  const char* name;
  int firstStep, lastStep;
  float facingDeg;
};
constexpr TestGroup kTestGroups[] = {
  {"B building to E building", 1, 8, 0},                // at the door
  {"E building to I building", 9, 15, 90},              // at wall E, facing along it
  {"I building to K building", 16, 21, 0},              // in front of the small pillar
  {"K building to H building, end of mission", 22, 24, 90},  // next to wall K, facing along it
};
constexpr int kTestGroupCount = sizeof(kTestGroups) / sizeof(kTestGroups[0]);

// Checked when you build: these numbers must fit together.
static_assert(kEndWallStopCm > kObstacleStopCm, "wall E: the obstacle stop would come first");
static_assert(kSmallPillarSeenCm > kObstacleStopCm + kLidarToFrontCm, "step 15: the obstacle stop would come first");
static_assert(kBeamStopCm > kObstacleStopCm + kLidarToFrontCm, "the beam check must come before the obstacle stop");
static_assert(kLastPillarStopCm > kObstacleStopCm + kLidarToFrontCm, "step 24: the obstacle stop would come first");
static_assert(kTurnPulseOnMs > 1000.0f * kTurnPower / kTurnRampPerSec, "a turn push must reach full power: make kTurnPulseOnMs longer or kTurnRampPerSec bigger");
static_assert(kTurnPower + kTurnForward <= 0.5f, "the turn asks the wheels for more than the rover gives: lower kTurnPower or kTurnForward");
static_assert(kEStopRampPerSec > kRampPerSec, "the emergency stop must brake faster than the normal stop");
static_assert(kObstacleHardStopCm < kObstacleStopCm, "the hard stop must be closer than the normal obstacle stop");

