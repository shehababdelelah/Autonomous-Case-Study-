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
// 13 two bench legs, then the panel    14 find the wall of I
// 15 stop at the small pillar          16 turn right 90
// 17 forward 1.3 m                     18 turn left 90
// 19 stop at wall G in front           20 turn right 90
// 21 along wall K, 2 pillars right     22 past the end of K, wall G left
// 23 turn right 90                     24 stop at the last pillar: mission finished

constexpr int kLastStep = 15;              // next test: new wall E rule (step 7) + right turn at E (step 8)

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
constexpr float kMaxPull = 0.10f;               // was 0.03f: this rover needed 0.08 (CSV drifitingleft)
constexpr float kMaxHeadingErrorDeg = 25.0f;    // lost the line: stop the run
constexpr uint32_t kHeadingErrorMs = 1500;      // ...if it lasts this long

// ---- Turn on the spot, values from Robot_8 ----
constexpr float kTurnPower = 0.50f;             // below this the rover barely turns
constexpr float kTurnPulseDeg = 15.0f;          // 20 optional, was 35.0f: faster turn outdoors
constexpr uint32_t kTurnPulseOnMs = 200;        // was 100: a longer push breaks the friction
constexpr uint32_t kTurnPulseOffMs = 200;       // was 300: shorter wait between pushes
constexpr float kTurnToleranceDeg = 4.0f;       // turn is finished inside this error
constexpr uint32_t kTurnHoldMs = 300;           // ...for this long
constexpr uint32_t kTurnTimeoutMs = 20000;
constexpr float kTurnWrongWayDeg = 115.0f;      // error got bigger: wrong direction

// Turn as a curve (false = the old turn on the spot above)
constexpr bool kCurveTurns = false;
constexpr float kTurnSpeed = 0.30f;             // forward part of the power; it ramps up like driving
constexpr float kTurnSharpness = 0.6f;          // outer side = speed x 1.6, inner side = speed x 0.4
constexpr float kTurnSlowDeg = 15.0f;           // in the last 15 deg the curve opens up

// ---- LiDAR directions: 0 = front, 90 = right, 180 = back, 270 = left ----
constexpr int kLidarAngleShift = 180;           // raw LD20 angle + this = rover angle
constexpr int kLidarMinCm = 20;                 // LidarScan ignores anything closer

// Obstacle in front: stop and wait until it moves away
constexpr int kObstacleStopCm = 25;             // measured from the front of the rover
constexpr int kLidarToFrontCm = 15;             // MEASURE: LiDAR centre to rover front
constexpr int kFrontConeDeg = 30;               // look this far to each side
constexpr int kObstacleMinHits = 3;             // directions that must see it
constexpr uint32_t kClearMs = 1000;             // front must stay clear this long

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
constexpr int kEPillarMinCm = 200;              // your measure: about 220
constexpr int kEPillarMaxCm = 240;
constexpr float kAlongEMinM = 0.0f;             // look for the pillar only after this far (0 = at once)
constexpr float kAfterEPillarM = 0.80f;         // then drive this far and stop
constexpr float kAlongEMaxM = 15.0f;            // safety: stop the run if the pillar is not found

// Step 11: the side wall of E on the left
constexpr int kESideFindMaxCm = 90;             // your measure: 80
constexpr float kFindESideMaxM = 5.0f;          // safety: stop the run if the wall is not found

// Step 12: along the side wall of E; after its end, on to the fire hydrant
constexpr int kESideNearCm = 60;                // follow the wall between these two
constexpr int kESideFarCm = 80;
constexpr float kESideGoneM = 0.5f;             // the wall must be gone this far to count as ended
constexpr int kHydrantMinCm = 115;              // your measure: 130-140
constexpr int kHydrantMaxCm = 155;
constexpr float kHydrantMaxM = 6.0f;            // safety: after the wall end (your measure: about 3.5 m)
constexpr float kESideMaxM = 40.0f;             // safety: stop the run if the wall never ends

// Step 13: two bench legs, then a panel, on the left
constexpr int kLegMinCm = 105;                  // your measure: 120-130
constexpr int kLegMaxCm = 145;
constexpr int kLegsNeeded = 2;
constexpr int kBenchPanelMinCm = 175;           // your measure: 190-200
constexpr int kBenchPanelMaxCm = 215;
constexpr float kBenchMaxM = 15.0f;             // safety

// Step 14: the wall of building I on the left
constexpr int kIWallMinCm = 245;                // your measure: 260-290
constexpr int kIWallMaxCm = 305;
constexpr float kIWallMaxM = 20.0f;             // safety

// Step 15: the small pillar in front, then stop
constexpr int kSmallPillarStopCm = 70;          // your measure: 50-60; it stops about 10 cm closer than this
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
  {"OpenSee urban canyon", 1, 8, 0},           // at the door
  {"Urban canyon to building I", 9, 15, 90},   // at wall E, facing along it
  {"Building I to building K", 16, 21, 0},     // in front of the small pillar
  {"To the end point", 22, 24, 90},            // next to wall K, facing along it
};
constexpr int kTestGroupCount = sizeof(kTestGroups) / sizeof(kTestGroups[0]);

// Checked when you build: these numbers must fit together.
static_assert(kEndWallStopCm > kObstacleStopCm, "wall E: the obstacle stop would come first");
static_assert(kSmallPillarStopCm > kObstacleStopCm + kLidarToFrontCm, "step 15: the obstacle stop would come first");
static_assert(kLastPillarStopCm > kObstacleStopCm + kLidarToFrontCm, "step 24: the obstacle stop would come first");

