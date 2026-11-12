#pragma once
#include <Arduino.h>

// All the numbers you may want to change are in this file.
// MEASURE means the value is only a first guess until you test it on the rover.

// ---- Test one step at a time: the rover stops after this step ----
// 1 forward 0.90 m       2 turn left 90
// 3 forward, no LiDAR    4 count 5 pillars and 2 walls
// 5 forward after C      6 turn right 90
// 7 drive to wall E      8 turn right 90 at wall E
// 9 forward along E      10 turn left 90
// 11 find the side wall of E   12 along it until it ends

constexpr int kLastStep = 10;              // next test: new wall E rule (step 7) + right turn at E (step 8)

// ---- Driving ----
constexpr float kDriveSpeed = 0.30f;    // was 0.30f: the weak left side needs more power
constexpr float kRightTrim  = 1.0f;    // 0.18f; was 0.20f: outdoors it pulls left
constexpr float kRampPerSec = 0.20f;    //0.3 was 0.20f: start and stop still take 1.5 s
constexpr float kRealSpeedMps = 0.27f;          // MEASURE: real speed at kDriveSpeed
constexpr uint32_t kPauseMs = 1000;             // stop time between two steps

// ---- Route distances, in metres ----
constexpr float kFirstForwardM =  0.90f;         // or 0.98f, see above was 0.90f, only for this test out of the door
constexpr float kNoLidarForwardM = 0.10f;        // was 0.50f: pillar 1 is right after the turn
constexpr float kAfterPillarsM = 1.50f;         // MEASURE: your map label says 6.66 m
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
constexpr int kEndWallStopCm = 40;              // stop when wall E is this close to the rover front
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

// ---- After wall E (steps 8 to 12) ----
constexpr float kAlongEM = 7.0f;                // MEASURE: step 9, along wall E
constexpr float kFindESideMaxM = 5.0f;          // step 11: stop if the side wall of E is not found
constexpr float kESideGoneM = 0.5f;             // step 12: the wall must be gone this far before stopping
constexpr float kESideMaxM = 40.0f;             // step 12: stop if the side wall never ends

