#pragma once
#include <Arduino.h>

// All the numbers you may want to change are in this file.
// MEASURE means the value is only a first guess until you test it on the rover.

// ---- Test one step at a time: the rover stops after this step ----
// 1 forward 0.90 m       2 turn left 90
// 3 forward, no LiDAR    4 count 5 pillars and 2 walls
// 5 forward after C      6 turn right 90
// 7 drive to wall E
constexpr int kLastStep = 1;

// ---- Driving ----
constexpr float kDriveSpeed = 0.30f;            // rover command, 0.5 = 100% power
constexpr float kRealSpeedMps = 0.50f;          // MEASURE: real speed at kDriveSpeed
constexpr float kRightTrim = 0.20f / 0.30f;     // right side is stronger, from Robot_8
constexpr float kRampPerSec = 0.20f;            // soft start and soft stop
constexpr uint32_t kPauseMs = 1000;             // stop time between two steps

// ---- Route distances, in metres ----
constexpr float kFirstForwardM = 0.90f;         // out of the door
constexpr float kNoLidarForwardM = 0.50f;       // after the left turn, before counting
constexpr float kAfterPillarsM = 3.00f;         // MEASURE: your map label says 6.66 m
constexpr float kCountMaxM = 40.0f;             // stop if the pillars are not found
constexpr float kToEndMaxM = 50.0f;             // stop if wall E is not found

// ---- Hold a straight line, values from Robot_8 ----
constexpr float kHeadingGain = 0.05f;           // wheel command per degree of error
constexpr float kHeadingDeadDeg = 1.0f;         // ignore errors smaller than this
constexpr float kMaxCorrection = 0.18f;         // largest steering command
constexpr float kPullGain = 0.004f;             // learns a steady pull to one side
constexpr float kMaxPull = 0.03f;               // largest learned pull
constexpr float kMaxHeadingErrorDeg = 25.0f;    // lost the line: stop the run
constexpr uint32_t kHeadingErrorMs = 1500;      // ...if it lasts this long

// ---- Turn on the spot, values from Robot_8 ----
constexpr float kTurnPower = 0.50f;             // below this the rover barely turns
constexpr float kTurnPulseDeg = 35.0f;          // closer than this: short pulses
constexpr uint32_t kTurnPulseOnMs = 100;
constexpr uint32_t kTurnPulseOffMs = 300;
constexpr float kTurnToleranceDeg = 4.0f;       // turn is finished inside this error
constexpr uint32_t kTurnHoldMs = 300;           // ...for this long
constexpr uint32_t kTurnTimeoutMs = 20000;
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

// Wall on the left (buildings B and C)
constexpr int kWallMaxCm = 200;                 // MEASURE
constexpr int kWallFanDeg = 10;
constexpr int kWallMinHits = 10;
constexpr uint32_t kWallSeenMs = 500;
constexpr uint32_t kWallGoneMs = 1000;

// Pillar on the right
constexpr int kPillarMinCm = 50;                // MEASURE
constexpr int kPillarMaxCm = 250;               // MEASURE
constexpr int kPillarFanDeg = 8;
constexpr int kPillarMinHits = 3;
constexpr uint32_t kPillarSeenMs = 150;
constexpr uint32_t kPillarGoneMs = 1000;
constexpr int kPillarsNeeded = 5;
constexpr int kWallsNeeded = 2;

// End of the route: front wall of E, after a panel on the right
constexpr int kEndWallCm = 150;                 // MEASURE: stop this far from wall E
constexpr int kEndWallFanDeg = 10;
constexpr int kEndWallMinHits = 12;
constexpr int kPanelMinCm = 40;                 // MEASURE
constexpr int kPanelMaxCm = 150;                // MEASURE
constexpr int kPanelFanDeg = 15;
constexpr int kPanelMinHits = 5;
constexpr uint32_t kPanelSeenMs = 300;
constexpr uint32_t kPanelGoneMs = 1000;
constexpr int kPanelsNeeded = 1;
