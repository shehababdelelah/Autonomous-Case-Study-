// TestLogger.h - zone test logger for the CAS box, kit 8 (v3).
//
// Saves one CSV file per sensor for every zone of every run in the ESP32 flash (LittleFS):
//   /Z1-R2/Z1-R2_z1_imu.csv     t_ms, drive command, yaw, pitch, pitch vs zero, ramp state (4 Hz)
//   /Z1-R2/Z1-R2_z1_lidar.csv   t_ms, front/left/right minimum, all 90 sectors              (2 Hz)
//   /Z1-R2/Z1-R2_z1_us.csv      t_ms, front upper/left/right/back/front lower               (4 Hz)
//   /Z1-R2/Z1-R2_z1_gps.csv     t_ms, fix, lat, lon                                          (2 Hz)
//   /Z1-R2/Z1-R2_z1_events.csv  start, waypoints, ramps, zone summary, stop
//
// Runs:
//   Zone run  "Zn-Rk"  one zone (1..4), files z<n>
//   Full path "F-Rk"   WP0 -> WP7, files switch to the next zone after WP2, WP3 and WP5
//
//   One zone is one test: press Start, and the zone records for its own time - 1:30 for zone 1,
//   3:00 for zone 2, 2:00 for zone 3, 1:00 for zone 4 (zoneRunMs) - then saves and ends by
//   itself. A run never walks on to the next zone: test the four zones one after the other,
//   each with its own press of Start. A waypoint that is still averaging finishes first, so a
//   run can overrun by up to kAvgMs.
//
//   With setTestDrive(speed) the test also DRIVES: autoDrive() asks for straight forward at that
//   speed for the whole run, and main.cpp passes it to the rover. The run - and with it the
//   driving - ends on any of four things: the zone's own timer, something closer than kStopCm in
//   front, Stop on the test page, or the red Stop on the driving page. Without setTestDrive()
//   the logger stays passive and you drive by hand, as before.
//
//   Zone  Section            Waypoints   Records   Ramp (ISO 8855 pitch, + = front down)
//   1     B to C             WP0 - WP2   1:30      R1 DOWN, pitch up to +3.5 deg
//   2     F to E             WP2 - WP3   3:00      R2 UP,   pitch down to -3.5 deg
//   3     D/E passage to K   WP3 - WP5   2:00      R3 UP,   pitch down to -3.5 deg
//   4     K to H             WP5 - WP7   1:00      none
//
// It only READS sensors and the drive command; it never drives the rover.
//
// Needs these public getters on your Sensors class (add them to Sensors.h):
//   bool imuOk(); float yawDeg(); float pitchDeg(); bool gpsFix(); double lat(); double lon();
//   uint16_t lidarCm(uint8_t i);  // 90 sectors of 4 deg, 0 = front, clockwise, 0 = no return
//   uint16_t usCm(uint8_t i);     // 0 front, 1 left, 2 right, 3 back, 4 front-angled, 300 = nothing
//
// Integration in cas_box.ino:
//   #include "TestLogger.h"
//   TestLogger testLog;
//   setup():  testLog.begin();                    // after Serial.begin()
//   loop():   testLog.update(sensors, l, r);      // right after rover.setDrive(l, r)
// and the phone page from TestLoggerWeb.h.
//
// Flash use is about 80 KB per minute of recording; the default 4 MB partition table
// leaves about 1.4 MB. Download and delete runs between test sessions.
// LittleFS.begin(true) formats the "spiffs" partition on first use.
#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NavLidar.h"   // sideSummary() for the zone 1 wall term

class TestLogger {
public:
  // ---- settings ----
  // Sensors::update() rebuilds its snapshot every 250 ms (kSnapshotMs in Sensors.cpp), so 4 Hz
  // is the real rate of every value below. Sampling the IMU and ultrasonic files at 5 Hz only
  // wrote the same reading a second time: it cost flash and looked like data that was not there.
  // For genuine 5 Hz, lower kSnapshotMs first - the sensors themselves can keep up.
  static constexpr uint32_t kImuMs        = 250;    // IMU file, 4 Hz
  static constexpr uint32_t kUsMs         = 250;    // ultrasonic file, 4 Hz, all five sensors
  static constexpr uint32_t kGpsMs        = 500;    // GPS file, 2 Hz
  static constexpr uint32_t kLidarMs      = 500;    // LiDAR file, 2 Hz (full scan)
  static constexpr uint32_t kFlushMs      = 3000;   // write each RAM buffer to flash at least this often
  static constexpr uint32_t kStatusMs     = 500;    // status text refresh
  static constexpr size_t   kBufSize      = 4096;   // RAM buffer per file
  static constexpr size_t   kFlushAt      = 3072;
  static constexpr uint32_t kReserveBytes = 32768;  // keep this much flash free
  // Estimate for the idle "minutes left": the four files together measure about 63 KB/min at the
  // rates above (LiDAR is three quarters of it). During a run the real measured rate is used.
  static constexpr uint32_t kBytesPerMin  = 64u * 1024u;
  static constexpr uint32_t kAvgMs        = 20000;  // waypoint averaging time
  static constexpr uint32_t kAvgSampleMs  = 250;
  // Recording time per zone: see zoneRunMs() below - each section needs a different amount.
  // Automatic test drive: the run aborts when the closest thing straight ahead (front LiDAR
  // sectors or the front ultrasonic) is nearer than this. Set to 0 to drive without the check.
  // The sensors refresh about 4 times per second, so at full speed the rover still travels a
  // fair distance after something appears - raise this rather than lower it.
  // All five ultrasonics and the front LiDAR sectors are checked, but not against the same
  // distance - each one is judged by where it points:
  //   front upper, front lower, LiDAR front   kStopCm, far, because we drive into it at speed
  //   left, right                             kSideCm, small: in a corridor these are looking at
  //                                           a wall the whole time, and kStopCm there would
  //                                           abort every run before it had started
  //   back                                    kBackCm, 0 = not checked, we only drive forward
  // Set kSideCm or kBackCm to kStopCm if you do want them equally strict - but check the width
  // of your narrowest passage first: a side limit of 120 cm needs a 2.4 m corridor to clear.
  //
  // OBSTACLE DETECTION IS OFF. All three limits are 0, which blockedBy() reads as "do not check
  // this sensor", so nothing stops a run early any more: not the ultrasonics, not the LiDAR. A
  // test drive ends only on its own zone timer, on Stop on the test page, or on the red Stop on
  // the driving page - and out of Wi-Fi range, only on the timer. The rover will drive into
  // whatever is in front of it.
  // To switch it back on, restore:  kStopCm = 120, kSideCm = 25, kBackCm = 0
  static constexpr uint16_t kStopCm       = 0;
  static constexpr uint16_t kSideCm       = 0;
  static constexpr uint16_t kBackCm       = 0;
  // True while any limit is still armed; the status text uses it to say so plainly.
  static constexpr bool     kObstacleCheck = (kStopCm > 0) || (kSideCm > 0) || (kBackCm > 0);
  // ...and only after it has stayed that close for this long. Sensors rebuilds its snapshot
  // every 250 ms, so this spans at least two readings: one stray short echo from an ultrasonic
  // (HC-SR04 modules do this, more so while driving) can no longer throw a whole run away. The
  // motors still go to zero on the very first close reading - only giving up the run waits.
  static constexpr uint32_t kBlockMs      = 400;
  static constexpr uint8_t  kMaxSamples   = 100;
  static constexpr uint8_t  kMaxWaypoints = 16;
  // Ramp detection, on pitch measured against the pitch zero of this run.
  // ISO 8855 (see ImuManager.h): pitch + = front down, so driving DOWN a ramp reads POSITIVE and
  // driving UP reads NEGATIVE.
  //
  // Everything here follows from two numbers measured on the course: the bare floor is not flat
  // and already swings about kGroundDeg of pitch on its own, and a route ramp peaks at about
  // kRampMaxDeg. Both thresholds must sit in the gap between them:
  //   kRampOffDeg above kGroundDeg - otherwise "back on level ground" is never true and a ramp
  //                                 that was detected never ends, so no RAMP row is ever written
  //   kRampOnDeg  below kRampMaxDeg - otherwise the ramp never triggers in the first place
  static constexpr float    kGroundDeg    = 1.2f;   // pitch range of the bare floor, measured
  static constexpr float    kRampMaxDeg   = 3.5f;   // expected ramp peak; a bigger one is flagged
  static constexpr float    kRampOffDeg   = 1.5f;   // below this -> back on the ground
  static constexpr float    kRampOnDeg    = 2.2f;   // |pitch - pitch zero| above this -> ramp
  static_assert(kGroundDeg < kRampOffDeg && kRampOffDeg < kRampOnDeg && kRampOnDeg < kRampMaxDeg,
                "ramp thresholds must sit between the floor's own pitch range and the ramp peak");
  static constexpr uint32_t kRampOnMs     = 300;    // must stay tilted this long (filters bumps)
  static constexpr uint32_t kRampOffMs    = 1000;   // must be level again this long to end
  // The rover stands still for this long at the start of a run while the average pitch is taken
  // as "level" for the zone. Ramp detection needs that zero, and before this existed it only
  // came from a marked waypoint - so a run where nobody pressed Mark found no ramps at all.
  static constexpr uint32_t kZeroMs       = 1000;

  // ---- straight-line hold for the automatic test drive ----
  // The yaw at the end of the settling window is the reference heading. The difference to it is
  // fed back as a difference between the two wheel speeds, so the box steers itself straight.
  //
  // Signs: ISO 8855 yaw is positive to the LEFT (ImuManager.h), and left wheel faster than right
  // turns the box RIGHT (the drive pad's own mapping in WebHMI.cpp). So a positive error - the
  // box has drifted left - adds to the left wheel and takes the same off the right.
  //
  // Proportional only - the yaw refreshes at 4 Hz, far too slow to differentiate without just
  // amplifying noise. What each piece is for:
  //   kYawDeadDeg   stops it hunting around zero, at the cost of leaving up to that much error
  //   deadband subtracted from the error, so the correction grows from zero at the edge of the
  //                 band instead of stepping to gain * deadband
  //   kYawMaxCorr   bounds it, so a large error cannot stall or reverse a wheel
  //   kYawSmooth    damps jitter coming from IMU yaw noise into the wheel commands
  //
  // TUNED AGAINST THE RECORDED RUNS, not against a guess. The first version of this controller
  // was tuned on a model assuming 150 deg/s of turn authority per unit of wheel difference. The
  // Z1-R1/Z2-R1 recordings give the real figure: while the correction sat pinned at 0.120 (a
  // wheel difference of 0.24) the box slewed 7.5 deg/s against a 3.6 deg/s disturbance, so the
  // authority is 11.1 deg/s at 0.24, i.e. 46 deg/s per unit - the model was 3.3x too responsive.
  // That is why gain 0.008 looked well damped in simulation and was far too weak on the floor:
  // it did not reach full authority until 16 deg of error had built up.
  //
  // Re-simulated against the measured plant and the measured 3.6 deg/s drift:
  //   gain 0.008 clamp 0.12 (as built) settles at +5.86 deg
  //   gain 0.050 clamp 0.18 (now)      settles at +1.78 deg, +2.56 at double the drift
  // No ringing appeared anywhere up to gain 0.080, so 0.050 keeps margin. Full authority is now
  // reached at 4.6 deg of error instead of 16. Proportional only, so a constant pull still
  // leaves a standing offset - it is now about 1.8 deg instead of 5.9.
  //
  // setTestDrive(speed) + kYawMaxCorr must stay inside the rover's +-0.5: 0.3 + 0.12 = 0.42 is
  // fine. Raise either one and check this again.
  static constexpr float    kYawGain      = 0.050f;  // drive units per degree of error
  // Activation threshold. Simulated against the rover model at 1.0 and 1.5 deg: 1.0 settles the
  // error to 0.84 deg instead of 1.08 (inside a +-1 deg target), still rejects +-1.5 deg of IMU
  // noise with 0.29 deg of residual wobble and negligible steering activity, and cuts the
  // standing offset under a 5 deg/s pull from 3.6 to 3.1 deg. Neither value hunts at this gain.
  static constexpr float    kYawDeadDeg   = 1.0f;    // inside this, leave the wheels alone
  static constexpr float    kYawMaxCorr   = 0.18f;   // largest difference put on one side
                                                     // 0.30 +- 0.18 = 0.12..0.48, inside +-0.5
  static constexpr float    kYawSmooth    = 0.35f;   // share of the new correction taken per step

  // ---- runaway guard ----
  // In Z1-R1 the heading error reached 55.8 deg and the box translated 190 cm sideways into a
  // wall (the left wall went from 110 cm to 300 cm while the front return fell to 20 cm).
  // Nothing stopped it. If the error stays past kErrStopDeg for kErrStopMs the run now ends:
  // at that point the box is no longer following the segment and steering harder is a guess.
  static constexpr float    kErrStopDeg   = 25.0f;
  static constexpr uint32_t kErrStopMs    = 1500;

  // ---- zone 1 only: lateral hold against a side wall ----
  // Heading control cannot remove a lateral offset once it exists - that is exactly how Z1-R1
  // ended up against the wall with its heading error already corrected back to -0.19 deg. Zone
  // 1 has a usable side wall, so its distance is held as well as the heading. Zone 2 has no
  // wall and falls back to heading only; see zoneUsesWall().
  //
  // There is NO surveyed target distance in this project, so none is invented: the distance
  // measured at the start of the segment, while the box stands still on the intended line, is
  // the reference. That assumes the box is placed on the line - it is an assumption, not a
  // measurement, and it is logged so it can be checked.
  // Swept against the measured plant, holding the measured 3.6 deg/s drift. Lateral offset after
  // 40 s: no wall term 619 mm and still climbing; 0.00039 -> 83 mm; 0.0008 -> 47 mm;
  // 0.0016 -> 17 mm but with visibly more heading activity. 0.0008 is the compromise. Note this
  // is a position loop closed through heading, so it is a double integrator: raise it carefully
  // and watch for weaving on the real floor, which the model cannot fully predict.
  static constexpr float    kWallGain     = 0.0008f;  // drive units per mm of lateral offset
  static constexpr float    kWallMaxCorr  = 0.06f;    // never allowed to dominate the heading
  static constexpr uint16_t kWallMinMm    = 250;      // nearer than this is not a wall, it is an obstacle
  static constexpr uint16_t kWallMaxMm    = 2500;     // further than this is not the corridor wall
  static constexpr uint16_t kWallJumpMm   = 400;      // a bigger step than this = different object
  static constexpr uint32_t kWallLostMs   = 1000;     // no usable wall this long -> heading only
  static constexpr uint8_t  kSectors      = 90;

  enum { F_IMU, F_LIDAR, F_US, F_GPS, F_EVENTS, F_FEAT, F_COUNT };

  // The features file records what the LiDAR geometry DETECTED, next to the raw scan that
  // lidar.csv already holds. It is written at kFeatMs rather than at the LiDAR rate because it
  // is derived data: the raw sectors remain the ground truth and any feature can be recomputed
  // from them offline, so there is no reason to pay twice for the same information.
  // At most kFeatMaxRuns segmented runs are written per sample, nearest first, which bounds the
  // file size - the count that was found is recorded even when some are left out.
  static constexpr uint32_t kFeatMs      = 1000;
  static constexpr uint8_t  kFeatMaxRuns = 8;

  // mode/zone 0 = full path, 1..4 = zones
  static const char* zoneName(uint8_t z) {
    static const char* const n[] = {"full path", "B to C", "F to E", "D/E passage to K", "K to H"};
    return n[z < 5 ? z : 0];
  }
  static uint8_t zoneFirstWp(uint8_t z) { static const uint8_t t[] = {0, 0, 2, 3, 5}; return t[z < 5 ? z : 0]; }
  static uint8_t zoneLastWp(uint8_t z)  { static const uint8_t t[] = {7, 2, 3, 5, 7}; return t[z < 5 ? z : 0]; }
  static uint8_t zoneRamps(uint8_t z)   { static const uint8_t t[] = {3, 1, 1, 1, 0}; return t[z < 5 ? z : 0]; }

  // Which way the route ramp of a zone tilts the box first, in the ISO 8855 pitch sign
  // (+1 = down, so pitch runs up to +kRampMaxDeg; -1 = up, down to -kRampMaxDeg; 0 = no ramp):
  //   zone 1  R1  DOWN  (+3.5)        zone 3  R3  UP  (-3.5)
  //   zone 2  R2  UP    (-3.5)        zone 4      no ramp
  // A ramp that tilts the other way is still recorded, but marked in the events file so you can
  // see that the box met the section from the other end.
  static int8_t zoneRampSign(uint8_t z) {
    static const int8_t t[] = {0, +1, -1, -1, 0};
    return t[z < 5 ? z : 0];
  }

  /**
   * @brief Whether a zone has a side wall good enough to hold a lateral position against.
   * @param z Zone 1..4 (0 = full path).
   * @return True only for zone 1. Everywhere else the controller is heading-only.
   * @note Zone 1 (B to C) runs between two buildings and the Z1-R1 recording shows a clean
   *       monotonic left-hand return (110 -> 300 cm). Zone 2 has no usable wall, so it must not
   *       pretend to measure lateral position - without a wall, encoders or vision there is no
   *       lateral measurement to be had, and heading control alone cannot supply one.
   */
  static bool zoneUsesWall(uint8_t z) { return z == 1; }

  // How long one press of Start records, per zone: zone 1 (B to C) 1:30, zone 2 (F to E) 3:00,
  // zone 3 (D/E passage to K) 2:00, zone 4 (K to H) 1:00 - long enough to cross each section in
  // one go. The run then saves and ends by itself. Index 0 is their sum, for a full path run.
  // A waypoint that is still averaging finishes first, so a run can overrun by up to kAvgMs.
  static uint32_t zoneRunMs(uint8_t z) {
    static const uint32_t t[] = {450000, 90000, 180000, 120000, 60000};
    return t[z < 5 ? z : 0];
  }

  void begin() {
    _fsOk = LittleFS.begin(true);
    if (!_fsOk) Serial.println(F("# WARN LittleFS mount failed - nothing can be saved"));
    _total = _fsOk ? LittleFS.totalBytes() : 0;
    _usedIdle = _fsOk ? LittleFS.usedBytes() : 0;
    Serial.println(F("# TestLogger v3 - zone tests. Phone page: http://192.168.4.1/test"));
    note("Idle. Choose a zone and press Start.");
    updateStatus(millis());
  }

  // ---- requests from the web page (safe from the web server task; run in update()) ----
  void requestStart(uint8_t mode) { if (mode <= 4) { _reqMode = mode; _reqStart = true; } }
  void requestStop()     { _reqStop = true; }
  void requestMark()     { _reqMark = true; }
  void requestUndo()     { _reqUndo = true; }
  void requestNextZone() { _reqNext = true; }
  // name of a run ("Z1-R2"), or "*" for all saved runs
  void requestDelete(const char* run) {
    strncpy(_delName, run, sizeof _delName - 1);
    _delName[sizeof _delName - 1] = '\0';
    _reqDelete = true;
  }

  bool isRecording() const { return _rec; }

  // Forward speed for the automatic part of a zone test, in the same units as rover.setDrive()
  // (0 = the logger never asks for a drive command). Call it once in setup().
  void setTestDrive(float speed) { _driveSpeed = speed > 0.0f ? speed : 0.0f; }

  // The drive command the running test wants: straight forward while it records, zero while a
  // waypoint is averaging (a Mark needs the rover to stand still) and zero as soon as something
  // is close in front. Returns false when the logger is not driving at all, so the caller keeps
  // whatever the phone or the Serial Monitor asked for.
  //
  // The front check is repeated here rather than left to update(), because this is what main.cpp
  // hands to rover.setDrive() in the same pass: the motors go to zero now, and update() writes
  // the reason and closes the files on the next pass.
  template <class S>
  bool autoDrive(S& s, float& left, float& right) const {
    if (!_rec || _driveSpeed <= 0.0f) return false;
    left = right = 0.0f;
    if (_avgActive) return true;                               // standing still for a waypoint
    if (millis() - _zoneT0 < kZeroMs) return true;             // settling for the pitch zero
    uint16_t cm;
    if (blockedBy(s, cm)) return true;                         // blocked: update() ends the run
    // Straight ahead, steered by the heading error: drifted left -> left wheel gets the faster
    // share. Clamped to the base speed so a correction can never reverse or stall a wheel.
    const float corr = constrain(_yawCorr, -_driveSpeed, _driveSpeed);
    left = _driveSpeed + corr;
    right = _driveSpeed - corr;
    return true;
  }

  // true if the path belongs to the run that is being recorded (no download then)
  bool isActivePath(const char* path) const {
    if (!_rec) return false;
    char p[24];
    snprintf(p, sizeof p, "/%s/", _run);
    return strncmp(path, p, strlen(p)) == 0;
  }

  void copyStatus(char* out, size_t n) const {
    strncpy(out, _status[_statusIdx], n - 1);
    out[n - 1] = '\0';
  }

  // JSON list of saved runs and their files, for the phone page (n >= 512).
  void listJson(char* out, size_t n) const {
    size_t len = 0;
    out[0] = '\0';
    catf(out, n, len, "{\"rec\":\"%s\",\"runs\":[", _rec ? _run : "");
    bool firstRun = true, more = false;
    File root = LittleFS.open("/");
    if (root && root.isDirectory()) {
      File d = root.openNextFile();
      while (d && !more) {
        if (d.isDirectory()) {
          if (n - len < 300) { more = true; break; }
          catf(out, n, len, "%s{\"run\":\"%s\",\"files\":[", firstRun ? "" : ",", baseName(d.name()));
          firstRun = false;
          bool firstFile = true;
          File f = d.openNextFile();
          while (f) {
            if (n - len < 120) { more = true; break; }
            catf(out, n, len, "%s{\"n\":\"%s\",\"s\":%lu}", firstFile ? "" : ",",
                 baseName(f.name()), (unsigned long)f.size());
            firstFile = false;
            f = d.openNextFile();
          }
          catf(out, n, len, "]}");
        }
        if (!more) d = root.openNextFile();
      }
    }
    catf(out, n, len, "],\"more\":%s}", more ? "true" : "false");
  }

  // Call on every loop() pass. S = your Sensors class.
  template <class S>
  void update(S& s, float l, float r) {
    const uint32_t now = millis();

    bool acted = false;
    if (_reqStop)   { _reqStop = false;   stopRun(now, "stopped by hand"); acted = true; }
    if (_reqStart)  { _reqStart = false;  startRun(_reqMode, now); acted = true; }
    if (_reqNext)   { _reqNext = false;   nextZone(now, true, "next zone pressed"); acted = true; }
    if (_reqMark)   { _reqMark = false;   markWaypoint(now);       acted = true; }
    if (_reqUndo)   { _reqUndo = false;   undoLast(now);           acted = true; }
    if (_reqDelete) { _reqDelete = false; deleteRuns(_delName);    acted = true; }
    if (acted) _lastStatus = now - kStatusMs;

    const bool  imu   = s.imuOk();
    const float yaw   = imu ? s.yawDeg()   : NAN;
    const float pitch = imu ? s.pitchDeg() : NAN;
    const bool  fix   = s.gpsFix();
    _lastImu = imu; _lastFix = fix;
    // for the status text and the abort: how far it can see ahead, and what is too close
    _frontCm = frontCm(s);
    _blockWhere = blockedBy(s, _blockCm);

    // Pitch zero for this run: the rover is held still for the first kZeroMs (see autoDrive), so
    // the average pitch over that window is what "level" means for this zone. Everything the
    // ramp detection does is measured against it.
    if (_rec && !_pitch0Set && imu) {
      if (now - _zoneT0 < kZeroMs) { _zeroSum += pitch; ++_zeroN; }
      else if (_zeroN) {
        _pitch0 = _zeroSum / _zeroN;
        _pitch0Set = true;
        char z[48];
        snprintf(z, sizeof z, "pitch zero %.2f deg from %u samples", _pitch0, (unsigned)_zeroN);
        evText(now, "PITCH_ZERO", z);
        Serial.printf("# %s\n", z);
        // Same moment, same standing-still window: this heading is what the drive will hold.
        _yaw0 = yaw;
        _yaw0Set = true;
        snprintf(z, sizeof z, "heading reference %.1f deg", _yaw0);
        evText(now, "HEADING_REF", z);
        Serial.printf("# %s\n", z);
      }
    }
    wallStep(s, now);          // zone 1 only; chooses and tracks the side wall
    headingStep(yaw, imu);
    // Runaway guard: a heading error this large for this long means the box is no longer on the
    // segment, and steering harder would be guessing. This is what Z1-R1 needed and did not have.
    if (_rec && _yaw0Set && _driveSpeed > 0.0f) {
      if (fabsf(_yawErr) >= kErrStopDeg) {
        if (!_errBadSince) _errBadSince = now ? now : 1;
        else if (now - _errBadSince >= kErrStopMs) {
          char why[56];
          snprintf(why, sizeof why, "heading error %+.0f deg, giving up", _yawErr);
          evText(now, "DRIFT_ABORT", why);
          Serial.printf("# DRIFT_ABORT: %s\n", why);
          stopRun(now, why);
          _lastStatus = now - kStatusMs;
        }
      } else {
        _errBadSince = 0;
      }
    }
    if (_avgActive) averageStep(s, now, yaw, pitch, fix, l, r);
    if (_rec && _pitch0Set && imu) rampStep(now, pitch - _pitch0);

    if (_rec) {
      const unsigned long t = now - _t0;
      if (now - _lastW[F_IMU] >= kImuMs) {
        _lastW[F_IMU] = now;
        // The heading controller's own signals go in the same row, so the recorded motor
        // commands can be read against what the correction was asking for. They are left EMPTY
        // rather than zeroed when no reference has been taken yet: an empty field means "not
        // available", a 0 would read as a measured zero error.
        // mode says which controller produced the row, so the two zones can be told apart in
        // analysis. The wall columns stay EMPTY when no wall is being held - not zero.
        char steer[80];
        if (_yaw0Set && _wallSide)
          snprintf(steer, sizeof steer, "%.1f,%+.2f,%+.3f,%s,%u,%u,%+.3f", _yaw0, _yawErr,
                   _yawCorr, _wallSide < 0 ? "yaw+wallL" : "yaw+wallR", (unsigned)_wallNowMm,
                   (unsigned)_wallRefMm, _wallCorr);
        else if (_yaw0Set)
          snprintf(steer, sizeof steer, "%.1f,%+.2f,%+.3f,yaw,,,", _yaw0, _yawErr, _yawCorr);
        else
          snprintf(steer, sizeof steer, ",,,none,,,");
        if (imu && _pitch0Set)
          add(F_IMU, "%lu,%.2f,%.2f,1,%.1f,%.1f,%.1f,%u,%s\n", t, l, r, yaw, pitch,
              pitch - _pitch0, rampState(), steer);
        else if (imu)
          add(F_IMU, "%lu,%.2f,%.2f,1,%.1f,%.1f,,0,%s\n", t, l, r, yaw, pitch, steer);
        else
          add(F_IMU, "%lu,%.2f,%.2f,0,,,,0,%s\n", t, l, r, steer);
      }
      if (now - _lastW[F_US] >= kUsMs) {
        _lastW[F_US] = now;
        add(F_US, "%lu,%u,%u,%u,%u,%u\n", t, (unsigned)s.usCm(0), (unsigned)s.usCm(1),
            (unsigned)s.usCm(2), (unsigned)s.usCm(3), (unsigned)s.usCm(4));
      }
      if (now - _lastW[F_GPS] >= kGpsMs) {
        _lastW[F_GPS] = now;
        ++_gpsN;
        if (fix) { ++_gpsFixN; add(F_GPS, "%lu,1,%.7f,%.7f\n", t, s.lat(), s.lon()); }
        else     add(F_GPS, "%lu,0,,\n", t);
      }
      if (now - _lastW[F_LIDAR] >= kLidarMs) {
        _lastW[F_LIDAR] = now;
        int p = snprintf(_line, sizeof _line, "%lu,%u,%u,%u", t,
                         (unsigned)minSectors(s, -3, 3),     // front +-12 deg
                         (unsigned)minSectors(s, 62, 72),    // left 270 deg +-20
                         (unsigned)minSectors(s, 17, 27));   // right 90 deg +-20
        for (uint8_t i = 0; i < kSectors && p > 0 && p < (int)sizeof _line - 8; ++i)
          p += snprintf(_line + p, sizeof _line - p, ",%u", (unsigned)s.lidarCm(i));
        if (p > 0 && p < (int)sizeof _line - 2) { _line[p++] = '\n'; _line[p] = '\0'; append(F_LIDAR, _line); }
      }
      // What the geometry made of that scan, recorded beside it rather than only displayed.
      if (now - _lastW[F_FEAT] >= kFeatMs) {
        _lastW[F_FEAT] = now;
        featureStep(s, t);
      }
      flushStep(now);
    } else if (_fsOk && now - _lastUsedCheck >= 5000) {
      _lastUsedCheck = now;
      _usedIdle = LittleFS.usedBytes();
    }

    // End of the run, without anyone pressing Stop: this zone's time is up, or the automatic
    // drive sees something close in front. One run is always one zone - it never walks on to the
    // next one. Checked after this pass has been written, so the last samples are in the files.
    if (_rec && !_avgActive) {
      const char* ev = nullptr;
      char why[48];
      const uint32_t runMs = zoneRunMs(_zone);
      if (now - _zoneT0 >= runMs) {
        ev = "AUTO_STOP";
        snprintf(why, sizeof why, "zone %u time done, %lu:%02lu", (unsigned)_zone,
                 (unsigned long)(runMs / 60000), (unsigned long)((runMs / 1000) % 60));
      } else if (_driveSpeed > 0.0f && _blockWhere) {
        if (!_blockSince) {
          _blockSince = now ? now : 1;      // 0 means "not blocked", so never store a plain 0
        } else if (now - _blockSince >= kBlockMs) {
          ev = "DRIVE_ABORT";
          snprintf(why, sizeof why, "%s %u cm", _blockWhere, (unsigned)_blockCm);
        }
      } else {
        _blockSince = 0;                    // clear again: a single stray reading is forgotten
      }
      if (ev) {
        evText(now, ev, why);
        Serial.printf("# %s: %s\n", ev, why);
        stopRun(now, why);
        _lastStatus = now - kStatusMs;  // show it on the phone at the next status refresh
      }
    }

    if (now - _lastStatus >= kStatusMs) { _lastStatus = now; updateStatus(now); }
  }

private:
  static constexpr double kMPerDeg = 6371000.0 * PI / 180.0;  // ~111195 m per degree

  // ---------- runs and zones ----------
  void startRun(uint8_t mode, uint32_t now) {
    if (!_fsOk) { note("No flash file system: cannot record."); return; }
    if (_rec)   { note("Stop the current run first."); return; }
    const unsigned k = nextRunNumber(mode);
    if (mode == 0) snprintf(_run, sizeof _run, "F-R%u", k);
    else           snprintf(_run, sizeof _run, "Z%u-R%u", (unsigned)mode, k);
    char dir[16];
    snprintf(dir, sizeof dir, "/%s", _run);
    LittleFS.mkdir(dir);
    _mode = mode; _zone = mode == 0 ? 1 : mode;
    _t0 = now;
    _wpCount = zoneFirstWp(mode);
    _avgActive = false; _pitch0Set = false; _zeroSum = 0; _zeroN = 0; _blockSince = 0;
    _yaw0Set = false; _yaw0 = 0; _yawErr = 0; _yawCorr = 0;
    _wallCorr = 0; _wallSide = 0; _wallRefMm = 0; _wallNowMm = 0; _wallLostSince = 0;
    _errBadSince = 0;
    resetRampState();
    _usedAtStart = LittleFS.usedBytes(); _written = 0; _full = false;
    _rec = true;
    openZoneFiles(now);
    Serial.printf("# start %s, zone %u (%s)\n", _run, (unsigned)_zone, zoneName(_zone));
    note("Recording %s. Press Mark at WP%u.", _run, (unsigned)_wpCount);
  }

  void stopRun(uint32_t now, const char* why) {
    if (!_rec) { note("Nothing is recording."); return; }
    if (_avgActive) { _avgActive = false; evText(now, "WP_CANCELLED", "stopped while averaging"); }
    evText(now, "STOP", why);
    closeZoneFiles(now);
    _rec = false;
    _usedIdle = LittleFS.usedBytes();
    Serial.printf("# stop %s (%s)\n", _run, why);
    note("Run %s saved - %s. Download it below.", _run, why);
  }

  // Full path only: close this zone's files and open the next zone's files.
  void nextZone(uint32_t now, bool manual, const char* why) {
    if (!_rec || _mode != 0) { if (manual) note("Next zone works only in a full path run."); return; }
    if (_zone >= 4)          { if (manual) note("Already in zone 4."); return; }
    if (_avgActive)          { if (manual) note("Wait until the waypoint is saved."); return; }
    evText(now, "ZONE_END", why);
    closeZoneFiles(now);
    ++_zone;
    openZoneFiles(now);
    evText(now, "ZONE_START", zoneName(_zone));
    note("Zone %u (%s) started. Next: Mark WP%u.", (unsigned)_zone, zoneName(_zone), (unsigned)_wpCount);
  }

  void openZoneFiles(uint32_t now) {
    static const char* const kNames[F_COUNT] = {"imu", "lidar", "us", "gps", "events", "features"};
    for (uint8_t i = 0; i < F_COUNT; ++i) {
      char path[48];
      snprintf(path, sizeof path, "/%s/%s_z%u_%s.csv", _run, _run, (unsigned)_zone, kNames[i]);
      _f[i] = LittleFS.open(path, FILE_WRITE);
      _fileOk[i] = (bool)_f[i];
      _bufLen[i] = 0;
      _lastFlush[i] = now;
      _lastW[i] = 0;
      if (!_fileOk[i]) Serial.printf("# WARN cannot open %s\n", path);
    }
    // imu_ok = 0 means the IMU columns of that row carry no measurement. The last three are the
    // heading controller: the reference it is holding, the error against it, and the wheel-speed
    // difference it applied. Empty = no reference taken yet, which is not the same as zero.
    append(F_IMU, "t_ms,l,r,imu_ok,yaw_deg,pitch_deg,pitch_rel_deg,ramp_state,"
                  "yaw_ref_deg,yaw_err_deg,steer,mode,wall_mm,wall_ref_mm,wall_corr\n");
    // All five ultrasonics, in the order UsManager measures them. front_upper is the straight
    // one (GPIO 32/25), front_lower the angled one (GPIO 23/26); 300 = nothing in range.
    append(F_US, "t_ms,front_upper_cm,left_cm,right_cm,back_cm,front_lower_cm\n");
    append(F_GPS, "t_ms,fix,lat,lon\n");
    // One row per detected feature, so a variable number of them per scan needs no fixed columns.
    // What the numeric columns mean depends on kind:
    //   RUN      a segmented run: all columns as named; note = what its shape resembles
    //   FRONT    near_mm = closest return straight ahead; note = blocking sensor, or "clear"
    //   ENDWALL  span_deg = degrees of the forward cone it fills; near_mm = its distance
    //   SIDE_L   bearing -90, SIDE_R bearing +90; span_deg = contributing bins;
    //            near_mm = nearest; note = wall or open
    //   OPEN_L   bearing_deg = first bearing of the gap, span_deg = last, near_mm = its width
    //   RUNS     span_deg = how many runs the scan produced, note says if any were left out
    append(F_FEAT, "t_ms,kind,bearing_deg,span_deg,near_mm,chord_mm,tilt_deg,note\n");
    append(F_EVENTS, "t_ms,event,wp,lat,lon,spread_m,rms_m,yaw_deg,samples,moved,"
                     "ramp,pitch_min,pitch_max,duration_ms,first,note\n");
    int p = snprintf(_line, sizeof _line, "t_ms,front_cm,left_cm,right_cm");
    for (uint8_t i = 0; i < kSectors && p > 0 && p < (int)sizeof _line - 8; ++i)
      p += snprintf(_line + p, sizeof _line - p, ",s%u", (unsigned)i);
    if (p > 0 && p < (int)sizeof _line - 2) { _line[p++] = '\n'; _line[p] = '\0'; append(F_LIDAR, _line); }
    _gpsN = _gpsFixN = 0;
    _rampsZone = 0;
    _wpZoneStart = _wpCount;
    _zoneT0 = now;          // start of this zone's zoneRunMs() window
    char s[64];
    snprintf(s, sizeof s, "run %s zone %u %s", _run, (unsigned)_zone, zoneName(_zone));
    evText(now, "START", s);
  }

  void closeZoneFiles(uint32_t now) {
    char s[120];
    const unsigned fixPct = _gpsN ? (unsigned)(100UL * _gpsFixN / _gpsN) : 0;
    snprintf(s, sizeof s, "WP%u-WP%u; ramps %u of %u expected; GPS fix %u%%",
             (unsigned)_wpZoneStart, (unsigned)(_wpCount ? _wpCount - 1 : 0), (unsigned)_rampsZone,
             (unsigned)zoneRamps(_zone), fixPct);
    evText(now, "ZONE_SUMMARY", s);
    for (uint8_t i = 0; i < F_COUNT; ++i) {
      flushFile(i, true);
      if (_fileOk[i]) _f[i].close();
      _fileOk[i] = false;
    }
  }

  unsigned nextRunNumber(uint8_t mode) const {
    char prefix[8];
    if (mode == 0) snprintf(prefix, sizeof prefix, "F-R");
    else           snprintf(prefix, sizeof prefix, "Z%u-R", (unsigned)mode);
    const size_t pl = strlen(prefix);
    unsigned best = 0;
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) return 1;
    File d = root.openNextFile();
    while (d) {
      const char* nm = baseName(d.name());
      if (d.isDirectory() && strncmp(nm, prefix, pl) == 0) {
        const unsigned k = (unsigned)atoi(nm + pl);
        if (k > best) best = k;
      }
      d = root.openNextFile();
    }
    return best + 1;
  }

  // Deletes one saved run, or all of them with "*". Never the run being recorded.
  void deleteRuns(const char* which) {
    const bool all = strcmp(which, "*") == 0;
    if (!all && !validRunName(which)) { note("Unknown run name."); return; }
    if (!all && _rec && strcmp(which, _run) == 0) { note("Cannot delete the run being recorded."); return; }
    char names[24][16];
    uint8_t count = 0;
    File root = LittleFS.open("/");
    if (root && root.isDirectory()) {
      File d = root.openNextFile();
      while (d && count < 24) {
        const char* nm = baseName(d.name());
        if (d.isDirectory() && (all || strcmp(nm, which) == 0) && !(_rec && strcmp(nm, _run) == 0)) {
          strncpy(names[count], nm, 15); names[count][15] = '\0'; ++count;
        }
        d = root.openNextFile();
      }
      root.close();
    }
    for (uint8_t i = 0; i < count; ++i) removeDir(names[i]);
    if (!_rec) _usedIdle = LittleFS.usedBytes();
    note(count ? "Deleted %u run(s)." : "Nothing deleted.", (unsigned)count);
  }

  static bool validRunName(const char* s) {
    if (!*s || strlen(s) > 12) return false;
    for (; *s; ++s)
      if (!((*s >= 'A' && *s <= 'Z') || (*s >= '0' && *s <= '9') || *s == '-')) return false;
    return true;
  }

  void removeDir(const char* run) {
    char dir[20], files[24][48];   // 24 = 4 zones x 6 files, the most a full path run can hold
    uint8_t n = 0;
    snprintf(dir, sizeof dir, "/%s", run);
    File d = LittleFS.open(dir);
    if (d && d.isDirectory()) {
      File f = d.openNextFile();
      while (f && n < 24) {
        snprintf(files[n++], sizeof files[0], "%s/%s", dir, baseName(f.name()));
        f = d.openNextFile();
      }
      d.close();
    }
    for (uint8_t i = 0; i < n; ++i) LittleFS.remove(files[i]);
    LittleFS.rmdir(dir);
  }

  // name() gives "x.csv" on core 2.x and "/dir/x.csv" on older cores
  static const char* baseName(const char* p) {
    const char* b = strrchr(p, '/');
    return b ? b + 1 : p;
  }

  // ---------- file output ----------
  void add(uint8_t i, const char* fmt, ...) {
    va_list a; va_start(a, fmt);
    vsnprintf(_line, sizeof _line, fmt, a);
    va_end(a);
    append(i, _line);
  }

  void append(uint8_t i, const char* s) {
    if (!_fileOk[i] || (_full && i != F_EVENTS)) return;
    const size_t n = strlen(s);
    if (_bufLen[i] + n > kBufSize) flushFile(i);
    if (n > kBufSize) return;
    memcpy(_buf[i] + _bufLen[i], s, n);
    _bufLen[i] += n;
  }

  // one file per pass, so flash writes are spread out
  void flushStep(uint32_t now) {
    const uint8_t i = _rr;
    _rr = (uint8_t)((_rr + 1) % F_COUNT);
    if (_bufLen[i] && (_bufLen[i] >= kFlushAt || now - _lastFlush[i] >= kFlushMs)) flushFile(i);
  }

  void flushFile(uint8_t i, bool final = false) {
    _lastFlush[i] = millis();
    if (!_bufLen[i] || !_fileOk[i]) { _bufLen[i] = 0; return; }
    if (freeBytes() < _bufLen[i] + (final ? 4096u : kReserveBytes)) {
      if (!_full) {
        _full = true;
        Serial.println(F("# WARN flash almost full - sensor files stopped. Press Stop."));
        note("Flash almost full. Press Stop, download and delete old runs.");
      }
      _bufLen[i] = 0;
      return;
    }
    _f[i].write(reinterpret_cast<const uint8_t*>(_buf[i]), _bufLen[i]);
    _f[i].flush();
    _written += _bufLen[i];
    _bufLen[i] = 0;
  }

  uint32_t freeBytes() const {
    const uint32_t used = _rec ? _usedAtStart + _written : _usedIdle;
    return _total > used ? _total - used : 0;
  }

  // events file: rows with 16 columns
  void evText(uint32_t now, const char* type, const char* text) {
    add(F_EVENTS, "%lu,%s,,,,,,,,,,,,,,%s\n", (unsigned long)(now - _t0), type, text);
  }

  void note(const char* fmt, ...) {
    va_list a; va_start(a, fmt);
    vsnprintf(_note, sizeof _note, fmt, a);
    va_end(a);
  }

  // appends to s, never past n - 1
  static void catf(char* s, size_t n, size_t& len, const char* fmt, ...) {
    if (len + 1 >= n) return;
    va_list a; va_start(a, fmt);
    const int k = vsnprintf(s + len, n - len, fmt, a);
    va_end(a);
    if (k > 0) len += (size_t)k;
    if (len > n - 1) len = n - 1;
  }

  void updateStatus(uint32_t now) {
    char* s = _status[1 - _statusIdx];
    const size_t n = sizeof _status[0];
    size_t len = 0;
    s[0] = '\0';
    const uint32_t freeKb = freeBytes() / 1024;
    if (!_fsOk) {
      catf(s, n, len, "NO FLASH FILE SYSTEM - nothing is saved\n");
    } else if (_rec) {
      const uint32_t sec = (now - _t0) / 1000;
      const uint32_t rate = sec > 30 ? (uint32_t)((_written + bufTotal()) * 60 / sec) : kBytesPerMin;
      const unsigned fixPct = _gpsN ? (unsigned)(100UL * _gpsFixN / _gpsN) : 0;
      catf(s, n, len, "RECORDING %s - zone %u (%s), %lu:%02lu\n", _run, (unsigned)_zone, zoneName(_zone),
           (unsigned long)(sec / 60), (unsigned long)(sec % 60));
      // this zone's own clock: how far it has got, and what happens when it runs out
      const uint32_t runMs = zoneRunMs(_zone);
      const uint32_t zoneMs = now - _zoneT0;
      const uint32_t leftSec = zoneMs >= runMs ? 0 : (runMs - zoneMs + 999) / 1000;
      catf(s, n, len, "Zone time %lu:%02lu of %lu:%02lu\n", (unsigned long)(zoneMs / 60000),
           (unsigned long)((zoneMs / 1000) % 60), (unsigned long)(runMs / 60000),
           (unsigned long)((runMs / 1000) % 60));
      if (_avgActive)
        catf(s, n, len, "The waypoint finishes before the run ends\n");
      else
        catf(s, n, len, "%s in %lu:%02lu\n", _driveSpeed > 0.0f ? "DRIVING FORWARD, stops"
             : "Saves itself", (unsigned long)(leftSec / 60), (unsigned long)(leftSec % 60));
      if (_avgActive)
        catf(s, n, len, "Averaging WP%u: %lu s left, keep still\n", (unsigned)_wpCount,
             (unsigned long)((kAvgMs - (now - _avgStart) + 999) / 1000));
      else if (_wpCount > zoneLastWp(_zone))
        catf(s, n, len, "%s done: press Stop and save\n", _mode == 0 ? "Full path" : "Zone");
      else
        catf(s, n, len, "Next: Mark WP%u (zone ends at WP%u)\n", (unsigned)_wpCount, (unsigned)zoneLastWp(_zone));
      catf(s, n, len, "Ramps: %u of %u   Front: %u cm%s   GPS fix: %u%%   IMU: %s\n",
           (unsigned)_rampsZone, (unsigned)zoneRamps(_zone), (unsigned)_frontCm,
           kObstacleCheck ? "" : " (no stop)", fixPct, _lastImu ? "ok" : "NOT READY");
      if (_driveSpeed > 0.0f) {
        if (_yaw0Set)
          catf(s, n, len, "Heading %+.1f of %.1f deg, steering %+.2f\n", _yawErr, _yaw0, _yawCorr);
        else
          catf(s, n, len, "Heading: no reference yet (IMU not ready)\n");
      }
      if (_full) catf(s, n, len, "FLASH FULL - press Stop\n");
      else catf(s, n, len, "Saved %lu KB, free %lu KB (about %lu min)\n",
                (unsigned long)((_written + bufTotal()) / 1024), (unsigned long)freeKb,
                (unsigned long)(rate ? freeBytes() / rate : 0));
    } else if (_driveSpeed > 0.0f) {
      catf(s, n, len, "IDLE - Start drives the rover FORWARD and records\n"
                      "Zone 1: 1:30   zone 2: 3:00   zone 3: 2:00   zone 4: 1:00\n");
      // The commonest reason a test seems to do nothing: a sensor is already too close, so the
      // run aborts in the same moment it starts. Name the sensor before Start is pressed.
      if (!kObstacleCheck)
        catf(s, n, len, "OBSTACLE DETECTION OFF - nothing will stop the run. Front %u cm\n",
             (unsigned)_frontCm);
      else if (_blockWhere)
        catf(s, n, len, "BLOCKED by %s at %u cm - clear it or Start will abort\n",
             _blockWhere, (unsigned)_blockCm);
      else
        catf(s, n, len, "Front %u cm, clear to start (front %u, sides %u cm)\n",
             (unsigned)_frontCm, (unsigned)kStopCm, (unsigned)kSideCm);
      catf(s, n, len, "Free %lu KB (about %lu min)   GPS: %s   IMU: %s\n",
           (unsigned long)freeKb, (unsigned long)(freeBytes() / kBytesPerMin),
           _lastFix ? "fix" : "NO FIX", _lastImu ? "ok" : "NOT READY");
    } else {
      catf(s, n, len, "IDLE - choose a zone and press Start\nFree %lu KB (about %lu min)   GPS: %s   IMU: %s\n",
           (unsigned long)freeKb, (unsigned long)(freeBytes() / kBytesPerMin),
           _lastFix ? "fix" : "NO FIX", _lastImu ? "ok" : "NOT READY");
    }
    catf(s, n, len, "%s", _note);
    _statusIdx = (uint8_t)(1 - _statusIdx);
  }

  size_t bufTotal() const { size_t t = 0; for (uint8_t i = 0; i < F_COUNT; ++i) t += _bufLen[i]; return t; }

  // ---------- waypoints ----------
  void markWaypoint(uint32_t now) {
    if (!_rec)                     { note("Press Start first."); return; }
    if (_avgActive)                { note("Busy: still averaging the last waypoint."); return; }
    if (_wpCount >= kMaxWaypoints) { note("Waypoint table full."); return; }
    _avgActive = true;
    _avgStart = _avgLastSample = now;
    _n = 0; _yawN = 0;
    _sumSin = _sumCos = _sumPitch = 0.0f;
    _moved = false;
    char s[48];
    snprintf(s, sizeof s, "averaging WP%u", (unsigned)_wpCount);
    evText(now, "WP_START", s);
  }

  void undoLast(uint32_t now) {
    if (!_rec) { note("Nothing is recording."); return; }
    if (_avgActive) { _avgActive = false; evText(now, "WP_CANCELLED", "undo"); note("Averaging cancelled."); return; }
    if (_wpCount <= _wpZoneStart) { note("No waypoint of this zone to remove."); return; }
    --_wpCount;
    char s[48];
    snprintf(s, sizeof s, "WP%u removed", (unsigned)_wpCount);
    evText(now, "UNDO", s);
    if (_wpCount == zoneFirstWp(_mode) && _wpCount == _wpZoneStart) _pitch0Set = false;
    note("WP%u removed. Press Mark to record it again.", (unsigned)_wpCount);
  }

  template <class S>
  void averageStep(S& s, uint32_t now, float yaw, float pitch, bool fix, float l, float r) {
    if (fabsf(l) > 0.01f || fabsf(r) > 0.01f) _moved = true;
    if (now - _avgLastSample >= kAvgSampleMs) {
      _avgLastSample = now;
      if (fix && _n < kMaxSamples) { _lat[_n] = s.lat(); _lon[_n] = s.lon(); ++_n; }
      if (!isnan(yaw)) {
        const float a = yaw * DEG_TO_RAD;
        _sumSin += sinf(a); _sumCos += cosf(a); _sumPitch += pitch; ++_yawN;
      }
    }
    if (now - _avgStart >= kAvgMs) finishWaypoint(now);
  }

  void finishWaypoint(uint32_t now) {
    _avgActive = false;
    double mLat = 0, mLon = 0, maxD = 0, sumSq = 0;
    if (_n) {
      for (uint8_t i = 0; i < _n; ++i) { mLat += _lat[i]; mLon += _lon[i]; }
      mLat /= _n; mLon /= _n;
      const double kx = kMPerDeg * cos(mLat * DEG_TO_RAD);
      for (uint8_t i = 0; i < _n; ++i) {
        const double dx = (_lon[i] - mLon) * kx, dy = (_lat[i] - mLat) * kMPerDeg;
        const double d2 = dx * dx + dy * dy;
        sumSq += d2;
        if (d2 > maxD) maxD = d2;
      }
    }
    const float spread = (float)sqrt(maxD), rms = _n ? (float)sqrt(sumSq / _n) : 0.0f;
    const float yawM = _yawN ? atan2f(_sumSin, _sumCos) * RAD_TO_DEG : NAN;
    const bool ok = _n > 0 && !_moved;
    const char* verdict = !_n ? "NO GPS FIX - undo and mark again" : (_moved ? "MOVED - undo and mark again" : "ok");
    add(F_EVENTS, "%lu,WP,%u,%.7f,%.7f,%.2f,%.2f,%.1f,%u,%d,,,,,,%s\n", (unsigned long)(now - _t0),
        (unsigned)_wpCount, mLat, mLon, spread, rms, yawM, (unsigned)_n, _moved ? 1 : 0, verdict);
    Serial.printf("# WP%u %s spread %.1f m\n", (unsigned)_wpCount, verdict, spread);
    if (!_pitch0Set && !_moved && _yawN) { _pitch0 = _sumPitch / _yawN; _pitch0Set = true; }
    const uint8_t done = _wpCount++;
    if (!ok) { note("WP%u: %s.", (unsigned)done, verdict); return; }
    if (done >= zoneLastWp(_zone) && (_mode != 0 || _zone == 4))
      note("WP%u saved, spread %.1f m. %s done: press Stop and save.", (unsigned)done, spread,
           _mode == 0 ? "Full path" : "Zone");
    else
      note("WP%u saved, GPS spread %.1f m. Drive on.", (unsigned)done, spread);
    if (_mode == 0 && _zone < 4 && done == zoneLastWp(_zone)) nextZone(now, false, "zone end waypoint saved");
  }

  // ---------- detected features ----------
  /**
   * @brief One-word guess at what a run's shape resembles. A hint for analysis, not a detection.
   * @param r The run to classify.
   * @param cal Calibration thresholds it is compared against.
   * @return A static string; "-" when it matches nothing. Never null.
   * @note Every threshold behind this is an uncalibrated placeholder (see NavCal), so the hint is
   *       only as good as those values. The numeric columns beside it are the real record.
   */
  static const char* featHint(const NavRun& r, const NavCal& cal) {
    if (r.chordMm >= cal.panelMinLenMm && r.tiltDeg <= cal.panelMaxTiltDeg) return "panel?";
    if (r.tiltDeg >= 60 && r.chordMm >= cal.panelMinLenMm) return "side wall";
    if (r.chordMm >= cal.pillarMinMm && r.chordMm <= cal.pillarMaxMm &&
        r.nearestMm <= cal.pillarMaxRangeMm) return "pillar?";
    return "-";
  }

  /**
   * @brief Writes everything the LiDAR geometry detected this sample into the features file.
   * @param s Sensors, read only; uses the 1-degree map through NavLidar.
   * @param t Milliseconds since the run started, for the t_ms column.
   * @return Nothing; rows are appended to the buffered features file.
   *
   * Records the forward summary, the zone 1 end-wall test, both side summaries, any measurable
   * side opening, and up to kFeatMaxRuns segmented runs. Nothing here steers or stops the robot
   * and nothing here consults the ultrasonics - it is a record of what the LiDAR saw, written
   * beside the raw sectors in lidar.csv rather than instead of them.
   *
   * Error handling: a feature that is not present simply produces no row, so an absence is an
   * absence rather than a zero. Called at kFeatMs from update(), on the same core as the sensor
   * refresh, so it never reads the LiDAR bins while they are being written.
   */
  template <class S>
  void featureStep(const S& s, unsigned long t) {
    NavCal cal;
    uint16_t cm = 0;
    // frontCm()/blockedBy() are this class's own helpers, not NavLidar's.
    const char* who = blockedBy(s, cm);
    add(F_FEAT, "%lu,FRONT,0,,%u,,,%s\n", t, (unsigned)frontCm(s), who ? who : "clear");

    const NavSegment w = NavLidar::endWallAhead(s, cal);
    if (w.found)
      add(F_FEAT, "%lu,ENDWALL,0,%u,%u,,,ahead\n", t, (unsigned)w.lengthMm, (unsigned)w.rangeMm);

    const NavSide L = NavLidar::sideSummary(s, cal, -90, 25);
    const NavSide R = NavLidar::sideSummary(s, cal, 90, 25);
    add(F_FEAT, "%lu,SIDE_L,-90,%u,%u,,,%s\n", t, (unsigned)L.hits, (unsigned)L.nearestMm,
        L.wall ? "wall" : "open");
    add(F_FEAT, "%lu,SIDE_R,90,%u,%u,,,%s\n", t, (unsigned)R.hits, (unsigned)R.nearestMm,
        R.wall ? "wall" : "open");

    uint16_t ow;
    int16_t of, ot;
    if (NavLidar::findSideOpening(s, cal, -90, 70, ow, of, ot))
      add(F_FEAT, "%lu,OPEN_L,%d,%d,%u,,,opening\n", t, (int)of, (int)ot, (unsigned)ow);
    if (NavLidar::findSideOpening(s, cal, 90, 70, ow, of, ot))
      add(F_FEAT, "%lu,OPEN_R,%d,%d,%u,,,opening\n", t, (int)of, (int)ot, (unsigned)ow);

    NavRunList runs;
    NavLidar::scanRuns(s, cal.wallMaxMm, 200, 3, runs);
    const uint8_t lim = runs.count < kFeatMaxRuns ? runs.count : kFeatMaxRuns;
    add(F_FEAT, "%lu,RUNS,,%u,,,,%s\n", t, (unsigned)runs.count,
        (runs.count > lim || runs.dropped) ? "some not logged" : "all logged");
    for (uint8_t i = 0; i < lim; ++i) {
      const NavRun& r = runs.run[i];
      add(F_FEAT, "%lu,RUN,%d,%u,%u,%u,%u,%s\n", t, (int)r.bearingDeg, (unsigned)r.spanDeg,
          (unsigned)r.nearestMm, (unsigned)r.chordMm, (unsigned)r.tiltDeg, featHint(r, cal));
    }
  }

  // ---------- straight-line hold ----------
  // shortest way round: any angle difference folded into -180..180
  static float wrapDeg(float d) {
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
  }

  // Works out the wheel-speed difference that brings the box back onto its reference heading.
  // Called every pass; autoDrive() applies the result, so the correction is always one loop pass
  // old - a few milliseconds against a yaw that only moves at 4 Hz.
  void headingStep(float yaw, bool imu) {
    if (!_rec || _driveSpeed <= 0.0f || !_yaw0Set || !imu || isnan(yaw)) {
      _yawCorr = 0;         // no heading to hold: drive both wheels the same
      _yawErr = 0;
      return;
    }
    _yawErr = wrapDeg(yaw - _yaw0);
    float want = 0.0f;
    if (fabsf(_yawErr) > kYawDeadDeg) {
      // deadband subtracted, so the correction starts at zero on the edge instead of stepping
      const float e = _yawErr > 0 ? _yawErr - kYawDeadDeg : _yawErr + kYawDeadDeg;
      want = constrain(kYawGain * e, -kYawMaxCorr, kYawMaxCorr);
    }
    // Zone 1 adds the lateral term. Sign is taken from the recording, not from a datasheet: in
    // Z1-R1 a positive yaw error coincided with the left wall receding (the box translating
    // right), and a positive correction demonstrably drove that error back to zero. So the same
    // positive correction is what pulls a right-translating box back, and a left-hand wall whose
    // distance has grown therefore calls for a positive term; a right-hand wall for a negative.
    _wallCorr = 0.0f;
    if (_wallSide && _wallRefMm) {
      const float off = (float)_wallNowMm - (float)_wallRefMm;   // + = moved away from that wall
      const float term = kWallGain * off * (_wallSide < 0 ? +1.0f : -1.0f);
      _wallCorr = constrain(term, -kWallMaxCorr, kWallMaxCorr);
    }
    want = constrain(want + _wallCorr, -kYawMaxCorr, kYawMaxCorr);
    _yawCorr += kYawSmooth * (want - _yawCorr);
  }

  /**
   * @brief Tracks the side wall of a zone-1 segment and keeps its reference distance.
   * @param s Sensors, read only; uses the 1-degree LiDAR map through NavLidar.
   * @param now millis().
   * @return Nothing; updates _wallSide, _wallRefMm, _wallNowMm and _wallLostSince.
   *
   * Picks the side whose returns are inside kWallMinMm..kWallMaxMm at the moment the heading
   * reference is taken, preferring the nearer one, and keeps that side for the whole segment so
   * it cannot swap walls halfway. A reading that jumps more than kWallJumpMm from the last one is
   * discarded as a different object rather than a movement of the box. If no usable reading
   * arrives for kWallLostMs the side is dropped and the controller falls back to heading only,
   * which is logged as a MODE event.
   *
   * Error handling: does nothing at all when the zone has no wall (zoneUsesWall), when no
   * heading reference exists yet, or when the LiDAR is not delivering frames.
   */
  template <class S>
  void wallStep(const S& s, uint32_t now) {
    if (!_rec || !zoneUsesWall(_zone) || !_yaw0Set || !s.lidarOk()) { _wallSide = 0; return; }
    NavCal cal;
    const NavSide left = NavLidar::sideSummary(s, cal, -90, 25);
    const NavSide right = NavLidar::sideSummary(s, cal, 90, 25);
    const bool lOk = left.nearestMm >= kWallMinMm && left.nearestMm <= kWallMaxMm;
    const bool rOk = right.nearestMm >= kWallMinMm && right.nearestMm <= kWallMaxMm;

    if (!_wallSide) {                       // choose a side once, at the start of the segment
      if (lOk && (!rOk || left.nearestMm <= right.nearestMm)) { _wallSide = -1; _wallRefMm = left.nearestMm; }
      else if (rOk)                                           { _wallSide = +1; _wallRefMm = right.nearestMm; }
      if (_wallSide) {
        _wallNowMm = _wallRefMm;
        _wallLostSince = 0;
        char m[56];
        snprintf(m, sizeof m, "wall %s, reference %u mm", _wallSide < 0 ? "LEFT" : "RIGHT",
                 (unsigned)_wallRefMm);
        evText(now, "WALL_REF", m);
        Serial.printf("# %s\n", m);
      }
      return;
    }
    const uint16_t mm = _wallSide < 0 ? left.nearestMm : right.nearestMm;
    const bool ok = (_wallSide < 0 ? lOk : rOk) &&
                    abs((int)mm - (int)_wallNowMm) <= (int)kWallJumpMm;
    if (ok) {
      _wallNowMm = mm;
      _wallLostSince = 0;
      return;
    }
    if (!_wallLostSince) _wallLostSince = now ? now : 1;
    else if (now - _wallLostSince >= kWallLostMs) {
      evText(now, "WALL_LOST", "no usable side wall, heading only");
      Serial.println(F("# WALL_LOST: heading only"));
      _wallSide = 0;
      _wallRefMm = 0;
    }
  }

  // ---------- ramps ----------
  uint8_t rampState() const { return _onRamp ? (_offPending ? 3 : 2) : (_cand ? 1 : 0); }

  void resetRampState() { _onRamp = _cand = _offPending = false; _rampsZone = 0; }

  void rampStep(uint32_t now, float p) {
    if (isnan(p)) return;
    const float ap = fabsf(p);
    if (!_onRamp) {
      if (ap > kRampOnDeg) {
        if (!_cand) { _cand = true; _candStart = now; _firstSign = p < 0 ? -1 : 1; _pMin = _pMax = p; }
        _pMin = fminf(_pMin, p); _pMax = fmaxf(_pMax, p);
        if (now - _candStart >= kRampOnMs) { _onRamp = true; _offPending = false; }
      } else {
        _cand = false;
      }
      return;
    }
    _pMin = fminf(_pMin, p); _pMax = fmaxf(_pMax, p);
    if (ap < kRampOffDeg) {
      if (!_offPending) { _offPending = true; _offSince = now; }
      else if (now - _offSince >= kRampOffMs) {
        ++_rampsZone;
        const char* first = _firstSign < 0 ? "UP" : "DOWN";
        const int8_t want = zoneRampSign(_zone);
        const float peak = fmaxf(fabsf(_pMin), fabsf(_pMax));
        char what[56];
        if (_rampsZone > zoneRamps(_zone) || _zone > 3)
          snprintf(what, sizeof what, "EXTRA ramp - check");
        else if (want != 0 && _firstSign != want)
          snprintf(what, sizeof what, "R%u went %s first, expected %s", (unsigned)_zone, first,
                   want < 0 ? "UP" : "DOWN");
        // The uneven floor adds up to kGroundDeg on top of the ramp itself, so only a peak past
        // their sum is really bigger than expected.
        else if (peak > kRampMaxDeg + kGroundDeg)
          snprintf(what, sizeof what, "R%u peak %.1f deg, over the %.1f expected", (unsigned)_zone,
                   peak, kRampMaxDeg);
        else
          snprintf(what, sizeof what, "route ramp R%u", (unsigned)_zone);
        add(F_EVENTS, "%lu,RAMP,,,,,,,,,%u,%.1f,%.1f,%lu,%s,%s\n", (unsigned long)(_candStart - _t0),
            (unsigned)_rampsZone, _pMin, _pMax, (unsigned long)(_offSince - _candStart), first, what);
        Serial.printf("# ramp %u: %.1f to %.1f deg, %s first\n", (unsigned)_rampsZone, _pMin, _pMax, first);
        note("Ramp %u done: pitch %.1f to %.1f deg, %s first.", (unsigned)_rampsZone, _pMin, _pMax, first);
        _onRamp = _cand = _offPending = false;
      }
    } else {
      _offPending = false;
    }
  }

  // Closest thing straight ahead in cm: the front LiDAR sectors and both front ultrasonics.
  // Each reports "nothing there" in its own way - the LiDAR gives 0, an ultrasonic its maximum -
  // so with no sensor working this stays large and the abort never fires: it cannot see.
  template <class S>
  static uint16_t frontCm(S& s) {
    uint16_t d = 300;
    const uint16_t cand[3] = {minSectors(s, -3, 3), (uint16_t)s.usCm(0), (uint16_t)s.usCm(4)};
    for (uint8_t i = 0; i < 3; ++i)
      if (cand[i] > 0 && cand[i] < d) d = cand[i];
    return d;
  }

  // Which sensor, if any, sees something too close to carry on driving forward. Returns its name
  // and writes that distance to cm; nullptr means clear. Every ultrasonic is checked, each
  // against the limit for the direction it faces (see kStopCm / kSideCm / kBackCm). When more
  // than one is blocked the closest one is named.
  template <class S>
  static const char* blockedBy(S& s, uint16_t& cm) {
    static const char* const kName[6] = {"LiDAR front", "front upper", "front lower",
                                         "left", "right", "back"};
    const uint16_t d[6] = {minSectors(s, -3, 3), (uint16_t)s.usCm(0), (uint16_t)s.usCm(4),
                           (uint16_t)s.usCm(1), (uint16_t)s.usCm(2), (uint16_t)s.usCm(3)};
    const uint16_t lim[6] = {kStopCm, kStopCm, kStopCm, kSideCm, kSideCm, kBackCm};
    const char* who = nullptr;
    cm = 0;
    for (uint8_t i = 0; i < 6; ++i)
      if (lim[i] && d[i] > 0 && d[i] < lim[i] && (!who || d[i] < cm)) { cm = d[i]; who = kName[i]; }
    return who;
  }

  // closest LiDAR return over sectors [from, to] (wraps around), 0 = none
  template <class S>
  static uint16_t minSectors(S& s, int from, int to) {
    uint16_t best = 0;
    for (int k = from; k <= to; ++k) {
      const uint8_t i = (uint8_t)(((k % kSectors) + kSectors) % kSectors);
      const uint16_t v = s.lidarCm(i);
      if (v > 0 && (best == 0 || v < best)) best = v;
    }
    return best;
  }

  // ---------- state ----------
  bool     _fsOk = false, _rec = false, _full = false;
  float    _driveSpeed = 0;   // forward speed of the automatic test, 0 = the logger never drives
  uint8_t  _mode = 1, _zone = 1;
  char     _run[12] = {0};
  uint32_t _t0 = 0, _zoneT0 = 0, _total = 0, _usedAtStart = 0, _usedIdle = 0, _written = 0,
           _lastUsedCheck = 0;
  File     _f[F_COUNT];
  bool     _fileOk[F_COUNT] = {false, false, false, false, false};
  char     _buf[F_COUNT][kBufSize];
  size_t   _bufLen[F_COUNT] = {0, 0, 0, 0, 0};
  uint32_t _lastFlush[F_COUNT] = {0, 0, 0, 0, 0};
  uint32_t _lastW[F_COUNT] = {0, 0, 0, 0, 0};
  uint8_t  _rr = 0;
  char     _line[512];
  uint32_t _gpsN = 0, _gpsFixN = 0;
  // web requests
  volatile bool    _reqStart = false, _reqStop = false, _reqMark = false, _reqUndo = false,
                   _reqNext = false, _reqDelete = false;
  volatile uint8_t _reqMode = 1;
  char     _delName[16] = {0};
  // status (double buffer)
  char     _status[2][448] = {{0}, {0}};
  volatile uint8_t _statusIdx = 0;
  uint32_t _lastStatus = 0;
  char     _note[96] = {0};
  bool     _lastFix = false, _lastImu = false;
  uint16_t _frontCm = 300;    // closest thing in front, last pass
  uint32_t _blockSince = 0;   // first pass something was too close, 0 = clear
  const char* _blockWhere = nullptr;  // which sensor is too close, nullptr = clear
  uint16_t _blockCm = 0;              // and at what distance
  // waypoints
  uint8_t  _wpCount = 0, _wpZoneStart = 0;
  bool     _avgActive = false;
  uint32_t _avgStart = 0, _avgLastSample = 0;
  uint8_t  _n = 0;
  uint16_t _yawN = 0;
  double   _lat[kMaxSamples], _lon[kMaxSamples];
  float    _sumSin = 0, _sumCos = 0, _sumPitch = 0;
  bool     _moved = false;
  // ramps
  bool     _pitch0Set = false;
  float    _pitch0 = 0;
  bool     _yaw0Set = false;  // reference heading taken, straight-line hold active
  float    _yaw0 = 0;         // the heading the drive holds, deg
  float    _yawErr = 0;       // current error against it, deg (status text)
  float    _yawCorr = 0;      // wheel-speed difference it asks for, smoothed
  float    _wallCorr = 0;     // the lateral part of it (zone 1 only)
  int8_t   _wallSide = 0;     // -1 left, +1 right, 0 = heading only
  uint16_t _wallRefMm = 0;    // distance held, captured at the segment start
  uint16_t _wallNowMm = 0;    // last accepted reading on that side
  uint32_t _wallLostSince = 0;
  uint32_t _errBadSince = 0;  // first pass the heading error went past kErrStopDeg
  float    _zeroSum = 0;      // pitch samples of the settling window at the start of a run
  uint16_t _zeroN = 0;
  bool     _onRamp = false, _cand = false, _offPending = false;
  uint32_t _candStart = 0, _offSince = 0;
  int8_t   _firstSign = 0;
  float    _pMin = 0, _pMax = 0;
  uint8_t  _rampsZone = 0;
};
