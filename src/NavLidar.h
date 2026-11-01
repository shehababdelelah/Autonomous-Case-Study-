#pragma once
// NavLidar - 2D LiDAR geometry for autonomous navigation of the CAS box, kit 8.
//
// Reads the full resolution map from Sensors (360 bins of 1 deg in mm, bin 270 = ahead, bins
// increase clockwise - see Sensors::lidarBinMm) and turns it into the few geometric facts the
// navigator needs: is something in my path, where are the walls beside me, is there a gap on
// the left, is there a pillar, is there a flat panel across my direction of travel.
//
// Coordinate frame used throughout (robot frame, millimetres):
//   x  forward,  y  to the RIGHT,  origin at the LiDAR
// A bin at angle a (clockwise from ahead) and range r maps to x = r*cos(a), y = r*sin(a).
// That matches the bin order: increasing bin = increasing a = swinging to the right.
//
// -------------------------------------------------------------------------------------------
// WHAT THIS FILE DOES NOT KNOW
// -------------------------------------------------------------------------------------------
// Every building measurement is a calibration parameter in NavCal below, because none of it is
// in this repository: corridor width, pillar diameter, panel length, the width of the opening
// between B and C, and the box's own footprint. The defaults are placeholders chosen only so
// the code compiles and can be unit tested - they are NOT measurements of your building and
// landmark detection will not work until they are measured on site.
//
// -------------------------------------------------------------------------------------------
// TWO HARDWARE LIMITS THAT THE CALLER MUST RESPECT
// -------------------------------------------------------------------------------------------
// 1. LidarScan.Config.minMm is 200, and points closer than that are thrown away before they
//    ever reach a bin ("ignore the box itself"). So the LiDAR physically cannot report an
//    obstacle at 120 mm. kStopMm is clamped to kLidarFloorMm for that reason, and a request
//    below the floor is reported by stopThresholdIsReachable() instead of being silently
//    accepted. Lowering minMm is not a fix: the box's own structure would then read as a
//    permanent obstacle directly ahead.
// 2. A bin needs >= 2 hits newer than 500 ms or it reads 0, and Sensors rebuilds its snapshot
//    at 4 Hz. At a forward command of 0.3 the box covers a useful fraction of the stopping
//    distance inside one update, so the threshold is a detection distance, not a guarantee of
//    stopping before contact. Raise it rather than lower it, and see scanAgeWarning().
#include <Arduino.h>
#include <math.h>

/// Calibration block. Every value marked MEASURE must be measured on the real route.
struct NavCal {
  // ---- robot footprint (MEASURE) ----
  uint16_t bodyHalfWidthMm = 150;   ///< MEASURE: half the widest part of the box
  uint16_t pathMarginMm = 60;       ///< clearance added either side when judging "in my path"

  // ---- obstacle stop ----
  uint16_t stopMm = 200;            ///< forward-path emergency stop distance (see limit 1)
  uint8_t  stopMinHits = 3;         ///< points inside the path box before it counts as real
  uint16_t forwardConeDeg = 60;     ///< bins scanned either side of ahead for path obstacles

  // ---- walls / corridor (MEASURE) ----
  uint16_t wallMaxMm = 4000;        ///< ignore returns beyond this when looking for side walls
  uint16_t wallMinHits = 12;        ///< bins needed before a side is called a wall

  // ---- pillar (MEASURE) ----
  uint16_t pillarMinMm = 120;       ///< MEASURE: narrowest the pillar may appear
  uint16_t pillarMaxMm = 700;       ///< MEASURE: widest the pillar may appear
  uint16_t pillarMaxRangeMm = 3000; ///< MEASURE: only trust pillars nearer than this
  uint16_t pillarStandoffMm = 400;  ///< MEASURE: free space needed behind it to be a pillar

  // ---- rectangular panel (MEASURE) ----
  uint16_t panelMinLenMm = 600;     ///< MEASURE: shortest segment still counted as the panel
  uint16_t panelMaxRangeMm = 3000;  ///< MEASURE
  uint8_t  panelMaxTiltDeg = 25;    ///< how far off perpendicular the panel may sit

  // ---- opening between B and C (MEASURE) ----
  uint16_t openingMinWidthMm = 1500; ///< MEASURE: narrowest gap that counts as the opening
  uint16_t openingProbeMm = 5000;    ///< a side bin beyond this reads as "open"

  // ---- end of zone 1 wall ----
  uint16_t endWallMm = 1200;        ///< MEASURE: how far ahead the end wall is recognised
  uint8_t  endWallMinSpanDeg = 30;  ///< it must fill at least this much of the forward cone
  uint16_t endWallFlatMm = 250;     ///< and be this flat across that span to be a wall
};

/// One LiDAR return in the robot frame.
struct NavPoint {
  int16_t angleDeg = 0;   ///< clockwise from ahead, -180..179
  uint16_t rangeMm = 0;   ///< 0 = invalid, never below the LiDAR floor
  float x = 0, y = 0;     ///< mm, x forward, y right
};

/// What a side of the robot looks like.
struct NavSide {
  bool wall = false;      ///< enough consistent returns to call it a wall
  uint16_t nearestMm = 0; ///< closest return on that side, 0 = nothing
  uint8_t hits = 0;       ///< how many bins contributed
};

/// A compact object, e.g. a pillar.
struct NavCluster {
  bool found = false;
  int16_t angleDeg = 0;   ///< bearing of its middle, clockwise from ahead
  uint16_t rangeMm = 0;   ///< range of its middle
  uint16_t widthMm = 0;   ///< apparent width
};

/// A straight run of returns, e.g. a wall or the panel.
struct NavSegment {
  bool found = false;
  uint16_t lengthMm = 0;  ///< end to end
  uint16_t rangeMm = 0;   ///< range of its middle
  int16_t angleDeg = 0;   ///< bearing of its middle
  uint8_t tiltDeg = 0;    ///< 0 = square across the direction of travel
};

/// One run of adjacent LiDAR returns at a continuous range: a wall, a panel or a pillar.
struct NavRun {
  int16_t fromDeg, toDeg;   ///< bearings of the two ends, clockwise from ahead
  int16_t bearingDeg;       ///< bearing of the middle
  uint16_t spanDeg;         ///< how many degrees it covers
  uint16_t nearestMm;       ///< closest return in the run
  uint16_t chordMm;         ///< straight-line distance between its two ends
  uint8_t tiltDeg;          ///< 0 = square across the path, 90 = running alongside it
};

/// Result of walking a whole scan. Fixed size: no allocation on the ESP32.
struct NavRunList {
  static const uint8_t kMax = 16;
  NavRun run[kMax];
  uint8_t count = 0;
  uint8_t dropped = 0;      ///< runs that did not fit in kMax
};

namespace NavLidar {

/// Lowest distance this LiDAR can ever report (LidarScan.Config.minMm). Nothing nearer exists.
static const uint16_t kLidarFloorMm = 200;

// forwardObstacleRequiresStop() needs this before it is defined below.
template <class S>
inline NavSegment endWallAhead(const S& s, const NavCal& cal);

/**
 * @brief Is a requested emergency-stop distance actually reachable by this LiDAR?
 * @param wantMm The configured stop distance in mm.
 * @return False when wantMm is below the hardware floor, i.e. unreachable.
 * @note Lets the caller report the clamp instead of appearing to honour an impossible value.
 *       The requested 120 mm is below the 200 mm floor and so returns false.
 */
inline bool stopThresholdIsReachable(uint16_t wantMm) { return wantMm >= kLidarFloorMm; }

/**
 * @brief Clamps a stop distance up to what the LiDAR can report.
 * @param wantMm Requested distance in mm.
 * @return wantMm, or kLidarFloorMm when the request is below the floor.
 */
inline uint16_t clampStopMm(uint16_t wantMm) {
  return wantMm < kLidarFloorMm ? kLidarFloorMm : wantMm;
}

/**
 * @brief Reads one bin as a validated point in the robot frame.
 * @param s Sensors, for lidarBinMm()/lidarBin().
 * @param angleDeg Bearing wanted, clockwise from ahead.
 * @param maxMm Ranges at or beyond this are treated as no return.
 * @return A NavPoint; rangeMm == 0 means the measurement was invalid and must be ignored.
 * @note Rejects 0 (no reliable return), anything under the LiDAR floor, and anything over
 *       maxMm. Those three are the only invalid cases the LD20 reader can produce.
 */
template <class S>
inline NavPoint readPoint(const S& s, int angleDeg, uint16_t maxMm) {
  NavPoint p;
  p.angleDeg = (int16_t)(((angleDeg + 180) % 360 + 360) % 360 - 180);
  p.x = p.y = 0;
  const uint16_t mm = s.lidarBinMm(S::lidarBin(angleDeg));
  p.rangeMm = (mm == 0 || mm < kLidarFloorMm || mm >= maxMm) ? 0 : mm;
  if (p.rangeMm) {
    const float a = p.angleDeg * (float)DEG_TO_RAD;
    p.x = p.rangeMm * cosf(a);
    p.y = p.rangeMm * sinf(a);
  }
  return p;
}

/**
 * @brief Decides whether a LiDAR return lies inside the strip the robot is about to drive through.
 * @param p A validated point in the robot frame.
 * @param cal Calibration: body half width and path margin set the strip's width.
 * @param withinMm Only points closer than this along x count.
 * @return True when the point is ahead, inside the strip, and nearer than withinMm.
 * @note This is what keeps a corridor wall from reading as an obstacle: a wall 700 mm to the
 *       side has |y| far outside the strip however close its slant range looks.
 */
inline bool pointIsInPath(const NavPoint& p, const NavCal& cal, uint16_t withinMm) {
  if (!p.rangeMm || p.x <= 0) return false;
  const float halfStrip = (float)cal.bodyHalfWidthMm + (float)cal.pathMarginMm;
  return fabsf(p.y) <= halfStrip && p.x <= (float)withinMm;
}

/**
 * @brief Determines whether a forward-path obstacle requires an emergency stop.
 * @param s Sensors, read only, for the 1-degree LiDAR map.
 * @param cal Calibration: stopMm, the footprint strip, forwardConeDeg and stopMinHits.
 * @param endWallExempt True while the navigator is deliberately closing on the zone 1 end
 *        wall, which is a route goal rather than an emergency; the 12 cm style stop is then
 *        suppressed only for returns that look like that flat wall across the whole cone.
 * @param nearestMm Out: range of the closest qualifying point, 0 when none.
 * @return True when a qualifying LiDAR obstacle requires stopping.
 *
 * Sensor data used: ONLY the 2D LiDAR. Ultrasonic readings are deliberately not consulted here
 * and must never be - that is required policy, not an oversight. GPS is likewise not consulted.
 *
 * Noise handling: a single stray return cannot stop the robot. At least cal.stopMinHits points
 * must fall inside the path strip in the same scan. Invalid bins (0, below the LiDAR floor,
 * beyond range) are discarded by readPoint() before they are considered.
 *
 * Assumptions and limits: the strip is a rectangle, so it over-approximates a turning box; the
 * threshold is a detection distance and not a guaranteed stopping distance (see the header);
 * and cal.stopMm is clamped up to the LiDAR floor, so a request below 200 mm acts at 200 mm.
 *
 * Side effects: none. Pure function of the current scan.
 */
template <class S>
inline bool forwardObstacleRequiresStop(const S& s, const NavCal& cal, bool endWallExempt,
                                        uint16_t& nearestMm) {
  nearestMm = 0;
  const uint16_t within = clampStopMm(cal.stopMm);
  const int cone = cal.forwardConeDeg;
  uint8_t hits = 0;
  for (int a = -cone; a <= cone; ++a) {
    // Full range here: pointIsInPath() does the distance and footprint test itself.
    const NavPoint p = readPoint(s, a, S::kLidarMaxMm);
    if (!pointIsInPath(p, cal, within)) continue;
    ++hits;
    if (!nearestMm || p.rangeMm < nearestMm) nearestMm = p.rangeMm;
  }
  if (hits < cal.stopMinHits) { nearestMm = 0; return false; }
  // The zone 1 end wall is approached on purpose. Suppress the emergency stop for it only, and
  // only while the navigator says so - any obstacle that is not that broad flat wall still stops.
  if (endWallExempt && endWallAhead(s, cal).found) return false;
  return true;
}

/**
 * @brief Looks for a broad flat surface square across the path, i.e. the end of zone 1.
 * @param s Sensors, read only.
 * @param cal Calibration: endWallMm, endWallMinSpanDeg, endWallFlatMm, forwardConeDeg.
 * @return A NavSegment; found == true when a wall-like surface fills the forward cone.
 * @note Distinguishes an end wall from a side wall by requiring returns across a wide span of
 *       the forward cone at a nearly constant forward distance (x), which a wall beside the
 *       robot cannot produce. Confirmation over several scans is the caller's job.
 */
template <class S>
inline NavSegment endWallAhead(const S& s, const NavCal& cal) {
  NavSegment seg;
  const int cone = cal.forwardConeDeg;
  float xMin = 1e9f, xMax = -1e9f, xSum = 0;
  int span = 0;
  for (int a = -cone; a <= cone; ++a) {
    const NavPoint p = readPoint(s, a, cal.endWallMm);
    if (!p.rangeMm || p.x <= 0) continue;
    ++span;
    xSum += p.x;
    if (p.x < xMin) xMin = p.x;
    if (p.x > xMax) xMax = p.x;
  }
  if (span < cal.endWallMinSpanDeg) return seg;
  if ((xMax - xMin) > (float)cal.endWallFlatMm) return seg;   // not flat: not a wall ahead
  seg.found = true;
  seg.lengthMm = (uint16_t)span;          // span in degrees, not a true length at this stage
  seg.rangeMm = (uint16_t)(xSum / span);
  seg.angleDeg = 0;
  seg.tiltDeg = 0;
  return seg;
}

/**
 * @brief Summarises one side of the robot: is there a wall, and how near is it.
 * @param s Sensors, read only.
 * @param cal Calibration: wallMaxMm and wallMinHits.
 * @param centreDeg Bearing of the side, +90 for right and -90 for left.
 * @param halfSpanDeg How wide a fan around centreDeg to look at.
 * @return A NavSide with the wall flag, nearest range and contributing bin count.
 * @note Used for corridor following and as the "is it still closed" half of opening detection.
 *       Never used to stop the robot; side walls are expected and are not obstacles.
 */
template <class S>
inline NavSide sideSummary(const S& s, const NavCal& cal, int centreDeg, int halfSpanDeg) {
  NavSide side;
  for (int d = -halfSpanDeg; d <= halfSpanDeg; ++d) {
    const NavPoint p = readPoint(s, centreDeg + d, cal.wallMaxMm);
    if (!p.rangeMm) continue;
    ++side.hits;
    if (!side.nearestMm || p.rangeMm < side.nearestMm) side.nearestMm = p.rangeMm;
  }
  side.wall = side.hits >= cal.wallMinHits;
  return side;
}

/**
 * @brief Segments a whole scan into runs of adjacent returns at a continuous range.
 * @param s Sensors, read only.
 * @param maxMm Ignore returns at or beyond this, so distant clutter does not form runs.
 * @param jumpMm A range step larger than this between neighbouring degrees ends the run.
 * @param minSpanDeg Runs narrower than this are discarded as noise.
 * @param out Filled with up to NavRunList::kMax runs; out.dropped counts any that did not fit.
 * @return Nothing; results are in out.
 *
 * This is the shared front end for every landmark detector and for the survey page. A pillar
 * comes out as a run with a small spanDeg and a small chordMm; a wall or the panel as a run with
 * a large chordMm; tiltDeg then separates a surface square across the path (near 0) from one
 * running alongside it (near 90).
 *
 * Assumptions: the scan is a single revolution of a 1-degree map, so neighbouring degrees really
 * are neighbouring directions. Invalid bins break a run, which is intended - a gap is a gap.
 * Side effects: none.
 */
/**
 * @brief Finishes one run and appends it, discarding anything narrower than minSpanDeg.
 * @param out The list to append to; out.dropped grows when the list is already full.
 * @param fromDeg,toDeg Bearings of the run's two ends.
 * @param nearest Closest range inside the run, mm.
 * @param x0,y0,x1,y1 The two end points in the robot frame, mm.
 * @param minSpanDeg Runs narrower than this are treated as noise and dropped.
 * @note Internal helper for scanRuns(). Computes the chord and the tilt: tiltDeg is the angle
 *       between the run's chord and the lateral axis, so 0 means the surface lies square across
 *       the direction of travel and 90 means it runs alongside it.
 */
inline void appendRun(NavRunList& out, int fromDeg, int toDeg, uint16_t nearest,
                      float x0, float y0, float x1, float y1, uint16_t minSpanDeg) {
  const int span = toDeg - fromDeg + 1;
  if (span < (int)minSpanDeg) return;
  if (out.count >= NavRunList::kMax) { ++out.dropped; return; }
  NavRun& r = out.run[out.count++];
  r.fromDeg = (int16_t)fromDeg;
  r.toDeg = (int16_t)toDeg;
  r.bearingDeg = (int16_t)((fromDeg + toDeg) / 2);
  r.spanDeg = (uint16_t)span;
  r.nearestMm = nearest;
  const float dx = x1 - x0, dy = y1 - y0;
  r.chordMm = (uint16_t)fminf(65000.0f, sqrtf(dx * dx + dy * dy));
  r.tiltDeg = (uint8_t)lroundf(atan2f(fabsf(dx), fabsf(dy)) * (float)RAD_TO_DEG);
}

template <class S>
inline void scanRuns(const S& s, uint16_t maxMm, uint16_t jumpMm, uint16_t minSpanDeg,
                     NavRunList& out) {
  out.count = 0;
  out.dropped = 0;
  bool open = false;
  int startDeg = 0, prevDeg = 0;
  uint16_t prevMm = 0, nearest = 0;
  float x0 = 0, y0 = 0, x1 = 0, y1 = 0;   // first and last point of the open run

  // -180..179, then one extra pass with no return so a run reaching the end is still closed.
  // A run straddling straight-behind is simply cut there: nothing on the route is identified
  // from directly behind the box.
  for (int a = -180; a <= 180; ++a) {
    NavPoint p;
    if (a == 180) {
      p.angleDeg = 180;
      p.rangeMm = 0;
      p.x = p.y = 0;
    } else {
      p = readPoint(s, a, maxMm);
    }
    if (p.rangeMm) {
      const bool cont = open && (a - prevDeg) == 1 &&
                        abs((int)p.rangeMm - (int)prevMm) <= (int)jumpMm;
      if (!cont) {
        if (open) appendRun(out, startDeg, prevDeg, nearest, x0, y0, x1, y1, minSpanDeg);
        open = true;
        startDeg = a;
        nearest = p.rangeMm;
        x0 = p.x;
        y0 = p.y;
      } else if (p.rangeMm < nearest) {
        nearest = p.rangeMm;
      }
      prevMm = p.rangeMm;
      prevDeg = a;
      x1 = p.x;
      y1 = p.y;
    } else if (open) {                     // an invalid bin ends the run: a gap is a gap
      appendRun(out, startDeg, prevDeg, nearest, x0, y0, x1, y1, minSpanDeg);
      open = false;
    }
  }
}

/**
 * @brief Finds the widest run of "open" directions on one side, i.e. a gap in a boundary.
 * @param s Sensors, read only.
 * @param cal Calibration: openingProbeMm decides what counts as open, openingMinWidthMm the size.
 * @param centreDeg Side to examine, -90 for left and +90 for right.
 * @param halfSpanDeg Fan either side of centreDeg to examine.
 * @param widthMm Out: estimated width of the gap across the robot's direction of travel.
 * @param fromDeg Out, toDeg Out: the bearings the gap spans.
 * @return True when a gap at least cal.openingMinWidthMm wide, WITH a boundary on both sides,
 *         was found. False when no such gap exists or when a gap cannot be measured.
 *
 * A direction counts as open when it has no valid return or a return beyond openingProbeMm.
 * Crucially, an open stretch is only measured when there is a real return immediately before
 * AND immediately after it: the width is the distance between those two boundary points. A
 * stretch of missing returns with nothing either side is therefore never reported as an
 * opening, which is what stops an empty scan, a blind spot or a dropout from looking like the
 * gap between the buildings. Confirming the result over consecutive scans is still the
 * navigator's job, not this function's.
 *
 * @note An earlier version estimated the width from openingProbeMm and the angular span. That
 *       is wrong: the probe distance is further away than the boundary, so it overestimated
 *       every gap and reported an opening even for a completely empty scan.
 */
template <class S>
inline bool findSideOpening(const S& s, const NavCal& cal, int centreDeg, int halfSpanDeg,
                            uint16_t& widthMm, int16_t& fromDeg, int16_t& toDeg) {
  widthMm = 0;
  fromDeg = toDeg = 0;
  bool found = false;
  // inRun must be its own flag. Using a negative runStart as the "not in a run" sentinel is
  // wrong here: bearings on the left side ARE negative, so the sentinel aliased real data and
  // the stretch was restarted on every open bearing, meaning it was never measured.
  bool inRun = false;
  int best = 0, runStart = 0;
  NavPoint edgeBefore, lastValid;
  edgeBefore.rangeMm = 0;
  lastValid.rangeMm = 0;

  for (int d = -halfSpanDeg; d <= halfSpanDeg; ++d) {
    const int a = centreDeg + d;
    const NavPoint p = readPoint(s, a, S::kLidarMaxMm);
    const bool isOpen = !p.rangeMm || p.rangeMm >= cal.openingProbeMm;
    if (isOpen) {
      if (!inRun) {               // a stretch begins: remember what bounded it
        inRun = true;
        runStart = a;
        edgeBefore = lastValid;
      }
      continue;
    }
    // A real return closes the stretch. Measurable only with a boundary on both sides.
    if (inRun && edgeBefore.rangeMm) {
      const float dx = p.x - edgeBefore.x, dy = p.y - edgeBefore.y;
      const int w = (int)sqrtf(dx * dx + dy * dy);
      if (w > best) {
        best = w;
        fromDeg = (int16_t)runStart;
        toDeg = (int16_t)(a - 1);
        found = true;
      }
    }
    inRun = false;
    lastValid = p;
  }
  widthMm = (uint16_t)(best > 65000 ? 65000 : best);
  return found && widthMm >= cal.openingMinWidthMm;
}

}  // namespace NavLidar
