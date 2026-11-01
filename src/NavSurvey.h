#pragma once
// NavSurvey - LiDAR calibration survey for the CAS box, kit 8.
//
// Purpose: NavLidar's landmark detectors need building measurements that are not in this
// repository - pillar width, panel length, the width of the opening between B and C, how far
// ahead the zone 1 end wall is, the corridor width. This page shows what the LiDAR actually
// reports right now, so those numbers can be read off the robot's own data at each landmark
// and written into NavCal, instead of being guessed.
//
// Open http://192.168.<10+BOX_ID>.1/survey on a phone, drive the box to a landmark, and read:
//   - the nearest return ahead / left / right / behind
//   - whether anything is inside the forward path strip, and the end-wall verdict
//   - every segmented run: bearing, angular span, nearest range, chord length and tilt
//     (a pillar is a short chord; the panel is a long chord with a low tilt; a side wall is a
//      long chord with a tilt near 90)
//   - the widest measurable opening on each side, with its true width
//
// Threading: update() runs in loop() on core 1, the same task that refreshes Sensors, so it
// never reads the LiDAR bins while they are being written. It writes into the spare half of a
// double buffer and then flips an index, exactly as TestLogger does for its status text.
// copyReport() runs in the web server task and only ever reads the finished half. No mutex is
// needed because the index is a single volatile byte and the halves are never written in place.
#include <Arduino.h>
#include "NavLidar.h"

class NavSurvey {
public:
  static constexpr uint32_t kUpdateMs = 500;   ///< rebuild the report twice a second
  static constexpr size_t kReportSize = 1600;

  /**
   * @brief Rebuilds the survey report from the current scan, at most every kUpdateMs.
   * @param s Sensors, read only; must be the same task that calls Sensors::update().
   * @param cal Calibration block, so the report shows the thresholds being compared against.
   * @return Nothing; the finished text is published for copyReport().
   * @note Cheap enough for loop(): one pass over 360 bins for the runs plus two side fans.
   *       Does not move the robot and does not touch the ultrasonics.
   */
  template <class S>
  void update(const S& s, const NavCal& cal) {
    const uint32_t now = millis();
    if (_lastMs && now - _lastMs < kUpdateMs) return;
    _lastMs = now;

    char* out = _buf[1 - _idx];
    size_t len = 0;
    out[0] = '\0';

    if (!s.lidarOk()) {
      _put(out, len, "NO LIDAR DATA - no frames are arriving.\nCheck GPIO27 and the 230400 baud link.\n");
      _publish();
      return;
    }

    // ---- the four cardinal directions, as the navigator sees them ----
    _put(out, len, "LIDAR SURVEY   bin 270 = ahead, + bearing = clockwise (right)\n");
    _put(out, len, "ahead %5u   left %5u   right %5u   behind %5u  mm\n",
         _nearestFan(s, 0, 5), _nearestFan(s, -90, 10), _nearestFan(s, 90, 10),
         _nearestFan(s, 180, 10));

    // ---- forward path and the end-wall test ----
    uint16_t nearest = 0;
    const bool stop = NavLidar::forwardObstacleRequiresStop(s, cal, false, nearest);
    const NavSegment wall = NavLidar::endWallAhead(s, cal);
    _put(out, len, "path strip +-%u mm: %s", (unsigned)(cal.bodyHalfWidthMm + cal.pathMarginMm),
         stop ? "BLOCKED at " : "clear");
    if (stop) _put(out, len, "%u mm (stop at %u)", nearest, NavLidar::clampStopMm(cal.stopMm));
    _put(out, len, "   end wall: %s", wall.found ? "YES at " : "no");
    if (wall.found) _put(out, len, "%u mm over %u deg", wall.rangeMm, wall.lengthMm);
    _put(out, len, "\n");

    // ---- segmented runs: this is what the landmark numbers are read from ----
    NavRunList runs;
    NavLidar::scanRuns(s, cal.wallMaxMm, kJumpMm, kMinSpanDeg, runs);
    _put(out, len, "\nRUNS (jump>%u mm splits, span<%u deg dropped)  %u found",
         (unsigned)kJumpMm, (unsigned)kMinSpanDeg, runs.count);
    if (runs.dropped) _put(out, len, ", %u dropped", runs.dropped);
    _put(out, len, "\n  bearing span  near  chord tilt  looks like\n");
    for (uint8_t i = 0; i < runs.count; ++i) {
      const NavRun& r = runs.run[i];
      _put(out, len, "  %+5d  %4u %5u %6u %4u  %s\n", r.bearingDeg, r.spanDeg, r.nearestMm,
           r.chordMm, r.tiltDeg, _looksLike(r, cal));
    }

    // ---- openings, measured between real boundaries ----
    uint16_t w;
    int16_t f, t;
    _put(out, len, "\nOPENINGS (need a boundary each side, min %u mm)\n",
         (unsigned)cal.openingMinWidthMm);
    const bool lOpen = NavLidar::findSideOpening(s, cal, -90, 70, w, f, t);
    _put(out, len, "  left : %s", lOpen ? "OPEN " : "none ");
    if (w) _put(out, len, "width %u mm, %+d to %+d deg", w, f, t);
    _put(out, len, "\n");
    const bool rOpen = NavLidar::findSideOpening(s, cal, 90, 70, w, f, t);
    _put(out, len, "  right: %s", rOpen ? "OPEN " : "none ");
    if (w) _put(out, len, "width %u mm, %+d to %+d deg", w, f, t);
    _put(out, len, "\n");

    _publish();
  }

  /**
   * @brief Copies the finished report out for the web server task.
   * @param dst Destination buffer, dstSize bytes; always NUL terminated.
   * @param dstSize Size of dst; kReportSize is enough for the whole report.
   * @note Safe to call from another task: only the published half is ever read.
   */
  void copyReport(char* dst, size_t dstSize) const {
    if (!dstSize) return;
    strncpy(dst, _buf[_idx], dstSize - 1);
    dst[dstSize - 1] = '\0';
  }

private:
  static constexpr uint16_t kJumpMm = 200;    ///< range step that splits one run from the next
  static constexpr uint16_t kMinSpanDeg = 3;  ///< narrower runs are noise

  void _publish() { _idx = (uint8_t)(1 - _idx); }

  /// Appends printf-style text, never past the end of the buffer.
  void _put(char* s, size_t& len, const char* fmt, ...) {
    if (len + 1 >= kReportSize) return;
    va_list a;
    va_start(a, fmt);
    const int k = vsnprintf(s + len, kReportSize - len, fmt, a);
    va_end(a);
    if (k > 0) len += (size_t)k;
    if (len > kReportSize - 1) len = kReportSize - 1;
  }

  /// Closest valid return in a fan of +-halfSpan around a bearing, 0 = nothing.
  template <class S>
  static uint16_t _nearestFan(const S& s, int centreDeg, int halfSpan) {
    uint16_t best = 0;
    for (int d = -halfSpan; d <= halfSpan; ++d) {
      const uint16_t mm = s.lidarBinMm(S::lidarBin(centreDeg + d));
      if (mm < S::kLidarMinMm || mm >= S::kLidarMaxMm) continue;
      if (!best || mm < best) best = mm;
    }
    return best;
  }

  /// A hint only, from the run's shape against the current calibration. Not a detection.
  static const char* _looksLike(const NavRun& r, const NavCal& cal) {
    if (r.chordMm >= cal.panelMinLenMm && r.tiltDeg <= cal.panelMaxTiltDeg) return "panel?";
    if (r.tiltDeg >= 60 && r.chordMm >= cal.panelMinLenMm) return "side wall";
    if (r.chordMm >= cal.pillarMinMm && r.chordMm <= cal.pillarMaxMm &&
        r.nearestMm <= cal.pillarMaxRangeMm) return "pillar?";
    return "-";
  }

  char _buf[2][kReportSize] = {{0}, {0}};
  volatile uint8_t _idx = 0;
  uint32_t _lastMs = 0;
};
