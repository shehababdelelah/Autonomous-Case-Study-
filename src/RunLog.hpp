#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <memory>
#include <new>

// Records one row every kLogPeriodMs (main.cpp) of a run, in RAM (no flash writes while driving).
// Download it at http://192.168.18.1/log.csv after the run, and the time of every step
// (turns too) at http://192.168.18.1/steps.csv.
// Start clears it, and a restart of the box loses it.
class RunLog {
 public:
  struct Row {
    uint32_t ms;                        // time since Start
    uint8_t step, blocked;
    int8_t left, right;                 // wheel command x 100
    uint16_t heading, target;           // degrees x 10
    int16_t pitch;                      // IMU pitch x 10, nose up or down
    uint16_t distanceCm, frontCm, leftCm, rightCm;
    uint8_t pillars, walls;
    uint8_t pillarHits, wallHits;       // LiDAR directions that see a pillar / a wall
    uint16_t volts;                     // battery volts x 100, 0 = not read yet
  };
  static constexpr int kMaxRows = 3000;   // 12.5 minutes at one row every 250 ms

  // One line of steps.csv: a step that ended, how long it took, and how it ended.
  struct StepTime {
    uint8_t step;
    uint32_t ms;          // time the step worked, without the pause after it
    const char* name;     // Route::name()
    const char* result;   // "done", or why the run stopped
  };
  static constexpr int kMaxSteps = 40;

  // One line of gps.csv, written once a second while the run is saved.
  struct GpsRow {
    uint32_t ms;          // time since Start
    int32_t lat, lon;     // degrees x 10 000 000 (about 1 cm), 0 = no fix
    uint8_t sats;
    uint8_t quality;      // 0 = no fix, 1 = GPS only, 2 = GPS + EGNOS
    uint8_t hdopX10;      // horizontal dilution of precision x 10
    int16_t altM;
  };
  static constexpr int kMaxGpsRows = 900;   // 15 minutes at one row a second

  // About 72 KB. If memory is short, the log stays off and the rover still works.
  void begin() {
    rows_ = new (std::nothrow) Row[kMaxRows];
    gpsRows_ = new (std::nothrow) GpsRow[kMaxGpsRows];
  }
  // group: 0 = the whole route, 1-4 = a test group (only used in the file names).
  void clear(int group = 0) {
    count_ = 0;
    steps_ = 0;
    gpsCount_ = 0;
    group_ = group;
  }

  void addGps(const GpsRow& row) {
    if (!gpsRows_ || gpsCount_ >= kMaxGpsRows) return;
    gpsRows_[gpsCount_] = row;
    gpsCount_ = gpsCount_ + 1;
  }
  void addStep(const StepTime& step) {
    if (steps_ >= kMaxSteps) return;
    stepTimes_[steps_] = step;
    steps_ = steps_ + 1;
  }
  void add(const Row& row) {
    if (!rows_ || count_ >= kMaxRows) return;
    rows_[count_] = row;
    count_ = count_ + 1;
  }

  // Sends the CSV a piece at a time, so the whole file never has to fit in memory.
  void send(AsyncWebServerRequest* request) {
    auto cursor = std::make_shared<Cursor>();
    cursor->rows = count_;
    AsyncWebServerResponse* response = request->beginChunkedResponse(
        "text/csv", [this, cursor](uint8_t* out, size_t maxLen, size_t) -> size_t {
          return fill(*cursor, out, maxLen);
        });
    response->addHeader("Content-Disposition", fileName("run"));
    request->send(response);
  }

  // gps.csv: one line a second. Sent a piece at a time, like run.csv.
  void sendGps(AsyncWebServerRequest* request) {
    auto cursor = std::make_shared<GpsCursor>();
    cursor->rows = gpsCount_;
    AsyncWebServerResponse* response = request->beginChunkedResponse(
        "text/csv", [this, cursor](uint8_t* out, size_t maxLen, size_t) -> size_t {
          return fillGps(*cursor, out, maxLen);
        });
    response->addHeader("Content-Disposition", fileName("gps"));
    request->send(response);
  }

  // steps.csv: one line per step. Small, so it is sent in one piece.
  void sendSteps(AsyncWebServerRequest* request) {
    String text = "step,name,seconds,result\n";
    for (int i = 0; i < steps_; ++i) {
      const StepTime& s = stepTimes_[i];
      char line[112];
      snprintf(line, sizeof(line), "%u,\"%s\",%.1f,\"%s\"\n", s.step, s.name, s.ms / 1000.0f, s.result);
      text += line;
    }
    AsyncWebServerResponse* response = request->beginResponse(200, "text/csv", text);
    response->addHeader("Content-Disposition", fileName("steps"));
    request->send(response);
  }

 private:
  // run.csv / steps.csv for the whole route, run-group2.csv / steps-group2.csv for test group 2.
  String fileName(const char* kind) const {
    String name = kind;
    if (group_ > 0) name += "-group" + String(group_);
    return "attachment; filename=\"" + name + ".csv\"";
  }

  struct Cursor {
    int rows = 0, next = -1;   // -1 = the header line comes first
    char line[160];            // the header is the longest line, 137 characters
    size_t len = 0, sent = 0;
  };

  size_t fill(Cursor& c, uint8_t* out, size_t maxLen) {
    size_t used = 0;
    while (used < maxLen) {
      if (c.sent == c.len) {
        if (c.next >= c.rows) break;   // everything is sent
        c.len = c.next < 0 ? header(c.line, sizeof(c.line))
                           : format(rows_[c.next], c.line, sizeof(c.line));
        c.sent = 0;
        ++c.next;
      }
      const size_t n = min(maxLen - used, c.len - c.sent);
      memcpy(out + used, c.line + c.sent, n);
      used += n;
      c.sent += n;
    }
    return used;
  }

  static size_t header(char* line, size_t size) {
    const int n = snprintf(line, size,
        "time_s,step,blocked,heading_deg,target_deg,error_deg,left,right,"
        "distance_m,front_cm,left_cm,right_cm,pillars,walls,pillar_hits,wall_hits,pitch_deg,volts\n");
    return n > 0 ? min((size_t)n, size - 1) : 0;
  }

  static size_t format(const Row& r, char* line, size_t size) {
    float error = (r.target - r.heading) / 10.0f;   // + = must turn right
    while (error > 180.0f) error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    const int n = snprintf(line, size, "%.2f,%u,%u,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%u,%.1f,%.2f\n",
        r.ms / 1000.0f, r.step, r.blocked, r.heading / 10.0f, r.target / 10.0f, error,
        r.left / 100.0f, r.right / 100.0f, r.distanceCm / 100.0f,
        r.frontCm, r.leftCm, r.rightCm, r.pillars, r.walls, r.pillarHits, r.wallHits,
        r.pitch / 10.0f, r.volts / 100.0f);
    return n > 0 ? min((size_t)n, size - 1) : 0;
  }

  struct GpsCursor {
    int rows = 0, next = -1;
    char line[96];
    size_t len = 0, sent = 0;
  };

  size_t fillGps(GpsCursor& c, uint8_t* out, size_t maxLen) {
    size_t used = 0;
    while (used < maxLen) {
      if (c.sent == c.len) {
        if (c.next >= c.rows) break;
        c.len = c.next < 0 ? gpsHeader(c.line, sizeof(c.line))
                           : gpsFormat(gpsRows_[c.next], c.line, sizeof(c.line));
        c.sent = 0;
        ++c.next;
      }
      const size_t n = min(maxLen - used, c.len - c.sent);
      memcpy(out + used, c.line + c.sent, n);
      used += n;
      c.sent += n;
    }
    return used;
  }

  static size_t gpsHeader(char* line, size_t size) {
    const int n = snprintf(line, size, "time_s,lat,lon,satellites,fix_quality,hdop,altitude_m\n");
    return n > 0 ? min((size_t)n, size - 1) : 0;
  }

  static size_t gpsFormat(const GpsRow& r, char* line, size_t size) {
    const int n = snprintf(line, size, "%.2f,%.7f,%.7f,%u,%u,%.1f,%d\n",
        r.ms / 1000.0f, r.lat / 1e7, r.lon / 1e7, r.sats, r.quality, r.hdopX10 / 10.0f, r.altM);
    return n > 0 ? min((size_t)n, size - 1) : 0;
  }

  Row* rows_ = nullptr;
  volatile int count_ = 0;
  GpsRow* gpsRows_ = nullptr;
  volatile int gpsCount_ = 0;
  StepTime stepTimes_[kMaxSteps];
  volatile int steps_ = 0;
  int group_ = 0;
};

extern RunLog runLog;
