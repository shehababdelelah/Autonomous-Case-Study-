#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <memory>
#include <new>

// Records one row every 100 ms of a run, in RAM (no flash writes while driving).
// Download it at http://192.168.18.1/log.csv after the run.
// Start clears it, and a restart of the box loses it.
class RunLog {
 public:
  struct Row {
    uint32_t ms;                        // time since Start
    uint8_t step, blocked;
    int8_t left, right;                 // wheel command x 100
    uint16_t heading, target;           // degrees x 10
    uint16_t distanceCm, frontCm, leftCm, rightCm;
    uint8_t pillars, walls;
  };
  static constexpr int kMaxRows = 3000;   // 5 minutes at 10 rows per second

  // About 72 KB. If memory is short, the log stays off and the rover still works.
  void begin() { rows_ = new (std::nothrow) Row[kMaxRows]; }
  void clear() { count_ = 0; }
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
    response->addHeader("Content-Disposition", "attachment; filename=\"run.csv\"");
    request->send(response);
  }

 private:
  struct Cursor {
    int rows = 0, next = -1;   // -1 = the header line comes first
    char line[128];
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
        "distance_m,front_cm,left_cm,right_cm,pillars,walls\n");
    return n > 0 ? min((size_t)n, size - 1) : 0;
  }

  static size_t format(const Row& r, char* line, size_t size) {
    float error = (r.target - r.heading) / 10.0f;   // + = must turn right
    while (error > 180.0f) error -= 360.0f;
    while (error <= -180.0f) error += 360.0f;
    const int n = snprintf(line, size, "%.2f,%u,%u,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u\n",
        r.ms / 1000.0f, r.step, r.blocked, r.heading / 10.0f, r.target / 10.0f, error,
        r.left / 100.0f, r.right / 100.0f, r.distanceCm / 100.0f,
        r.frontCm, r.leftCm, r.rightCm, r.pillars, r.walls);
    return n > 0 ? min((size_t)n, size - 1) : 0;
  }

  Row* rows_ = nullptr;
  volatile int count_ = 0;
};

extern RunLog runLog;
