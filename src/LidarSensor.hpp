#pragma once

#include <Arduino.h>

static const uint8_t kSensorLogLidarPoints = 90;

// LidarScan - LD20 LiDAR reader with a stable distance map, one value per degree.
//
// Why a filter: the raw LD20 points are steady (2-60 mm), but about a third of all points are
// "no return" (distance 0). An older reader shrank the stored value on every such point, so the
// view jumped by up to 1 m although nothing moved. Here a "no return" point changes nothing;
// instead:
//   - a bin keeps its last 3 valid hits; its value is their median (single outliers drop out)
//   - a bin counts only with >= 2 hits younger than expireMs (removes ghost points in open
//     directions), otherwise it reads 0 = no return
//   - hits closer than minMm (the box itself) are ignored
// Angle convention: raw angle + 90, rounded to 1 degree -> front = bin 270, clockwise.
// LD20 protocol: 47-byte frames, header 0x54 0x2C, 12 points, CRC8 at the end; ~340 frames/s.
class LidarScan {
public:
  struct Config {
    int rxPin = 27, txPin = 33;   // ESP32 RX <- LD20 TX, ESP32 TX -> LD20 RX (unused by the LD20)
    uint32_t baud = 230400;       // LD20 default
    int angleShiftDeg = 90;   // raw angle + 90 -> front = bin 270
    uint16_t minMm = 200;     // ignore the box itself
    uint16_t maxMm = 12000;   // LD20 range
    uint16_t expireMs = 500;  // LD20 turns at ~10 Hz -> ~5 revolutions
  };

  LidarScan();
  void begin(HardwareSerial& port, const Config& cfg);
  void update();
  void feed(uint8_t c);
  uint16_t distanceMm(int bin) const;
  uint32_t packets() const { return _packets; }
  uint32_t packetAgeMs() const;
  float crcErrorPct() const;

private:
  struct __attribute__((packed)) Pt { uint16_t dist; uint8_t inten; };
  struct __attribute__((packed)) Frame {
    uint8_t header, verLen; uint16_t speed, startAngle; Pt p[12]; uint16_t endAngle, ts; uint8_t crc;
  };
  struct Bin { uint16_t mm[3] = {0, 0, 0}; uint32_t ms[3] = {0, 0, 0}; uint8_t head = 0; };

  void _handle(const Frame* f);

  Config _cfg;
  HardwareSerial* _port = nullptr;
  uint8_t _crcTab[256];
  uint8_t _buf[sizeof(Frame)];
  uint8_t _idx = 0;
  mutable portMUX_TYPE _packetMux = portMUX_INITIALIZER_UNLOCKED;
  uint32_t _packets = 0, _crcErrors = 0;
  uint32_t _lastPacketMs = 0;
  Bin _bins[360];
};


struct SensorLogSnapshot {
  uint32_t timestampMs = 0;
  bool imuOk = false;
  float headingDeg = 0;
  float rollDeg = 0;
  float pitchDeg = 0;
  uint8_t calibration[4] = {0, 0, 0, 0};
  bool lidarOk = false;
  float lidarPacketsPerSecond = 0;
  uint16_t lidarFrontCm = 0;
  uint16_t lidarRightCm = 0;
  uint16_t lidarLeftCm = 0;
  uint16_t lidarCm[kSensorLogLidarPoints] = {0};
};

class LidarSceneor {
public:
  static const uint8_t kPointCount = 90;

  void begin(int offsetDeg);
  void update();
  String toJson() const;
  uint16_t medianSector(uint8_t centerIndex) const;
  void copyDistances(uint16_t* distances, size_t count) const;
  bool ok() const;
  uint32_t scanAgeMs() const { return _scan.packetAgeMs(); }
  float packetsPerSecond() const;

private:
  void _snapshot();

  LidarScan _scan;
  int _offsetDeg = 0;
  uint32_t _lastSnapshotMs = 0;
  uint32_t _lastPpsMs = 0;
  uint32_t _lastPackets = 0;
  bool _ok = false;
  mutable portMUX_TYPE _statusMux = portMUX_INITIALIZER_UNLOCKED;
  mutable portMUX_TYPE _distanceMux = portMUX_INITIALIZER_UNLOCKED;
  float _packetsPerSecond = 0;
  float _crcErrorPct = 0;
  uint16_t _distanceCm[kPointCount] = {0};
};

struct LidarTestRecord {
  uint32_t timestampMs = 0;
  uint16_t distanceCm = 0;
  char direction[8] = {};
  char objectType[8] = {};
};

struct LidarLogMessage {
  enum Kind : uint8_t { LabeledReading, MovementSnapshot };
  Kind kind = LabeledReading;
  LidarTestRecord labeled;
  uint32_t runStartMs = 0;
  SensorLogSnapshot snapshot;
};

struct MovementLogStatus {
  bool ready = false;
  bool recording = false;
  bool error = false;
  bool full = false;
  bool writing = false;
  UBaseType_t queued = 0;
};

class LidarTestLogger {
public:
  static const size_t kRecentRecordCount = 32;

  bool begin();
  bool enqueue(const char* direction, const char* objectType, uint16_t distanceCm);
  bool startMovementRecording();
  bool startMovementDrive(float speed);
  bool clearMovementFiles();
  void stopMovementRecording();
  void stopMovementDrive();
  bool getMovementDrive(float& speed);
  void getMovementDriveStatus(bool& active, float& speed, uint32_t& remainingMs) const;
  bool enqueueMovementSnapshot(const SensorLogSnapshot& snapshot);
  MovementLogStatus movementStatus() const;
  size_t copyRecent(LidarTestRecord* records, size_t capacity) const;
  bool ready() const;
  bool hasError() const;
  bool isFull() const;
  static void taskEntry(void* context);

private:
  void task();
  bool prepareFile();
  bool prepareImuFile();
  bool prepareLidarFile();
  void loadRecentRecords();
  bool appendRecord(const LidarTestRecord& record);
  bool appendImuRecord(const LidarLogMessage& message);
  bool appendLidarRecord(const LidarLogMessage& message);
  void rememberRecord(const LidarTestRecord& record);

  static const char* kFilePath;
  static const char* kImuFilePath;
  static const char* kLidarFilePath;
  static const size_t kMaxFileBytes = 16384;
  static const size_t kMaxImuFileBytes = 128 * 1024;
  static const size_t kMaxLidarFileBytes = 700 * 1024;
  static const uint32_t kMovementDriveDurationMs = 60000;

  QueueHandle_t _queue = nullptr;
  mutable portMUX_TYPE _stateMux = portMUX_INITIALIZER_UNLOCKED;
  LidarTestRecord _recent[kRecentRecordCount] = {};
  size_t _recentCount = 0;
  size_t _recentNext = 0;
  bool _ready = false;
  bool _error = false;
  bool _full = false;
  bool _movementRecording = false;
  bool _movementError = false;
  bool _movementFull = false;
  bool _movementWriting = false;
  bool _movementClearing = false;
  uint32_t _movementRunStartMs = 0;
  uint16_t _movementOutstanding = 0;
  SemaphoreHandle_t _fileMutex = nullptr;
  mutable portMUX_TYPE _movementDriveMux = portMUX_INITIALIZER_UNLOCKED;
  bool _movementDriveStarting = false;
  bool _movementDriveActive = false;
  float _movementDriveSpeed = 0;
  uint32_t _movementDriveStartedMs = 0;
};
