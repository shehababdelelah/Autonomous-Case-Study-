#include "LidarSensor.hpp"

#include <LittleFS.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char* LidarTestLogger::kFilePath = "/lidar_test.csv";
const char* LidarTestLogger::kImuFilePath = "/imu_euler_data.csv";
const char* LidarTestLogger::kLidarFilePath = "/lidar_data.csv";
static const char* kLegacyMovementFilePath = "/movement_data.csv";
static const char* kLegacyImuFilePath = "/imu_data.csv";

LidarScan::LidarScan() {
  for (int i = 0; i < 256; ++i) {
    uint8_t crc = i;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x4D : crc << 1;
    }
    _crcTab[i] = crc;
  }
}

void LidarScan::begin(HardwareSerial& port, const Config& cfg) {
  _cfg = cfg;
  _port = &port;
  _port->setRxBufferSize(4096);
  _port->begin(_cfg.baud, SERIAL_8N1, _cfg.rxPin, _cfg.txPin);
}

void LidarScan::update() {
  if (!_port) return;
  while (_port->available()) feed(_port->read());
}

void LidarScan::feed(uint8_t byte) {
  if (_idx == 0 && byte != 0x54) return;
  if (_idx == 1 && byte != 0x2C) {
    _idx = byte == 0x54 ? 1 : 0;
    return;
  }
  _buf[_idx++] = byte;
  if (_idx < sizeof(Frame)) return;
  _idx = 0;
  uint8_t crc = 0;
  for (size_t i = 0; i < sizeof(Frame) - 1; ++i) crc = _crcTab[crc ^ _buf[i]];
  if (crc != _buf[sizeof(Frame) - 1]) {
    ++_crcErrors;
    return;
  }
  portENTER_CRITICAL(&_packetMux);
  ++_packets;
  _lastPacketMs = millis();
  portEXIT_CRITICAL(&_packetMux);
  _handle(reinterpret_cast<const Frame*>(_buf));
}

uint32_t LidarScan::packetAgeMs() const {
  portENTER_CRITICAL(&_packetMux);
  const uint32_t packets = _packets;
  const uint32_t lastPacketMs = _lastPacketMs;
  portEXIT_CRITICAL(&_packetMux);
  return packets ? millis() - lastPacketMs : UINT32_MAX;
}

uint16_t LidarScan::distanceMm(int bin) const {
  const Bin& value = _bins[((bin % 360) + 360) % 360];
  const uint32_t now = millis();
  uint16_t samples[3];
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    if (value.mm[i] && now - value.ms[i] <= _cfg.expireMs) samples[count++] = value.mm[i];
  }
  if (count < 2) return 0;
  if (count == 2) return (samples[0] + samples[1]) / 2;
  if (samples[0] > samples[1]) {
    const uint16_t temp = samples[0];
    samples[0] = samples[1];
    samples[1] = temp;
  }
  if (samples[1] > samples[2]) samples[1] = samples[2];
  return samples[0] > samples[1] ? samples[0] : samples[1];
}

float LidarScan::crcErrorPct() const {
  const uint32_t total = _packets + _crcErrors;
  return total ? 100.0f * _crcErrors / total : 0;
}

void LidarScan::_handle(const Frame* frame) {
  float startAngle = frame->startAngle / 100.0f;
  float endAngle = frame->endAngle / 100.0f;
  if (endAngle < startAngle) endAngle += 360;
  const float step = (endAngle - startAngle) / 11;
  const uint32_t now = millis();
  for (int i = 0; i < 12; ++i) {
    const uint16_t distance = frame->p[i].dist;
    if (distance < _cfg.minMm || distance >= _cfg.maxMm) continue;
    const int bin = ((int)lroundf(startAngle + step * i) + _cfg.angleShiftDeg) % 360;
    Bin& value = _bins[bin];
    value.mm[value.head] = distance;
    value.ms[value.head] = now;
    value.head = (value.head + 1) % 3;
  }
}

void LidarSceneor::begin(int offsetDeg) {
  _offsetDeg = offsetDeg;
  LidarScan::Config config;
  _scan.begin(Serial1, config);
}

void LidarSceneor::update() {
  _scan.update();
  const uint32_t now = millis();
  if (now - _lastSnapshotMs < 250) return;
  _lastSnapshotMs = now;
  _snapshot();
}

void LidarSceneor::_snapshot() {
  const uint32_t now = millis();
  float packetsPerSecond = _packetsPerSecond;
  if (now - _lastPpsMs >= 1000) {
    const uint32_t packets = _scan.packets();
    packetsPerSecond = (packets - _lastPackets) * 1000.0f / (now - _lastPpsMs);
    _lastPackets = packets;
    _lastPpsMs = now;
  }
  portENTER_CRITICAL(&_statusMux);
  _packetsPerSecond = packetsPerSecond;
  _ok = packetsPerSecond > 1;
  _crcErrorPct = _scan.crcErrorPct();
  portEXIT_CRITICAL(&_statusMux);

  const int degreesPerPoint = 360 / kPointCount;
  uint16_t distances[kPointCount] = {};
  for (int bin = 0; bin < 360; ++bin) {
    const uint16_t mm = _scan.distanceMm(bin);
    if (!mm) continue;
    const int degrees = ((bin + _offsetDeg) % 360 + 360) % 360;
    const uint8_t index = degrees / degreesPerPoint;
    const uint16_t cm = mm / 10;
    if (!distances[index] || cm < distances[index]) distances[index] = cm;
  }
  portENTER_CRITICAL(&_distanceMux);
  memcpy(_distanceCm, distances, sizeof(_distanceCm));
  portEXIT_CRITICAL(&_distanceMux);
}

uint16_t LidarSceneor::medianSector(uint8_t centerIndex) const {
  uint16_t distances[kPointCount];
  copyDistances(distances, kPointCount);
  uint16_t values[5];
  uint8_t count = 0;
  for (int8_t offset = -2; offset <= 2; ++offset) {
    const uint8_t index = (centerIndex + kPointCount + offset) % kPointCount;
    const uint16_t value = distances[index];
    if (!value) continue;
    uint8_t insertAt = count;
    while (insertAt && values[insertAt - 1] > value) {
      values[insertAt] = values[insertAt - 1];
      --insertAt;
    }
    values[insertAt] = value;
    ++count;
  }
  return count >= 2 ? values[count / 2] : 0;
}

void LidarSceneor::copyDistances(uint16_t* distances, size_t count) const {
  const size_t copyCount = count < kPointCount ? count : kPointCount;
  portENTER_CRITICAL(&_distanceMux);
  memcpy(distances, _distanceCm, copyCount * sizeof(_distanceCm[0]));
  portEXIT_CRITICAL(&_distanceMux);
}

String LidarSceneor::toJson() const {
  uint16_t distances[kPointCount];
  copyDistances(distances, kPointCount);
  portENTER_CRITICAL(&_statusMux);
  const bool scanOk = _ok;
  const float packetsPerSecond = _packetsPerSecond;
  const float crcErrorPct = _crcErrorPct;
  portEXIT_CRITICAL(&_statusMux);
  String json;
  json.reserve(430);
  json = "{\"ok\":";
  json += scanOk && scanAgeMs() <= 500 ? "true" : "false";
  json += ",\"pps\":"; json += String(packetsPerSecond, 0);
  json += ",\"crc\":"; json += String(crcErrorPct, 1);
  json += ",\"d\":[";
  for (uint8_t i = 0; i < kPointCount; ++i) {
    if (i) json += ',';
    json += distances[i];
  }
  json += "]}";
  return json;
}

bool LidarSceneor::ok() const {
  portENTER_CRITICAL(&_statusMux);
  const bool statusOk = _ok;
  portEXIT_CRITICAL(&_statusMux);
  return statusOk && scanAgeMs() <= 500;
}

float LidarSceneor::packetsPerSecond() const {
  portENTER_CRITICAL(&_statusMux);
  const float rate = _packetsPerSecond;
  portEXIT_CRITICAL(&_statusMux);
  return rate;
}

bool LidarTestLogger::begin() {
  _fileMutex = xSemaphoreCreateMutex();
  if (!_fileMutex) {
    portENTER_CRITICAL(&_stateMux);
    _error = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }

  _queue = xQueueCreate(8, sizeof(LidarLogMessage));
  if (!_queue) {
    portENTER_CRITICAL(&_stateMux);
    _error = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }

  // The project does not use this partition yet; format only if LittleFS cannot mount it.
  if (!LittleFS.begin(true)) {
    portENTER_CRITICAL(&_stateMux);
    _error = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }
  if (!prepareFile()) {
    portENTER_CRITICAL(&_stateMux);
    _error = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }
  if (!prepareImuFile() || !prepareLidarFile()) {
    portENTER_CRITICAL(&_stateMux);
    _movementError = true;
    portEXIT_CRITICAL(&_stateMux);
  }
  loadRecentRecords();

  if (xTaskCreatePinnedToCore(taskEntry, "lidarLog", 4096, this, 1, nullptr, 1) != pdPASS) {
    portENTER_CRITICAL(&_stateMux);
    _error = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }

  portENTER_CRITICAL(&_stateMux);
  _ready = true;
  _error = false;
  portEXIT_CRITICAL(&_stateMux);
  return true;
}

bool LidarTestLogger::prepareFile() {
  if (LittleFS.exists(kFilePath)) {
    File existing = LittleFS.open(kFilePath, FILE_READ);
    if (!existing) return false;
    const bool empty = existing.size() == 0;
    existing.close();
    if (!empty) return true;
  }

  File file = LittleFS.open(kFilePath, FILE_WRITE);
  if (!file) return false;
  const bool wroteHeader =
      file.println("uptime_ms,direction,object_type,distance_cm") > 0;
  file.close();
  return wroteHeader;
}

bool LidarTestLogger::prepareImuFile() {
  if (LittleFS.exists(kImuFilePath)) {
    File existing = LittleFS.open(kImuFilePath, FILE_READ);
    if (!existing) return false;
    const bool empty = existing.size() == 0;
    existing.close();
    if (!empty) return true;
  }

  File file = LittleFS.open(kImuFilePath, FILE_WRITE);
  if (!file) return false;
  const bool wroteHeader =
      file.println("uptime_ms,run_start_ms,imu_ok,heading_yaw_deg,roll_deg,pitch_deg,"
                   "sys_cal,gyro_cal,accel_cal,mag_cal") > 0;
  file.close();
  return wroteHeader;
}

bool LidarTestLogger::prepareLidarFile() {
  if (LittleFS.exists(kLidarFilePath)) {
    File existing = LittleFS.open(kLidarFilePath, FILE_READ);
    if (!existing) return false;
    const bool empty = existing.size() == 0;
    existing.close();
    if (!empty) return true;
  }

  File file = LittleFS.open(kLidarFilePath, FILE_WRITE);
  if (!file) return false;
  char header[1600];
  size_t used = 0;
  int written = snprintf(header, sizeof(header),
                         "uptime_ms,run_start_ms,lidar_ok,lidar_packets_per_second,"
                         "lidar_front_cm,lidar_right_cm,lidar_left_cm");
  if (written < 0 || (size_t)written >= sizeof(header)) {
    file.close();
    return false;
  }
  used = (size_t)written;
  for (uint8_t i = 0; i < kSensorLogLidarPoints; ++i) {
    written = snprintf(header + used, sizeof(header) - used, ",range_%03u_cm", i * 4);
    if (written < 0 || (size_t)written >= sizeof(header) - used) {
      file.close();
      return false;
    }
    used += (size_t)written;
  }
  if (used + 1 >= sizeof(header)) {
    file.close();
    return false;
  }
  header[used++] = '\n';
  const bool wroteHeader = file.write((const uint8_t*)header, used) == used;
  file.close();
  return wroteHeader;
}

void LidarTestLogger::loadRecentRecords() {
  File file = LittleFS.open(kFilePath, FILE_READ);
  if (!file) return;

  char line[96];
  while (file.available()) {
    const size_t length = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';
    if (!length || line[0] < '0' || line[0] > '9') continue;

    char* save = nullptr;
    char* timestamp = strtok_r(line, ",", &save);
    char* direction = strtok_r(nullptr, ",", &save);
    char* objectType = strtok_r(nullptr, ",", &save);
    char* distance = strtok_r(nullptr, ",", &save);
    if (!timestamp || !direction || !objectType || !distance) continue;

    LidarTestRecord record;
    record.timestampMs = strtoul(timestamp, nullptr, 10);
    record.distanceCm = (uint16_t)strtoul(distance, nullptr, 10);
    strlcpy(record.direction, direction, sizeof(record.direction));
    strlcpy(record.objectType, objectType, sizeof(record.objectType));
    rememberRecord(record);
  }
  file.close();
}

bool LidarTestLogger::enqueue(const char* direction, const char* objectType,
                              uint16_t distanceCm) {
  if (!ready() || !direction || !objectType || !distanceCm) return false;

  LidarLogMessage message;
  message.kind = LidarLogMessage::LabeledReading;
  message.labeled.timestampMs = millis();
  message.labeled.distanceCm = distanceCm;
  strlcpy(message.labeled.direction, direction, sizeof(message.labeled.direction));
  strlcpy(message.labeled.objectType, objectType, sizeof(message.labeled.objectType));
  return xQueueSend(_queue, &message, 0) == pdPASS;
}

bool LidarTestLogger::startMovementRecording() {
  portENTER_CRITICAL(&_stateMux);
  const bool canStart = _ready && !_movementError && !_movementFull && !_movementClearing;
  if (canStart && !_movementRecording) {
    _movementRunStartMs = millis();
    _movementRecording = true;
  }
  portEXIT_CRITICAL(&_stateMux);
  return canStart;
}

bool LidarTestLogger::startMovementDrive(float speed) {
  if (!isfinite(speed) || speed < 0.1f || speed > 0.5f) return false;
  portENTER_CRITICAL(&_movementDriveMux);
  if (_movementDriveActive || _movementDriveStarting) {
    portEXIT_CRITICAL(&_movementDriveMux);
    return false;
  }
  _movementDriveStarting = true;
  portEXIT_CRITICAL(&_movementDriveMux);

  if (!startMovementRecording()) {
    portENTER_CRITICAL(&_movementDriveMux);
    _movementDriveStarting = false;
    portEXIT_CRITICAL(&_movementDriveMux);
    return false;
  }
  portENTER_CRITICAL(&_movementDriveMux);
  if (!_movementDriveStarting) {
    portEXIT_CRITICAL(&_movementDriveMux);
    stopMovementRecording();
    return false;
  }
  _movementDriveStarting = false;
  _movementDriveSpeed = speed;
  _movementDriveStartedMs = millis();
  _movementDriveActive = true;
  portEXIT_CRITICAL(&_movementDriveMux);
  return true;
}

bool LidarTestLogger::clearMovementFiles() {
  if (!_fileMutex || xSemaphoreTake(_fileMutex, 0) != pdTRUE) return false;

  portENTER_CRITICAL(&_stateMux);
  const bool canClear = !_movementRecording && !_movementWriting &&
                        !_movementOutstanding && !_movementClearing;
  if (canClear) _movementClearing = true;
  portEXIT_CRITICAL(&_stateMux);
  if (!canClear) {
    xSemaphoreGive(_fileMutex);
    return false;
  }

  bool driveActive;
  float driveSpeed;
  uint32_t remainingMs;
  getMovementDriveStatus(driveActive, driveSpeed, remainingMs);
  if (driveActive) {
    portENTER_CRITICAL(&_stateMux);
    _movementClearing = false;
    portEXIT_CRITICAL(&_stateMux);
    xSemaphoreGive(_fileMutex);
    return false;
  }

  bool removed = true;
  const char* paths[] = {
      kImuFilePath, kLidarFilePath, kLegacyMovementFilePath, kLegacyImuFilePath};
  for (const char* path : paths) {
    if (LittleFS.exists(path) && !LittleFS.remove(path)) removed = false;
  }
  const bool prepared = removed && prepareImuFile() && prepareLidarFile();

  portENTER_CRITICAL(&_stateMux);
  _movementClearing = false;
  if (prepared) {
    _movementError = false;
    _movementFull = false;
    _movementRunStartMs = 0;
  } else {
    _movementError = true;
  }
  portEXIT_CRITICAL(&_stateMux);
  xSemaphoreGive(_fileMutex);
  return prepared;
}

void LidarTestLogger::stopMovementRecording() {
  portENTER_CRITICAL(&_stateMux);
  _movementRecording = false;
  portEXIT_CRITICAL(&_stateMux);
}

void LidarTestLogger::stopMovementDrive() {
  portENTER_CRITICAL(&_movementDriveMux);
  _movementDriveStarting = false;
  _movementDriveActive = false;
  portEXIT_CRITICAL(&_movementDriveMux);
  stopMovementRecording();
}

bool LidarTestLogger::getMovementDrive(float& speed) {
  bool active = false;
  bool expired = false;
  portENTER_CRITICAL(&_movementDriveMux);
  if (_movementDriveActive) {
    if (millis() - _movementDriveStartedMs >= kMovementDriveDurationMs) {
      _movementDriveActive = false;
      expired = true;
    } else {
      speed = _movementDriveSpeed;
      active = true;
    }
  }
  portEXIT_CRITICAL(&_movementDriveMux);
  if (expired) stopMovementRecording();
  if (active) {
    portENTER_CRITICAL(&_stateMux);
    const bool recording = _movementRecording && !_movementError && !_movementFull;
    portEXIT_CRITICAL(&_stateMux);
    if (!recording) {
      portENTER_CRITICAL(&_movementDriveMux);
      _movementDriveActive = false;
      portEXIT_CRITICAL(&_movementDriveMux);
      active = false;
    }
  }
  return active;
}

void LidarTestLogger::getMovementDriveStatus(bool& active, float& speed,
                                            uint32_t& remainingMs) const {
  portENTER_CRITICAL(&_movementDriveMux);
  active = _movementDriveActive;
  speed = _movementDriveSpeed;
  const uint32_t elapsedMs = millis() - _movementDriveStartedMs;
  remainingMs = active && elapsedMs < kMovementDriveDurationMs
      ? kMovementDriveDurationMs - elapsedMs : 0;
  portEXIT_CRITICAL(&_movementDriveMux);
}

bool LidarTestLogger::enqueueMovementSnapshot(const SensorLogSnapshot& snapshot) {
  LidarLogMessage message;
  portENTER_CRITICAL(&_stateMux);
  if (!_movementRecording || !_ready || _movementError || _movementFull) {
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }
  message.kind = LidarLogMessage::MovementSnapshot;
  message.runStartMs = _movementRunStartMs;
  ++_movementOutstanding;
  portEXIT_CRITICAL(&_stateMux);
  message.snapshot = snapshot;

  if (xQueueSend(_queue, &message, 0) == pdPASS) return true;
  portENTER_CRITICAL(&_stateMux);
  --_movementOutstanding;
  _movementRecording = false;
  _movementError = true;
  portEXIT_CRITICAL(&_stateMux);
  return false;
}

MovementLogStatus LidarTestLogger::movementStatus() const {
  MovementLogStatus status;
  portENTER_CRITICAL(&_stateMux);
  status.ready = _ready && !_movementError;
  status.recording = _movementRecording;
  status.error = _movementError;
  status.full = _movementFull;
  status.writing = _movementWriting;
  status.queued = _movementOutstanding;
  portEXIT_CRITICAL(&_stateMux);
  return status;
}

size_t LidarTestLogger::copyRecent(LidarTestRecord* records, size_t capacity) const {
  if (!records || !capacity) return 0;
  portENTER_CRITICAL(&_stateMux);
  const size_t count = _recentCount < capacity ? _recentCount : capacity;
  const size_t start =
      (_recentNext + kRecentRecordCount - _recentCount) % kRecentRecordCount;
  for (size_t i = 0; i < count; ++i) {
    records[i] = _recent[(start + i) % kRecentRecordCount];
  }
  portEXIT_CRITICAL(&_stateMux);
  return count;
}

bool LidarTestLogger::ready() const {
  portENTER_CRITICAL(&_stateMux);
  const bool value = _ready;
  portEXIT_CRITICAL(&_stateMux);
  return value;
}

bool LidarTestLogger::hasError() const {
  portENTER_CRITICAL(&_stateMux);
  const bool value = _error;
  portEXIT_CRITICAL(&_stateMux);
  return value;
}

bool LidarTestLogger::isFull() const {
  portENTER_CRITICAL(&_stateMux);
  const bool value = _full;
  portEXIT_CRITICAL(&_stateMux);
  return value;
}

void LidarTestLogger::taskEntry(void* context) {
  static_cast<LidarTestLogger*>(context)->task();
}

void LidarTestLogger::task() {
  LidarLogMessage message;
  for (;;) {
    if (xQueueReceive(_queue, &message, portMAX_DELAY) != pdPASS) continue;
    if (message.kind == LidarLogMessage::LabeledReading) {
      xSemaphoreTake(_fileMutex, portMAX_DELAY);
      const bool appended = appendRecord(message.labeled);
      xSemaphoreGive(_fileMutex);
      if (!appended) {
        portENTER_CRITICAL(&_stateMux);
        _error = true;
        portEXIT_CRITICAL(&_stateMux);
        continue;
      }
      rememberRecord(message.labeled);
      continue;
    }

    portENTER_CRITICAL(&_stateMux);
    _movementWriting = true;
    portEXIT_CRITICAL(&_stateMux);
    xSemaphoreTake(_fileMutex, portMAX_DELAY);
    const bool imuAppended = appendImuRecord(message);
    const bool lidarAppended = appendLidarRecord(message);
    xSemaphoreGive(_fileMutex);
    portENTER_CRITICAL(&_stateMux);
    _movementWriting = false;
    if (_movementOutstanding) --_movementOutstanding;
    if (!imuAppended || !lidarAppended) {
      _movementRecording = false;
      if (!_movementFull) _movementError = true;
    }
    portEXIT_CRITICAL(&_stateMux);
  }
}

bool LidarTestLogger::appendRecord(const LidarTestRecord& record) {
  File file = LittleFS.open(kFilePath, FILE_APPEND);
  if (!file) return false;

  char row[80];
  const int length = snprintf(row, sizeof(row), "%lu,%s,%s,%u\n",
                              (unsigned long)record.timestampMs, record.direction,
                              record.objectType, record.distanceCm);
  if (length <= 0 || (size_t)length >= sizeof(row) ||
      file.size() + (size_t)length > kMaxFileBytes) {
    file.close();
    portENTER_CRITICAL(&_stateMux);
    _full = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }

  const bool wroteRow = file.write((const uint8_t*)row, (size_t)length) == (size_t)length;
  file.flush();
  file.close();
  return wroteRow;
}

bool LidarTestLogger::appendImuRecord(const LidarLogMessage& message) {
  File file = LittleFS.open(kImuFilePath, FILE_APPEND);
  if (!file) return false;

  const SensorLogSnapshot& snapshot = message.snapshot;
  char row[160];
  const int written = snprintf(row, sizeof(row),
                               "%lu,%lu,%u,%.1f,%.1f,%.1f,%u,%u,%u,%u\n",
                               (unsigned long)snapshot.timestampMs,
                               (unsigned long)message.runStartMs,
                               snapshot.imuOk ? 1 : 0, snapshot.headingDeg,
                               snapshot.rollDeg, snapshot.pitchDeg,
                               snapshot.calibration[0], snapshot.calibration[1],
                               snapshot.calibration[2], snapshot.calibration[3]);
  if (written <= 0 || (size_t)written >= sizeof(row)) {
    file.close();
    return false;
  }
  if (file.size() + (size_t)written > kMaxImuFileBytes) {
    file.close();
    portENTER_CRITICAL(&_stateMux);
    _movementFull = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }
  const bool wroteRow = file.write((const uint8_t*)row, (size_t)written) == (size_t)written;
  file.flush();
  file.close();
  return wroteRow;
}

bool LidarTestLogger::appendLidarRecord(const LidarLogMessage& message) {
  File file = LittleFS.open(kLidarFilePath, FILE_APPEND);
  if (!file) return false;

  char row[600];
  const SensorLogSnapshot& snapshot = message.snapshot;
  int written = snprintf(row, sizeof(row), "%lu,%lu,%u,%.0f,%u,%u,%u",
                         (unsigned long)snapshot.timestampMs,
                         (unsigned long)message.runStartMs,
                         snapshot.lidarOk ? 1 : 0, snapshot.lidarPacketsPerSecond,
                         snapshot.lidarFrontCm, snapshot.lidarRightCm,
                         snapshot.lidarLeftCm);
  if (written < 0 || (size_t)written >= sizeof(row)) {
    file.close();
    return false;
  }
  size_t used = (size_t)written;
  for (uint8_t i = 0; i < kSensorLogLidarPoints; ++i) {
    written = snprintf(row + used, sizeof(row) - used, ",%u", snapshot.lidarCm[i]);
    if (written < 0 || (size_t)written >= sizeof(row) - used) {
      file.close();
      return false;
    }
    used += (size_t)written;
  }
  if (used + 1 >= sizeof(row)) {
    file.close();
    return false;
  }
  row[used++] = '\n';
  if (file.size() + used > kMaxLidarFileBytes) {
    file.close();
    portENTER_CRITICAL(&_stateMux);
    _movementFull = true;
    portEXIT_CRITICAL(&_stateMux);
    return false;
  }
  const bool wroteRow = file.write((const uint8_t*)row, used) == used;
  file.flush();
  file.close();
  return wroteRow;
}

void LidarTestLogger::rememberRecord(const LidarTestRecord& record) {
  portENTER_CRITICAL(&_stateMux);
  _recent[_recentNext] = record;
  _recentNext = (_recentNext + 1) % kRecentRecordCount;
  if (_recentCount < kRecentRecordCount) ++_recentCount;
  portEXIT_CRITICAL(&_stateMux);
}
