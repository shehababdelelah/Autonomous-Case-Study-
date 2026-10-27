#pragma once
// Sensors - all box sensors behind one object.
//
// Load budget: the ultrasonic sensors run in their own task on core 0 (UsManager); LiDAR, GPS and
// IMU are read in update() on core 1. The web page never touches a sensor: it gets a compact
// snapshot (toJson) that is rebuilt 4 times per second; the LiDAR is reduced to 90 points of 4 deg.
// Missing sensors never block: the IMU is started only once it answers on I2C (retry every 5 s).
#include <Arduino.h>
#include "UsManager.h"
#include "LidarScan.h"
#include "GpsManager.h"
#include "ImuManager.h"

class Sensors {
public:
  static const uint8_t kLidarPoints = 90;  // LiDAR points of 4 deg, index 0 = front, clockwise
  static const int kUsMaxCm = 300;         // ultrasonic: "nothing in range" reads as this value
  uint16_t lidarCm(uint8_t i) const { return i < kLidarPoints ? _lidarCm[i] : 0; }

  // ---- full resolution LiDAR, for navigation geometry ----
  // lidarCm() above is downsampled to 4 deg for the web page. Navigation needs the real map:
  // 360 bins of 1 deg in millimetres, median filtered, 0 = no reliable return (see LidarScan.h).
  // Bin 270 is straight ahead and bins increase clockwise, which is what _snapshotLidar()'s
  // (bin + 90) % 360 shift produces; lidarBin() converts an angle to the matching bin.
  static const int kLidarFrontBin = 270;
  static const uint16_t kLidarMinMm = 200;    // LidarScan discards anything closer: hard floor
  static const uint16_t kLidarMaxMm = 12000;

  /**
   * @brief Distance of one 1-degree LiDAR bin.
   * @param bin Bin index; wrapped into 0..359, so negative and over-360 values are fine.
   * @return Distance in mm, or 0 when that direction has no reliable return.
   */
  uint16_t lidarBinMm(int bin) const { return _lidar.distanceMm(bin); }

  /**
   * @brief Bin index for an angle measured clockwise from straight ahead.
   * @param degClockwise 0 = ahead, +90 = right, -90 (or 270) = left, 180 = behind.
   * @return Bin index in 0..359 for lidarBinMm().
   */
  static int lidarBin(int degClockwise) {
    return ((kLidarFrontBin + degClockwise) % 360 + 360) % 360;
  }

  bool lidarOk() const { return _lidarOk; }  // frames are arriving
  bool   imuOk()    const { return _imuReady; }
  float  yawDeg()   const { return _yaw; }
  float  pitchDeg() const { return _pitch; }
  bool   gpsFix()   const { return _gpsFix; }
  double lat()      const { return _lat; }
  double lon()      const { return _lon; }
  // Starts I2C and all sensors. lidarOffsetDeg rotates the LiDAR view so that point 0 is the box
  // front (90 for the box as mounted).
  void begin(int lidarOffsetDeg);
  void update();               // call every loop pass (core 1)
  String toJson() const;       // snapshot for the web page (~650 bytes)

  // Ultrasonic distance in cm, updated 4 times per second (kUsMaxCm = nothing in range).
  // i: 0 front, 1 left, 2 right, 3 back, 4 front angled
  int usCm(uint8_t i) const { return i < 5 ? _usCm[i] : kUsMaxCm; }

private:
  void _snapshotLidar();
  void _snapshotGps();
  void _startImuIfPresent();

  UsManager _us;
  LidarScan _lidar;
  GpsManager _gps;
  ImuManager _imu;

  int _lidarOffsetDeg = 0;
  uint32_t _lastSnapMs = 0, _lastImuTryMs = 0, _lastPpsMs = 0, _lastPackets = 0;

  // snapshot values (loop() only)
  int _usCm[5] = {kUsMaxCm, kUsMaxCm, kUsMaxCm, kUsMaxCm, kUsMaxCm};
  bool _lidarOk = false;
  float _lidarPps = 0, _lidarCrcPct = 0;
  uint16_t _lidarCm[kLidarPoints] = {0};  // closest reliable return per 4 deg in cm, 0 = none
  bool _imuReady = false;
  float _yaw = 0, _pitch = 0, _roll = 0;
  uint8_t _cal[4] = {0, 0, 0, 0};
  bool _gpsFix = false;
  int _sats = 0;
  float _hdop = 0;
  double _lat = 0, _lon = 0;
  uint32_t _gpsChars = 0;
};

bool i2cPresent(uint8_t addr);  // true if a device acknowledges on the I2C bus
