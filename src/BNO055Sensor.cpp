#include "BNO055Sensor.hpp"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <Wire.h>

static constexpr uint8_t kBnoAddress = 0x28;
static constexpr int32_t kBnoIdAddress = 0;
static constexpr size_t kEepromSize =
sizeof(int32_t) + sizeof(adafruit_bno055_offsets_t);
static constexpr uint32_t kCalibrationPollMs = 100;
static constexpr uint32_t kPrintIntervalMs = 500;

// Gyro Z is positive counter-clockwise, heading is positive clockwise.
// Check this on your rover with the hand test.
static constexpr float kYawRateSign = -1.0f;

static uint32_t lastPrintMs = 0;

// Protects the newest reading. Same idea as _mux in RoverLink.
static portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;
static ImuData latestImu = {};
static bool hasImuData = false;

static constexpr bool kClearSavedCalibration = false;
static Adafruit_BNO055 bno(55, kBnoAddress, &Wire);
static bool bnoReady = false;


static void displaySensorDetails() {
  sensor_t sensor;
  bno.getSensor(&sensor);

  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
}

static void displaySensorStatus() {
  uint8_t systemStatus = 0;
  uint8_t selfTestResults = 0;
  uint8_t systemError = 0;
  bno.getSystemStatus(&systemStatus, &selfTestResults, &systemError);

  Serial.println();
  Serial.print("System Status: 0x"); Serial.println(systemStatus, HEX);
  Serial.print("Self Test:     0x"); Serial.println(selfTestResults, HEX);
  Serial.print("System Error:  0x"); Serial.println(systemError, HEX);
}

static void displayCalStatus() {
  uint8_t system = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print('\t');
  if (system == 0) Serial.print("! ");
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

static void displaySensorOffsets(const adafruit_bno055_offsets_t& offsets) {
  Serial.print("Accelerometer: ");
  Serial.print(offsets.accel_offset_x); Serial.print(' ');
  Serial.print(offsets.accel_offset_y); Serial.print(' ');
  Serial.println(offsets.accel_offset_z);

  Serial.print("Gyro: ");
  Serial.print(offsets.gyro_offset_x); Serial.print(' ');
  Serial.print(offsets.gyro_offset_y); Serial.print(' ');
  Serial.println(offsets.gyro_offset_z);

  Serial.print("Mag: ");
  Serial.print(offsets.mag_offset_x); Serial.print(' ');
  Serial.print(offsets.mag_offset_y); Serial.print(' ');
  Serial.println(offsets.mag_offset_z);

  Serial.print("Accel Radius: ");
  Serial.println(offsets.accel_radius);
  Serial.print("Mag Radius: ");
  Serial.println(offsets.mag_radius);
}

static bool waitForBnoCalibration(uint32_t timeoutMs) {
  const uint32_t startedMs = millis();

  while (!bno.isFullyCalibrated()) {
    if (millis() - startedMs >= timeoutMs) {
      Serial.println("BNO055 calibration timed out");
      return false;
    }

    displayCalStatus();
    Serial.println();
    delay(kCalibrationPollMs);
  }

  return true;
}
bool initBNO055(int sdaPin, int sclPin) {
  Wire.begin(sdaPin, sclPin);

  if (!EEPROM.begin(kEepromSize)) {
    Serial.println("Could not initialize EEPROM for BNO055 calibration.");
    return false;
  }
//   To recalibrate manually:
// 1. Set the flag to true.
// 2. Upload and let calibration complete.
// 3. Set it back to false before the next upload.
// Leaving it true forces calibration on every boot.
  if (kClearSavedCalibration) {
  const int32_t invalidId = -1;
  EEPROM.put(kBnoIdAddress, invalidId);

  if (!EEPROM.commit()) {
    Serial.println("Failed to clear saved calibration");
    return false;
  }
}
  // if i need north i test NDOF agian
  Serial.println("Initializing BNO055...");
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("No BNO055 detected. Check wiring and I2C address.");
    return false;
  }

  int32_t savedBnoId = -1;
  EEPROM.get(kBnoIdAddress, savedBnoId);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  bno.getSensor(&sensor);
  bool foundCalibration = false;

  if (savedBnoId == sensor.sensor_id) {
    EEPROM.get(kBnoIdAddress + sizeof(savedBnoId), calibrationData);
    bno.setSensorOffsets(calibrationData);
    foundCalibration = true;
    Serial.println("Restored BNO055 calibration from EEPROM.");
    displaySensorOffsets(calibrationData);
  } else {
    Serial.println("No saved calibration found for this sensor.");
  }

  // Match the Adafruit example: configure the external crystal after loading offsets.
  bno.setExtCrystalUse(true);

  displaySensorDetails();
  displaySensorStatus();

if (foundCalibration) {
  Serial.println("Using saved BNO055 calibration.");
} else {
  Serial.println(
      "Keep still, then hold the sensor in six positions.");

  if (!waitForBnoCalibration(120000)) {
    return false;
  }

  Serial.println("Fully calibrated!");

  adafruit_bno055_offsets_t newCalibration;

  if (!bno.getSensorOffsets(newCalibration)) {
    Serial.println("Could not read calibration offsets");
    return false;
  }

  displaySensorOffsets(newCalibration);

  bno.getSensor(&sensor);
  const int32_t currentBnoId = sensor.sensor_id;

  EEPROM.put(kBnoIdAddress, currentBnoId);
  EEPROM.put(
      kBnoIdAddress + sizeof(currentBnoId),
      newCalibration);

  if (!EEPROM.commit()) {
    Serial.println("Could not save calibration");
    return false;
  }

  Serial.println("Calibration saved");
}
    
  bnoReady = true;
  return true;
}

// For Euler and gyro registers, whose values use a scale of 16.
static bool readBnoVector16(uint8_t reg, float values[3]) {
  uint8_t bytes[6];

  Wire.beginTransmission(kBnoAddress);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received =
      Wire.requestFrom(kBnoAddress, static_cast<uint8_t>(6));

  if (received != sizeof(bytes)) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    bytes[i] = Wire.read();
  }

  for (int i = 0; i < 3; ++i) {
    const uint16_t word =
        static_cast<uint16_t>(bytes[2 * i]) |
        (static_cast<uint16_t>(bytes[2 * i + 1]) << 8);

    const int16_t raw = static_cast<int16_t>(word);
    values[i] = raw / 16.0f;
  }

  return true;
}

void updateBNO055() {
if (!bnoReady) return;

  float euler[3];
  float gyro[3];

  const bool readOk =
      readBnoVector16(
          Adafruit_BNO055::BNO055_EULER_H_LSB_ADDR, euler) &&
      readBnoVector16(
          Adafruit_BNO055::BNO055_GYRO_DATA_X_LSB_ADDR, gyro);

  if (!readOk) return;   // keep the last sample; main.cpp checks its age

  ImuData sample{};
  sample.headingDeg = euler[0];
  sample.rollDeg = euler[1];
  sample.pitchDeg = euler[2];
  sample.yawRateDps = kYawRateSign * gyro[2];
  sample.sampleMs = millis();

  portENTER_CRITICAL(&imuMux);
  latestImu = sample;
  hasImuData = true;
  portEXIT_CRITICAL(&imuMux);

  if (sample.sampleMs - lastPrintMs < kPrintIntervalMs) return;
  lastPrintMs = sample.sampleMs;

  Serial.print("Heading: ");
  Serial.print(sample.headingDeg, 1);
  Serial.print("\tRoll: ");
  Serial.print(sample.rollDeg, 1);
  Serial.print("\tPitch: ");
  Serial.print(sample.pitchDeg, 1);
  Serial.print("\tYaw rate: ");
  Serial.print(sample.yawRateDps, 1);
  displayCalStatus();
  Serial.println();
}

bool getBNO055Data(ImuData& data) {
  portENTER_CRITICAL(&imuMux);
  data = latestImu;
  const bool ready = hasImuData;
  portEXIT_CRITICAL(&imuMux);
  return ready;
}
