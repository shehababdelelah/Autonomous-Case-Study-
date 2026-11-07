#include "Ultrasonics.hpp"
#include "BNO055Sensor.hpp"
#include "RoverLink.hpp"
#include "LidarSensor.hpp"
#include "LCDDisplay.hpp"
#include "WEBpage.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <WiFi.h>

//******************** object declarations   ****************** */ 
LCDDisplay oled;
Ultrasonics ultrasonics;
RoverLink rover;
HardwareSerial LidarSerial(1);
LD20_Lidar lidar(&LidarSerial);
LidarPins lidarPins;
TaskHandle_t roverLinkTaskHandle = nullptr;

// Heading hold settings. PID output is a wheel speed, not an angle.
static constexpr float kKp = 0.004f;            // wheel speed per degree of error
static constexpr float kKi = 0.001f;            // wheel speed per degree, per second
static constexpr float kKd = 0.0003f;           // wheel speed per degree/s of yaw rate
static constexpr float kMaxCorrection = 0.10f;  // largest change on each wheel
static constexpr float kMaxIntegral = 0.05f;    // anti-windup limit

static constexpr uint32_t kDrivePeriodMs = 50;
static constexpr float kDriveDtSeconds = kDrivePeriodMs / 1000.0f;
static constexpr uint32_t kMaxCommandAgeMs = 400;
static constexpr uint32_t kMaxImuAgeMs = 200;
static constexpr float kMaxWheelSpeed = 0.5f;   // same limit as RoverLink
static constexpr float kStraightTolerance = 0.02f;
static constexpr float kMinHoldSpeed = 0.05f;
static constexpr float kSettleRateDps = 20.0f;  // lock heading only below this yaw rate
static constexpr float kMaxTiltDeg = 35.0f;

// Keeps an angle between -180 and +180, so 359 to 1 degree is a 2 degree error.
static float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

struct HeadingPid {
  float integral = 0;

  void reset() { integral = 0; }

  float compute(float errorDeg, float yawRateDps, float dtSeconds) {
    integral = constrain(integral + kKi * errorDeg * dtSeconds, -kMaxIntegral, kMaxIntegral);
    const float output = kKp * errorDeg + integral - kKd * yawRateDps;
    return constrain(output, -kMaxCorrection, kMaxCorrection);
  }
};

void imuTask(void* parameter) {
  if (!initBNO055(21, 22)) {
    Serial.println("BNO055 initialization failed");
    while (true) {
      delay(1000);
    }
  }

  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    updateBNO055();
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
  }
}
void imuDriveTestTask(void* parameter) {
  // Start with false. Change only after checking sensor directions.
  constexpr bool allowMovement = false;

  constexpr uint32_t periodMs = 50;
  constexpr uint32_t maxSampleAgeMs = 200;
  constexpr uint32_t stableTimeMs = 1500;
  constexpr uint32_t testDurationMs = 3000;

  constexpr float stableTiltDeg = 5.0f;
  constexpr float stableYawRateDps = 3.0f;
  constexpr float stableHeadingBandDeg = 1.5f;

  constexpr float maxTiltDeg = 15.0f;
  constexpr float maxHeadingErrorDeg = 15.0f;

  constexpr float baseSpeed = 0.10f;
  constexpr float kp = 0.004f;
  constexpr float kd = 0.0003f;
  constexpr float maxCorrection = 0.03f;

  auto readUsableImu = [&](ImuData& sample) -> bool {
    if (!getBNO055Data(sample)) return false;

    return millis() - sample.sampleMs <= maxSampleAgeMs &&
           isfinite(sample.headingDeg) &&
           isfinite(sample.rollDeg) &&
           isfinite(sample.pitchDeg) &&
           isfinite(sample.yawRateDps);
  };

  TickType_t lastWake = xTaskGetTickCount();

  bool trackingStability = false;
  uint32_t stableSinceMs = 0;
  uint32_t lastSampleMs = 0;
  uint32_t lastPrintMs = 0;

  float referenceHeading = 0;
  float targetHeading = 0;

  Serial.println(
      allowMovement ? "[IMU test] Movement enabled"
                    : "[IMU test] Diagnostic mode: movement disabled");

  // Stage 1: wait for stable readings.
  for (;;) {
    rover.stop();

    ImuData sample{};
    const bool imuOk = readUsableImu(sample);
    const uint32_t now = millis();

    const bool newSample =
        imuOk && sample.sampleMs != lastSampleMs;

    const bool linkOk =
        !allowMovement || rover.connected();

    const bool quiet =
        imuOk &&
        fabsf(sample.rollDeg) <= stableTiltDeg &&
        fabsf(sample.pitchDeg) <= stableTiltDeg &&
        fabsf(sample.yawRateDps) <= stableYawRateDps;

    if (!linkOk || !quiet || !newSample) {
      trackingStability = false;
    } else if (!trackingStability ||
               fabsf(wrap180(sample.headingDeg -
                             referenceHeading)) >
                   stableHeadingBandDeg) {
      referenceHeading = sample.headingDeg;
      stableSinceMs = now;
      trackingStability = true;
    } else if (now - stableSinceMs >= stableTimeMs) {
      targetHeading = sample.headingDeg;
      Serial.printf(
          "[IMU test] Stable. Target heading: %.1f deg\n",
          targetHeading);
      break;
    }

    if (imuOk) {
      lastSampleMs = sample.sampleMs;
    }

    if (now - lastPrintMs >= 500) {
      lastPrintMs = now;

      if (!imuOk) {
        Serial.println("[IMU test] Waiting for fresh valid data");
      } else {
        Serial.printf(
            "[IMU test] heading=%.1f roll=%.1f pitch=%.1f "
            "yawRate=%.1f age=%lu ms stable=%d link=%d\n",
            sample.headingDeg,
            sample.rollDeg,
            sample.pitchDeg,
            sample.yawRateDps,
            static_cast<unsigned long>(now - sample.sampleMs),
            trackingStability,
            rover.connected());
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
  }

  // Stage 2: calculate heading-hold commands for three seconds.
  const uint32_t testStartedMs = millis();
  lastPrintMs = 0;

  for (;;) {
    ImuData sample{};

    if (!readUsableImu(sample)) {
      Serial.println("[IMU test] Aborted: IMU data unavailable");
      break;
    }

    if (allowMovement && !rover.connected()) {
      Serial.println("[IMU test] Aborted: rover disconnected");
      break;
    }

    if (fabsf(sample.rollDeg) > maxTiltDeg ||
        fabsf(sample.pitchDeg) > maxTiltDeg) {
      Serial.println("[IMU test] Aborted: tilt limit");
      break;
    }

    const float error =
        wrap180(targetHeading - sample.headingDeg);

    if (fabsf(error) > maxHeadingErrorDeg) {
      Serial.println("[IMU test] Aborted: heading error limit");
      break;
    }

    if (millis() - testStartedMs >= testDurationMs) {
      Serial.println("[IMU test] Test completed");
      break;
    }

    const float correction = constrain(
        kp * error - kd * sample.yawRateDps,
        -maxCorrection,
        maxCorrection);

    const float left = baseSpeed + correction;
    const float right = baseSpeed - correction;

    if (allowMovement) {
      rover.setDrive(left, right);
    } else {
      rover.stop();
    }

    const uint32_t now = millis();

    if (now - lastPrintMs >= 250) {
      lastPrintMs = now;

      Serial.printf(
          "[IMU test] heading=%.1f target=%.1f error=%.2f "
          "yawRate=%.2f correction=%.3f L=%.3f R=%.3f\n",
          sample.headingDeg,
          targetHeading,
          error,
          sample.yawRateDps,
          correction,
          left,
          right);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
  }

  // Stage 3: remain stopped. No automatic restart.
  for (;;) {
    rover.stop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void printDistance(const char* label, int cm) {
  Serial.print(label);

  if (cm < 0) {
    Serial.println(": no valid echo");
  } else {
    Serial.printf(": %d cm\n", cm);
  }
}

bool frontObjectDetection(const int frontCm){
    return frontCm >= 0 && frontCm < 30;
}

void ultrasonicTask(void* parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    ultrasonics.update();
    const int frontCm = ultrasonics.readFrontCm();
    if (frontObjectDetection(frontCm)) {
      Serial.println("Front obstacle within 30 cm");
    }
    printDistance("Front", ultrasonics.readFrontCm());
    printDistance("Left", ultrasonics.readLeftCm());
    printDistance("Right", ultrasonics.readRightCm());
    printDistance("Back", ultrasonics.readBackCm());
    printDistance("Front angled", ultrasonics.readFrontAngledCm());
    Serial.println("--");
    // Start the next measurement cycle every 500 ms.
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
  }
}

void lidarTask(void* parameter) {
  uint32_t lastPrintMs = 0;

  for (;;) {
    const bool newPacket = lidar.readPacket();
    const uint32_t now = millis();

    if (newPacket && now - lastPrintMs >= 500) {
      lastPrintMs = now;

      // Print one sample instead of every point from every packet.
      Serial.printf(
          "LiDAR sample: angle %.1f deg, distance %.1f cm\n",
          lidar.points[0].angle,
          lidar.points[0].distance / 10.0f
      );
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void roverLinkTask(void* parameter) {
  constexpr uint32_t periodMs = 100;

  for (;;) {
    const uint32_t startedMs = millis();

    rover.update();

    const uint32_t elapsedMs = millis() - startedMs;

    const uint32_t waitMs =
        elapsedMs < periodMs ? periodMs - elapsedMs : 1;

    TickType_t waitTicks = pdMS_TO_TICKS(waitMs);

    if (waitTicks == 0) {
      waitTicks = 1;
    }

    vTaskDelay(waitTicks);
  }
}

// // The only task that commands the rover.
// void driveTask(void* parameter) {
//   HeadingPid pid;
//   bool holding = false;
//   float targetHeading = 0;
//   float errorDeg = 0;
//   float correction = 0;
//   uint32_t lastShowMs = 0;
//   TickType_t lastWakeTime = xTaskGetTickCount();

//   for (;;) {
//     vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(kDrivePeriodMs));

//     float left;
//     float right;
//     uint32_t ageMs;
//     ImuData imu;
//     const bool imuOk = getBNO055Data(imu) && millis() - imu.sampleMs <= kMaxImuAgeMs;

//     const bool tooTilted = imuOk &&
//         (fabsf(imu.rollDeg) > kMaxTiltDeg || fabsf(imu.pitchDeg) > kMaxTiltDeg);

//     // Stop if the command is old, IMU data is unavailable,
//     // or the rover is tilted too far.
//     if (ageMs > kMaxCommandAgeMs || !imuOk || tooTilted) {
//       holding = false;
//       pid.reset();
//       rover.stop();
//       continue;
//     }

//     const float base = (left + right) / 2.0f;  // forward part of the command
//     const float turn = (right - left) / 2.0f;  // turning part of the command
//     const bool drivingStraight =
//         fabsf(turn) < kStraightTolerance && fabsf(base) > kMinHoldSpeed;

//     // Turning, stopped, or no IMU: drive exactly as commanded.
//     if (!imuOk || !drivingStraight) {
//       holding = false;
//       rover.setDrive(left, right);
//       continue;
//     }

//     if (!holding) {
//       // The rover may still be finishing a turn. Lock the heading after it settles.
//       if (fabsf(imu.yawRateDps) > kSettleRateDps) {
//         rover.setDrive(left, right);
//         continue;
//       }

//       targetHeading = imu.headingDeg;
//       pid.reset();
//       holding = true;
//     }

//     errorDeg = wrap180(targetHeading - imu.headingDeg);
//     correction = pid.compute(errorDeg, imu.yawRateDps, kDriveDtSeconds);

//     // Keep room for the correction, so one wheel is not clipped at the speed limit.
//     const float maxBase = kMaxWheelSpeed - fabsf(correction);
//     const float safeBase = constrain(base, -maxBase, maxBase);

//     // Positive error = rover must turn clockwise = left wheel faster.
//     // This is also correct when driving backward.
//     rover.setDrive(safeBase + correction, safeBase - correction);
//   }
// }

void setup() {
  Serial.begin(115200);
  
  // Initialize sensors.
  ultrasonics.begin(300, 17500);
  lidar.begin(230400, lidarPins.rxPin,  lidarPins.txPin);
  
  const BaseType_t imuTestResult = xTaskCreatePinnedToCore(
        imuDriveTestTask,
        "imuDriveTest",
        4096,
        nullptr,
        2,
        nullptr,
        1
    );

    if (imuTestResult != pdPASS) {
      Serial.println("Failed to create IMU drive test task");
      return;
    }
   // Create sensor tasks.
  const BaseType_t imuResult = xTaskCreatePinnedToCore(
      imuTask, "imu", 4096, nullptr, 3, nullptr, 1);

  const BaseType_t ultrasonicResult = xTaskCreatePinnedToCore(
      ultrasonicTask, "ultrasonic", 4096, nullptr, 1, nullptr, 1);

  const BaseType_t lidarResult = xTaskCreatePinnedToCore(
      lidarTask, "lidar", 4096, nullptr, 1, nullptr, 1);

  if (imuResult != pdPASS) {
    Serial.println("Failed to create a IMU task");
    return;
  }
    if (ultrasonicResult != pdPASS) {
    Serial.println("Failed to create a ultrasonic task");
    return;
  }
    if (lidarResult != pdPASS) {
    Serial.println("Failed to create a lidar task");
    return;
  }
    rover.begin();
    WiFi.mode(WIFI_AP);

    const IPAddress localIp(192, 168, 18, 1);
    const IPAddress subnet(255, 255, 255, 0);

    if (!WiFi.softAPConfig(localIp, localIp, subnet)) {
      Serial.println("Failed to configure access point");
      return;
    }

    // These must match the network configured on your rover.
    if (!WiFi.softAP("CASbox_08", "87654321", 1)) {
      Serial.println("Failed to start access point");
      return;
    }

    Serial.print("Access point IP: ");
    Serial.println(WiFi.softAPIP());

      const BaseType_t result = xTaskCreatePinnedToCore(
        roverLinkTask,
        "roverLink",
        6144,
        nullptr,
        1,
        &roverLinkTaskHandle,
        0
    );

    if (result != pdPASS) {
      roverLinkTaskHandle = nullptr;
      Serial.println("Failed to create RoverLink task");
      return;
    }
   //     // Only this task may call rover.setDrive() or rover.stop().
  // const BaseType_t driveResult = xTaskCreatePinnedToCore(
  //     driveTask, "drive", 4096, nullptr, 2, nullptr, 1);

  // if (driveResult != pdPASS) {
  //   Serial.println("Failed to create drive task");
  //   }
}

void loop() {
 vTaskDelay(pdMS_TO_TICKS(1000));
}



// #include <SoftwareSerial.h>
// #include <Arduino.h>

// #define POINTS_PER_PACK 12
// #define HEADER 0x54
// #define LIDAR_BAUD 230400
// #define POINT_BUFF_SZ 360

// typedef struct __attribute__((packed)) {
//   uint16_t distance;
//   uint8_t intensity;
// } LidarPoint;

// typedef struct __attribute__((packed)) {
//   uint8_t header;
//   uint8_t ver_len;
//   uint16_t speed;
//   uint16_t start_angle;
//   LidarPoint point[POINTS_PER_PACK];
//   uint16_t end_angle;
//   uint16_t timestamp;
//   uint8_t crc8;
// } LiDARFrame;

// static const uint8_t CrcTable[256] = {
// 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
// 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
// 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
// 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
// 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
// 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
// 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
// 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
// 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
// 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
// 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
// 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
// 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
// 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
// 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
// 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
// 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
// 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
// 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
// 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
// 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
// };

// uint8_t CalCRC8(uint8_t *p, uint8_t len) {
//   uint8_t crc = 0;
//   for (int i = 0; i < len; i++) {
//     crc = CrcTable[(crc ^ *p++) & 0xff];
//   }
//   return crc;
// }

// // Change the RX and TX pins as per your connections.
// SoftwareSerial mySerial(10, 11); // RX, TX

// LiDARFrame lidarFrame;

// void setup() {
//   Serial.begin(9600);
//   // set the data rate for the SoftwareSerial port
//   mySerial.begin(LIDAR_BAUD);
// }

// void loop() {
//   if (mySerial.available()) {
//     if (mySerial.read() == HEADER) {

//       mySerial.readBytes((char*)&lidarFrame + 1, sizeof(LiDARFrame) - 1);

//       if (CalCRC8((uint8_t*)&lidarFrame, sizeof(LiDARFrame) - 1) == lidarFrame.crc8) {

//         double step = (lidarFrame.end_angle - lidarFrame.start_angle) / (POINTS_PER_PACK - 1);

//         for (int i = 0; i < POINTS_PER_PACK; i++) {

//           double angle = (lidarFrame.start_angle + step * i);
//           angle = fmod(angle + 360, 360);

//           //angle = 360 - angle; // Convert to counterclockwise direction
//           angle += 90;
//           angle = fmod(angle + 360, 360);

//           int distance = lidarFrame.point[i].distance;
//           uint8_t intensity = lidarFrame.point[i].intensity;

//           if (intensity < 10) continue;

//           if (distance > 0 and distance <= 200) {
//             Serial.print(angle);
//             Serial.print(F(" degrees, distance: "));
//             Serial.print(distance);
//             Serial.print(F(", intensity: "));
//             Serial.println(intensity);
//           }
//         }
//       } else {
//         //Serial.println("CRC check failed!");
//       }
//     }
//   }
// }