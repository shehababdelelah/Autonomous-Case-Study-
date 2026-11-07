// #include "Ultrasonics.hpp"
// #include "BNO055Sensor.hpp"
// #include "RoverLink.hpp"
// #include "LidarSensor.hpp"
// #include "LCDDisplay.hpp"
// #include "WEBpage.hpp"
// #include "CampusNavTest.hpp"
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <math.h>
// #include <WiFi.h>

// // Start disabled; enable only after bench tests, measured speed calibration,
// // sonar threshold calibration and confirmation of rover turn conventions.
// static constexpr bool kEnableNavigationTest = true;
// WEBpage web;
// struct SonarSnapshot {
//   int front = -1;
//   int right = -1;
//   int left = -1;
//   uint32_t seq = 0;
//   uint32_t updatedMs = 0;
// };
// static SonarSnapshot sonarData;
// static portMUX_TYPE sonarMux = portMUX_INITIALIZER_UNLOCKED;

// //******************** object declarations   ****************** */ 
// LCDDisplay oled;
// Ultrasonics ultrasonics;
// RoverLink rover;
// HardwareSerial LidarSerial(1);
// LD20_Lidar lidar(&LidarSerial);
// LidarPins lidarPins;
// TaskHandle_t roverLinkTaskHandle = nullptr;

// // Heading hold settings. PID output is a wheel speed, not an angle.
// static constexpr float kKp = 0.004f;            // wheel speed per degree of error
// static constexpr float kKi = 0.001f;            // wheel speed per degree, per second
// static constexpr float kKd = 0.0003f;           // wheel speed per degree/s of yaw rate
// static constexpr float kMaxCorrection = 0.10f;  // largest change on each wheel
// static constexpr float kMaxIntegral = 0.05f;    // anti-windup limit

// static constexpr uint32_t kDrivePeriodMs = 50;
// static constexpr float kDriveDtSeconds = kDrivePeriodMs / 1000.0f;
// static constexpr uint32_t kMaxCommandAgeMs = 400;
// static constexpr uint32_t kMaxImuAgeMs = 200;
// static constexpr float kMaxWheelSpeed = 0.5f;   // same limit as RoverLink
// static constexpr float kStraightTolerance = 0.02f;
// static constexpr float kMinHoldSpeed = 0.05f;
// static constexpr float kSettleRateDps = 20.0f;  // lock heading only below this yaw rate
// static constexpr float kMaxTiltDeg = 35.0f;

// // Keeps an angle between -180 and +180, so 359 to 1 degree is a 2 degree error.
// static float wrap180(float deg) {
//   while (deg > 180.0f) deg -= 360.0f;
//   while (deg <= -180.0f) deg += 360.0f;
//   return deg;
// }

// struct HeadingPid {
//   float integral = 0;

//   void reset() { integral = 0; }

//   float compute(float errorDeg, float yawRateDps, float dtSeconds) {
//     integral = constrain(integral + kKi * errorDeg * dtSeconds, -kMaxIntegral, kMaxIntegral);
//     const float output = kKp * errorDeg + integral - kKd * yawRateDps;
//     return constrain(output, -kMaxCorrection, kMaxCorrection);
//   }
// };

// void imuTask(void* parameter) {
//   if (!initBNO055(21, 22)) {
//     Serial.println("BNO055 initialization failed");
//     while (true) {
//       delay(1000);
//     }
//   }

//   TickType_t lastWakeTime = xTaskGetTickCount();

//   for (;;) {
//     updateBNO055();
//     vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
//   }
// }
// void printDistance(const char* label, int cm) {
//   Serial.print(label);

//   if (cm < 0) {
//     Serial.println(": no valid echo");
//   } else {
//     Serial.printf(": %d cm\n", cm);
//   }
// }

// bool frontObjectDetection(const int frontCm){
//     return frontCm >= 0 && frontCm < 30;
// }

// void ultrasonicTask(void* parameter) {
//   TickType_t lastWakeTime = xTaskGetTickCount();

//   for (;;) {
//     ultrasonics.update();
//     const int frontCm = ultrasonics.readFrontCm();
//     const int rightCm = ultrasonics.readRightCm();
//     const int leftCm = ultrasonics.readLeftCm();
//     portENTER_CRITICAL(&sonarMux);
//     sonarData.front = frontCm;
//     sonarData.right = rightCm;
//     sonarData.left = leftCm;
//     sonarData.updatedMs = millis();
//     ++sonarData.seq;
//     portEXIT_CRITICAL(&sonarMux);
//     if (frontObjectDetection(frontCm)) {
//       Serial.println("Front obstacle within 30 cm");
//     }
//     printDistance("Front", ultrasonics.readFrontCm());
//     printDistance("Left", ultrasonics.readLeftCm());
//     printDistance("Right", ultrasonics.readRightCm());
//     printDistance("Back", ultrasonics.readBackCm());
//     printDistance("Front angled", ultrasonics.readFrontAngledCm());
//     Serial.println("--");
//     // Start the next measurement cycle every 500 ms.
//     vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
//   }
// }



// constexpr int kBins = 360;
// constexpr int kMaxObjects = 16;
// constexpr int kMinObjectBins = 2;       // ignore single-degree noise
// constexpr uint16_t kMinMm = 150;
// constexpr uint16_t kMaxMm = 8000;
// constexpr uint8_t kMinIntensity = 10;   // starting value, tune it

// struct LidarObject {
//   float angleDeg;
//   float distanceCm;
//   float widthCm;
//   float xCm;          // + is right
//   float yCm;          // + is front
//   const char* side;
// };

// static void addPoint(uint16_t* scan, const LidarPoint& p) {
//   if (p.distance < kMinMm || p.distance > kMaxMm) return;
//   if (p.intensity < kMinIntensity) return;
//   const int bin = static_cast<int>(p.angle) % kBins;
//   if (scan[bin] == 0 || p.distance < scan[bin]) scan[bin] = p.distance;
// }

// // Same object if the distance changes by less than 15 cm or 10%.
// static bool sameObject(uint16_t a, uint16_t b) {
//   const int gap = abs(static_cast<int>(a) - static_cast<int>(b));
//   const int limit = max(150, static_cast<int>(min(a, b)) / 10);
//   return gap < limit;
// }

// static const char* sideName(float angle) {
//   if (angle < 45.0f || angle >= 315.0f) return "Front";
//   if (angle < 135.0f) return "Right";
//   if (angle < 225.0f) return "Back";
//   return "Left";
// }
// static int findObjects(uint16_t* scan, LidarObject* out, int maxOut) {
//   // Fill one-degree holes, so a missing point does not split an object.
//   for (int i = 0; i < kBins; ++i) {
//     const uint16_t before = scan[(i + kBins - 1) % kBins];
//     const uint16_t after = scan[(i + 1) % kBins];
//     if (scan[i] == 0 && before != 0 && after != 0 && sameObject(before, after)) {
//       scan[i] = min(before, after);
//     }
//   }

//   // Start at the edge of an object, so one crossing 360/0 stays whole.
//   int start = -1;
//   for (int i = 0; i < kBins; ++i) {
//     const uint16_t before = scan[(i + kBins - 1) % kBins];
//     if (scan[i] != 0 && (before == 0 || !sameObject(before, scan[i]))) {
//       start = i;
//       break;
//     }
//   }
//   if (start < 0) return 0;

//   int count = 0;
//   int i = 0;
//   while (i < kBins) {
//     const int first = (start + i) % kBins;
//     if (scan[first] == 0) {
//       ++i;
//       continue;
//     }

//     int length = 1;
//     uint16_t nearest = scan[first];
//     while (i + length < kBins) {
//       const uint16_t current = scan[(first + length) % kBins];
//       const uint16_t previous = scan[(first + length - 1) % kBins];
//       if (current == 0 || !sameObject(previous, current)) break;
//       nearest = min(nearest, current);
//       ++length;
//     }

//     if (length >= kMinObjectBins && count < maxOut) {
//       const float angle = fmodf(first + length / 2.0f, 360.0f);
//       const float radians = angle * DEG_TO_RAD;
//       const float cm = nearest / 10.0f;
//       LidarObject& object = out[count++];
//       object.angleDeg = angle;
//       object.distanceCm = cm;
//       object.widthCm = cm * length * DEG_TO_RAD;
//       object.xCm = cm * sinf(radians);
//       object.yCm = cm * cosf(radians);
//       object.side = sideName(angle);
//     }
//     i += length;
//   }
//   return count;
// }

// // void lidarTask(void* parameter) {
// //   static uint16_t scanMm[kBins] = {};
// //   LidarObject objects[kMaxObjects];
// //   float lastAngle = 0;
// //   uint32_t lastPrintMs = 0;

// //   for (;;) {
// //     if (lidar.readPacket()) {
// //       for (int i = 0; i < LD20_Lidar::POINTS_PER_FRAME; ++i) {
// //         const LidarPoint& p = lidar.points[i];

// //         if (p.angle + 180.0f < lastAngle) {  // one rotation is finished
// //           const int count = findObjects(scanMm, objects, kMaxObjects);
// //           memset(scanMm, 0, sizeof(scanMm));

// //           if (millis() - lastPrintMs >= 500) {
// //             lastPrintMs = millis();
// //             for (int k = 0; k < count; ++k) {
// //               Serial.printf("%s: angle %.1f, distance %.1f cm, width %.1f cm\n",
// //                             objects[k].side, objects[k].angleDeg,
// //                             objects[k].distanceCm, objects[k].widthCm);
// //             }
// //             Serial.println("--");
// //           }
// //         }
// //         lastAngle = p.angle;
// //         addPoint(scanMm, p);
// //       }
// //     }
// //     vTaskDelay(pdMS_TO_TICKS(1));
// //   }
// // }

// void roverLinkTask(void* parameter) {
//   constexpr uint32_t periodMs = 100;

//   for (;;) {
//     const uint32_t startedMs = millis();

//     rover.update();

//     const uint32_t elapsedMs = millis() - startedMs;

//     const uint32_t waitMs =
//         elapsedMs < periodMs ? periodMs - elapsedMs : 1;

//     TickType_t waitTicks = pdMS_TO_TICKS(waitMs);

//     if (waitTicks == 0) {
//       waitTicks = 1;
//     }

//     vTaskDelay(waitTicks);
//   }
// }

// // // The only task that commands the rover.
// // void driveTask(void* parameter) {
// //   HeadingPid pid;
// //   bool holding = false;
// //   float targetHeading = 0;
// //   float errorDeg = 0;
// //   float correction = 0;
// //   uint32_t lastShowMs = 0;
// //   TickType_t lastWakeTime = xTaskGetTickCount();

// //   for (;;) {
// //     vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(kDrivePeriodMs));

// //     float left;
// //     float right;
// //     uint32_t ageMs;
// //     ImuData imu;
// //     const bool imuOk = getBNO055Data(imu) && millis() - imu.sampleMs <= kMaxImuAgeMs;

// //     const bool tooTilted = imuOk &&
// //         (fabsf(imu.rollDeg) > kMaxTiltDeg || fabsf(imu.pitchDeg) > kMaxTiltDeg);

// //     // Stop if the command is old, IMU data is unavailable,
// //     // or the rover is tilted too far.
// //     if (ageMs > kMaxCommandAgeMs || !imuOk || tooTilted) {
// //       holding = false;
// //       pid.reset();
// //       rover.stop();
// //       continue;
// //     }

// //     const float base = (left + right) / 2.0f;  // forward part of the command
// //     const float turn = (right - left) / 2.0f;  // turning part of the command
// //     const bool drivingStraight =
// //         fabsf(turn) < kStraightTolerance && fabsf(base) > kMinHoldSpeed;

// //     // Turning, stopped, or no IMU: drive exactly as commanded.
// //     if (!imuOk || !drivingStraight) {
// //       holding = false;
// //       rover.setDrive(left, right);
// //       continue;
// //     }

// //     if (!holding) {
// //       // The rover may still be finishing a turn. Lock the heading after it settles.
// //       if (fabsf(imu.yawRateDps) > kSettleRateDps) {
// //         rover.setDrive(left, right);
// //         continue;
// //       }

// //       targetHeading = imu.headingDeg;
// //       pid.reset();
// //       holding = true;
// //     }

// //     errorDeg = wrap180(targetHeading - imu.headingDeg);
// //     correction = pid.compute(errorDeg, imu.yawRateDps, kDriveDtSeconds);

// //     // Keep room for the correction, so one wheel is not clipped at the speed limit.
// //     const float maxBase = kMaxWheelSpeed - fabsf(correction);
// //     const float safeBase = constrain(base, -maxBase, maxBase);

// //     // Positive error = rover must turn clockwise = left wheel faster.
// //     // This is also correct when driving backward.
// //     rover.setDrive(safeBase + correction, safeBase - correction);
// //   }
// // }


// // Sole task allowed to command the wheels during this test.
// void campusNavTask(void* parameter) {
//   if (!kEnableNavigationTest) {
//     rover.stop();
//     Serial.println("[NAV] DISABLED: calibrate and explicitly enable first");
//     vTaskDelete(nullptr);
//     return;
//   }

//   // Avoid motion until rover is connected, sensors have updated, and IMU settled.
//   uint32_t startedMs = millis();
//   ImuData imu{};
//   SonarSnapshot sonar{};
//   while (true) {
//     portENTER_CRITICAL(&sonarMux);
//     sonar = sonarData;
//     portEXIT_CRITICAL(&sonarMux);
//     const bool imuOk = getBNO055Data(imu) &&
//         millis() - imu.sampleMs <= 200 && isfinite(imu.headingDeg) &&
//         isfinite(imu.yawRateDps) && isfinite(imu.pitchDeg) &&
//         isfinite(imu.rollDeg);
//     if (rover.connected() && imuOk && sonar.seq > 0 &&
//         millis() - sonar.updatedMs < 1000 &&
//         fabsf(imu.yawRateDps) < 4.0f) break;
//     rover.stop();
//     if (millis() - startedMs > 30000) {
//       Serial.println("[NAV] Initial checks timed out: not driving");
//       vTaskDelete(nullptr);
//       return;
//     }
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }

//   CampusNavTest nav(imu.headingDeg, millis());
//   TickType_t wake = xTaskGetTickCount();
//   uint32_t lastPrintMs = 0;
//   while (true) {
//     vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
//     portENTER_CRITICAL(&sonarMux);
//     sonar = sonarData;
//     portEXIT_CRITICAL(&sonarMux);

//     const uint32_t now = millis();
//     const bool imuOk = getBNO055Data(imu) && now - imu.sampleMs <= 200 &&
//         isfinite(imu.headingDeg) && isfinite(imu.yawRateDps) &&
//         isfinite(imu.pitchDeg) && isfinite(imu.rollDeg);
//     if (!rover.connected() || !imuOk) {
//       rover.stop();
//       Serial.println("[NAV] STOP: link lost or IMU invalid");
//       break;
//     }

//     CampusNavTest::Input input{
//       imu.headingDeg, imu.yawRateDps, imu.pitchDeg, imu.rollDeg,
//       sonar.front, sonar.right, sonar.left,
//       sonar.seq, sonar.updatedMs, now
//     };
//     const auto result = nav.update(input);
//     if (result.hardStop || nav.state() == CampusNavTest::State::DONE) {
//       rover.stop();
//       Serial.printf("[NAV] TERMINATED: %s\n", CampusNavTest::name(nav.state()));
//       break;
//     }
//     rover.setDrive(result.left, result.right);

//     if (now - lastPrintMs >= 500) {
//       lastPrintMs = now;
//       Serial.printf("[NAV] %s pillars=%u est=%.2fm H=%.1f front=%d right=%d left=%d L=%.3f R=%.3f\n",
//         CampusNavTest::name(nav.state()),
//         static_cast<unsigned>(nav.pillarCount()), nav.estimatedMeters(),
//         imu.headingDeg, sonar.front, sonar.right, sonar.left,
//         result.left, result.right);
//     }
//   }
//   rover.stop();
//   vTaskDelete(nullptr);
// }

// void setup() {
//   Serial.begin(115200);
  
//   // Initialize sensors.
//   ultrasonics.begin(300, 17500);
//   lidar.begin(230400, lidarPins.rxPin,  lidarPins.txPin);
  
// // Create sensor tasks.
//   const BaseType_t imuResult = xTaskCreatePinnedToCore(
//       imuTask, "imu", 4096, nullptr, 3, nullptr, 1);

//   const BaseType_t ultrasonicResult = xTaskCreatePinnedToCore(
//       ultrasonicTask, "ultrasonic", 4096, nullptr, 1, nullptr, 1);

//   // const BaseType_t lidarResult = xTaskCreatePinnedToCore(
//   //     lidarTask, "lidar", 4096, nullptr, 1, nullptr, 1);

//   if (imuResult != pdPASS) {
//     Serial.println("Failed to create a IMU task");
//     return;
//   }
//     if (ultrasonicResult != pdPASS) {
//     Serial.println("Failed to create a ultrasonic task");
//     return;
//   }
//   //   if (lidarResult != pdPASS) {
//   //   Serial.println("Failed to create a lidar task");
//   //   return;
//   // }
//     rover.begin();
//     WiFi.mode(WIFI_AP);

//     const IPAddress localIp(192, 168, 18, 1);
//     const IPAddress subnet(255, 255, 255, 0);

//     if (!WiFi.softAPConfig(localIp, localIp, subnet)) {
//       Serial.println("Failed to configure access point");
//       return;
//     }

//     // These must match the network configured on your rover.
//     if (!WiFi.softAP("CASbox_08", "87654321", 1)) {
//       Serial.println("Failed to start access point");
//       return;
//     }

//     Serial.print("Access point IP: ");
//     Serial.println(WiFi.softAPIP());
//     if (!web.begin()) {
//     Serial.println("Navigation web server failed");
//     return;
//     }
//       const BaseType_t result = xTaskCreatePinnedToCore(
//         roverLinkTask,
//         "roverLink",
//         6144,
//         nullptr,
//         1,
//         &roverLinkTaskHandle,
//         0
//     );

//     if (result != pdPASS) {
//       roverLinkTaskHandle = nullptr;
//       Serial.println("Failed to create RoverLink task");
//       return;
//     }
//     const BaseType_t navTaskResult = xTaskCreatePinnedToCore(
//         campusNavTask, "campusNav", 6144, nullptr, 2, nullptr, 1);
//     if (navTaskResult != pdPASS) {
//       Serial.println("Failed to create campus navigation test task");
//       rover.stop();
//     }

//    //     // Only this task may call rover.setDrive() or rover.stop().
//   // const BaseType_t driveResult = xTaskCreatePinnedToCore(
//   //     driveTask, "drive", 4096, nullptr, 2, nullptr, 1);

//   // if (driveResult != pdPASS) {
//   //   Serial.println("Failed to create drive task");
//   //   }
// }

// void loop() {
//  vTaskDelay(pdMS_TO_TICKS(1000));
// }



#include "Ultrasonics.hpp"
#include "BNO055Sensor.hpp"
#include "RoverLink.hpp"
#include "LidarSensor.hpp"
#include "LCDDisplay.hpp"
#include "WEBpage.hpp"
#include "CampusNavTest.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <WiFi.h>

// Start disabled; enable only after bench tests, measured speed calibration,
// sonar threshold calibration and confirmation of rover turn conventions.
static constexpr bool kEnableNavigationTest = true;
WEBpage web;
struct SonarSnapshot {
  int front = -1;
  int right = -1;
  int left = -1;
  uint32_t seq = 0;
  uint32_t updatedMs = 0;
};
static SonarSnapshot sonarData;
static portMUX_TYPE sonarMux = portMUX_INITIALIZER_UNLOCKED;

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
    const int rightCm = ultrasonics.readRightCm();
    const int leftCm = ultrasonics.readLeftCm();
    portENTER_CRITICAL(&sonarMux);
    sonarData.front = frontCm;
    sonarData.right = rightCm;
    sonarData.left = leftCm;
    sonarData.updatedMs = millis();
    ++sonarData.seq;
    portEXIT_CRITICAL(&sonarMux);
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



constexpr int kBins = 360;
constexpr int kMaxObjects = 16;
constexpr int kMinObjectBins = 2;       // ignore single-degree noise
constexpr uint16_t kMinMm = 150;
constexpr uint16_t kMaxMm = 8000;
constexpr uint8_t kMinIntensity = 10;   // starting value, tune it

struct LidarObject {
  float angleDeg;
  float distanceCm;
  float widthCm;
  float xCm;          // + is right
  float yCm;          // + is front
  const char* side;
};

static void addPoint(uint16_t* scan, const LidarPoint& p) {
  if (p.distance < kMinMm || p.distance > kMaxMm) return;
  if (p.intensity < kMinIntensity) return;
  const int bin = static_cast<int>(p.angle) % kBins;
  if (scan[bin] == 0 || p.distance < scan[bin]) scan[bin] = p.distance;
}

// Same object if the distance changes by less than 15 cm or 10%.
static bool sameObject(uint16_t a, uint16_t b) {
  const int gap = abs(static_cast<int>(a) - static_cast<int>(b));
  const int limit = max(150, static_cast<int>(min(a, b)) / 10);
  return gap < limit;
}

static const char* sideName(float angle) {
  if (angle < 45.0f || angle >= 315.0f) return "Front";
  if (angle < 135.0f) return "Right";
  if (angle < 225.0f) return "Back";
  return "Left";
}
static int findObjects(uint16_t* scan, LidarObject* out, int maxOut) {
  // Fill one-degree holes, so a missing point does not split an object.
  for (int i = 0; i < kBins; ++i) {
    const uint16_t before = scan[(i + kBins - 1) % kBins];
    const uint16_t after = scan[(i + 1) % kBins];
    if (scan[i] == 0 && before != 0 && after != 0 && sameObject(before, after)) {
      scan[i] = min(before, after);
    }
  }

  // Start at the edge of an object, so one crossing 360/0 stays whole.
  int start = -1;
  for (int i = 0; i < kBins; ++i) {
    const uint16_t before = scan[(i + kBins - 1) % kBins];
    if (scan[i] != 0 && (before == 0 || !sameObject(before, scan[i]))) {
      start = i;
      break;
    }
  }
  if (start < 0) return 0;

  int count = 0;
  int i = 0;
  while (i < kBins) {
    const int first = (start + i) % kBins;
    if (scan[first] == 0) {
      ++i;
      continue;
    }

    int length = 1;
    uint16_t nearest = scan[first];
    while (i + length < kBins) {
      const uint16_t current = scan[(first + length) % kBins];
      const uint16_t previous = scan[(first + length - 1) % kBins];
      if (current == 0 || !sameObject(previous, current)) break;
      nearest = min(nearest, current);
      ++length;
    }

    if (length >= kMinObjectBins && count < maxOut) {
      const float angle = fmodf(first + length / 2.0f, 360.0f);
      const float radians = angle * DEG_TO_RAD;
      const float cm = nearest / 10.0f;
      LidarObject& object = out[count++];
      object.angleDeg = angle;
      object.distanceCm = cm;
      object.widthCm = cm * length * DEG_TO_RAD;
      object.xCm = cm * sinf(radians);
      object.yCm = cm * cosf(radians);
      object.side = sideName(angle);
    }
    i += length;
  }
  return count;
}

// void lidarTask(void* parameter) {
//   static uint16_t scanMm[kBins] = {};
//   LidarObject objects[kMaxObjects];
//   float lastAngle = 0;
//   uint32_t lastPrintMs = 0;

//   for (;;) {
//     if (lidar.readPacket()) {
//       for (int i = 0; i < LD20_Lidar::POINTS_PER_FRAME; ++i) {
//         const LidarPoint& p = lidar.points[i];

//         if (p.angle + 180.0f < lastAngle) {  // one rotation is finished
//           const int count = findObjects(scanMm, objects, kMaxObjects);
//           memset(scanMm, 0, sizeof(scanMm));

//           if (millis() - lastPrintMs >= 500) {
//             lastPrintMs = millis();
//             for (int k = 0; k < count; ++k) {
//               Serial.printf("%s: angle %.1f, distance %.1f cm, width %.1f cm\n",
//                             objects[k].side, objects[k].angleDeg,
//                             objects[k].distanceCm, objects[k].widthCm);
//             }
//             Serial.println("--");
//           }
//         }
//         lastAngle = p.angle;
//         addPoint(scanMm, p);
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(1));
//   }
// }

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


// Sole task allowed to command the wheels during this test.
// The only normal wheel-command owner. Lives in main.cpp.
void campusNavTask(void*) {
  CampusNavTest nav(0, millis());
  WEBpage::Status view;
  bool active=false, navStarted=false, tracking=false;
  uint32_t started=0, stableSince=0, lastSample=0, elapsed=0;
  unsigned stableSamples=0;
  float reference=0;
  const char* state="IDLE";
  const char* phase="idle";
  const char* note=kEnableNavigationTest ? "Press Start on the navigation page"
    : "Navigation disabled: kEnableNavigationTest is false";
  uint8_t lastStage=0;
  TickType_t wake=xTaskGetTickCount();

  for (;;) {
    SonarSnapshot sonar;
    portENTER_CRITICAL(&sonarMux);sonar=sonarData;portEXIT_CRITICAL(&sonarMux);
    ImuData imu{};
    const bool hasImu=getBNO055Data(imu);
    const uint32_t now=millis(); // Capture after copying samples to avoid future timestamp subtraction.
    const bool imuOk=hasImu && now-imu.sampleMs<=200 &&
      isfinite(imu.headingDeg) && isfinite(imu.yawRateDps) &&
      isfinite(imu.rollDeg) && isfinite(imu.pitchDeg);
    const bool sonarOk=sonar.seq>0 && now-sonar.updatedMs<=1500;
    const bool linkOk=rover.connected();
    const auto command=web.takeCommand();

    if(command==WEBpage::Command::Stop) {
      active=false;state="STOPPED";phase="stopped";note="Stop requested from webpage";
    } else if(command==WEBpage::Command::Start) {
      if(!kEnableNavigationTest) {
        active=false;state="FAULT";phase="fault";note="Navigation disabled by kEnableNavigationTest";
    } else if(!linkOk) {
        active=false;state="IDLE";phase="idle";note="Start refused: rover is not connected";
    } else {
        active=true;navStarted=false;tracking=false;stableSamples=0;lastSample=0;
        started=now;elapsed=0;lastStage=0;nav=CampusNavTest(0,now);
        state="WAITING";phase="running";note="Waiting for fresh inputs and stable IMU";
      }
    }

    float left=0,right=0;
    if(active) {
      elapsed=now-started;
      if(!kEnableNavigationTest) {
        active=false;state="FAULT";phase="fault";note="Navigation disabled";
      } else if(!imuOk || !sonarOk || !linkOk) {
        tracking=false;stableSamples=0;
        note="Waiting for fresh IMU, sonar and rover link";
        if(navStarted) {
          active=false;state="FAULT";phase="fault";note="Stopped: IMU, sonar or rover link unavailable";
        }
      } else if(fabsf(imu.rollDeg)>18 || fabsf(imu.pitchDeg)>18) {
        active=false;state="FAULT";phase="fault";note="Stopped: tilt above 18 degrees";
      } else if(sonar.front<CampusNavTest::kFrontStopCm) {
        // Deliberate wrapper policy: unknown front (-1) also blocks movement.
        active=false;state="FAULT";phase="fault";
        note=sonar.front<0?"Stopped: front range unknown":"Stopped: front obstacle below 35 cm";
      } else {
        if(!navStarted && imu.sampleMs!=lastSample) {
          lastSample=imu.sampleMs;
          const bool quiet=fabsf(imu.yawRateDps)<=3 && fabsf(imu.rollDeg)<=5 && fabsf(imu.pitchDeg)<=5;
          if(!quiet) {tracking=false;stableSamples=0;}
          else if(!tracking || fabsf(wrap180(imu.headingDeg-reference))>1.5f) {
            tracking=true;stableSince=now;stableSamples=1;reference=imu.headingDeg;
          } else ++stableSamples;
          if(tracking && stableSamples>=20 && now-stableSince>=1500) {
            nav=CampusNavTest(imu.headingDeg,now);navStarted=true;
          }
        }
        if(navStarted) {
          CampusNavTest::Input in{imu.headingDeg,imu.yawRateDps,imu.pitchDeg,imu.rollDeg,
            sonar.front,sonar.right,sonar.left,sonar.seq,sonar.updatedMs,now};
          const auto out=nav.update(in);
          state=CampusNavTest::name(nav.state());note=state;
          if(out.hardStop || nav.state()==CampusNavTest::State::FAULT) {
            active=false;phase="fault";note="CampusNavTest fault; inspect phase/sensor logs";
          } else if(nav.state()==CampusNavTest::State::DONE) {
            active=false;phase="completed";note="Configured navigation stage completed";
          } else {
            left=out.left;right=out.right;lastStage=static_cast<uint8_t>(nav.state());
          }
        } else note="Waiting for 1.5 seconds of stable IMU";
      }
      if(active && !navStarted && elapsed>30000) {
        active=false;state="FAULT";phase="fault";note="Startup checks timed out after 30 seconds";
      }
    }
    if(!active) left=right=0;
    if(left==0 && right==0) rover.stop(); else rover.setDrive(left,right);

    view.enabled=kEnableNavigationTest;view.connected=linkOk;view.imuOk=imuOk;view.imu=imu;
    view.frontCm=sonar.front;view.rightCm=sonar.right;view.leftCm=sonar.left;
    view.sonarMs=sonar.updatedMs;view.sonarSequence=sonar.seq;
    snprintf(view.state,sizeof(view.state),"%s",state);
    snprintf(view.phase,sizeof(view.phase),"%s",phase);
    snprintf(view.note,sizeof(view.note),"%s",note);
    view.pillars=nav.pillarCount();view.estimatedM=nav.estimatedMeters();
    view.segmentM=nav.segmentMeters();view.targetHeadingDeg=navStarted?nav.targetHeadingDeg():NAN;
    view.headingErrorDeg=navStarted && imuOk?wrap180(nav.targetHeadingDeg()-imu.headingDeg):NAN;
    view.pillarArmed=nav.pillarArmed();view.nearSamples=nav.nearSamples();view.farSamples=nav.farSamples();
    view.lastStageIndex=lastStage;view.leftCommand=left;view.rightCommand=right;view.elapsedMs=elapsed;
    web.publish(view);
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize sensors.
  ultrasonics.begin(300, 17500);
  lidar.begin(230400, lidarPins.rxPin,  lidarPins.txPin);
  
// Create sensor tasks.
  const BaseType_t imuResult = xTaskCreatePinnedToCore(
      imuTask, "imu", 4096, nullptr, 3, nullptr, 1);

  const BaseType_t ultrasonicResult = xTaskCreatePinnedToCore(
      ultrasonicTask, "ultrasonic", 4096, nullptr, 1, nullptr, 1);

  // const BaseType_t lidarResult = xTaskCreatePinnedToCore(
  //     lidarTask, "lidar", 4096, nullptr, 1, nullptr, 1);

  if (imuResult != pdPASS) {
    Serial.println("Failed to create a IMU task");
    return;
  }
    if (ultrasonicResult != pdPASS) {
    Serial.println("Failed to create a ultrasonic task");
    return;
  }
  //   if (lidarResult != pdPASS) {
  //   Serial.println("Failed to create a lidar task");
  //   return;
  // }
    rover.begin();
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    const IPAddress localIp(192, 168, 18, 1);
    const IPAddress subnet(255, 255, 255, 0);

    if (!WiFi.softAPConfig(localIp, localIp, subnet)) {
      Serial.println("Failed to configure access point");
      return;
    }

    // These must match the network configured on your rover.
        if (!WiFi.softAP("CAS-Box-08", "CASbox08", 11)) {
      Serial.println("Failed to start access point");
      return;
    }

    Serial.print("Access point IP: ");
    Serial.println(WiFi.softAPIP());

    if (!web.begin()) {
      Serial.println("Navigation web server failed");
      return;
    }


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
    const BaseType_t navTaskResult = xTaskCreatePinnedToCore(
        campusNavTask, "campusNav", 6144, nullptr, 2, nullptr, 1);
    if (navTaskResult != pdPASS) {
      Serial.println("Failed to create campus navigation test task");
      rover.stop();
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





