// =============================================================================================
//  CAS Box - starter program for the sensor box on the WAVE ROVER
//  Case Study Cooperative and Autonomous Systems, version 1.0 (2026-10-15)
// =============================================================================================
//
//  WHAT IT DOES
//    The box ESP32 is the "brain" of your robot. It opens its own Wi-Fi, serves a web page for
//    your phone, reads all box sensors and drives the WAVE ROVER underneath it over Wi-Fi.
//
//      phone ──Wi-Fi──► box (this program): web page, sensors ──Wi-Fi──► rover (motors)
//
//    Web page: drive pad (hold a key = drive, release = stop), speed slider, LiDAR top view,
//    ultrasonic distances, IMU (roll/pitch/yaw), GPS. "Dev" (top right) shows raw values.
//
//  BEFORE YOU FLASH: set BOX_ID below to the number of your kit (sticker on box and rover).
//    Everything else follows from it - do not change the naming scheme:
//      Wi-Fi name  CAS-Box-<NN>      e.g. kit 7 -> CAS-Box-07
//      password    CASbox<NN>        e.g. kit 7 -> CASbox07
//      web page    http://192.168.<10+N>.1   e.g. kit 7 -> http://192.168.17.1
//      channel     odd kits 1, even kits 11
//    Your rover is already set up to join exactly this Wi-Fi. A wrong BOX_ID means: your rover
//    does not find your box, or your box takes over another team's network.
//
//  START ORDER: switch on the box first, then the rover (the rover looks for its box only
//    while it boots). Phone: join CAS-Box-<NN>, open the web page.
//
//  ARDUINO IDE: board "ESP32 Dev Module", ESP32 core 3.3.x, default partition scheme.
//    Required libraries and versions: README.md. Serial monitor: 115200 baud.
//
//  FILES
//    cas_box.ino        this file: settings, setup(), loop()
//    RoverLink.*        box -> rover link (sends drive commands, reads battery voltage)
//    Sensors.*          all sensors behind one object (ultrasonic, LiDAR, IMU, GPS)
//    LidarScan.h        LiDAR (LD20) reader with noise filtering
//    WebHMI.*           web server and web page
//    UsManager.*        5 ultrasonic sensors (HC-SR04), measured in a background task
//    ImuManager.*       orientation sensor BNO055 (roll / pitch / yaw in the ISO 8855 vehicle frame)
//    GpsManager.*       GPS module NEO-6M
//    DisplayManager.*   small OLED on the box
//
//  SAFETY / BEHAVIOUR
//    The rover stops when you release the key, when the page loses focus, when the box gets no
//    update from the phone for 0.4 s, when RoverLink gets no new set point for 0.5 s, and - in
//    the rover itself - after 3 s without a command. Do not remove these stops.
//    The rover firmware must not be changed.
// =============================================================================================

#include "WebHMI.h"
#include "RoverLink.h"
#include "Sensors.h"
#include "DisplayManager.h"
// Team B4: Strict parsing and finite-number validation for serial speed values.
#include <stdlib.h>
#include <math.h>

// ---- YOUR KIT ------------------------------------------------------------------------------
const int BOX_ID = 8;                     // 1..10 = kit number on the box

// ---- settings --------------------------------------------------------------------------------
const float MAX_SPEED = 0.5f;             // drive pad maximum; 0.5 = full rover speed (100 % PWM)
const uint32_t PAD_TIMEOUT_MS = 400;      // no update from the phone for this long -> stop
// Team B4: Serial Monitor test speed and automatic stop timeout.
const float SERIAL_TEST_SPEED = 0.2f;     // speed for Serial Monitor tests
// Team B4: Run each serial movement command for five seconds, then stop automatically.
const uint32_t SERIAL_TIMEOUT_MS = 5000;
const int LIDAR_OFFSET_DEG = 90;          // rotates the LiDAR view so that "front" is up

// ---- objects ---------------------------------------------------------------------------------
WebHMI hmi;           // web server + page
RoverLink rover;      // connection to the rover
Sensors sensors;      // all box sensors
DisplayManager disp;  // small OLED on the box
bool oledReady = false;
char boxSsid[16], boxPassword[16];
// Team B4: State for serial control; 'w' returns control to the web page.
bool serialDriveMode = false;             // first serial direction takes control; 'w' returns to web
float serialLeft = 0, serialRight = 0;
uint32_t serialDriveMs = 0;

// Team B4: Parse a complete line, e.g. f0.3 or l 0.4; a letter alone uses the default speed.
void applySerialDrive(String command) {
    command.trim();
    if (command.length() == 0) return;
    char c = command[0];
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    String value = command.substring(1);
    value.trim();
    float s = SERIAL_TEST_SPEED;
    bool valid = c == 'f' || c == 'b' || c == 'l' || c == 'r' || c == 's' || c == 'w';
    if (value.length() > 0) {
      char* end = nullptr;
      s = strtof(value.c_str(), &end);
      valid = valid && end != value.c_str() && *end == '\0' && isfinite(s)
              && s >= 0.0f && s <= MAX_SPEED && c != 's' && c != 'w';
    }
    if (!valid) {
      serialDriveMode = true;
      serialLeft = serialRight = 0;
      Serial.println("[Team B4] Invalid command: stopped. Use f0.3 / b0.3 / l0.3 / r0.3, speed 0..0.5, s=stop, w=web.");
      return;
    }
    if (c == 'w') {
      serialDriveMode = false;
      serialLeft = serialRight = 0;
      Serial.println("[drive] Web control enabled");
      return;
    }
    s = constrain(s, 0.0f, MAX_SPEED);
    switch (c) {
      case 'f': serialLeft = s;  serialRight = s;  break;
      case 'b': serialLeft = -s; serialRight = -s; break;
      case 'l': serialLeft = -s; serialRight = s;  break;
      case 'r': serialLeft = s;  serialRight = -s; break;
      case 's': serialLeft = 0;  serialRight = 0;  break;
      default:
        serialLeft = serialRight = 0;
        Serial.println("[drive] Unknown command: stopped. Use f/b/l/r/s, w=web");
        break;
    }
    serialDriveMode = true;
    serialDriveMs = millis();
    Serial.printf("[drive] %c -> L=%.2f R=%.2f (timeout %lu ms)\n",
                  c, serialLeft, serialRight, (unsigned long)SERIAL_TIMEOUT_MS);
    // Team B4: Distinguish received serial input from an available rover Wi-Fi link.
    Serial.printf("[Team B4] %s; HTTP ok=%lu fail=%lu\n",
                  rover.statusText().c_str(), (unsigned long)rover.okCount(),
                  (unsigned long)rover.failCount());
    if (!rover.connected()) {
      Serial.println("[Team B4] Rover not connected: power the box first, then restart the rover. Check BOX_ID.");
      // Team B4: Do not leave a movement command pending while the rover is disconnected.
      serialLeft = serialRight = 0;
    }
}

// Team B4: Nonblocking line input; CR/LF completes a command. Discard oversized lines safely.
void readSerialDrive() {
  static String line;
  static bool overflow = false;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!overflow) applySerialDrive(line);
      line = "";
      overflow = false;
    } else if (!overflow) {
      if (line.length() < 31) {
        line += c;
      } else {
        overflow = true;
        line = "";
        serialDriveMode = true;
        serialLeft = serialRight = 0;
        Serial.println("[Team B4] Command too long: stopped. Send a new line.");
      }
    }
  }
}

// OLED: Wi-Fi name, web address, rover state. Started only when the display answers on I2C.
void updateOled() {
  static uint32_t lastTry = 0;
  if (!oledReady) {
    if (millis() - lastTry < 5000) return;
    lastTry = millis();
    if (!i2cPresent(0x3C)) return;
    oledReady = disp.begin();
  }
  String ip = WiFi.softAPIP().toString();
  String r = rover.connected() ? "Rover " + String(rover.voltage(), 1) + "V" : "Rover: search";
  disp.show(boxSsid, ip.c_str(), r.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(200);
  // Team B4: Explain direction-plus-speed commands and required Serial Monitor line ending.
  Serial.println("[Team B4] Serial: f0.3 / b0.3 / l0.3 / r0.3; speed 0..0.5; s=stop w=web");
  Serial.println("[Team B4] Select Newline or Both NL & CR in Serial Monitor, then send a command.");
  // Team B4: Show the configured movement duration; a new command restarts the timer.
  Serial.printf("[drive] Movement stops after %lu ms; s=stop immediately.\n",
                (unsigned long)SERIAL_TIMEOUT_MS);

  // naming scheme, derived from BOX_ID only (see header)
  snprintf(boxSsid, sizeof(boxSsid), "CAS-Box-%02d", BOX_ID);
  snprintf(boxPassword, sizeof(boxPassword), "CASbox%02d", BOX_ID);
  uint8_t channel = (BOX_ID % 2) ? 1 : 11;

  hmi.setTitle(String("CAS Box ") + BOX_ID);
  hmi.enableDrivePad(MAX_SPEED);
  hmi.setStateText("Starting ...");
  hmi.begin(boxSsid, boxPassword, IPAddress(192, 168, 10 + BOX_ID, 1), channel);

  sensors.begin(LIDAR_OFFSET_DEG);
  rover.begin(100);  // sends the current drive command every 100 ms
}

void loop() {
  uint32_t t0 = micros();
  hmi.tick();
  sensors.update();

  // ---- driving -------------------------------------------------------------------------------
  // Manual driving from the phone. For your own (autonomous) behaviour, compute l and r
  // yourself from the sensors, e.g. sensors.usCm(0) = front ultrasonic in cm, and call
  // rover.setDrive(l, r) every loop pass (range -0.5..0.5 per side).
  float l, r;
  uint32_t age;
  hmi.getDrive(l, r, age);
  if (age > PAD_TIMEOUT_MS) l = r = 0;  // key released, page closed or phone gone
  // Team B4: Apply serial commands while serial control is active, with an automatic stop.
  readSerialDrive();
  if (serialDriveMode) {
    l = serialLeft;
    r = serialRight;
    // Team B4: Report the automatic stop once, then clear the expired serial speeds.
    if (millis() - serialDriveMs > SERIAL_TIMEOUT_MS) {
      if (serialLeft != 0 || serialRight != 0) {
        Serial.println("[Team B4] Serial timeout: motors stopped; send another direction to test again.");
      }
      serialLeft = serialRight = 0;
      l = r = 0;
    }
  }
  rover.setDrive(l, r);

  // ---- web page and OLED -----------------------------------------------------------------------
  uint32_t now = millis();
  static uint32_t lastSnap = 0, lastUi = 0, lastOled = 0;
  if (now - lastSnap >= 250) {  // sensor snapshot for the page, 4 times per second
    lastSnap = now;
    hmi.setSensorJson(sensors.toJson());
  }
  if (now - lastUi >= 500) {  // status line + values in the "Dev" view
    lastUi = now;
    hmi.setStateText(rover.statusText());
    hmi.showValue("rover.ip", rover.ip().toString());
    hmi.showValue("rover.voltage_V", rover.voltage());
    hmi.showValue("rover.rtt_ms", rover.rttMs());
    hmi.showValue("rover.ok", (double)rover.okCount());
    hmi.showValue("rover.fail", (double)rover.failCount());
    hmi.showValue("sys.heap_kB", ESP.getFreeHeap() / 1024.0);
  }
  if (now - lastOled >= 1000) {
    lastOled = now;
    updateOled();
  }

  // ---- load report on Serial every 10 s (loop time, free memory, sensor snapshot) -------------
  static uint32_t lastRep = 0, n = 0, maxUs = 0;
  static uint64_t sumUs = 0;
  uint32_t dt = micros() - t0;
  n++; sumUs += dt; if (dt > maxUs) maxUs = dt;
  if (now - lastRep >= 10000) {
    Serial.printf("[sys] loop avg %.2f ms max %.1f ms, heap %u kB, rover %s\n",
                  sumUs / 1000.0 / n, maxUs / 1000.0, ESP.getFreeHeap() / 1024,
                  rover.connected() ? "ok" : "searching");
    Serial.printf("[sys] %s\n", sensors.toJson().c_str());
    lastRep = now; n = 0; sumUs = 0; maxUs = 0;
  }
  delay(2);
}
