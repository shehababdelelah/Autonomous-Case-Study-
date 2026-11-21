#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <time.h>

// GpsDriver - GY-GPS6MV2 (u-blox NEO-6M) on the CAS box.
//   Wiring (from the course code GpsManager.cpp):
//     ESP32 GPIO16 (RX2) <- GPS TX     (positions from the GPS)
//     ESP32 GPIO17 (TX2) -> GPS RX     (commands to the GPS, used for EGNOS)
//   9600 baud, NMEA, one position per second.
//
// What it adds to plain TinyGPSPlus:
//   - Turns on EGNOS (SBAS corrections) in begin(). Software only, no new hardware.
//     The command is sent at every start, so the module's saved settings do not matter.
//   - Tells you if EGNOS is really used: fixQuality 2 = GPS + EGNOS, 1 = GPS only.
//   - One snapshot struct with everything (GpsData).
//
// Use (3 lines):
//   GpsDriver gps;
//   setup():  gps.begin();
//   loop():   gps.poll();               // every pass, never blocks
//   anywhere in the same task:  GpsData d = gps.data();
// Call poll() and data() from the same task (normally loop()).

// One GPS reading. Values that are not known are 0 (and fix is false).
struct GpsData {
  bool fix;             // true = position younger than GpsDriver::kMaxFixAgeMs
  double latDeg;        // latitude in degrees
  double lonDeg;        // longitude in degrees
  uint8_t satellites;   // satellites used
  float hdop;           // horizontal dilution of precision, lower is better
  uint8_t fixQuality;   // 0 = no fix, 1 = GPS only, 2 = GPS + EGNOS (SBAS)
  uint32_t fixAgeMs;    // age of the position in ms (UINT32_MAX = no position yet)
  uint32_t utcDate;     // ddmmyy
  uint32_t utcTime;     // hhmmsscc
  float altitudeM;
  float speedMps;       // not useful at walking / rover speed
  float courseDeg;      // not useful at walking / rover speed
};

enum class EgnosState : uint8_t {
  Off,       // begin(false) was used
  Waiting,   // command sent, waiting for the GPS answer
  On,        // the GPS accepted the command (ACK)
  Failed     // no answer after 3 tries (check the GPIO17 -> GPS RX wire)
};

class GpsDriver {
public:
  static constexpr int kRxPin = 16;
  static constexpr int kTxPin = 17;
  static constexpr uint32_t kBaud = 9600;
  static constexpr uint32_t kMaxFixAgeMs = 3000;

  // Opens Serial2 and sends the EGNOS command. Does not wait: the answer is read in poll().
  void begin(bool enableEgnos = true);

  // Closes Serial2. Used when the GPS is switched off on the page.
  void end();

  // Reads every waiting byte from the GPS. Call it every loop pass. Never blocks.
  void poll();

  // The newest reading.
  GpsData data();

  EgnosState egnos() const { return _egnos; }
  const char* egnosText() const;            // "ON", "FAILED", "waiting", "off"
  static const char* fixQualityText(uint8_t q);   // "GPS + EGNOS", "GPS only", "no fix"

  // Bytes received from the GPS since start. 0 after a few seconds = no data (wiring / power).
  uint32_t charsReceived() { return _gps.charsProcessed(); }

private:
  void _sendEgnosCommand();
  void _checkAck(uint8_t c);

  TinyGPSPlus _gps;
  TinyGPSCustom _ggaQuality{_gps, "GPGGA", 6};   // fix quality field of the GGA sentence

  EgnosState _egnos = EgnosState::Off;
  uint8_t _tries = 0;
  uint32_t _sentMs = 0;
  uint8_t _ackPos = 0;
  uint8_t _nakPos = 0;
};

// ---- Rtc: the clock inside the ESP32, set from the GPS ----
// The ESP32 has its own clock. Once the GPS has given it a date and a time, the box reads the
// time from here and not from the GPS, so the time keeps running when the GPS is switched off
// or loses its fix. It is lost when the power is cut, and set again at the next GPS fix.
// The clock itself keeps UTC; kTimeZoneHours is added only when the time is written out.
class Rtc {
 public:
  static constexpr int kTimeZoneHours = 1;          // Germany in winter. Summer time: 2.
  static constexpr uint32_t kResyncMs = 600000;     // set it again from the GPS every 10 minutes

  // utcDate = ddmmyy and utcTime = hhmmsscc, exactly as GpsData carries them.
  // Returns true the moment the clock was set or re-set.
  bool setFromGps(uint32_t utcDate, uint32_t utcTime);

  bool isSet() const { return set_; }
  void text(char* out, size_t size) const;   // "21.11.2026 14:03:12", or "no time yet"
  void dateText(char* out, size_t size) const;   // "21.11.2026"
  void timeText(char* out, size_t size) const;   // "14:03:12"

 private:
  void parts(struct tm& t) const;

  bool set_ = false;
  uint32_t lastSyncMs_ = 0;
};
