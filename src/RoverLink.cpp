#include "RoverLink.hpp"

#include <HTTPClient.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <math.h>

static const uint16_t kHttpTimeoutMs = 400;
static const uint8_t kStopRepeats = 3;
static const uint8_t kLostAfterFails = 10;
static const uint32_t kDeadmanMs = 500;
static const uint32_t kSearchEveryMs = 2000;
static const uint32_t kIdleEveryMs = 500;

// Speeds use the range -0.5 to 0.5.
static const float kMaxSpeed = 0.5f;

// Speed units per second. Increase for faster response; decrease for gentler movement.
static const float kAcceleration = 0.4f;
static const float kDeceleration = 0.8f;

// Limit ramp time after delays so one delayed task cycle cannot cause a large speed jump.
static const float kMaxRampStepSeconds = 0.1f;

static float clampDriveSpeed(float speed) {
  if (speed > kMaxSpeed) return kMaxSpeed;
  if (speed < -kMaxSpeed) return -kMaxSpeed;
  return speed;
}

static float clampDirectionSpeed(float speed) {
  if (speed < 0) return 0;
  if (speed > kMaxSpeed) return kMaxSpeed;
  return speed;
}

// Moves current toward target by at most rate * elapsedSeconds.
// When changing direction, it slows to zero before speeding up the other way.
static float rampSpeed(float current, float target, float elapsedSeconds) {
  if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
    target = 0;
  }

  const bool slowingDown = fabsf(target) < fabsf(current);
  const float rate = slowingDown ? kDeceleration : kAcceleration;
  const float maxChange = rate * elapsedSeconds;

  if (current < target) {
    const float next = current + maxChange;
    return next > target ? target : next;
  }

  if (current > target) {
    const float next = current - maxChange;
    return next < target ? target : next;
  }

  return current;
}

// Percent-encodes JSON so it can safely go in the HTTP query string.
static String urlEncode(const String& text) {
  const char* hex = "0123456789ABCDEF";
  String encoded;

  for (size_t i = 0; i < text.length(); ++i) {
    const char c = text[i];

    if (isalnum(static_cast<unsigned char>(c)) ||
        c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else {
      encoded += '%';
      encoded += hex[(c >> 4) & 0x0F];
      encoded += hex[c & 0x0F];
    }
  }

  return encoded;
}

void RoverLink::begin() {
  lastFindMs = 0;
  lastIdleMs = 0;
  lastRampMs = millis();

  stopsLeft = 0;
  idleStatus = false;
  currentLeft = 0;
  currentRight = 0;

  portENTER_CRITICAL(&_mux);
  _setL = 0;
  _setR = 0;
  _setMs = millis();
  portEXIT_CRITICAL(&_mux);
}

void RoverLink::setDrive(float left, float right) {
  portENTER_CRITICAL(&_mux);
  _setL = clampDriveSpeed(left);
  _setR = clampDriveSpeed(right);
  _setMs = millis();
  portEXIT_CRITICAL(&_mux);
}

void RoverLink::forward(float speed) {
  speed = clampDirectionSpeed(speed);
  setDrive(speed, speed);
}

void RoverLink::backward(float speed) {
  speed = clampDirectionSpeed(speed);
  setDrive(-speed, -speed);
}

void RoverLink::turnLeft(float speed) {
  speed = clampDirectionSpeed(speed);
  setDrive(-speed, speed);
}

void RoverLink::turnRight(float speed) {
  speed = clampDirectionSpeed(speed);
  setDrive(speed, -speed);
}

void RoverLink::stop() {
  setDrive(0, 0);
}

String RoverLink::statusText() const {
  if (!connected()) return "Rover: searching ...";

  String text = "Rover " + ip().toString();
  const float batteryVoltage = _voltage;

  if (batteryVoltage > 0) {
    text += ", " + String(batteryVoltage, 2) + " V";
  }

  text += ", " + String(_rttMs, 0) + " ms";
  return text;
}

// Sends one JSON command. HTTP 200 means the rover replied successfully.
int RoverLink::_cmd(uint32_t ipAddress, const String& json, String* reply) {
  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  http.setConnectTimeout(kHttpTimeoutMs);

  const uint32_t startMs = millis();
  const String url =
      "http://" + IPAddress(ipAddress).toString() + "/js?json=" + urlEncode(json);

  if (!http.begin(url)) return -100;

  const int code = http.GET();

  if (code == 200) {
    const String body = http.getString();
    if (reply != nullptr) *reply = body;

    const float roundTripMs = static_cast<float>(millis() - startMs);
    _rttMs = (_rttMs == 0)
        ? roundTripMs
        : 0.8f * _rttMs + 0.2f * roundTripMs;

    ++_ok;
    _failRow = 0;
  } else {
    ++_fail;

    if (++_failRow >= kLostAfterFails && _ipRaw != 0) {
      Serial.println("[rover] lost, searching again");
      _ipRaw = 0;
    }
  }

  http.end();
  return code;
}

// Finds the rover among devices connected to this box's Wi-Fi access point.
bool RoverLink::_find() {
  wifi_sta_list_t list = {};

  const esp_err_t listResult =
      esp_wifi_ap_get_sta_list(&list);

  if (listResult != ESP_OK) {
    Serial.printf(
        "[rover] station-list error: %d\n",
        static_cast<int>(listResult));
    return false;
  }

  static uint32_t lastLogMs = 0;

  if (millis() - lastLogMs >= 10000) {
    lastLogMs = millis();

    Serial.printf(
        "[rover] searching, %d station(s) on the AP\n",
        list.num);
  }

  esp_netif_t* ap =
      esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

  if (ap == nullptr) {
    Serial.println("[rover] AP network interface not found");
    return false;
  }

  for (int i = 0; i < list.num; ++i) {
    esp_netif_pair_mac_ip_t pair = {};
    memcpy(pair.mac, list.sta[i].mac, sizeof(pair.mac));

    const esp_err_t ipResult =
        esp_netif_dhcps_get_clients_by_mac(ap, 1, &pair);

    if (ipResult != ESP_OK || pair.ip.addr == 0) {
      Serial.printf(
          "[rover] client %d: IP lookup failed, result=%d\n",
          i,
          static_cast<int>(ipResult));
      continue;
    }

    // Probe THIS device while pair is still in scope.
    String reply;

    const int httpCode =
        _cmd(pair.ip.addr, "{\"T\":405}", &reply);

    const bool recognized =
        reply.indexOf("wifi_mode_on_boot") >= 0;

    Serial.printf(
        "[rover] probe %s HTTP=%d recognized=%s\n",
        IPAddress(pair.ip.addr).toString().c_str(),
        httpCode,
        recognized ? "yes" : "no");

    if (httpCode == 200 && recognized) {
      _failRow = 0;
      _ipRaw = pair.ip.addr;

      Serial.printf(
          "[rover] found at %s\n",
          IPAddress(_ipRaw).toString().c_str());

      return true;
    }
  }  // Close the loop AFTER checking each device.

  return false;  // No matching device was found.
}
// Runs in the background: finds the rover, ramps wheel speeds, and sends commands.
void RoverLink::update() {
  const uint32_t startMs = millis();
  const uint32_t roverIp = _ipRaw;

  if (roverIp == 0) {
    if (startMs - lastFindMs >= kSearchEveryMs) {
      lastFindMs = startMs;
      _find();
    }

    return;
  }

    float targetLeft;
    float targetRight;
    uint32_t setMs;

    portENTER_CRITICAL(&_mux);
    targetLeft = _setL;
    targetRight = _setR;
    setMs = _setMs;
    portEXIT_CRITICAL(&_mux);

    // If main has not refreshed its command recently, request a stop.
    if (millis() - setMs > kDeadmanMs) {
      targetLeft = 0;
      targetRight = 0;
    }

    float elapsedSeconds = (startMs - lastRampMs) / 1000.0f;
    lastRampMs = startMs;

    if (elapsedSeconds > kMaxRampStepSeconds) {
      elapsedSeconds = kMaxRampStepSeconds;
    }

    if (targetLeft == 0.0f && targetRight == 0.0f) {
    if (currentLeft != 0.0f || currentRight != 0.0f) {
    currentLeft = 0.0f;
    currentRight = 0.0f;
    stopsLeft = kStopRepeats;
    }
    } else {
    currentLeft  = rampSpeed(currentLeft, targetLeft, elapsedSeconds);
    currentRight = rampSpeed(currentRight, targetRight, elapsedSeconds);
    }
    
    if (currentLeft != 0 || currentRight != 0) {
      const String command =
          String("{\"T\":1,\"L\":") + String(currentLeft, 2) +
          ",\"R\":" + String(currentRight, 2) + "}";

      _cmd(roverIp, command, nullptr);
      stopsLeft = kStopRepeats;
    } else if (stopsLeft > 0) {
      // Repeat stop commands until the rover confirms them.
      if (_cmd(roverIp, "{\"T\":1,\"L\":0,\"R\":0}", nullptr) == 200) {
        --stopsLeft;
      }
    } else if (startMs - lastIdleMs >= kIdleEveryMs) {
      lastIdleMs = startMs;
      idleStatus = !idleStatus;

      String reply;
      if (!idleStatus) {
        _cmd(roverIp, "{\"T\":1,\"L\":0,\"R\":0}", nullptr);
      } else if (_cmd(roverIp, "{\"T\":130}", &reply) == 200) {
        const int voltageIndex = reply.indexOf("\"v\":");
        if (voltageIndex >= 0) {
          _voltage = reply.substring(voltageIndex + 4).toFloat();
        }
      }
  }
}