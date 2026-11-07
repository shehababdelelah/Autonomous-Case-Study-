#ifndef LD20_LIDAR_H
#define LD20_LIDAR_H

#include <Arduino.h>

// --- Protocol & Configuration Constants ---
#define LIDAR_HEADER 0x54
#define LIDAR_VERLEN 0x2C
#define POINT_PER_PACK 12
#define PACKET_SIZE 47

#ifdef robot
  #define lidar_offset 180
#else
  #define lidar_offset 90
#endif

// --- Data Structures ---
struct LidarPoint {
  uint16_t distance;
  uint8_t intensity;
  float angle;
};
// // ESP32 RX <- LD20 TX, ESP32 TX -> LD20 RX (unused by the LD20)
struct LidarPins {
   int rxPin = 27;
   int txPin = 33;  
};
// --- LD20 LiDAR driver ---
class LD20_Lidar {
public:
  static const int POINTS_PER_FRAME = POINT_PER_PACK;

  // Latest decoded packet.
  uint16_t speed;
  LidarPoint points[POINTS_PER_FRAME];

  LD20_Lidar(HardwareSerial* serialPort);

  void begin(uint32_t baud, int rx_pin, int tx_pin);
  bool readPacket();

private:
  HardwareSerial* _lidarSerial;
  uint8_t _rxBuffer[PACKET_SIZE];
  int _bufferIndex;

  struct RawLidarPacket {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    struct {
      uint16_t distance;
      uint8_t intensity;
    } raw_points[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
  };

  static const uint8_t CrcTable[256];

  uint8_t calculateCRC8(const uint8_t* data, uint8_t len) const;
  bool parsePacket(const uint8_t* packet, RawLidarPacket* raw_packet) const;
  void calculateAngles(const RawLidarPacket& raw_packet);
};

#endif // LD20_LIDAR_H
