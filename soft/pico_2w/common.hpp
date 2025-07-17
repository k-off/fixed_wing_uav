#pragma once
#include <stdint.h>

/**
 * @brief Packet sent from Vehicle to controller
 */
struct VPayload {
  float lon;
  float lat;
  float alt;
  float vel3d;
  float accel3d;
  uint32_t time;
  uint8_t reserved[6];
  uint16_t crc16;
  void print() {
    if (Serial)
      Serial.printf("\"vehicle\": { \"lon\": %f, \"lat\": %f, \"alt\": %f, \"velocity\": %f, \"accel\": %f, \"time\": %d }\n", lon, lat, alt, vel3d, accel3d, time);
  }
};

/**
 * @brief Packet sent from Controller to vehicle
 */
struct CPayload {
  uint8_t m1;
  uint8_t m2;
  uint8_t m3;
  uint8_t m4;
  int8_t  s1;
  int8_t  s2;
  int8_t  s3;
  int8_t  s4;
  uint8_t reserved[22];
  uint16_t crc16;
  void print() {
    if (Serial)
      Serial.printf("\"controller\": {\"mot1\": %u, \"mot2\": %u, \"mot3\": %u, \"mot4\": %u, \"sig1\": %u, \"sig2\": %u, \"sig3\": %u, \"sig4\": %u}\n", m1, m2, m3, m4, s1, s2, s3, s4);
  }
};


enum Role { CONTROLLER, VEHICLE };
// address of the vehicle and controller
uint8_t gAddr[][6] = {"ctrl1", "vhcl1"};

// default pico 2w SPI0 pins
constexpr const uint8_t gCsPin   = 17u;  // chip select
constexpr const uint8_t gSckPin  = 18u;  // clock
constexpr const uint8_t gTxPin   = 19u;  // data transmit
constexpr const uint8_t gRxPin   = 16u;  // data receive
constexpr const uint8_t gCePin   = 20u;  // nrf chip enable
constexpr const uint8_t gIrcPin  = 21u;  // receive nrf interrupt

enum MotorPins {M1=12, M2=13, M3=14, M4=15};

/**
 * @brief allows to morse-code an error on built-in led when Serial is unavailable
 *  provide an array of delays where even delays for led enabled, and odd ones for led disabled
 * @param err_msg array of delays in ms
 * @param err_length length of array
 */
void blink_errcode(volatile const uint16_t* err_msg, uint16_t err_len) {
  volatile uint8_t state = LOW;
  digitalWrite(LED_BUILTIN, state);
  delay(1000);
  for (uint16_t i = 0; i < err_len; ++i) {
    state != state;
    digitalWrite(LED_BUILTIN, state);
    delay(err_msg[i]);
  }
}

uint16_t crc16_ccitt(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    const uint16_t polynomial = 0x1021;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
