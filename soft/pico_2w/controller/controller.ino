#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "/Users/user/Desktop/repos/fixed_wing/soft/pico_2w/common.hpp"

#define NDEBUG // comment out to enable prints

CPayload txPayload {128, 128, 128, 128, 1, 2, 3, 4, {}};
VPayload rxPayload {};

struct Joystick {
  uint8_t x, y, btn;

  Joystick(uint8_t _pinX, uint8_t _pinY, uint8_t _pinBtn) : 
      x(), y(), btn(), pinX(_pinX), pinY(_pinY), pinBtn(_pinBtn)
  {
    pinMode(pinX, INPUT);
    pinMode(pinY, INPUT);
    pinMode(pinBtn, INPUT_PULLUP);
  }
  void update() {
    x = analogRead(pinX) >> 2;
    y = analogRead(pinY) >> 2;
    btn = digitalRead(pinBtn);
  }
  void println() {
    if (Serial) {
      Serial.printf("\"joystick\": { \"x\": %u, \"y\": %u, \"btn\": %s }", x, y, (btn ? "up" : "down"));
    }
  }
private:
  const uint8_t pinX, pinY, pinBtn;
  float minX, minY, maxX, maxY;
};

Joystick joy1(26, 27, 28);

RF24 radio(gCePin, gCsPin);
const uint8_t radioNumber = 0;
uint32_t timeout = 1000000u;

float rxTotal = 0, rxFail = 0, txTotal = 0, txFail = 0;
uint32_t start_timer = 0;
uint32_t end_timer = 0;
uint32_t gRxTimeoutUS = 3000u;


volatile const uint16_t eNoSerial[] = {200, 350, 200, 350, 200, 1150};
volatile const uint16_t eNoRadio[] = {500, 350, 200, 350, 200, 500, 1150};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

#ifndef NDEBUG
  Serial.begin(115200);
  while (!Serial) {
    blink_errcode(eNoSerial, sizeof(eNoSerial) / sizeof(eNoSerial[0]));
  }
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nStarting setup\n");
#endif

  if (!radio.begin()) {
#ifndef NDEBUG
    Serial.print(F("radio hardware is not responding!!\n"));
#endif
    while (1) {
      blink_errcode(eNoRadio, sizeof(eNoRadio) / sizeof(eNoRadio[0]));
    }
  }
  
  radio.setChannel(64);
  radio.setDataRate(RF24_250KBPS);
  radio.disableCRC();
  radio.disableAckPayload();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(rxPayload));
  radio.setAddressWidth(sizeof(gAddr[CONTROLLER]) - 1);
  radio.stopListening();
  radio.openWritingPipe(gAddr[!CONTROLLER]);
  radio.openReadingPipe(1, gAddr[CONTROLLER]);
  radio.startListening();

#ifndef NDEBUG
  radio.printPrettyDetails();
  Serial.printf("Setup success\n");
#endif
}


void transmit() {
  txTotal += 1;
  radio.stopListening();
  delayMicroseconds(130);
  start_timer = micros();
  bool report = radio.write(&txPayload, sizeof(CPayload));
  end_timer = micros();

  if (report) {
#ifndef NDEBUG
    Serial.print(F("Transmission successful! "));
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);
    Serial.print(F(" us: "));
    txPayload.print();
#endif
  } else {
    txFail += 1;
#ifndef NDEBUG
    Serial.println(F("Transmission failed or timed out\n"));
#endif
  }
  radio.startListening();
  delayMicroseconds(130);
  receive();
}

void receive() {
  rxTotal += 1;
  uint8_t pipe;
  while (!radio.available(&pipe) && micros() - end_timer < gRxTimeoutUS) ;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize();
    radio.read(&rxPayload, bytes);
    decrypt_payload((uint32_t*)&rxPayload, sizeof(rxPayload), keys[!CONTROLLER]);
    if (rxPayload.crc16 != crc16_ccitt((const uint8_t*)&rxPayload, sizeof(rxPayload) - 2)) {
#ifndef NDEBUG
      rxPayload.println();
      rxFail += 1;
#endif
      rxPayload = {};
    }
#ifndef NDEBUG
    Serial.print(F("Received "));
    Serial.print(bytes);
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);
    Serial.print(F(": "));
    rxPayload.print();
#endif
  } else {
    rxFail += 1;
#ifndef NDEBUG
    Serial.printf("\t\t\t\tController didn't receive a response from the vehicle.\n");
#endif
  }
}

void tick_alive() {
  volatile static uint32_t prev_time = micros();
  volatile static uint8_t counter = 1u;
  uint32_t curr_time = micros();
  if (curr_time - prev_time >= timeout) {
    float failRate = rxFail / rxTotal * 100;
    Serial.printf("\nTotal packets sent %.0f, response fail rate = %5.2f%%\n", txTotal, failRate);
    rxPayload.print();
    rxPayload = {};
    
    joy1.println();
    txTotal = txFail = rxTotal = rxFail = 0;
    Serial.print((counter ? "." : "\n"));
    digitalWrite(LED_BUILTIN, counter & 1);
    prev_time = curr_time;
    ++counter;
  }
}

// prepare infos to be sent to the vehicle here
// crc must be added after all fields
// encryptrion must be done after crc added
void update_payload() {
  joy1.update();
  txPayload.m1 = joy1.x;
  txPayload.m2 = joy1.x;
  txPayload.m3 = joy1.x;
  txPayload.m4 = joy1.x;
  
  txPayload.s1 = joy1.y;
  txPayload.s2 = joy1.btn;
  txPayload.crc16 = crc16_ccitt((const uint8_t*)&txPayload, sizeof(txPayload) - 2);
  encrypt_payload((uint32_t*)&txPayload, sizeof(txPayload), keys[CONTROLLER]);
}

void loop() {
  update_payload();
  transmit();
  tick_alive();
}
