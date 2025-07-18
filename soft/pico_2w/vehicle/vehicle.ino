#include <printf.h>
#include <RF24.h>

// either copy this header in both controller and vehicle directories, or update the path
#include "/Users/user/Desktop/repos/fixed_wing/soft/pico_2w/common.hpp"


#define NDEBUG // comment out to enable prints

RF24 radio(gCePin, gCsPin);
VPayload txPayload {1.1, 2.2, 3.3, 4.4, 5.5, 134, {}};
CPayload rxPayload {};

uint32_t gTickTimeout = 1000000u;

float rxTotal = 0, rxFail = 0, txTotal = 0, txFail = 0;
uint32_t start_timer = 0;
uint32_t end_timer = 0;

volatile const uint16_t eNoSerial[] = {200, 350, 200, 350, 200, 1150};
volatile const uint16_t eNoRadio[] = {500, 350, 200, 350, 200, 500, 1150};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  analogWriteFreq(20000);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);

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

  radio.stopListening(gAddr[VEHICLE]);
  radio.setChannel(64);
  radio.setDataRate(RF24_250KBPS);
  radio.disableCRC();
  radio.disableAckPayload();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);
  radio.setPayloadSize(sizeof(rxPayload));
  radio.setAddressWidth(sizeof(gAddr[!VEHICLE]) - 1);
  radio.stopListening();
  radio.openWritingPipe(gAddr[!VEHICLE]);
  radio.openReadingPipe(1, gAddr[VEHICLE]);
  radio.startListening();

#ifndef NDEBUG
  radio.printPrettyDetails();
  Serial.printf("Setup success\n");
#endif
}

// prepare infos to be sent to the controller here
// crc must be added after all fields
// encryption must be done after crc added
void update_payload() {
  txPayload.crc16 = crc16_ccitt((const uint8_t*)&txPayload, sizeof(txPayload) - 2);
  encrypt_payload((uint32_t*)&txPayload, sizeof(txPayload), keys[VEHICLE]);
}

void transmit() {
  radio.stopListening();
  delayMicroseconds(130);
  start_timer = micros();
  update_payload();
  bool report = radio.write(&txPayload, sizeof(VPayload)); 
  end_timer = micros();

  txTotal += 1;
  if (report) {
#ifndef NDEBUG
    Serial.print(F("Transmission successful! "));
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);
    Serial.print(F(" us."));
#endif
  } else {
    txFail += 1;
#ifndef NDEBUG
    Serial.println(F("Transmission failed or timed out"));
#endif
  }
  radio.startListening();
  delayMicroseconds(130);
}

void receive() {
  uint8_t pipe;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize();
    radio.read(&rxPayload, bytes);
    rxTotal += 1;
    decrypt_payload((uint32_t*)&rxPayload, sizeof(rxPayload), keys[!VEHICLE]);
    if (rxPayload.crc16 != crc16_ccitt((const uint8_t*)&rxPayload, sizeof(rxPayload) - 2)) {
      rxFail += 1;
      Serial.println(rxPayload.crc16);
      Serial.println(crc16_ccitt((const uint8_t*)&rxPayload, sizeof(rxPayload) - 2));
      rxPayload = {};
      Serial.print("CRC mismatch\n");
    }
    transmit();
#ifndef NDEBUG
    Serial.print(F("Received\n"));
    Serial.print(bytes);
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);
    Serial.print(F(": "));
    rxPayload.print();
#endif
  }
}

void tick_alive() {
  volatile static uint32_t prev_time = micros();
  volatile static uint8_t counter = 1u;
  uint32_t curr_time = micros();
  if (curr_time - prev_time >= gTickTimeout) {
#ifndef NDEBUG
    Serial.printf("Total packets received %.0f (fail rate %5.2f%%), total packets sent %.0f (fail rate %5.2f%%)\n", rxTotal, rxFail/rxTotal*100, txTotal, txFail/txTotal*100);
    rxPayload.print();
    Serial.print((counter ? "." : "\n"));
#endif
    txTotal = txFail = rxTotal = rxFail = 0;
    digitalWrite(LED_BUILTIN, counter & 1);
    prev_time = curr_time;
    rxPayload = {}; // shutdown all if connection is lost
    ++counter;
  }
}

void loop() {
  tick_alive();
  receive();
  analogWrite(M1, rxPayload.m1);
  analogWrite(M2, rxPayload.m2);
  analogWrite(M3, rxPayload.m3);
  analogWrite(M4, rxPayload.m4);
}
