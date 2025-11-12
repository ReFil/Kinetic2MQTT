#include <Arduino.h>
#include <RadioLib.h>

// CC1101 pins (adjust as needed)
#define CS_PIN 15
#define GDO0_PIN 5
#define GDO2_PIN 16

// Radio settings
#define FREQUENCY 433.3
#define BITRATE 100.0
#define FDEV 50.0
#define RXBW 135.0
#define OUTPUT_POWER 10
#define PREAMBLE_LENGTH 32

// 10 data bytes, with 0b1 0x23 bitshifted in from the left to account for third sync word byte 
// Actual data packet is B115EDDF0B0107016492
uint8_t payload1[12] = {0x11, 0xD8, 0x8A, 0xF6, 0xEF, 0x85, 0x80, 0x83, 0x80, 0xB2, 0x49};
uint8_t payload[10] = {0xB1, 0x15, 0xED, 0xDF, 0x0B, 0x01, 0x07, 0x00, 0x74, 0xB3};

// CC1101 module
CC1101 radio = new Module(CS_PIN, GDO0_PIN, RADIOLIB_NC, GDO2_PIN);

// Sync word: only 2 bytes supported, bitshift 0x21a4 to the right so we can add the extra 0 in
const uint8_t syncWord[2] = {0x10, 0xd2};

void encodeForTransmission(uint8_t *outBuf, uint8_t *inBuf){
  uint8_t remainder;
  // Takes the 10 byte data packet and returns a 12 byte data packet with the bytes shifted 9 bits along and 0b000100011 inserted at the beginning
  for(int i=0; i<10; i++){
    outBuf[i+1] = (inBuf[i] >> 1) | (i==0 ? 0b10000000 : ((inBuf[i-1] & 0b00000001) << 7));
  }
  outBuf[0] = 0x11;
  outBuf[11] = (inBuf[9] << 7);
}

void cc1101_prepareNextTransmit() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x3B); // SFTX
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(10);

  radio.standby(); // ready for next TX
  SPI.endTransaction();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("Initializing CC1101...");

  int state = radio.begin(FREQUENCY, BITRATE, FDEV, RXBW, OUTPUT_POWER, PREAMBLE_LENGTH);
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("Radio init failed, code ");
    Serial.println(state);
    while(true);
  }

  radio.setCrcFiltering(false);
  radio.fixedPacketLengthMode(12);
  radio.setSyncWord(syncWord, 2);

  Serial.println("Ready to transmit!");
}

void loop() {
  if(Serial.available()) {
    String input = Serial.readString();
    if(input == "0\n") {
      payload[7] = 0x00;
      payload[8] = 0x74;
      payload[9] = 0xb3;
    }
    else {
      payload[7] = 0x01;
      payload[8] = 0x64;
      payload[9] = 0x92;
    }
    uint8_t outdata[13] = {0x00};
    encodeForTransmission(outdata, payload);
    Serial.print("Transmitting packet: ");
    for(int i=0; i<12; i++) {
      Serial.print(outdata[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    int16_t res = radio.startTransmit(outdata, 12);

    if(res == RADIOLIB_ERR_NONE) {
      Serial.println("Transmit success!");
    } else {
      Serial.print("Transmit failed, code ");
      Serial.println(res);
    }

    res = radio.finishTransmit();
    if(res == RADIOLIB_ERR_NONE) {
      Serial.println("Transmit success!");
    } else {
      Serial.print("Transmit failed, code ");
      Serial.println(res);
    }
    delay(50); // wait before next send
  }
}
