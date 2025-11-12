#include <Arduino.h>
#include <RadioLib.h>
#include <AceCRC.h>

// Select the type of CRC algorithm we'll be using
using namespace ace_crc::crc16ccitt_byte;

// CC1101 pins (adjust as needed)
#define CS_PIN 15
#define GDO0_PIN 5
#define GDO2_PIN 4

// Radio settings
#define FREQUENCY 433.3
#define BITRATE 100.0
#define FDEV 50.0
#define RXBW 135.0
#define OUTPUT_POWER 10
#define PREAMBLE_LENGTH 32
#define PACKET_LENGTH 12 // bytes
#define MIN_RSSI -85 // dBm

// CC1101 module
CC1101 radio = new Module(CS_PIN, GDO0_PIN, RADIOLIB_NC, GDO2_PIN);

// Sync word: only 2 bytes supported, bitshift 0x21a4 to the right so we can add the extra 0 in
const uint8_t syncWord[2] = {0x10, 0xd2};

enum messageType {
  POLL,
  STATE,
  SET,
};


void encodeForTransmission(uint8_t *outBuf, uint32_t deviceID, messageType msgType, bool state){
  // First 4 bytes are device ID
  // 5th Byte is 0x0D if polling, 0x0B if setting
  // 6th Byte is 0x00 if polling, 0x01 if setting
  // 7th Byte is 0x55 if polling, 0x07 if setting a receiver switch, 0x02 if setting a relay (IDK why) 
  // (Relay device IDs seem to start with 0x00 based on a sample size of 4 ^w^)
  // 8th byte is 0xAA for polling, 0x00 to set off, 0x01 to set on
  // Final two bytes are CRC16/AUG-CCITT of first 8 bytes
  uint8_t txBuf[10] = {(deviceID >> 24) & 0xFF,
                      (deviceID >> 16) & 0xFF,
                      (deviceID >> 8) & 0xFF,
                      (deviceID >> 0) & 0xFF,
                      (msgType == POLL) ? 0x0D : 0x0B,
                      (msgType == POLL) ? 0x00 : 0x01,
                      (msgType == POLL) ? 0x55 : ((deviceID >> 24) & 0xFF) ? 0x07 : 0x02,
                      (msgType == POLL) ? 0xAA : (state ? 0x01 : 0x00),
                      0x00,
                      0x00};

  // Calculate CRC and populate final bytes
  crc_t crc = crc_init();
  crc = crc_update(crc, txBuf, 8);
  crc = crc_finalize(crc);

  txBuf[8] = (crc >> 8) & 0xFF;
  txBuf[9] = crc & 0xFF;

  // Takes the 10 byte data packet and returns a 12 byte data packet with the bytes shifted 9 bits along and 0b000100011 inserted at the beginning
  // This accounts for the extra 0 bit in between preamble and actual sync word, and the third byte of the sync word, unsupported by cc1101
  for(int i=0; i<10; i++){
    outBuf[i+1] = (txBuf[i] >> 1) | (i==0 ? 0b10000000 : ((txBuf[i-1] & 0b00000001) << 7));
  }
  outBuf[0] = 0x11;
  outBuf[11] = (txBuf[9] << 7);
}

void decodeTransmission(uint8_t *outBuf, uint8_t *inBuf){
  // Take the 12 byte input packet and deshift it, returning the original 10 byte packet
  for(int i=0; i<10; i++)
    outBuf[i] = (inBuf[i+1] << 1) | ((inBuf[i+2] >> 7) & 0b1);

}


// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setTxFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
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
  radio.setPacketSentAction(setTxFlag);
  radio.setPacketReceivedAction(setRxFlag);
  radio.setSyncWord(syncWord, 2);

  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("done!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true);
  }


  Serial.println("Ready to transmit!");
}

void loop() {

  if(Serial.available()) {
    String input = Serial.readString();
    // Parse string for device ID and chosen state
    String deviceIDStr = input.substring(0, 8);
    uint32_t deviceID = (uint32_t) strtoul(deviceIDStr.c_str(), NULL, 16);

    bool state = input.substring(9,10) == "1" ? true : false;

    uint8_t outdata[12];

    radio.finishReceive();
    delay(50);

    encodeForTransmission(outdata, deviceID, SET, state);

    Serial.print("Transmitting packet: ");
    for(int i=0; i<12; i++) {
      Serial.print(outdata[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    transmissionState = radio.startTransmit(outdata, 12);
  }
  //If transmission finished report status and cleanup
  if(transmittedFlag) {
    transmittedFlag = false;
    if(transmissionState == RADIOLIB_ERR_NONE) {
      Serial.println("Transmit success!");
    } else {
      Serial.print("Transmit failed, code ");
      Serial.println(transmissionState);
    }
    int err = radio.finishTransmit();
    if(err == RADIOLIB_ERR_NONE) {
      Serial.println("cleanup success!");
    } else {
      Serial.print("cleanup failed, code ");
      Serial.println(err);
    }
    delay(25);
    radio.startReceive();
  }

  if(receivedFlag) {
    byte byteArr[PACKET_LENGTH];
    int state = radio.readData(byteArr, PACKET_LENGTH);
    
    if (state == RADIOLIB_ERR_NONE) {
      // Only process the message if RSSI is high enough, this prevents uncessary calculating CRC for noise
      if (radio.getRSSI() > MIN_RSSI) {
        // Last 2 bytes are the CRC-16/AUG-CCITT of the first 3 bytes
        // Verify CRC and only continue if valid
        Serial.print(radio.getRSSI());
        Serial.print(",");
        uint8_t dataBuf[10];
        decodeTransmission(dataBuf, byteArr);
        char data[41] = "";
        bytesToHexString(dataBuf, 10, data);
        Serial.println(data);

      }
    } else {
      // An error occurred receiving the message
      Serial.print("RadioLib error: ");
      Serial.println(state);
    }

    // Reset flag and put module back into listening mode
    // This should be the last thing in the loop
    receivedFlag = false;
    radio.startReceive();
  }
}

// Convert array of bytes into a string containing the HEX representation of the array
void bytesToHexString(byte array[], unsigned int len, char buffer[]) {
    for (unsigned int i = 0; i < len; i++) {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

// Convert array of bytes into a string containing the HEX representation of the array
void byteToHexString(byte mybyte, char buffer[]) {
    byte nib1 = (mybyte >> 4) & 0x0F;
    byte nib2 = (mybyte >> 0) & 0x0F;
    buffer[0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    buffer[2] = '\0';
}
