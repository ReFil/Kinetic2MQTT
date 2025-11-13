#include <Arduino.h>
#include <RadioLib.h>
#include <AceCRC.h>
#include "kinetic_helpers.h"

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

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;
// flag to track whether we are actively transmitting
volatile bool txInProgress = false;

// Falling edge interrupt signalling completion of packet transmission/reception
// Used to signal end of transmission

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setTxFlag(void) {
  // Both GDO0 and GDO2 signal the same events, only set the flag if we did actually transmit something
  // This interrupt will fire often but there's not much we can do about that without lib changes
  if(txInProgress)
    transmittedFlag = true;
  
}

volatile bool receivedFlag = false;

volatile bool rxInProgress = false;

// Rising edge interrupt signalling the sync word has been received
// Used to signal the start of data we want to receive

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  // Both GDO0 and GDO2 signal the same events, only set the flag if we're actually receiving something
  // This interrupt will fire often but there's not much we can do about that without lib changes
  if(rxInProgress)
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

  rxInProgress = true;
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

    // Clear the transmitted flag before starting new transmission
    transmittedFlag = false;
    
    // Stop receiving before transmitting
    rxInProgress = false;
    radio.finishReceive();

    encodeForTransmission(outdata, deviceID, SET, state);

    Serial.print("Transmitting packet: ");
    for(int i=0; i<12; i++) {
      Serial.print(outdata[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // flag that we're attempting a transmit so the ISR knows to accept the
    // GDO2 interrupt as a TX completion event
    txInProgress = true;
    transmissionState = radio.startTransmit(outdata, 12);
    if(transmissionState != RADIOLIB_ERR_NONE) {
      // startTransmit failed; clear the transmit-in-progress flag so
      // that stray GDO2 events aren't treated as successful transmits
      txInProgress = false;
      Serial.print("startTransmit failed, code ");
      Serial.println(transmissionState);
    }
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
    
    // clear the transmit-in-progress flag now tx completed
    txInProgress = false;


    // go back to receiving
    rxInProgress = true;
    radio.startReceive();
  }

  if(receivedFlag) {
    rxInProgress = false;
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
        unshiftTransmission(dataBuf, byteArr);
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
    rxInProgress = true;
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
