#include <Arduino.h>
#include <RadioLib.h>
#include <AceCRC.h>
#include "kinetic_helpers.h"

using namespace ace_crc::crc16ccitt_byte;

// Print aggregated device ID + packet count stats
 #define STATS_PRINT

// Print the decoded message to the serial terminal
// #define DECODED_DEBUG

// Print the raw hex message to the terminal
// #define RAW_DEBUG

// Disable filtering decoded message for known device types
#define ALLOW_UNKNOWN_DEV

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

// Device tracking settings
#define MAX_TRACKED_DEVICES 64
#define REPORT_INTERVAL_MS 10000 // Print stats every 10 seconds

// CC1101 module
CC1101 radio = new Module(CS_PIN, GDO0_PIN, RADIOLIB_NC, GDO2_PIN);

// Sync word: only 2 bytes supported
const uint8_t syncWord[2] = {0x10, 0xd2};

// Device tracking structure
struct TrackedDevice {
  uint32_t deviceID;
  uint32_t packetCount;
};

// Array to store tracked devices
TrackedDevice trackedDevices[MAX_TRACKED_DEVICES];
uint8_t numTrackedDevices = 0;

// Timing for periodic reports
unsigned long lastReportTime = 0;

// RX interrupt handling
volatile bool receivedFlag = false;
volatile bool rxInProgress = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  if(rxInProgress)
    receivedFlag = true;
}

// Function to find or add a device to the tracking list
void trackDevice(uint32_t deviceID) {
  // Search for existing device
  for(uint8_t i = 0; i < numTrackedDevices; i++) {
    if(trackedDevices[i].deviceID == deviceID) {
      trackedDevices[i].packetCount++;
      return;
    }
  }
  
  // Device not found, add it if there's space
  if(numTrackedDevices < MAX_TRACKED_DEVICES) {
    trackedDevices[numTrackedDevices].deviceID = deviceID;
    trackedDevices[numTrackedDevices].packetCount = 1;
    numTrackedDevices++;
  } else {
    Serial.println("WARNING: Tracked device list full!");
  }
}

// Function to print all tracked devices and their packet counts
void printDeviceStats(void) {
  Serial.print("Total unique devices: ");
  Serial.println(numTrackedDevices);
  Serial.println();
  
  for(uint8_t i = 0; i < numTrackedDevices; i++) {
    Serial.print("Device, ");
    Serial.print(i);
    Serial.print(", ");
    Serial.print(trackedDevices[i].deviceID, HEX);
    Serial.print(", Packets, ");
    Serial.println(trackedDevices[i].packetCount);
  }
  
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  int state = radio.begin(FREQUENCY, BITRATE, FDEV, RXBW, OUTPUT_POWER, PREAMBLE_LENGTH);
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("Radio init failed, code ");
    Serial.println(state);
    while(true);
  }

  radio.setCrcFiltering(false);
  radio.fixedPacketLengthMode(PACKET_LENGTH);
  radio.setPacketReceivedAction(setRxFlag);
  radio.setSyncWord(syncWord, 2);

  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Radio ready!");
  } else {
    Serial.print("Failed to start receive, code ");
    Serial.println(state);
    while (true);
  }

  rxInProgress = true;
  lastReportTime = millis();
}

void loop() {
  // Check if it's time to print statistics
  #ifdef STATS_PRINT
  if(millis() - lastReportTime >= REPORT_INTERVAL_MS) {
    printDeviceStats();
    lastReportTime = millis();
  }
  #endif

  if(receivedFlag) {
    rxInProgress = false;
    byte byteArr[PACKET_LENGTH];
    int state = radio.readData(byteArr, PACKET_LENGTH);
    
    if (state == RADIOLIB_ERR_NONE) {
      // Only process the message if RSSI is high enough
      if (radio.getRSSI() > MIN_RSSI) {
        uint8_t dataBuf[10];
        unshiftTransmission(dataBuf, byteArr);

        #ifdef RAW_DEBUG
        char data[41] = "";
        bytesToHexString(dataBuf, 10, data);
        Serial.println(data);
        #endif

        
        // Decode the kinetic message
        kineticMessage msg = decodeTransmission(dataBuf);
        
        if(
          #ifndef ALLOW_UNKNOWN_DEV
          msg.devType != UNKNOWN
          #else
          true
          #endif
          ) {
          // Track this device
          trackDevice(msg.deviceID);
          #ifdef DECODED_DEBUG
          Serial.print("ID, ");
          Serial.print(msg.deviceID, HEX);
          Serial.print(" , Type, ");
          
          switch(msg.devType) {
            case SWITCH_ONLY:
              Serial.print("Switch");
              break;
            case SWITCH_RELAY:
              Serial.print("Switch+Relay");
              break;
            case RELAY:
              Serial.print("Relay");
              break;
            default:
              Serial.print("Unknown");
          }
          
          Serial.print(" , Msg, ");
          switch(msg.msgType) {
            case POLL:
              Serial.print("Poll");
              break;
            case STATE:
              Serial.print("State");
              break;
            case SET:
              Serial.print("Set");
              break;
            default:
              Serial.print("Unknown");
          }
          
          Serial.print(" , State, 0x");
          Serial.print(msg.state, HEX);
          Serial.print(" , RSSI (dBm), ");
          Serial.println(radio.getRSSI());

          #endif
        }
      }
    } else {
      Serial.print("RadioLib error: ");
      Serial.println(state);
    }

    // Reset flag and put module back into listening mode
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
