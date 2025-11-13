#include <Arduino.h>
#include <RadioLib.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <PubSubClient.h>
#include <AceCRC.h>

// Select the type of CRC algorithm we'll be using
using namespace ace_crc::crc16ccitt_byte;

// Radio Config
#define PACKET_LENGTH 12 // bytes
#define MIN_RSSI -85 // dBm
#define CARRIER_FREQUENCY 433.3 // MHz
#define BIT_RATE 100.0 // kbps
#define FREQUENCY_DEVIATION 50.0 // kHz
#define RX_BANDWIDTH 135.0 // kHz
#define OUTPUT_POWER 10 // dBm
#define PREAMBLE_LENGTH 16 // bits

// IotWebConf Config
#define CONFIG_PARAM_MAX_LEN 128
#define CONFIG_VERSION "mqt4"
#define CONFIG_PIN 4 // D2
#define STATUS_PIN 16 // D0
#define MAX_DEVICE_IDS 32
#define DEVICE_ID_LENGTH 9  // 8 hex chars + null terminator

// Kinetic2MQTT Config
#define DEBOUNCE_MILLIS 15

// CC1101 has the following connections:
// CS pin:    15
// GDO0 pin:  5
// RST pin:   unused
// GDO2 pin:  unused (optional)
CC1101 radio = new Module(15, 5, RADIOLIB_NC, RADIOLIB_NC);

// Set up IotWebConf
const char deviceName[] = "kinetic2mqtt";
const char apPassword[] = "EMWhP56Q"; // Default password for SSID and configuration page, can be changed after first boot

void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

// Storage for device IDs
char deviceIDValues[MAX_DEVICE_IDS][DEVICE_ID_LENGTH] = {0};

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
PubSubClient mqttClient(net);

char mqttServerValue[CONFIG_PARAM_MAX_LEN];
char mqttUserNameValue[CONFIG_PARAM_MAX_LEN];
char mqttUserPasswordValue[CONFIG_PARAM_MAX_LEN];
char mqttTopicValue[CONFIG_PARAM_MAX_LEN];

IotWebConf iotWebConf(deviceName, &dnsServer, &server, apPassword, CONFIG_VERSION);
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfParameterGroup deviceIdGroup = IotWebConfParameterGroup("deviceids", "Receiver Device IDs");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, CONFIG_PARAM_MAX_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttTopicParam = IotWebConfTextParameter("MQTT Topic", "mqttTopic", mqttTopicValue, CONFIG_PARAM_MAX_LEN);

bool needReset = false;

// Create device ID parameters array
IotWebConfTextParameter* deviceIDParams[MAX_DEVICE_IDS];

// parametrically initialise device ID parameters
void initializeDeviceIDParams() {
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    char* paramId = new char[16];
    char* paramLabel = new char[32];
    
    sprintf(paramId, "devId%d", i);
    sprintf(paramLabel, "Device ID %d", i + 1);
    
    deviceIDParams[i] = new IotWebConfTextParameter(
      paramLabel,
      paramId,
      deviceIDValues[i],
      DEVICE_ID_LENGTH
    );
  }
}

// Global State Variables
char lastSentSwitchID[5] = "";
char lastSentButtonAction[8] = "";
unsigned long lastSentMillis = 0;
int last_polled_index;


void setup() {
  Serial.begin(115200);
  pinMode(CONFIG_PIN, INPUT_PULLUP);

  // Initialise and configure CC1101
  Serial.print("Initializing Radio... ");

  int state = radio.begin(CARRIER_FREQUENCY, BIT_RATE, FREQUENCY_DEVIATION, RX_BANDWIDTH, OUTPUT_POWER, PREAMBLE_LENGTH);
  radio.setCrcFiltering(false);
  radio.fixedPacketLengthMode(PACKET_LENGTH);

  radio.setPreambleLength(PREAMBLE_LENGTH, 0);


  // Sync word: only 2 bytes supported, bitshift 0x21a4 to the right so we can add the extra 0 in
  uint8_t syncWord[] = {0x10, 0xd2};
  radio.setSyncWord(syncWord, 2);
  radio.setGdo0Action(setRxFlag, RISING);

  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("done!");
  } else {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true);
  }

  // Initialise IOTWebConf

  Serial.println("Initialising IotWebConf... ");
  
  // Initialize device ID parameters
  initializeDeviceIDParams();
  
  // Add all device ID parameters to the group
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    deviceIdGroup.addItem(deviceIDParams[i]);
  }
  
  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttTopicParam);

  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.addParameterGroup(&deviceIdGroup);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);

  bool validConfig = iotWebConf.init();
  if (!validConfig) {
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
    mqttTopicValue[0] = '\0';
  }

  // This will disable the device sitting in AP mode for 30s on startup
  // Not requred due to presence of reset button to manually enable AP mode
  iotWebConf.setApTimeoutMs(0);

  server.on("/", []{ 
    // Build custom form with chained device ID inputs
    String html = "";
    html += iotWebConf.getThingName();
    html += " - Configuration\n";
    
    // Let IotWebConf handle the base config
    iotWebConf.handleConfig(); 
  });
  server.onNotFound([](){ iotWebConf.handleNotFound(); });

  // Initialise MQTT
  mqttClient.setServer(mqttServerValue, 1883);
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setRxFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void loop() {
  iotWebConf.doLoop();

  if (needReset) {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  // If WiFi is connected but MQTT is not, establish MQTT connection
 // if ((iotWebConf.getState() == iotwebconf::OnLine) && !mqttClient.connected()) {
 //   connectMqtt();
 // }
 // mqttClient.loop();

  // We want to poll every switch once per minute
  // Run a transmit loop every 60/number of devices seconds and iterate through each device
  // We collect device changed events anyway so this is arguably unnecessary
  if(!millis()%60000){
    // Poll all known kinetic switches
  }

  // If a message has been received and flag has been set by interrupt, process the message
  if(receivedFlag) {
    byte byteArr[PACKET_LENGTH];
    int state = radio.readData(byteArr, PACKET_LENGTH);
    
    if (state == RADIOLIB_ERR_NONE) {
      // Only process the message if RSSI is high enough, this prevents uncessary calculating CRC for noise
      if (radio.getRSSI() > MIN_RSSI) {
        // Last 2 bytes are the CRC-16/AUG-CCITT of the first 3 bytes
        // Verify CRC and only continue if valid

        // Does calculted CRC match the CRC in the message?
        if (true) {
        
          // Do not send a message if this received message is the same as the previous one and the previous one was sent less than DEBOUNCE_MILLIS miliseconds ago
          // This is because Kinetic switches continuously send messages every milisecond or so until the power runs out, this logic deduplicates these
          if (!((strcmp(switchID, lastSentSwitchID) == 0) && (strcmp(buttonAction, lastSentButtonAction) == 0) && ((millis() - lastSentMillis) < DEBOUNCE_MILLIS))) {
            Serial.print("Button pressed: ");
            Serial.print(switchID);
            Serial.print(", action: ");
            Serial.print(buttonAction);
            Serial.print(", value: ");
            Serial.println(buttonState);

            //publishFullMessage(switchID, buttonState, rssiString);  // usertopic/switchid/state 
            
            strcpy(lastSentButtonAction, buttonAction);
            strcpy(lastSentSwitchID, switchID);
            lastSentMillis = millis();
          }

          char data[49] = "";
          bytesToHexString(byteArr, 11, data);
          Serial.println(data);
        } else {
          // Message CRC was invalid
          Serial.println("Error: CRC Mismatch!");
        }
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

// Publish [value] to MQTT at topic: [mqttTopicValue]/[topicLevel1]/[topicLevel2]
void publishMqtt(char value[]) {
  char compiledTopic[strlen(mqttTopicValue) + 7] = "";
  strcat(compiledTopic, mqttTopicValue);
  strcat(compiledTopic, "/system");
  
  mqttClient.publish(compiledTopic, value);
}

void publishFullMessage(char switchIdent[], char value[], char rssi[]) {
  char compiledTopic[strlen(mqttTopicValue) + strlen(switchIdent) + 9] = "";
  strcat(compiledTopic, mqttTopicValue);
  strcat(compiledTopic, "/state/");
  strcat(compiledTopic, switchIdent);

  char compiledValue[28 + strlen(value) + strlen(rssi)] = "";
  strcat(compiledValue,"{\"rssi\":\"");
  strcat(compiledValue, rssi);
  strcat(compiledValue,"\",\"buttonstate\":\"");
  strcat(compiledValue, value);
  strcat(compiledValue,"\"}");

  mqttClient.publish(compiledTopic, compiledValue);
}

// Establish an MQTT connection, if connection fails this function will delay for 5 seconds and then return
// Connection will be re-attempted at next execution of loop()
void connectMqtt() {
  // Loop until we're reconnected
  Serial.println("Attempting MQTT connection...");

  // Create a random client ID
  String clientId = iotWebConf.getThingName();
  clientId += "-";
  clientId += String(random(0xffff), HEX);

  Serial.println(clientId);

  // Attempt to connect
  if (connectMqttOptions()) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    char topicLevel1[] = "status";
    char topicLevel2[] = "connection";
    char messageValue[] = "connected";
    publishMqtt(messageValue);
  } else {
    Serial.print("MQTT Connection Failed:");
    Serial.print(mqttClient.state());
    Serial.println(".  Trying again in 5 seconds");
    // Wait 5 seconds before retrying
    iotWebConf.delay(5000);
  }
}

bool connectMqttOptions()
{
  bool result;
  if (mqttUserPasswordValue[0] != '\0')
  {
    String userName = mqttUserNameValue;
    String password = mqttUserPasswordValue;
    result = mqttClient.connect(iotWebConf.getThingName(), userName.c_str(), password.c_str());
  }
  else
  {
    result = mqttClient.connect(iotWebConf.getThingName());
  }
  return result;
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

// If configuration is saved in IOTWebConf, reboot the device
void configSaved() {
  Serial.println("Configuration was updated.");
  needReset = true;
}

// Validate the data entered into the IOTWebConf configuration page
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper) {
  Serial.println("Validating form.");
  bool valid = true;

  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3) {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  l = webRequestWrapper->arg(mqttTopicParam.getId()).length();
  if (l < 3) {
    mqttTopicParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  // Validate Device IDs
  // Check that IDs are in proper format (8 hex characters) and there are no empty IDs in between filled ones
  int lastFilledIndex = -1;
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    String devIdStr = webRequestWrapper->arg(deviceIDParams[i]->getId());
    
    if (devIdStr.length() > 0) {
      // ID is filled - validate format
      if (devIdStr.length() != 8) {
        deviceIDParams[i]->errorMessage = "Must be exactly 8 hex characters (32-bit)!";
        valid = false;
      } else {
        // Check all characters are valid hex
        bool isValidHex = true;
        for (char c : devIdStr) {
          if (!isxdigit(c)) {
            isValidHex = false;
            break;
          }
        }
        if (!isValidHex) {
          deviceIDParams[i]->errorMessage = "Must be valid hexadecimal (0-9, A-F)!";
          valid = false;
        }
      }
      
      // Check chaining: previous ID must be filled if current ID is filled
      if (i > 0 && lastFilledIndex != i - 1) {
        deviceIDParams[i]->errorMessage = "Previous Device ID must be filled first!";
        valid = false;
      }
      
      lastFilledIndex = i;
    }
  }

  return valid;
}

// Get the number of filled device IDs (respects chaining)
int getFilledDeviceIDCount() {
  int count = 0;
  for (int i = 0; i < MAX_DEVICE_IDS; i++) {
    if (strlen(deviceIDValues[i]) > 0) {
      count++;
    } else {
      break;  // Chain breaks here
    }
  }
  return count;
}

// Get device ID at index as a uint32_t, or 0 if index is out of range or ID is not filled in the web form
const uint32_t getDeviceID(int index) {
  if (index >= 0 && index < MAX_DEVICE_IDS && strlen(deviceIDValues[index]) > 0) {
    return strtol(deviceIDValues[index], 0, 16);
  }
  return 0;
}
