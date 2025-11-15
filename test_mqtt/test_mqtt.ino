#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <PubSubClient.h>

// Set up IotWebConf
const char deviceName[] = "kinetic2mqtt";
const char apPassword[] = "EMWhP56Q"; // Default password for SSID and configuration page, can be changed after first boot

// IotWebConf Config
#define CONFIG_PARAM_MAX_LEN 128
#define CONFIG_VERSION "mq2"
#define CONFIG_PIN 2 // D4
#define STATUS_PIN 16 // D0

void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

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
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, CONFIG_PARAM_MAX_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttTopicParam = IotWebConfTextParameter("MQTT Topic", "mqttTopic", mqttTopicValue, CONFIG_PARAM_MAX_LEN);

bool needReset = false;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}

void setup() {
  Serial.begin(115200);


  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttTopicParam);

  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameterGroup(&mqttGroup);
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
  mqttClient.setCallback(callback);
  mqttClient.setServer(mqttServerValue, 1883);
}

void loop() {
  // put your main code here, to run repeatedly:
  iotWebConf.doLoop();

  if (needReset) {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  // If WiFi is connected but MQTT is not, establish MQTT connection
  if ((iotWebConf.getState() == iotwebconf::OnLine) && !mqttClient.connected()) {
    Serial.println("connecting to MQTT");
    connectMqtt();
  }
  mqttClient.loop();

  if(Serial.available()) {
    String input = Serial.readString();
    // Parse string for device ID and chosen state
    String deviceIDStr = input.substring(0, 8);

    char deviceID[9];
    deviceIDStr.toCharArray(deviceID, 9);
    char state[2];
    input.substring(9, 10).toCharArray(state, 2);
    char rssi[] = "-30";

    publishFullMessage(deviceID, state, rssi);
  }
}

// Publish [value] to MQTT at topic: [mqttTopicValue]/system
void publishMqtt(char value[]) {
  char compiledTopic[strlen(mqttTopicValue) + 7] = "";
  strcat(compiledTopic, mqttTopicValue);
  strcat(compiledTopic, "/system");
  
  mqttClient.publish(compiledTopic, value);
}

void publishFullMessage(char switchIdent[], char value[], char rssi[]) {
  char compiledValue[32 + strlen(value) + strlen(rssi)] = "";
  strcat(compiledValue,"{\"rssi\":\"");
  strcat(compiledValue, rssi);
  strcat(compiledValue,"\",\"state\":\"");
  strcat(compiledValue, value);
  strcat(compiledValue,"\"}"); 
 
 
  char compiledTopic[strlen(mqttTopicValue) + strlen(switchIdent) + 9] = "";
  strcat(compiledTopic, mqttTopicValue);
  strcat(compiledTopic, "/state/");
  strcat(compiledTopic, switchIdent);

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

  // subscribe to our configuration topic
  char compiledTopic[strlen(mqttTopicValue) + 4] = "";
  strcat(compiledTopic, mqttTopicValue);
  strcat(compiledTopic, "/set");

  // Attempt to connect
  if (connectMqttOptions()) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    char topicLevel1[] = "status";
    char topicLevel2[] = "connection";
    char messageValue[] = "connected";
    publishMqtt(messageValue);
    Serial.print("Subscribing to: ");
    Serial.println(compiledTopic);
    mqttClient.subscribe(compiledTopic);
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

  return valid;
}
