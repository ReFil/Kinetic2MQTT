# Kinetic2MQTT

A bidirectional MQTT gateway for Quinetic wireless switches, relays, and smart plugs. This project enables Home Assistant and other MQTT-based systems to control Quinetic devices while maintaining their standalone operation. This may work for other kinetic switches, but this has only been tested with the Quinetic brand (available here: https://www.tlc-direct.co.uk/Main_Index/Quinetic/index.html)

## Overview

This is an Arduino sketch for ESP8266 microcontrollers connected to a CC1101 433MHz radio module. It bridges Quinetic wireless devices to MQTT, allowing remote monitoring and control via Home Assistant or similar platforms.

This work is based off the excellent prior work of Cameron Gray, who first got the switches being received by an ESP: https://github.com/camerongray1515/Kinetic2MQTT and Secfault Security, who reverse engineered the fundamental radio protocol: https://secfault-security.com/blog/kineticswitches.html. My developments on top would not have been possible without their original developments on this.

For technical details on how the relay and switch protocol was reverse-engineered, see the blog article at https://willowherring.dev/blogs/blog-17.html

## Hardware Setup

The CC1101 module should be connected to the ESP8266 via SPI with the following pinout:
- CS pin: 15
- GDO0 pin: 5 (RX interrupt)
- GDO2 pin: 4 (TX interrupt)

I used a D1 mini ESP8266 and a generic CC1101 module, you don't need anything fancy for this

## Configuration

This project uses IotWebConf to provide a setup and configuration interface.  When first powered on and for 30 seconds on every subsequent power on, the ESP will broadcast a WiFi network with an SSID of "kinetic2mqtt" - connnect to this using the password in the sketch and navigate to 192.168.4.1 or neverssl.com in a web browser (Most devices will automatically redirect you to the portal anyway).  You will then be presented with a web interface allowing you to set a management password, WiFi connection details and MQTT broker settings. Please note changing the stock AP password is mandatory as a security measure.

Device IDs can be found on the physical device labels (except on the smart plugs, which differ for some reason) or discovered by running the `receiver.ino` sketch with `#define STATS_PRINT` enabled. After running this script press the buttons on the devices multiple times and their device IDs will be periodically printed to the serial console. N.B. this is only for devices with a controllable switching element, so the hub knows what devices to periodically poll, switch messages will always be published, and you can even send commands to device IDs not in the list and it will transmit them, and it will receive and publish messages from unknown device IDs, but not unknown device types. Also relays/smart plugs have device IDs that start with 00, and the receiver utility won't put those leading zeros on, but you should add them into the topics and ID table when configuring, I haven't checked to see what happens if you dont

## MQTT Messaging

### Topic Structure
```
{yourconfiguredtopic}/state/DEVICEID    ← Device state changes (published by receiver)
{yourconfiguredtopic}/set/DEVICEID      ← Commands to control devices (subscribe)
{yourconfiguredtopic}/system            ← System status (one message published when it connects)
```

### Message Format

**State updates** (received from devices):
```json
{
  "state": 1,           // 0 = off, 1 = on for relay devices, 4 for single channel switch pressed, 192 for switch released 
  "type": 1,            // 0 = POLL (you shouldn't see this unless you have the stock hub operational), 1 = STATE, 2 = SET
  "rssi": -65,          // Signal strength in dBm
  "device_type": 1      // 0 = SWITCH_ONLY, 1 = SWITCH_RELAY, 2 = RELAY
}
```

**Control commands** (sent to devices):
```json
{
  "state": 1            // 0 = off, 1 = on
}
```

## Home Assistant Integration

To control Quinetic devices from Home Assistant, add the following to your `configuration.yaml` for each relay/in line switch relay:

```yaml
mqtt:
  light:
    - name: "Light Name"
      unique_id: quinetic_DEVICEID
      command_topic: "{yourconfiguredtopic}/set/DEVICEID"
      state_topic: "{yourconfiguredtopic}/state/DEVICEID"
      schema: template
      command_on_template: "{{ {'state': 1} | tojson }}"
      command_off_template: "{{ {'state': 0} | tojson }}"
      state_template: "{% if value_json.state == 1 %}on{% else %}off{% endif %}"
```

Replace `{yourconfiguredtopic}` with your MQTT topic prefix and `DEVICEID` with the 8-character hex ID of your device.

The light entity will:
- Show the current state of the device (on/off)
- Update automatically when the device changes state (either physically via switch or remotely)
- Allow control from Home Assistant UI or automations

# Sketches

- **Kinetic2MQTT.ino**: Main bidirectional hub code
- **receiver/receiver.ino**: Device discovery utility - prints all devices detected with packet counts (Useful if you've already glued the switches into your walls), can also print raw packet data if you're trying to reverse engineer a new type of device. If you do please PR it in!

### Debugging sketches

- **simultaneous_txrx/simultaneous_txrx.ino**: TX/RX testing sketch with manual serial control
- **test_mqtt/test_mqtt.ino**: MQTT connectivity and JSON messaging test
- **test_sending/test_sending.ino**: Radio transmission testing

## Limitations

This project is still very much a proof of concept with very little testing and documentation.  If you wish to use this, you
should be familar with Arduino and ESP8266 development to be able to resolve any issues that occur.

Also bear in mind the following:
* This has only been tested with the following single-gang switches: Quinetic QU WS1S, Quinetic QU WS1W, Quinetic QU GDMK.  Other switches may or 
may not work.
* This has also been tested with the following two-gang switches : Quinetic QUDGMK R.
* Relays and other devices confirmed to work are: Quinetic QU RS1W, Quinetic QU R303/R305, Quinetic QU A313
* The outputs of the switches differ on multi channel switches, with one of the last 6 bits of the signal byte representing which physical 
button on the device is triggered. 

* The state for releasing a button on any tested device does not indicate which button is released.  You're resonsible for 
maintaining which button is in a presed state if you need that functionality.

## Requirements

The following libraries are used by Kinetic2MQTT.  They should be installed in your Arduino development environment prior to building:
* IotWebConf version 3.2.1: https://github.com/prampec/IotWebConf
* PubSubClient version 2.8: https://github.com/knolleary/pubsubclient/
* RadioLib version 7.4.0: https://github.com/jgromes/RadioLib
* AceCRC version 1.1.1: https://github.com/bxparks/AceCRC
* ArduinoJson version 6.x: https://github.com/bblanchon/ArduinoJson

## Debug Flag
The main Kinetic2MQTT.ino sketch has a flag to enable printing of debug messages, which is useful when you're having issues making it work. Simply uncomment the `#define DEBUG_SERIAL` at the top of the file and lots of messaging gets printed to the serial console