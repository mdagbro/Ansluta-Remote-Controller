# MQTT JSON Light - Home Assistant
A simple example to control a set of IKEA ANSLUTA led strips with a NodeMCU board (ESP8266) and a CC2500 radio

NOTE: UNTESTED as of 2024-11-09 - only pushed for safekeeping

## Features
- MQTT discovery
- MQTT JSON Light
  - State
  - Brightness

## How to
1. Rename `config.example.h`to `config.h`
2. Define your WiFi SSID and password
3. Define your MQTT settings
4. Install the external libraries (PubSubClient, ArduinoJson)
4. Define `MQTT_MAX_PACKET_SIZE` to `256`in `Arduino/libraries/pubsubclient/src/PubSubClient.h`
