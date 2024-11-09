/*
  MQTT Discovery and MQTT JSON Light for Home Assistant
  
  Samuel Mertenat
  04.2017
  ORIGINAL SOURCE: 
  https://github.com/SamZorSec/Open-Home-Automation/blob/master/ha_mqtt_rgbw_light_with_discovery/ha_mqtt_rgbw_light_with_discovery.ino

  Adapted to IKEA Ansluta radio controlled lights by:
  Magnus Dagbro
  11.2024
*/

#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <PubSubClient.h>         // https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson
#include <ArduinoOTA.h>
#include "ha_mqtt_ansluta_light_with_discovery.h"

#include "../AnslutaDemoCode/cc2500_REG.h"
#include "../AnslutaDemoCode/cc2500_VAL.h"

// Radio stuff
#define CC2500_SIDLE    0x36      // Exit RX / TX
#define CC2500_STX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_SFTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_SRES     0x30      // Reset chip
#define CC2500_FIFO     0x3F      // TX and RX FIFO
#define CC2500_SRX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_SFRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states

// Light stuff
#define Light_OFF       0x01      // Command to turn the light off
#define Light_ON_50     0x02      // Command to turn the light on 50%
#define Light_ON_100    0x03      // Command to turn the light on 100%
#define Light_PAIR      0xFF      // Command to pair a remote to the light

const byte delayA = 1;  //1++ 0-- No delay is also possible
const unsigned int delayB = 2000; // 10000-- 20000++ 15000++     //KRITISCH
const byte delayC = 10;  //255++ 128++ 64--
const byte delayD = 0;  //200++ 128+++ 64+++ 32+++ 8++
const byte delayE = 200;

const boolean DEBUG = true;   //Some simple communication by RS232

// fixed address of ANSLUTA remote
static const byte baseAddress = 0x12;

// Number of different remote channels
static const int CHANNEL_NUM = 5;
// Brightness thresholds / offsets for the different channels.
// each channel will turn ON 50% when brightness + channelThreshold > 127
static const int[] CHANNEL_THRESHOLD = {85, 29, 57, 1, 113};

#if defined(DEBUG_TELNET)
WiFiServer  telnetServer(23);
WiFiClient  telnetClient;
#define     DEBUG_PRINT(x)    telnetClient.print(x)
#define     DEBUG_PRINTLN(x)  telnetClient.println(x)
#elif defined(DEBUG_SERIAL)
#define     DEBUG_PRINT(x)    Serial.print(x)
#define     DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define     DEBUG_PRINT(x)
#define     DEBUG_PRINTLN(x)
#endif

#if defined(MQTT_HOME_ASSISTANT_SUPPORT)
StaticJsonBuffer<256> staticJsonBuffer;
char jsonBuffer[256] = {0};
#endif

volatile uint8_t cmd = CMD_NOT_DEFINED;

WiFiClient    wifiClient;
PubSubClient  mqttClient(wifiClient);

///////////////////////////////////////////////////////////////////////////
//   TELNET
///////////////////////////////////////////////////////////////////////////
/*
   Function called to handle Telnet clients
   https://www.youtube.com/watch?v=j9yW10OcahI
*/
#if defined(DEBUG_TELNET)
void handleTelnet(void) {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) {
        telnetClient.stop();
      }
      telnetClient = telnetServer.available();
    } else {
      telnetServer.available().stop();
    }
  }
}
#endif

///////////////////////////////////////////////////////////////////////////
//   WiFi
///////////////////////////////////////////////////////////////////////////
/*
   Function called to handle WiFi events
*/
void handleWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case WIFI_EVENT_STAMODE_GOT_IP:
      DEBUG_PRINTLN(F("INFO: WiFi connected"));
      DEBUG_PRINT(F("INFO: IP address: "));
      DEBUG_PRINTLN(WiFi.localIP());
      break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      DEBUG_PRINTLN(F("ERROR: WiFi losts connection"));
      /*
         TODO: Do something smarter than rebooting the device
      */
      delay(5000);
      ESP.restart();
      break;
    default:
      DEBUG_PRINT(F("INFO: WiFi event: "));
      DEBUG_PRINTLN(event);
      break;
  }
}

/*
   Function called to setup the connection to the WiFi AP
*/
void setupWiFi() {
  DEBUG_PRINT(F("INFO: WiFi connecting to: "));
  DEBUG_PRINTLN(WIFI_SSID);

  delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(handleWiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  randomSeed(micros());
}

///////////////////////////////////////////////////////////////////////////
//   OTA
///////////////////////////////////////////////////////////////////////////
#if defined(OTA)
/*
   Function called to setup OTA updates
*/
void setupOTA() {
#if defined(OTA_HOSTNAME)
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  DEBUG_PRINT(F("INFO: OTA hostname sets to: "));
  DEBUG_PRINTLN(OTA_HOSTNAME);
#endif

#if defined(OTA_PORT)
  ArduinoOTA.setPort(OTA_PORT);
  DEBUG_PRINT(F("INFO: OTA port sets to: "));
  DEBUG_PRINTLN(OTA_PORT);
#endif

#if defined(OTA_PASSWORD)
  ArduinoOTA.setPassword((const char *)OTA_PASSWORD);
  DEBUG_PRINT(F("INFO: OTA password sets to: "));
  DEBUG_PRINTLN(OTA_PASSWORD);
#endif

  ArduinoOTA.onStart([]() {
    DEBUG_PRINTLN(F("INFO: OTA starts"));
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_PRINTLN(F("INFO: OTA ends"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINT(F("INFO: OTA progresses: "));
    DEBUG_PRINT(progress / (total / 100));
    DEBUG_PRINTLN(F("%"));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINT(F("ERROR: OTA error: "));
    DEBUG_PRINTLN(error);
    if (error == OTA_AUTH_ERROR)
      DEBUG_PRINTLN(F("ERROR: OTA auth failed"));
    else if (error == OTA_BEGIN_ERROR)
      DEBUG_PRINTLN(F("ERROR: OTA begin failed"));
    else if (error == OTA_CONNECT_ERROR)
      DEBUG_PRINTLN(F("ERROR: OTA connect failed"));
    else if (error == OTA_RECEIVE_ERROR)
      DEBUG_PRINTLN(F("ERROR: OTA receive failed"));
    else if (error == OTA_END_ERROR)
      DEBUG_PRINTLN(F("ERROR: OTA end failed"));
  });
  ArduinoOTA.begin();
}

/*
   Function called to handle OTA updates
*/
void handleOTA() {
  ArduinoOTA.handle();
}
#endif

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////

char MQTT_CLIENT_ID[7] = {0};
#if defined(MQTT_HOME_ASSISTANT_SUPPORT)
char MQTT_CONFIG_TOPIC[sizeof(MQTT_HOME_ASSISTANT_DISCOVERY_PREFIX) + sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_CONFIG_TOPIC_TEMPLATE) - 4] = {0};
#else

#endif

char MQTT_STATE_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_STATE_TOPIC_TEMPLATE) - 2] = {0};
char MQTT_COMMAND_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_COMMAND_TOPIC_TEMPLATE) - 2] = {0};
char MQTT_STATUS_TOPIC[sizeof(MQTT_CLIENT_ID) + sizeof(MQTT_STATUS_TOPIC_TEMPLATE) - 2] = {0};

volatile unsigned long lastMQTTConnection = MQTT_CONNECTION_TIMEOUT;
/*
   Function called when a MQTT message has arrived
   @param p_topic   The topic of the MQTT message
   @param p_payload The payload of the MQTT message
   @param p_length  The length of the payload
*/
void handleMQTTMessage(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concatenates the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  DEBUG_PRINTLN(F("INFO: New MQTT message received"));
  DEBUG_PRINT(F("INFO: MQTT topic: "));
  DEBUG_PRINTLN(p_topic);
  DEBUG_PRINT(F("INFO: MQTT payload: "));
  DEBUG_PRINTLN(payload);
  
  if (String(MQTT_COMMAND_TOPIC).equals(p_topic)) {
    DynamicJsonBuffer dynamicJsonBuffer;
    JsonObject& root = dynamicJsonBuffer.parseObject(p_payload);
    if (!root.success()) {
      DEBUG_PRINTLN(F("ERROR: parseObject() failed"));
      return;
    }

    if (root.containsKey("state")) {
      if (strcmp(root["state"], MQTT_STATE_ON_PAYLOAD) == 0) {
        if (bulb.setState(true)) {
          DEBUG_PRINT(F("INFO: State changed to: "));
          DEBUG_PRINTLN(bulb.getState());
          cmd = CMD_STATE_CHANGED;
        }
      } else if (strcmp(root["state"], MQTT_STATE_OFF_PAYLOAD) == 0) {
        // stops the possible current effect
        bulb.setEffect(EFFECT_NOT_DEFINED_NAME);
        
        if (bulb.setState(false)) {
          DEBUG_PRINT(F("INFO: State changed to: "));
          DEBUG_PRINTLN(bulb.getState());
          cmd = CMD_STATE_CHANGED;
        }
      }
    }

    if (root.containsKey("brightness")) {
      if (bulb.setBrightness(root["brightness"])) {
        DEBUG_PRINT(F("INFO: Brightness changed to: "));
        DEBUG_PRINTLN(bulb.getBrightness());
        cmd = CMD_STATE_CHANGED;
      }
    }
  }
}

/*
  Function called to subscribe to a MQTT topic
*/
void subscribeToMQTT(char* p_topic) {
  if (mqttClient.subscribe(p_topic)) {
    DEBUG_PRINT(F("INFO: Sending the MQTT subscribe succeeded for topic: "));
    DEBUG_PRINTLN(p_topic);
  } else {
    DEBUG_PRINT(F("ERROR: Sending the MQTT subscribe failed for topic: "));
    DEBUG_PRINTLN(p_topic);
  }
}

/*
  Function called to publish to a MQTT topic with the given payload
*/
void publishToMQTT(char* p_topic, char* p_payload) {
  if (mqttClient.publish(p_topic, p_payload, true)) {
    DEBUG_PRINT(F("INFO: MQTT message published successfully, topic: "));
    DEBUG_PRINT(p_topic);
    DEBUG_PRINT(F(", payload: "));
    DEBUG_PRINTLN(p_payload);
  } else {
    DEBUG_PRINTLN(F("ERROR: MQTT message not published, either connection lost, or message too large. Topic: "));
    DEBUG_PRINT(p_topic);
    DEBUG_PRINT(F(" , payload: "));
    DEBUG_PRINTLN(p_payload);
  }
}

/*
  Function called to connect/reconnect to the MQTT broker
*/
void connectToMQTT() {
  if (!mqttClient.connected()) {
    if (lastMQTTConnection + MQTT_CONNECTION_TIMEOUT < millis()) {
      if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, 0, 1, "dead")) {
        DEBUG_PRINTLN(F("INFO: The client is successfully connected to the MQTT broker"));
        publishToMQTT(MQTT_STATUS_TOPIC, "alive");

#if defined(MQTT_HOME_ASSISTANT_SUPPORT)
        // MQTT discovery for Home Assistant
        JsonObject& root = staticJsonBuffer.createObject();
        root["name"] = FRIENDLY_NAME;
        root["platform"] = "mqtt_json";
        root["state_topic"] = MQTT_STATE_TOPIC;
        root["command_topic"] = MQTT_COMMAND_TOPIC;
        root["brightness"] = true;
        root.printTo(jsonBuffer, sizeof(jsonBuffer));
        publishToMQTT(MQTT_CONFIG_TOPIC, jsonBuffer);
#endif

        subscribeToMQTT(MQTT_COMMAND_TOPIC);
      } else {
        DEBUG_PRINTLN(F("ERROR: The connection to the MQTT broker failed"));
        DEBUG_PRINT(F("INFO: MQTT username: "));
        DEBUG_PRINTLN(MQTT_USERNAME);
        DEBUG_PRINT(F("INFO: MQTT password: "));
        DEBUG_PRINTLN(MQTT_PASSWORD);
        DEBUG_PRINT(F("INFO: MQTT broker: "));
        DEBUG_PRINTLN(MQTT_SERVER);
      }
        lastMQTTConnection = millis();
    }
  }
}

///////////////////////////////////////////////////////////////////////////
//  CMD
///////////////////////////////////////////////////////////////////////////

void handleCMD() {
  switch(cmd) {
    case CMD_NOT_DEFINED:
      break;
    case CMD_STATE_CHANGED:
      cmd = CMD_NOT_DEFINED;
      DynamicJsonBuffer dynamicJsonBuffer;
      JsonObject& root = dynamicJsonBuffer.createObject();
      root["state"] = bulb.getState() ? MQTT_STATE_ON_PAYLOAD : MQTT_STATE_OFF_PAYLOAD;
      root["brightness"] = bulb.getBrightness();
      root.printTo(jsonBuffer, sizeof(jsonBuffer));
      publishToMQTT(MQTT_STATE_TOPIC, jsonBuffer);
      break;
  }
}

///////////////////////////////////////////////////////////////////////////
// Set brightness level
///////////////////////////////////////////////////////////////////////////

void setBrightness(int brightness){
  byte lightLevel = Light_OFF;
  for (int i = 0; i < CHANNEL_NUM; i++) {
    lightLevel = Light_OFF;
    if (brightness >= CHANNEL_THRESHOLD[i]) {
      lightLevel = Light_ON_50
    }
    if (brightness >= CHANNEL_THRESHOLD[i] + 140) {
      lightLevel = Light_ON_100
    }
    SendCommand(baseAddress, i, lightLevel);
  }
}



///////////////////////////////////////////////////////////////////////////
// Radio functions
///////////////////////////////////////////////////////////////////////////

byte ReadReg(byte addr){
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  byte x = SPI.transfer(addr);
  delay(10);
  byte y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

void SendStrobe(byte strobe){
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delayMicroseconds(delayB);
}

void SendCommand(byte AddressByteA, byte AddressByteB, byte Command){
    if(DEBUG){
      Serial.print("Send command ");
      Serial.print(Command,HEX);
      Serial.print(" to ");
      if(AddressByteA<0x10){Serial.print("0");}  //Print leading zero
      Serial.print(AddressByteA,HEX);
      if(AddressByteB<0x10){Serial.print("0");}
      Serial.print(AddressByteB,HEX);
    }
    for(byte i=0;i<50;i++){       //Send 50 times
      Serial.print(".");
      SendStrobe(CC2500_SIDLE);   //0x36 SIDLE Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
      SendStrobe(CC2500_SFTX);    //0x3B SFTX Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
      digitalWrite(SS,LOW);
      while (digitalRead(MISO) == HIGH) { /*TODO: timeout*/ };  //Wait untill MISO high
      SPI.transfer(0x7F);
      delayMicroseconds(delayA);
      SPI.transfer(0x06);
      delayMicroseconds(delayA);
      SPI.transfer(0x55);
      delayMicroseconds(delayA);
      SPI.transfer(0x01);                 
      delayMicroseconds(delayA);
      SPI.transfer(AddressByteA);                 //Address Byte A
      delayMicroseconds(delayA);
      SPI.transfer(AddressByteB);                 //Address Byte B
      delayMicroseconds(delayA);
      SPI.transfer(Command);                      //Command 0x01=Light OFF 0x02=50% 0x03=100% 0xFF=Pairing
      delayMicroseconds(delayA);
      SPI.transfer(0xAA);
      delayMicroseconds(delayA);
      SPI.transfer(0xFF);
      digitalWrite(SS,HIGH);
      SendStrobe(CC2500_STX);                 //0x35 STX In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear
      delayMicroseconds(delayC);      //Longer delay for transmitting
    }
    if(DEBUG){
      Serial.println(" - Done");
    }
}


void WriteReg(byte addr, byte value){
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  SPI.transfer(addr);
  delayMicroseconds(delayE);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
  delayMicroseconds(delayD);
}


void init_CC2500(){
  WriteReg(REG_IOCFG2,VAL_IOCFG2);
  WriteReg(REG_IOCFG0,VAL_IOCFG0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);
  WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
  WriteReg(REG_MDMCFG4,VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3,VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2,VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1,VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0,VAL_MDMCFG0);
  WriteReg(REG_DEVIATN,VAL_DEVIATN);
  WriteReg(REG_MCSM2,VAL_MCSM2);
  WriteReg(REG_MCSM1,VAL_MCSM1);
  WriteReg(REG_MCSM0,VAL_MCSM0);
  WriteReg(REG_FOCCFG,VAL_FOCCFG);
  WriteReg(REG_BSCFG,VAL_BSCFG);
  WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);
  WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,VAL_WORCTRL);
  WriteReg(REG_FREND1,VAL_FREND1);
  WriteReg(REG_FREND0,VAL_FREND0);
  WriteReg(REG_FSCAL3,VAL_FSCAL3);
  WriteReg(REG_FSCAL2,VAL_FSCAL2);
  WriteReg(REG_FSCAL1,VAL_FSCAL1);
  WriteReg(REG_FSCAL0,VAL_FSCAL0);
  WriteReg(REG_RCCTRL1,VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0,VAL_RCCTRL0);
  WriteReg(REG_FSTEST,VAL_FSTEST);
  WriteReg(REG_TEST2,VAL_TEST2);
  WriteReg(REG_TEST1,VAL_TEST1);
  WriteReg(REG_TEST0,VAL_TEST0);
  WriteReg(REG_DAFUQ,VAL_DAFUQ);
}


///////////////////////////////////////////////////////////////////////////
//  SETUP() AND LOOP()
///////////////////////////////////////////////////////////////////////////

void setup() {
#if defined(DEBUG_SERIAL)
  Serial.begin(115200);
#elif defined(DEBUG_TELNET)
  telnetServer.begin();
  telnetServer.setNoDelay(true);
#endif

  setupWiFi();

  sprintf(MQTT_CLIENT_ID, "%06X", ESP.getChipId());
#if defined(MQTT_HOME_ASSISTANT_SUPPORT)
  sprintf(MQTT_CONFIG_TOPIC, MQTT_CONFIG_TOPIC_TEMPLATE, MQTT_HOME_ASSISTANT_DISCOVERY_PREFIX, MQTT_CLIENT_ID);
  DEBUG_PRINT(F("INFO: MQTT config topic: "));
  DEBUG_PRINTLN(MQTT_CONFIG_TOPIC);
#else

#endif

  sprintf(MQTT_STATE_TOPIC, MQTT_STATE_TOPIC_TEMPLATE, MQTT_CLIENT_ID);
  sprintf(MQTT_COMMAND_TOPIC, MQTT_COMMAND_TOPIC_TEMPLATE, MQTT_CLIENT_ID);
  sprintf(MQTT_STATUS_TOPIC, MQTT_STATUS_TOPIC_TEMPLATE, MQTT_CLIENT_ID);

  DEBUG_PRINT(F("INFO: MQTT state topic: "));
  DEBUG_PRINTLN(MQTT_STATE_TOPIC);
  DEBUG_PRINT(F("INFO: MQTT command topic: "));
  DEBUG_PRINTLN(MQTT_COMMAND_TOPIC);
  DEBUG_PRINT(F("INFO: MQTT status topic: "));
  DEBUG_PRINTLN(MQTT_STATUS_TOPIC);

  mqttClient.setServer(MQTT_SERVER, MQTT_SERVER_PORT);
  mqttClient.setCallback(handleMQTTMessage);

  connectToMQTT();

#if defined(OTA)
  setupOTA();
#endif

  bulb.init();

  cmd = CMD_STATE_CHANGED;

  // Init radio
  pinMode(SS,OUTPUT);
  if(DEBUG){
    Serial.begin(115200);
    Serial.println("Debug mode");
    Serial.print("Initialisation");
  }
  SPI.begin();
  SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0));    //Faster SPI mode, maximal speed for the CC2500 without the need for extra delays
  digitalWrite(SS,HIGH);
  SendStrobe(CC2500_SRES); //0x30 SRES Reset chip.
  init_CC2500();
  //  SendStrobe(CC2500_SPWD); //Enter power down mode    -   Not used in the prototype
  //WriteReg(0x3E,0xFF);  //Maximum transmit power - write 0xFF to 0x3E (PATABLE)
  if(DEBUG){
    Serial.println(" - Done");
  }

  delay(1000);
  for (int i = 0; i < CHANNEL_NUM; i++) {
    SendCommand(baseAddress, i, Light_ON_50);
  }
}

void loop() {
#if defined(DEBUG_TELNET)
  // handle the Telnet connection
  handleTelnet();
#endif

  yield();

#if defined(OTA)
  handleOTA();
#endif

  yield();

  connectToMQTT();
  mqttClient.loop();

  yield();

  handleCMD();

  yield();

  bulb.loop();
}
