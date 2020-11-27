#include <Arduino.h>

#include "ESP8266WiFi.h"
#include "PubSubClient.h"
#include <cstdio>
#include <iostream>  
#include <string>  

using namespace std;

/*******************************
 * CONSTANT VALUES AND DEFINES * 
 ******************************/

#define SERIAL_BAUD_RATE        9600
#define SCAN_PERIOD_MS          1000

/***********************************
 * FUNCTION PROPOTYPES DECLARATION * 
 **********************************/

void messageCallback(char* topic, byte* payload, unsigned int length);

/************************
 * FILE SCOPE VARIABLES * 
 ***********************/

const char*   wifiSsid = "Fibertel WiFi568 2.4GHz";
const char*   wifiPassword = "0103052182";
IPAddress     ip(192, 168, 0, 130);         // IP Address of the MQTT Server
uint16_t       port = 1883;                 // Port of the MQTT Server

const char*   mqttUser = "lucaskammann";    // User for the MQTT connection
const char*   mqttPassword = "password";    // Password for the MQTT connection
const char*   mqttClientID = "someid";      // ID to identify as unique user

PubSubClient  mqttClient;                   // Instance of the MQTT Client
WiFiClient    wifiClient;                   // Instance of the WiFi Client

uint32_t      lastMillis;
uint32_t      currentMillis;

typedef struct {
  uint8_t topic;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} pixel_t;

void setup() 
{
  // Initialize the serial communication driver
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize the MQTT client
  mqttClient.setServer(ip, port);
  mqttClient.setCallback(messageCallback);
  mqttClient.setClient(wifiClient);

  // Initialize the connection to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.println("< MQTT Client > Trying to connect to WiFi network...");
  while ( WiFi.status() != WL_CONNECTED )
  {
    delay(100);
  }
  Serial.println("< MQTT Client > WiFi connection established!");
}

void loop() 
{
  // When the MQTT Client is not connected, try to connect using the
  // client id, and user credentials corresponding...
  while ( !mqttClient.connected() )
  {
    Serial.println("< MQTT Client > Trying to connect to MQTT Broker");
    if (mqttClient.connect(mqttClientID, mqttUser, mqttPassword))
    {
      Serial.println("< MQTT Client > Connection established to the MQTT Broker");
      mqttClient.subscribe("esp/player_colour");
      mqttClient.subscribe("esp/obstacle_colour");
    }
  }

  // Run the loop of the MQTT client
  mqttClient.loop();
}

/***********************
 * FUNCTION DEFINITION * 
 **********************/

void messageCallback(char* topic, byte* payload, unsigned int length)
{
  // Set the terminator of the payload
  pixel_t pixel_colour;
  payload[length] = 0;
  int r_pos = string((char*)payload).find("(") + 1;
  int g_pos = string((char*)payload).find_first_of(",") + 2;
  int b_pos = string((char*)payload).find_last_of(",") + 2;
  int end_pos = string((char*)payload).find_last_of(")");

  char r_value[4];
  char g_value[4];
  char b_value[4];
  int i = 0;
  for(i = 0; i < g_pos - r_pos - 2; i++)
  {
    r_value[i] = payload[r_pos + i];
  }
  r_value[i] = 0;

  for(i = 0; i < b_pos - g_pos - 2; i++)
  {
    g_value[i] = payload[g_pos + i];
  }
  g_value[i] = 0;

  for(i = 0; i < end_pos - b_pos; i++)
  {
    b_value[i] = payload[b_pos + i];
  }
  b_value[i] = 0;

  pixel_colour.r = atoi(r_value);
  pixel_colour.g = atoi(g_value);
  pixel_colour.b = atoi(b_value);

  if(string(topic) == "esp/obstacle_colour")
  {
    pixel_colour.topic = 0;
    Serial.write((const char*)(&pixel_colour), sizeof(pixel_colour));
  }
  else if (string(topic) == "esp/player_colour")
  {
    pixel_colour.topic = 1;
    Serial.write((const char*)(&pixel_colour), sizeof(pixel_colour));
  }
}

