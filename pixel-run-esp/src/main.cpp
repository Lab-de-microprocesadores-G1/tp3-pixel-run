/*******************************************************************************
  @file     main.c
  @brief    ES8266 WiFi application
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <Arduino.h>
#include <iostream>  
#include <string> 
#include <cstdio>

#include "ESP8266WiFi.h"
#include "PubSubClient.h"

extern "C"
{
  #include "protocol.h"
}

using namespace std;

/*******************************
 * CONSTANT VALUES AND DEFINES * 
 ******************************/

// Debuggin mode used only to print in PC monitor, when using
// the K64F remove this define!
// #define DEBUG_MODE

// MQTT topic subscriptions
#define TOPIC_PLAYER_PIXEL      "player_pixel"
#define TOPIC_OBSTACLE_PIXEL    "obstacle_pixel"
#define TOPIC_LEVEL             "level"

// General application settings
#define SERIAL_BAUD_RATE        9600
#define DEBUG_MODE

/***********************************
 * FUNCTION PROPOTYPES DECLARATION * 
 **********************************/

/*
 * @brief Dispatch the topic of the MQTT event
 * @param topic   Topic of the event received
 * @param payload Data of the event
 * @param length  Length of data
 */
void mqttDispatcher(char* topic, byte* payload, unsigned int length);

/**
 * @brief Dispatch the packet received via UART as a MQTT event
 * @param packet    New packet received
 */
void packetDispatcher(protocol_packet_t packet);

/*
 * @brief Parses pixel data
 * @param payload Data
 * @param length  Size of data
 */
protocol_pixel_data_t parsePixelData(byte* payload, unsigned int length); 

/************************
 * FILE SCOPE VARIABLES * 
 ***********************/

uint8_t       receivedByte;                       // Buffer for the next received byte
uint8_t       buffer[PROTOCOL_MAX_SIZE];          // Buffer for the data to be transmitted via UART

const char*   wifiSsid      = "Fibertel WiFi062 2.4GHz";
const char*   wifiPassword  = "0141649034";
IPAddress     ip(192, 168, 1, 12);               // IP Address of the MQTT Server 
// IPAddress     ip((const uint8_t*)"broker.mqtt-dashboard.com");    // IP Address of the MQTT Server
uint16_t      port = 1883;                        // Port of the MQTT Server

const char*   mqttUser      = "lucaskammann";     // User for the MQTT connection
const char*   mqttPassword  = "password";         // Password for the MQTT connection
const char*   mqttClientID  = "someid";           // ID to identify as unique user

PubSubClient  mqttClient;                         // Instance of the MQTT Client
WiFiClient    wifiClient;                         // Instance of the WiFi Client

/******************************
 * APPLICATION SETUP AND LOOP * 
 *****************************/

void setup() 
{
  // Initialize the protocol module
  protocolInit();

  // Initialize the serial communication driver
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize the MQTT client
  mqttClient.setServer(ip, port);
  mqttClient.setCallback(mqttDispatcher);
  mqttClient.setClient(wifiClient);

  // Initialize the connection to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(wifiSsid, wifiPassword);
  while ( WiFi.status() != WL_CONNECTED )
  {
    delay(100);
  }

  #ifdef DEBUG_MODE
  Serial.println("<MQTT Client> Connection established!");
  #endif
}

void loop() 
{
  // When the MQTT Client is not connected, try to connect using the
  // client id, and user credentials corresponding. Subscribes to the
  // topics desired by the client.
  while ( !mqttClient.connected() )
  {
    
    if (mqttClient.connect(mqttClientID, mqttUser, mqttPassword))
    {
      mqttClient.subscribe(TOPIC_PLAYER_PIXEL);
      mqttClient.subscribe(TOPIC_OBSTACLE_PIXEL);

      #ifdef DEBUG_MODE
      Serial.println("<MQTT Client> Client subscribed!");
      #endif
    }
    else
    {
      #ifdef DEBUG_MODE
      Serial.println("<MQTT Client> Client not subscribed!");
      #endif
    }
  }

  // Run the loop of the MQTT client
  mqttClient.loop();

  // Verify if a new message via UART has been received
  if (Serial.available() > 0)
  {
    receivedByte = Serial.read();
    Serial.println(receivedByte);
    protocolDecode(receivedByte);
    if (protocolHasPackets())
    {
      packetDispatcher(protocolGetNextPacket());
    }
  }
}

/***********************
 * FUNCTION DEFINITION * 
 **********************/

void packetDispatcher(protocol_packet_t packet)
{
  switch (packet.topic)
  {
    case PROTOCOL_TOPIC_LEVEL:
      packet.data.level.level += '0';
      mqttClient.publish(TOPIC_LEVEL, &packet.data.level.level, 1);
      break;
    default:
      break;
  }
}

void mqttDispatcher(char* topic, byte* payload, unsigned int length)
{
  #ifdef DEBUG_MODE
  Serial.println("<MQTT Client> Data received from MQTT!");
  #endif

  protocol_packet_t packet;
  uint8_t size;

  // Create the packet for each case of topic
  if(string(topic) == TOPIC_OBSTACLE_PIXEL)
  {
    packet.topic = PROTOCOL_TOPIC_OBSTACLE_PIXEL;
    packet.data.pixel = parsePixelData(payload, length);
  }
  else if (string(topic) == TOPIC_PLAYER_PIXEL)
  {
    packet.topic = PROTOCOL_TOPIC_PLAYER_PIXEL;
    packet.data.pixel = parsePixelData(payload, length);
  }
  
  // Encode the packet to raw data to be transmitted via UART
  size = protocolEncode(packet, buffer);

  // Send via UART
  for (uint8_t i = 0 ; i < size ; i++)
  {
    while (!Serial.availableForWrite());
    Serial.print((char)buffer[i]);
  }
}

protocol_pixel_data_t parsePixelData(byte* payload, unsigned int length)
{
  protocol_pixel_data_t data;

  // Find the index where each component starts in the string received
  payload[length] = 0;
  int r_pos = string((char*)payload).find("(") + 1;
  int g_pos = string((char*)payload).find_first_of(",") + 2;
  int b_pos = string((char*)payload).find_last_of(",") + 2;
  int end_pos = string((char*)payload).find_last_of(")");

  // Converts to individual string values of R, G, B components
  char r_value[4], g_value[4], b_value[4];
  uint8_t i = 0;
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

  // Save and return the pixel data
  data.r = atoi(r_value);
  data.g = atoi(g_value);
  data.b = atoi(b_value);
  return data;
}