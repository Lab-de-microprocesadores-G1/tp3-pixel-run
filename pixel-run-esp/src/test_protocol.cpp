/*******************************************************************************
  @file     test_protocol.c
  @brief    Protocol library testbench
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <Arduino.h>
#include <stdint.h>

extern "C"
{
#include "protocol.h"
}

/*******************************
 * CONSTANT VALUES AND DEFINES * 
 ******************************/

#define SERIAL_BAUD_RATE        9600
#define SCAN_PERIOD_MS          1000

/***********************************
 * FUNCTION PROPOTYPES DECLARATION * 
 **********************************/

/************************
 * FILE SCOPE VARIABLES * 
 ***********************/

static uint8_t  nextByte;

void setup() 
{
  // Initialize the serial communication driver
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize the protocol library
  protocolInit();

  // Initialize the message in the monitor
  Serial.println("<Test Protocol> ¡Ready!");
}

void loop() 
{
    if (Serial.available())
    {
        nextByte = Serial.read();
        protocolDecode(nextByte);
        if (protocolHasPackets())
        {
            protocol_packet_t packet = protocolGetNextPacket();
            Serial.print("<Test Protocol> Topico: ");
            Serial.print(packet.topic);
            Serial.println("");

            Serial.print("<Test Protocol> R: ");
            Serial.print(packet.data.pixel.r);
            Serial.println("");

            Serial.print("<Test Protocol> G: ");
            Serial.print(packet.data.pixel.g);
            Serial.println("");

            Serial.print("<Test Protocol> B: ");
            Serial.print(packet.data.pixel.b);
            Serial.println("");
        }
    }
}

/***********************
 * FUNCTION DEFINITION * 
 **********************/