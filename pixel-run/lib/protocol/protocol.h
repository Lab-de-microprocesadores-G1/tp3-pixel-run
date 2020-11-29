/*******************************************************************************
  @file     protocol.h
  @brief    Custom protocol for data link layer of OSI model
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef __PROTOCOL_HEADER__
#define __PROTOCOL_HEADER__

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define PROTOCOL_MAX_SIZE       30      // Maximum size of an encoded data packet

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Pixel topic data structure
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} protocol_pixel_data_t;

// Protocol supported topics
typedef enum {
    PROTOCOL_TOPIC_PLAYER_PIXEL,
    PROTOCOL_TOPIC_OBSTACLE_PIXEL,

    PROTOCOL_TOPIC_COUNT
} protocol_topic_t;

// Protocol packet data
typedef union {
    protocol_pixel_data_t   pixel;
} protocol_data_t;

// Protocol packet
typedef struct {
    protocol_topic_t        topic;
    protocol_data_t         data;
} protocol_packet_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initializes the protocol module.
 */
void protocolInit(void);

/*
 * @brief Run the decodification algorithm with the next byte received over the physical layer.
 * @param data      Next byte to decode
 */
void protocolDecode(uint8_t data);

/*
 * @brief Returns true if there is a new packet.
 */
bool protocolHasPackets(void);

/*
 * @brief Returns the packet.
 */
protocol_packet_t protocolGetNextPacket(void);

/*
 * @brief Run the codification algorithm.
 * @param packet      Packet to encode.
 * @param encoded     Direction to encoded data.
 * @return Length of codified data.
 */
size_t protocolEncode(protocol_packet_t packet, uint8_t* encoded);

/*******************************************************************************
 ******************************************************************************/

#endif 
