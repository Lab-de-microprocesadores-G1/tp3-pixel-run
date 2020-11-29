/*******************************************************************************
  @file     protocol.c
  @brief    Custom protocol for data link layer of OSI model
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "protocol.h"
#include "../queue/queue.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Declaring general parameters of the module
#define PROTOCOL_BUFFER_SIZE    10

// Declaring the special characters used to parse raw data
#define PROTOCOL_START_BYTE     0x02
#define PROTOCOL_ESCAPE_BYTE    0x1B
#define PROTOCOL_STOP_BYTE      0x04
#define PROTOCOL_ENCODE_BYTE    0x20

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the protocol states of the decoding state machine
typedef enum {
    PS_START,
    PS_TOPIC,
    PS_ESCAPE_TOPIC,
    PS_DATA,
    PS_ESCAPE_DATA
} protocol_state_t;

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Parses data according to the current topic detected
 * @param data      Byte to be parse
 */
static void protocolDecodeData(uint8_t data);
static void protocolDecodePixelData(uint8_t data);

/**
 * @brief Encodes pixel data
 * @param data      Data to be encoded
 * @param encode    Pointer of the result
 */
static size_t protocolEncodePixelData(protocol_pixel_data_t data, uint8_t* encoded);

/*
 * @brief Applies the escaping algorithm to a byte received or sent
 * @param data      Byte to be encoded
 */
static uint8_t protocolEscapeAlgorithm(uint8_t data);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static protocol_packet_t    packetBuffer[PROTOCOL_BUFFER_SIZE];
static queue_t              packetQueue;
static protocol_packet_t    currentPacket;
static protocol_state_t     currentState;
static uint8_t              currentByte;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void protocolInit(void)
{
    // Initialize the queue to save the packets decoded
    packetQueue = createQueue(&packetBuffer, PROTOCOL_BUFFER_SIZE, sizeof(protocol_packet_t));

    // Initialize protocol decode state
    currentState = PS_START;
}

void protocolDecode(uint8_t data)
{
    switch (currentState)
    {
        case PS_START:
            if (data == PROTOCOL_START_BYTE)
            {
                currentState = PS_TOPIC;
            }
            break;
        case PS_TOPIC:
            if (data == PROTOCOL_ESCAPE_BYTE)
            {
                currentState = PS_ESCAPE_TOPIC;
            }
            else if (data < PROTOCOL_TOPIC_COUNT)
            {
                currentByte = 0;
                currentState = PS_DATA;
                currentPacket.topic = (protocol_topic_t)data;
            }
            else
            {
                currentState = PS_START;
            }
            break;
        case PS_ESCAPE_TOPIC:
            currentPacket.topic = (protocol_topic_t)protocolEscapeAlgorithm(data);
            if (currentPacket.topic < PROTOCOL_TOPIC_COUNT)
            {
                currentByte = 0;
                currentState = PS_DATA;
            }
            else
            {
                currentState = PS_START;
            }
            break;
        case PS_DATA:
            if (data == PROTOCOL_START_BYTE )
            {
                currentState = PS_START;
            }
            else if (data == PROTOCOL_STOP_BYTE)
            {
                currentState = PS_START;
                push(&packetQueue, &currentPacket);
            }
            else if (data == PROTOCOL_ESCAPE_BYTE)
            {
                currentState = PS_ESCAPE_DATA;
            }
            else
            {
                protocolDecodeData(data);
            }
            break;
        case PS_ESCAPE_DATA:
            protocolDecodeData(protocolEscapeAlgorithm(data));
            currentState = PS_DATA;
            break;
    }
}

bool protocolHasPackets(void)
{
    return !isEmpty(&packetQueue);
}

protocol_packet_t protocolGetNextPacket(void)
{
    return *(protocol_packet_t*)pop(&packetQueue);
}

size_t protocolEncode(protocol_packet_t packet, uint8_t* encoded)
{
    uint8_t* startPointer = encoded;
    size_t   encodedSize;
    
    // Create the standard protocol packet
    *(encoded++) = PROTOCOL_START_BYTE;
    *(encoded++) = packet.topic;
    switch (currentPacket.topic)
    {
        case PROTOCOL_TOPIC_OBSTACLE_PIXEL:
        case PROTOCOL_TOPIC_PLAYER_PIXEL:
            encoded += protocolEncodePixelData(packet.data.pixel, encoded);
            break;
        default:
            break;
    }
    *(encoded++) = PROTOCOL_STOP_BYTE;
    encodedSize = encoded - startPointer;

    // Apply the byte stuffing
    for (uint8_t i = 1 ; i < (encodedSize - 1) ; i++)
    {
        uint8_t data = *(startPointer + i);
        if (data == PROTOCOL_START_BYTE || data == PROTOCOL_ESCAPE_BYTE || data == PROTOCOL_STOP_BYTE)
        {
            for (uint8_t j = encodedSize ; j > i ; j--)
            {
                if (j > i + 1)
                {
                    *(startPointer + j) = *(startPointer + j - 1);
                }
                else 
                {
                    *(startPointer + j) = protocolEscapeAlgorithm(data);
                    *(startPointer + j - 1) = PROTOCOL_ESCAPE_BYTE;
                    encodedSize++;
                }
            }
        }
    }

    return encodedSize;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void protocolDecodeData(uint8_t data)
{
    // Data dispatcher, selects the parser according to the current topic being received
    switch (currentPacket.topic)
    {
        case PROTOCOL_TOPIC_OBSTACLE_PIXEL:
        case PROTOCOL_TOPIC_PLAYER_PIXEL:
            protocolDecodePixelData(data);
            break;
        default:
            break;
    }

    // Increment the counter to the following byte
    currentByte++;
}

static void protocolDecodePixelData(uint8_t data)
{
    if (currentByte == 0)
    {
        currentPacket.data.pixel.r = data;
    }
    else if (currentByte == 1)
    {
        currentPacket.data.pixel.g = data;
    }
    else if (currentByte == 2)
    {
        currentPacket.data.pixel.b = data;
    }
}

static size_t protocolEncodePixelData(protocol_pixel_data_t data, uint8_t* encoded)
{
    *(encoded++) = data.r;
    *(encoded++) = data.g;
    *(encoded++) = data.b;
    return sizeof(data);
}

static uint8_t protocolEscapeAlgorithm(uint8_t data)
{
    return data ^ PROTOCOL_ENCODE_BYTE;
}

/*******************************************************************************
 *******************************************************************************
						 INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

/******************************************************************************/


