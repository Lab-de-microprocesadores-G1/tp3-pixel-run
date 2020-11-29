/*******************************************************************************
  @file     node_red.h
  @brief    node- red controller
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef NODE_RED_H
#define NODE_RED_H

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "lib/protocol/protocol.h"

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef enum {
	OBSTACLE,
	PLAYER
} node_red_topic_t;

// Pixel data structure
typedef struct {
  node_red_topic_t topic;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} node_red_pixel_t;


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*
 * @brief Initializes the driver
 */
void nodeRedInit(void);

/*
 * @brief Returns the next packet received from node red via mqtt
 */
protocol_packet_t nodeRedGetValue();

/*
 * @brief True if new value available
 */
bool nodeRedHasNewValue();

/*
 * @brief Sends message to node red via mqtt
 * @param packet  Packet with data and topic to be sent
 * @return Whether it could send or not
 */
bool nodeRedSendValue(protocol_packet_t packet);

/*******************************************************************************
 ******************************************************************************/


#endif /* NODE_RED_H_ */
