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
#include "../../../lib/protocol/protocol.h"

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


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
 * @brief Initializes the node_red driver
 */
void nodeRedInit(void);

/*
 * @brief Gets new colour
 * @return Struct with topic and rgb colours
 */
protocol_packet_t nodeRedGetValue();

/*
 * @brief True if new value
 */
bool nodeRedHasNewValue();

/*
 * @brief Sends message to node-red
 * @param msg: message to send
 * @param length: length of message to be sent
 */
void nodeRedSendValue(protocol_packet_t packet, uint8_t* encoded);
/*******************************************************************************
 ******************************************************************************/


#endif /* NODE_RED_H_ */
