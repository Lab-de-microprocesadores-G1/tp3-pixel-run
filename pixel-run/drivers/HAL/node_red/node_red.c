/*******************************************************************************
  @file     node_red.h
  @brief    node- red controller
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "node_red.h"
#include  "../../MCAL/uart/uart.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define FRAMESIZE 		1
#define UART_INSTANCE	UART_INSTANCE_3

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void updateColourCallback();
/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static uart_cfg_t config = {
	.baudRate 		= UART_BAUD_RATE_9600,
	.length			= 0,
	.parityEnable	= 0,
	.parityMode		= 0,
	.stopMode		= 0
};


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void nodeRedInit(void)
{
	//UART initialize
	uartInit(UART_INSTANCE, config);
	uartSubscribeRxMsg(UART_INSTANCE, updateColourCallback, FRAMESIZE);

	//Protocol initialize
	protocolInit();
}

protocol_packet_t nodeRedGetValue()
{
	return protocolGetNextPacket();
}

bool nodeRedHasNewValue()
{
	return protocolHasPackets();
}

void nodeRedSendValue(protocol_packet_t packet, uint8_t* encoded)
{
	size_t length = protocolEncode(packet, encoded);
	if(uartCanTx(UART_INSTANCE, length))
	{
		uartWriteMsg(UART_INSTANCE, encoded, length);
	}
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void updateColourCallback()
{
	word_t newByte;
	uartReadMsg(UART_INSTANCE, &newByte, 1);
	protocolDecode((char)newByte);
}

/*******************************************************************************
 *******************************************************************************
						            INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

/******************************************************************************/
