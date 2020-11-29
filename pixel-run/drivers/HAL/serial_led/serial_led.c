/***************************************************************************//**
  @file     serial_led.c
  @brief    ...
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "serial_led.h"
#include "drivers/MCAL/spi/spi_master.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define LED_SPI_INSTANCE	SPI_INSTANCE_0
#define LED_SPI_SLAVE_ID	SPI_SLAVE_0
#define LED_SPI_BAUD_RATE	1000
#define LED_SPI_FRAME_SIZE	8
#define LED_SPI_ACTIVE		true

#define	LED_SPI_MESSAGE(x)	(LED_SPI_ACTIVE ? x : ~x)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void updateOutput(void);

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static spi_cfg_t spiConfig = {
		.baudRate = LED_SPI_BAUD_RATE,
		.frameSize = LED_SPI_FRAME_SIZE,
		.slaveSelectPolarity = SPI_SS_INACTIVE_HIGH,	// This is to use SS as RCLK
		.continuousPcs		 = SPI_CONTINUOUS_PCS_DIS	// Make sure RCLK has a rising edge after transfer
};

static uint16_t ledStates 	= 0;
static bool 	alreadyInit = false;

void serialLedInit()
{
	if (!alreadyInit)
	{
		// Intialize SPI driver
		spiInit(LED_SPI_INSTANCE, spiConfig);

		// Turn off all LEDs
		serialLedWriteAll(0);

		// Set flag
		alreadyInit = true;
	}
}

void serialLedWrite(serial_led_id_t id, bool active)
{
	// Update corresponding state
	ledStates = (ledStates & ~(1 << id)) | ((active ? 1 : 0) << id);

	// Write Output
	updateOutput();
}

void serialLedWriteAll(uint8_t status)
{
	// Update all states
	ledStates = status;

	// Write Output
	updateOutput();
}


void serialLedSetNumber(uint8_t number)
{
	// Clear previous state
	ledStates = 0;

	// Set corresponding state
	for (uint8_t i = 0 ; i < number ; i++)
	{
		ledStates |= (1 << i);
	}

	// Write Output
	updateOutput();
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void updateOutput(void)
{
	uint16_t message = LED_SPI_MESSAGE(ledStates);
	if (spiCanSend(LED_SPI_INSTANCE, 2))
	{
		// Send led state
		spiSend(LED_SPI_INSTANCE, LED_SPI_SLAVE_ID, &message, 1);
	}
}


/*******************************************************************************
 *******************************************************************************
						            INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

/******************************************************************************/
