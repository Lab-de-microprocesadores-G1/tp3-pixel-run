/***************************************************************************//**
  @file     serial_led.h
  @brief    Driver for multiple LED handling using SPI and HC595 IC
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef HAL_SERIAL_LED_SERIAL_LED_H_
#define HAL_SERIAL_LED_SERIAL_LED_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Parallel LED Outputs.
typedef enum {
	SERIAL_LED_0,	// QA
	SERIAL_LED_1,	// QB
	SERIAL_LED_2,	// QC
	SERIAL_LED_3,	// QD
	SERIAL_LED_4,	// QE
	SERIAL_LED_5,	// QF
	SERIAL_LED_6,	// QG
	SERIAL_LED_7,	// QH
	SERIAL_LED_COUNT
} serial_led_id_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initializes the serial driver
 */
void serialLedInit();

/**
 * @brief Updates the state of one of the LEDs
 * @param id		Led to update
 * @param active	New LED state
 */
void serialLedWrite(serial_led_id_t id, bool active);

/**
 * @brief Updates the state of all LEDs
 * @param status	State of LED "i" is given by bit "i"
 */
void serialLedWriteAll(uint8_t status);

/**
 * @brief Writes a number from 0 to 8.
 * @param number	Value to be written.
 * For example, sending number=2 will turn on QA and QB.
 */
void serialLedSetNumber(uint8_t number);

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 ******************************************************************************/


#endif /* HAL_SERIAL_LED_SERIAL_LED_H_ */
