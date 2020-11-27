/***************************************************************************//**
  @file     board.h
  @brief    Board management
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef _BOARD_H_
#define _BOARD_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "../drivers/MCAL/gpio/gpio.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/***** BOARD defines **********************************************************/

// On Board User LEDs
#define PIN_LED_RED     PORTNUM2PIN(PB, 22) // PTB22
#define PIN_LED_GREEN   PORTNUM2PIN(PE, 26) // PTE26
#define PIN_LED_BLUE    PORTNUM2PIN(PB, 21) // PTB21

// On Board User Switches
#define PIN_SW2         PORTNUM2PIN(PC, 6)  // PTC6
#define PIN_SW3         PORTNUM2PIN(PA, 4)  // PTA4

#define SW2_ACTIVE		LOW
#define SW3_ACTIVE		LOW

// Active Status
#define LED_ACTIVE      LOW

// Joystick ports
#define PIN_JOYSTICK_AXIS_X		PORTNUM2PIN(PB, 10)	// PTB10
#define PIN_JOYSTICK_AXIS_Y		PORTNUM2PIN(PB, 3)	// PTB3
#define PIN_JOYSTICK_BUTTON		PORTNUM2PIN(PC, 5)	// PTC5

// Display ports
#define PIN_DISPLAY_DATA		PORTNUM2PIN(PC,1)	// PTC1
    

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Basic board initialization, default settings
 */
void boardInit(void);

/*******************************************************************************
 ******************************************************************************/

#endif // _BOARD_H_
