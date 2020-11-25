/*******************************************************************************
  @file     joystick.c
  @brief    Joystick driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef HAL_JOYSTICK_JOYSTICK_C_
#define HAL_JOYSTICK_JOYSTICK_C_

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

// Joystick's position data structure
// The standby position in both axes, horizontal and vertical, is the middle value in the
// range. For example, with 8 bits of conversion for the position, 0 and 255 are the limits,
// and the standby position will be at 127 (approx...)
typedef struct {
	uint8_t	x;
	uint8_t y;
} joystick_position_t;

// Joystick's fixed directions
typedef enum {
	JOYSTICK_DIRECTION_RIGHT,
	JOYSTICK_DIRECTION_RIGHT_UP,
	JOYSTICK_DIRECTION_UP,
	JOYSTICK_DIRECTION_LEFT_UP,
	JOYSTICK_DIRECTION_LEFT,
	JOYSTICK_DIRECTION_LEFT_DOWN,
	JOYSTICK_DIRECTION_DOWN,
	JOYSTICK_DIRECTION_RIGHT_DOWN,

	JOYSTICK_DIRECTION_COUNT
} joystick_fixed_direction_t;

// Joystick's callback type
typedef void (*js_pressed_cb_t)			(void);
typedef void (*js_fixed_direction_cb_t)	(joystick_fixed_direction_t direction);
typedef void (*js_any_direction_cb_t)	(uint16_t angle);

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initializes the joystick driver.
 */
void 				joystickInit(void);

/********************
 * POLLING SERVICES *
 *******************/

/**
 * @brief Returns the current position of the joystick, as a structure containing
 * 		  the values of position x and y.
 */
joystick_position_t joystickGetPosition(void);

/*
 * @brief Returns whether the joystick's button is being pressed.
 */
bool 				joystickIsPressed(void);

/*************************
 * EVENT-DRIVEN SERVICES *
 ************************/

/**
 * @brief Registers a callback to be called when the joystick's button is pressed.
 * @param callback		Callback being registered
 */
void				joystickOnPressed(js_pressed_cb_t callback);

/**
 * @brief Registers a callback to be called when the joystick position changes towards a fixed
 * 	      direction and the position exceeds the given threshold.
 * 	      NOTE, the standby position of the joystick is the middle in the 8 bit range.
 * @param callback		Callback being registered
 * @param threshold		Threshold value to detect the movement in fixed direction.
 */
void				joystickOnFixedDirection(js_fixed_direction_cb_t callback, uint8_t threshold);

/**
 * @brief Registers a callback to be called when the joystick position changes, and the distance moved
 * 		  from the standby position is greater than the threshold.
 * @param callback		Callback being registered
 * @param threshold		Threshold value to detect the movement in fixed direction.
 */
void				joystickOnAnyDirection(js_any_direction_cb_t callback, uint8_t threshold);

/*******************************************************************************
 ******************************************************************************/


#endif /* HAL_JOYSTICK_JOYSTICK_C_ */
