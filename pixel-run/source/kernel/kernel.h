/*******************************************************************************
  @file     kernel.h
  @brief    Kernel abstraction layer between application and the MCU
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef KERNEL_H_
#define KERNEL_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define KERNEL_DISPLAY_SIZE	       	  8	// Display side number of digits (8x8)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Kernel event IDs
typedef enum 
{
	KERNEL_NO_EVENT,
	KERNEL_ENTER,
	KERNEL_LEFT,
	KERNEL_RIGHT,
	KERNEL_TIMEOUT,
	KERNEL_FPS
} kernel_event_id_t;

typedef struct 
{
    kernel_event_id_t   id;
} kernel_event_t;

// // Kernel available timers
// // typedef enum 
// // {
// // 	KERNEL_TIMER,
// // 	KERNEL_TIMER_FPS,
// // 	KERNEL_TIMER_COUNT
// // } kernel_timer_t;

typedef enum
{
	KERNEL_BLACK,
	KERNEL_RED,
	KERNEL_GREEN,
	KERNEL_BLUE,
} kernel_color_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * SERVICES
 ******************************************************************************/

/**
 * @brief Kernel module initialization
 */
void kernelInit(void);

/**
 * @brief Writes to the display
 * @param stream 	Array of digits, length DISPLAY_LENGTH
 */
void kernelDisplay(const kernel_color_t matrix[KERNEL_DISPLAY_SIZE][KERNEL_DISPLAY_SIZE], uint8_t runnerPos);

/**
 * @brief Changes display intensity
 * @param increase whether to increase or decrease the intensity
 */
void kernelChangeDisplayIntensity(bool increase);

/**
 * @brief Starts timer, KERNEL_TIMEOUT_x is emitted on timeout
 * @param ms		Amount of milliseconds to wait
 * @param periodic	Whether the timer stops after timeout or keeps running
 */
void kernelStartTimer(uint32_t ms, bool periodic);

/**
 * @brief Stops timer
 */
void kernelStopTimer(void);

/**
 * @brief If running, stops the timer, and restarts from the beginning
 */
void kernelRestartTimer(void);

/**
 * @brief Print debug message
 * @param msg	Message to print.
 * @param len	Length of message to print.
 */
void kernelPrint(uint8_t * msg, uint8_t len);

/*******************************************************************************
 * EVENT GENERATORS INTERFACE
 ******************************************************************************/

/**
 * @brief Returns the next event in the event queue of the Kernel.
 *
 * @emits 	KERNEL_NO_EVENT,
 * @emits	KERNEL_ENTER,
 * @emits	KERNEL_LEFT,
 * @emits	KERNEL_RIGHT,
 * @emits	KERNEL_TIMEOUT
 */
kernel_event_t kernelGetNextEvent(void);

/*******************************************************************************
 ******************************************************************************/

#endif

