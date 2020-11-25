/*******************************************************************************
  @file     joystick.c
  @brief    Joystick driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <math.h>

#include "board.h"
#include "joystick.h"

#include "drivers/MCAL/systick/systick.h"
#include "drivers/MCAL/gpio/gpio.h"
#include "drivers/MCAL/adc/adc.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Joystick general settings, related to hardware or driver configuration
#define	JOYSTICK_BUTTON_PIN			PIN_JOYSTICK_BUTTON
#define JOYSTICK_BUTTON_ACTIVE		LOW
#define JOYSTICK_SAMPLE_RATE_MS		50
#define JOYSTICK_SAMPLE_RATE_TICKS	SYSTICK_MS2TICKS(JOYSTICK_SAMPLE_RATE_MS)
#define JOYSTICK_STANDBY			127
#define JOYSTICK_SAMPLE_RESET		-1

// Math defines
#define PI							3.14159265
#define RAD2DEGREES(x)				(((x) * 360) / (2*PI))

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the joystick's context data structure
typedef struct {
	/* General variables */
	bool 				alreadyInitialized;		// The driver has already been initialized
	uint32_t			tickCounter;			// Used to control the sample rate to read joystick status
	uint8_t				threshold;				// Threshold used to detect the direction
	joystick_position_t	bufferPosition;			// Holds the current position during the sample interval
	uint8_t				samplingPositionStep;	// Step in the position sampling process, used to control simple state machine

	/* Current status */
	bool						buttonPressed;		// If the button is currently pressed
	joystick_position_t			position;			// Current position of the joystick
	uint16_t					angle;				// Current angle of the joystick
	joystick_fixed_direction_t	direction;			// Current direction of the joystick

	/* Flags */
	bool 						angleReset;			// When the angle should be reset
	bool						directionReset;		// When the direction should be reset
	/* Callbacks */
	js_pressed_cb_t			onButtonPressed;	// When the button is pressed
	js_fixed_direction_cb_t	onFixedDirection;	// When the position is moved towards a fixed direction
	js_any_direction_cb_t	onAnyDirection;		// When the position is moved towards any direction
} js_context_t;

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Joystick service routine to be run periodically, to update the
 * 		  current status of the joystick.
 */
static void joystickPeriodicISR(void);

/**
 * @brief Joystick routine used to sample both horizontal and vertical positions
 * @param sample	Current sample taken, the first time when the measurement
 * 					process starts, will receive invalid value.
 */
static void joystickSamplePosition(int16_t sample);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static js_context_t	context;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void joystickInit(void)
{
	if (!context.alreadyInitialized)
	{
		// Update the context flag, driver already initialized
		context.alreadyInitialized = true;

		// Initialization of the driver's context
		context.tickCounter = JOYSTICK_SAMPLE_RATE_TICKS;
		context.samplingPositionStep = 0;
		context.angleReset = true;
		context.directionReset = true;

		// Initialization of the adc driver
		adc_cfg_t adcSettings = {
			.diff = 0,
			.resolution = ADC_8_BIT_SINGLE_CONV,
			.usingAverage = 1,
			.averageSamples = ADC_8_SAMPLES_AVERAGE
		};

		adcInit();
		adcConfig(ADC_JOYSTICK_AXIS_X, adcSettings);
		adcConfig(ADC_JOYSTICK_AXIS_Y, adcSettings);
		adcOnConversion(ADC_JOYSTICK_AXIS_X, joystickSamplePosition);
		adcOnConversion(ADC_JOYSTICK_AXIS_Y, joystickSamplePosition);

		// Initialization of the gpio driver
		gpioMode(JOYSTICK_BUTTON_PIN, INPUT | (JOYSTICK_BUTTON_ACTIVE == LOW ? PULLUP : PULLDOWN));

		// Initialization of the systick driver
		systickInit(joystickPeriodicISR);
	}
}

joystick_position_t joystickGetPosition(void)
{
	return context.position;
}

bool joystickIsPressed(void)
{
	return context.buttonPressed;
}

void joystickOnPressed(js_pressed_cb_t callback)
{
	context.onButtonPressed = callback;
}

void joystickOnFixedDirection(js_fixed_direction_cb_t callback, uint8_t threshold)
{
	context.onFixedDirection = callback;
	context.threshold = threshold;
}

void joystickOnAnyDirection(js_any_direction_cb_t callback, uint8_t threshold)
{
	context.onAnyDirection = callback;
	context.threshold = threshold;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 *******************************************************************************
						 INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

static void joystickPeriodicISR(void)
{
	if (--context.tickCounter == 0)
	{
		// Reload the value of the tickCounter
		context.tickCounter = JOYSTICK_SAMPLE_RATE_TICKS;

		// Sample the status of the joystick's button
		bool previousButtonPressed = context.buttonPressed;
		context.buttonPressed = gpioRead(JOYSTICK_BUTTON_PIN) == JOYSTICK_BUTTON_ACTIVE ? true : false;
		if (!previousButtonPressed && context.buttonPressed)
		{
			if (context.onButtonPressed)
			{
				context.onButtonPressed();
			}
		}

		// Sample the status of the joystick's position
		joystickSamplePosition(JOYSTICK_SAMPLE_RESET);
	}
}

static void joystickSamplePosition(int16_t sample)
{
	if (sample == JOYSTICK_SAMPLE_RESET)
	{
		context.samplingPositionStep = 0;
	}
	else
	{
		context.samplingPositionStep = (context.samplingPositionStep + 1) % 3;
	}
	switch (context.samplingPositionStep)
	{
		case 0:		// Start the sampling process on the x axis of the joystick's position
			adcStartConversion(ADC_JOYSTICK_AXIS_X);
			break;
		case 1:		// Conversion completed on the x axis, starting the sampling on the y axis
			adcStartConversion(ADC_JOYSTICK_AXIS_Y);
			context.bufferPosition.x = (uint8_t)sample;
			break;
		case 2:		// Conversion completed, update the position status and reset the simple state machine
			context.bufferPosition.y = (uint8_t)sample;
			context.position = context.bufferPosition;

			// Performs the algorithm to detect whether the position is greater than the threshold
			double posX = context.position.x;
			double posY = context.position.y;
			double deltaX = posX - 127;
			double deltaY = posY - 127;
			double angle;
			double slice;
			joystick_fixed_direction_t direction;
			if (sqrt(deltaX * deltaX + deltaY * deltaY) >= context.threshold)
			{
				// Compute the angle, and makes a correction if needed
				angle = RAD2DEGREES(atan(deltaY / deltaX));
				if (deltaX < 0)
				{
					angle = angle + 180;
				}
				if (angle < 0)
				{
					angle = angle + 360;
				}

				// Raises the event corresponding
				if (context.onAnyDirection)
				{
					// If the event changed after the previous value
					if (context.angleReset | (context.angle != (uint16_t)angle))
					{
						context.angleReset = false;
						context.angle = angle;
						context.onAnyDirection(context.angle);
					}
				}

				if (context.onFixedDirection)
				{
					slice = 180 / JOYSTICK_DIRECTION_COUNT;
					direction = (uint8_t)((angle + slice) / (slice * 2)) % JOYSTICK_DIRECTION_COUNT;

					// If the event changed after the previous value
					if (context.directionReset | (context.direction != direction))
					{
						context.directionReset = false;
						context.direction = direction;
						context.onFixedDirection(context.direction);
					}
				}
			}
			else
			{
				context.angle = true;
				context.directionReset = true;
			}
			break;
		default: 	// On invalid values, reset the state machine
			context.samplingPositionStep = 0;
			break;
	}
}

/******************************************************************************/
