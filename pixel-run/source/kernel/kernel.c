/*******************************************************************************
  @file     kernel.c
  @brief    Kernel abstraction layer between application and the MCU
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/* kernel.c TODO */
// todo check accelerometer initialisation (parameters)
// todo check joystick threshold value

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "kernel.h"

// HAL Drivers
#include  "../../drivers/HAL/WS2812/WS2812.h"
#include  "../../drivers/HAL/timer/timer.h"
#include  "../../drivers/HAL/joystick/joystick.h"
#include  "../../drivers/HAL/FXOS8700/fxos8700_accelerometer.h"

// lib
#include  "../../lib/event_queue/event_queue.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define MAX_KERNEL_EVENT_QTY  50
#define EVGEN_QUEUE_SIZE      25

/* Joystick params */
#define JOYSTICK_THRESHOLD    50

/* Accelerometer params */

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct
{
  uint8_t           queueBuffer[MAX_KERNEL_EVENT_QTY + 1];    // array reserved for the queue
  event_queue_t     eventQueue;                               // queue for HW events
  tim_id_t          timer;                                    // timer to use
}kernel_context_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
static void* hardwareEvGen(void);

static void joystickCallback(joystick_fixed_direction_t direction);
static void timerCallback(void);
static void accelerometerCallback(void);

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

const ws2812_pixel_t kernelBlackPixel = {0, 0, 0};
const ws2812_pixel_t kernelRedPixel = {1, 0, 0};
const ws2812_pixel_t kernelGreenPixel = {0, 1, 0};
const ws2812_pixel_t kernelBluePixel = {0, 0, 1};

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
static kernel_context_t kernelContext;
static ws2812_pixel_t kernelDisplayMatrix[KERNEL_DISPLAY_SIZE][KERNEL_DISPLAY_SIZE];

/* queues for event generators */
static kernel_event_t   kernelQueueBuffer[EVGEN_QUEUE_SIZE];
static queue_t          kernelEvsInternalQueue;

/* Controls brightness of display */
static uint8_t kernelBrightness = 5;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void kernelInit(void)
{
  /* HAL drivers initialisation */
  joystickInit();
  timerInit();
  kernelContext.timer = timerGetId();
  WS2812Init();
  WS2812SetDisplayBuffer(kernelDisplayMatrix, KERNEL_DISPLAY_SIZE*KERNEL_DISPLAY_SIZE);
  FXOSInit(FXOS_ZLOCK_13_DEG, FXOS_BKFR_THRESHOLD_0, FXOS_THRESHOLD_15_DEG, FXOS_HYSTERESIS_11_DEG);

  /* event generators queue init */
  kernelEvsInternalQueue = createQueue(kernelQueueBuffer, EVGEN_QUEUE_SIZE, sizeof(kernel_event_t));
  
  /* event queue initialisation */
  kernelContext.eventQueue = createEventQueue(kernelContext.queueBuffer, MAX_KERNEL_EVENT_QTY + 1, sizeof(kernel_event_t));
  
  /* subscribe to input peripherals */
  joystickOnFixedDirection(joystickCallback, JOYSTICK_THRESHOLD);
  FXOSSubscribeOrientationChanged(accelerometerCallback);  
  
  /* event generators registration */
  registerEventGenerator(&(kernelContext.eventQueue), hardwareEvGen);
}

void kernelDisplay(const kernel_color_t matrix[KERNEL_DISPLAY_SIZE][KERNEL_DISPLAY_SIZE], uint8_t runnerPos)
{
  for (uint8_t i=0; i<KERNEL_DISPLAY_SIZE; i++)
  {
    for (uint32_t j=0; j<KERNEL_DISPLAY_SIZE; j++)
    {
      switch (matrix[i][j])
      {
        case KERNEL_BLACK: 
        {
          kernelDisplayMatrix[i][j].r = kernelBlackPixel.r;
          kernelDisplayMatrix[i][j].g = kernelBlackPixel.g;
          kernelDisplayMatrix[i][j].b = kernelBlackPixel.b;
        } break;
        case KERNEL_RED: 
        {
          kernelDisplayMatrix[i][j].r = kernelRedPixel.r * kernelBrightness;
          kernelDisplayMatrix[i][j].g = kernelRedPixel.g * kernelBrightness;
          kernelDisplayMatrix[i][j].b = kernelRedPixel.b * kernelBrightness;
        } break;
        case KERNEL_GREEN: 
        {
          kernelDisplayMatrix[i][j].r = kernelGreenPixel.r * kernelBrightness;
          kernelDisplayMatrix[i][j].g = kernelGreenPixel.g * kernelBrightness;
          kernelDisplayMatrix[i][j].b = kernelGreenPixel.b * kernelBrightness;
        } break;
        case KERNEL_BLUE: 
        {
          kernelDisplayMatrix[i][j].r = kernelBluePixel.r *  kernelBrightness;
          kernelDisplayMatrix[i][j].g = kernelBluePixel.g *  kernelBrightness;
          kernelDisplayMatrix[i][j].b = kernelBluePixel.b *  kernelBrightness;
        } break;
      }

      if (i == 7 && j == runnerPos)
      {
    	kernelDisplayMatrix[i][j].r = kernelGreenPixel.r * kernelBrightness;
    	kernelDisplayMatrix[i][j].g = kernelGreenPixel.g * kernelBrightness;
    	kernelDisplayMatrix[i][j].b = kernelGreenPixel.b * kernelBrightness;
      }
    }
  }

  WS2812Update();
}

void kernelChangeDisplayIntensity(bool increase)
{
  for (uint8_t i=0; i<KERNEL_DISPLAY_SIZE; i++)
  {
    for (uint32_t j=0; j<KERNEL_DISPLAY_SIZE; j++)
    {
      kernelDisplayMatrix[i][j].r = kernelDisplayMatrix[i][j].r / kernelBrightness * (kernelBrightness+1);
      kernelDisplayMatrix[i][j].g = kernelDisplayMatrix[i][j].g / kernelBrightness * (kernelBrightness+1);
      kernelDisplayMatrix[i][j].b = kernelDisplayMatrix[i][j].b / kernelBrightness * (kernelBrightness+1);
        
    }
  }

  kernelBrightness += 1;

  WS2812Update();
}

void kernelStartTimer(uint32_t ms, bool periodic)
{
  uint8_t mode = periodic ? TIM_MODE_PERIODIC : TIM_MODE_SINGLESHOT;
  timerStart(kernelContext.timer, TIMER_MS2TICKS(ms), mode, timerCallback);
}

void kernelStopTimer(void)
{
  timerPause(kernelContext.timer);
}

void kernelRestartTimer()
{
	timerRestart(kernelContext.timer);
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
kernel_event_t kernelGetNextEvent(void)
{
	return *((kernel_event_t*)getNextEvent(&kernelContext.eventQueue));
}

void* hardwareEvGen(void)
{
  return pop(&kernelEvsInternalQueue);
}

void joystickCallback(joystick_fixed_direction_t direction)
{
  if (emptySize(&kernelEvsInternalQueue))
  {
    kernel_event_t ev;
    switch(direction)
    {
      case JOYSTICK_DIRECTION_RIGHT:
        ev.id = KERNEL_RIGHT;
        push(&kernelEvsInternalQueue, &ev);
        break;
        
      case JOYSTICK_DIRECTION_LEFT:
        ev.id = KERNEL_LEFT;
        push(&kernelEvsInternalQueue, &ev);
        break;

      default:
        break;
    }
  }
}

void accelerometerCallback(void)
{
  acc_orientation_t currentOrientation;
  if (FXOSGetOrientation(&currentOrientation) && emptySize(&kernelEvsInternalQueue))
  {
    kernel_event_t ev;
    switch(currentOrientation.landscapePortrait)
    {
      case ACC_LANDSCAPE_RIGHT:
        ev.id = KERNEL_RIGHT;
        push(&kernelEvsInternalQueue, &ev);
        break;

      case ACC_LANDSCAPE_LEFT:
        ev.id = KERNEL_LEFT;
        push(&kernelEvsInternalQueue, &ev);
        break;

      default:
        break;
    }
  }
}

void timerCallback(void)
{
  kernel_event_t newTimerEv = { .id = KERNEL_TIMEOUT };
  if (emptySize(&kernelEvsInternalQueue))
  {
    push(&kernelEvsInternalQueue, (void*)&newTimerEv);
  }
}

/******************************************************************************/
