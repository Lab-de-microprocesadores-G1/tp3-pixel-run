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
#include  "../../drivers/HAL/node_red/node_red.h"
#include  "../../drivers/HAL/serial_led/serial_led.h"

// MCAL Drivers (for debug)
#include  "../../drivers/MCAL/uart/uart.h"

// lib
#include  "../../lib/event_queue/event_queue.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define MAX_KERNEL_EVENT_QTY  50
#define EVGEN_QUEUE_SIZE      25

/* Joystick params */
#define JOYSTICK_THRESHOLD    50

/* FPS for display refresh */
#define KERNEL_REF_PERIOD     (1000/60.0)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef struct
{
  kernel_event_t    queueBuffer[MAX_KERNEL_EVENT_QTY + 1];    // array reserved for the queue
  event_queue_t     eventQueue;                               // queue for HW events
  tim_id_t          timer;                                    // timer to use
  tim_id_t          fpsTimer;                                 // timer for fps
  bool              fpsReady;     
}kernel_context_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/
static void* hardwareEvGen(void);
static void* nodeRedEvGen(void);

static void joystickCallback(joystick_fixed_direction_t direction);
static void joystickButtonCallback(void);
static void timerCallback(void);
static void accelerometerCallback(void);

static void fpsTimerCallback(void);

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
static kernel_context_t kernelContext;
static ws2812_pixel_t kernelDisplayMatrix[KERNEL_DISPLAY_SIZE][KERNEL_DISPLAY_SIZE];

/* queues for event generators */
static kernel_event_t   kernelQueueBuffer[EVGEN_QUEUE_SIZE];
static queue_t          kernelEvsInternalQueue;


static ws2812_pixel_t kernelBlackPixel = {0, 0, 0};
static ws2812_pixel_t kernelRedPixel = {5, 0, 0};
static ws2812_pixel_t kernelGreenPixel = {0, 5, 0};
static ws2812_pixel_t kernelBluePixel = {0, 0, 5};

static kernel_event_t newColourEv; 

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void kernelInit(void)
{
  /* MCAL drivers initialisation */
  uart_cfg_t config = { .baudRate = UART_BAUD_RATE_9600, .length = 0, .parityEnable = 0, .parityMode = 0, .stopMode = 0 };
  uartInit(UART_INSTANCE_0, config);

  /* HAL drivers initialisation */
  joystickInit();
  timerInit();
  kernelContext.timer = timerGetId();
  kernelContext.fpsTimer = timerGetId();
  kernelContext.fpsReady = true;
  WS2812Init();
  WS2812SetDisplayBuffer(kernelDisplayMatrix, KERNEL_DISPLAY_SIZE*KERNEL_DISPLAY_SIZE);
  FXOSInit(FXOS_ZLOCK_13_DEG, FXOS_BKFR_THRESHOLD_0, FXOS_THRESHOLD_15_DEG, FXOS_HYSTERESIS_11_DEG);
  nodeRedInit();
  serialLedInit();

  /* event generators queue init */
  kernelEvsInternalQueue = createQueue(kernelQueueBuffer, EVGEN_QUEUE_SIZE, sizeof(kernel_event_t));
  
  /* event queue initialisation */
  kernelContext.eventQueue = createEventQueue(kernelContext.queueBuffer, MAX_KERNEL_EVENT_QTY + 1, sizeof(kernel_event_t));
  
  /* subscribe to input peripherals */
  joystickOnFixedDirection(joystickCallback, JOYSTICK_THRESHOLD);
  joystickOnPressed(joystickButtonCallback);
  FXOSSubscribeOrientationChanged(accelerometerCallback);
  
  /* event generators registration */
  registerEventGenerator(&(kernelContext.eventQueue), hardwareEvGen);
  registerEventGenerator(&(kernelContext.eventQueue), nodeRedEvGen);
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
          kernelDisplayMatrix[i][j].r = kernelRedPixel.r;
          kernelDisplayMatrix[i][j].g = kernelRedPixel.g;
          kernelDisplayMatrix[i][j].b = kernelRedPixel.b;
        } break;
        case KERNEL_GREEN: 
        {
          kernelDisplayMatrix[i][j].r = kernelGreenPixel.r;
          kernelDisplayMatrix[i][j].g = kernelGreenPixel.g;
          kernelDisplayMatrix[i][j].b = kernelGreenPixel.b;
        } break;
        case KERNEL_BLUE: 
        {
          kernelDisplayMatrix[i][j].r = kernelBluePixel.r;
          kernelDisplayMatrix[i][j].g = kernelBluePixel.g;
          kernelDisplayMatrix[i][j].b = kernelBluePixel.b;
        } break;
      }

      if (i == 7 && j == runnerPos)
      {
    	kernelDisplayMatrix[i][j].r = kernelGreenPixel.r;
    	kernelDisplayMatrix[i][j].g = kernelGreenPixel.g;
    	kernelDisplayMatrix[i][j].b = kernelGreenPixel.b;
      }
    }
  }

  WS2812Update();
}

void kernelStartTimer(uint32_t ms, bool periodic)
{
  uint8_t mode = periodic ? TIM_MODE_PERIODIC : TIM_MODE_SINGLESHOT;
  timerStart(kernelContext.timer, TIMER_MS2TICKS(ms), mode, timerCallback);
}

void kernelStartFpsTimer(void)
{
  /* Initalising FPS timer */
  timerStart(kernelContext.fpsTimer, TIMER_MS2TICKS(KERNEL_REF_PERIOD), false, fpsTimerCallback);
  kernelContext.fpsReady = false;
}

bool kernelFpsReady(void)
{
  return kernelContext.fpsReady;
}

void kernelStopTimer(void)
{
  timerPause(kernelContext.timer);
}

void kernelRestartTimer(void)
{
	timerRestart(kernelContext.timer);
}

void kernelChangeNodeRedColour(protocol_packet_t newColourPacket)
{
  if (newColourPacket.topic == PROTOCOL_TOPIC_PLAYER_PIXEL)
  {
    kernelGreenPixel.r = newColourPacket.data.pixel.r;
    kernelGreenPixel.g = newColourPacket.data.pixel.g;
    kernelGreenPixel.b = newColourPacket.data.pixel.b;
  }
  else if (newColourPacket.topic == PROTOCOL_TOPIC_OBSTACLE_PIXEL)
  {
    kernelBluePixel.r = newColourPacket.data.pixel.r;
    kernelBluePixel.g = newColourPacket.data.pixel.g;
    kernelBluePixel.b = newColourPacket.data.pixel.b;
  }
}

void kernelPrint(uint8_t * msg, uint8_t len)
{
  if (uartCanTx(UART_INSTANCE_0, len))
  {
    uartWriteMsg(UART_INSTANCE_0, msg, len);
  }
}

void kernelLevelLed(uint8_t level)
{
  serialLedSetNumber(level + 1);

  protocol_packet_t packet;
  packet.topic = PROTOCOL_TOPIC_LEVEL;
  packet.data.level.level = level + 1;
  nodeRedSendValue(packet);
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

void* nodeRedEvGen(void)
{
  if (nodeRedHasNewValue())
  {
    newColourEv.id = KERNEL_NODE_RED_COLOUR;
    newColourEv.nodeRedColour = nodeRedGetValue();

    if (newColourEv.nodeRedColour.data.pixel.r != 0 || newColourEv.nodeRedColour.data.pixel.g != 0 || newColourEv.nodeRedColour.data.pixel.b != 0)
    {
      return &newColourEv;
    }
  }

  return NO_EVENTS;
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

void joystickButtonCallback(void)
{
  kernel_event_t newTimerEv = { .id = KERNEL_ENTER };
  if (emptySize(&kernelEvsInternalQueue))
  {
    push(&kernelEvsInternalQueue, (void*)&newTimerEv);
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
      case ACC_PORTRAIT_UP:
        ev.id = KERNEL_RIGHT;
        push(&kernelEvsInternalQueue, &ev);
        break;

      case ACC_PORTRAIT_DOWN:
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

void fpsTimerCallback(void)
{
  kernelContext.fpsReady = true;

  kernel_event_t newTimerEv = { .id = KERNEL_FPS };
  if (emptySize(&kernelEvsInternalQueue))
  {
    push(&kernelEvsInternalQueue, (void*)&newTimerEv);
  }
}

/******************************************************************************/
