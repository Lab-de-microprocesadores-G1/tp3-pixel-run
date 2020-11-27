/*******************************************************************************
  @file     i2c_master.c
  @brief    I2C peripheral driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "i2c_master.h"
#include "MK64F12.h"
#include "hardware.h"
#include "drivers/MCAL/gpio/gpio.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define I2C_BUS_CLOCK	50000000U

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring I2C instance data structure
typedef struct {
  /* General data */
  i2c_state_t   currentState;     // Current state of the I2C instance
  bool          alreadyInit;      // Whether the instance has already been initialized or not
  
  /* Transaction baud rate */
  uint32_t      baudrate;
  
  /* Transaction user data */
  uint8_t*      writeBuffer;
  uint8_t*      readBuffer;
  size_t        bytesToWrite;
  size_t        bytesToRead;
  uint32_t      address;

  /* Transaction control data */
  uint8_t       bytesWritten;     // Bytes already written by the master
  uint8_t       bytesRead;        // Bytes already read from the slave
  bool          isReadMode;  	  // True when currently in read mode

  /* Event's callbacks */
  i2c_callback  onError;
  i2c_callback  onFinished;
} i2c_instance_t;

// Declaring the baud rate configuration structure
typedef struct {
  uint8_t mul; 	// Multiplier Factor, register value
  uint8_t icr;	// ClockRate, register value

  uint32_t targetBaudRate;	// The target baud rate with the given register values
} baudrate_cfg_t;

// Declaring I2C peripheral pin out
enum {
	I2C_SCL,
	I2C_SDA,
	I2C_PINOUT_COUNT
};

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Computes what value should be used in the frequency register of the I2C
 * peripheral to get the desired baud rate passed as argument.
 */
static baudrate_cfg_t computeBaudRateSettings(uint32_t desiredBaudRate);

/**
 * @brief Interrupt service routines, handler and dispatcher
 */
void      I2C_IRQDispatcher(i2c_id_t id);
__ISR__   I2C0_IRQHandler(void);
__ISR__   I2C1_IRQHandler(void);
__ISR__   I2C2_IRQHandler(void);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// I2C memory map pointer and context instances used to control the peripheral
static uint8_t 			    i2cIrqs[] = I2C_IRQS;
static I2C_Type * 		  i2cPointers[] = I2C_BASE_PTRS;
static i2c_instance_t 	i2cInstances[I2C_INSTANCE_COUNT] = {
  { I2C_STATE_IDLE },
  { I2C_STATE_IDLE },
  { I2C_STATE_IDLE }
};

// Look-up table for the clock divider, nice. :'|
static uint32_t ICR2SCLDivider[] = {
	20, 22, 24, 26, 28, 30, 34, 40, 28, 32, 36, 40, 44, 48, 56,
	68, 48, 56, 64, 72, 80, 88, 104, 128, 80, 96, 112, 128, 144,
	160, 192, 240, 160, 192, 224, 256, 288, 320, 384, 480, 320,
	384, 448, 512, 576, 640, 768, 960, 640, 768, 896, 1024, 1152,
	1280, 1536, 1920, 1280, 1536, 1792, 2048, 2304, 2560, 3072, 3840
};

// Look-up table for the signal routing
static uint8_t 	i2cPinout[I2C_INSTANCE_COUNT][I2C_PINOUT_COUNT] = {
  //   	I2C_SCL			  		I2C_SDA
  {		PORTNUM2PIN(PE, 24)	, 	PORTNUM2PIN(PE, 25) 		},	// I2C INSTANCE 0
  { 	PORTNUM2PIN(PC, 10)	, 	PORTNUM2PIN(PC, 11)			},	// I2C INSTANCE 1
  { 	PORTNUM2PIN(PA, 14)	, 	PORTNUM2PIN(PA, 13)			}		// I2C INSTANCE 2
};

// Look-up table for the pin alternative
static uint8_t i2cAlternative[I2C_INSTANCE_COUNT][I2C_PINOUT_COUNT] = {
  //   	I2C_SCL	 I2C_SDA
  {		5					,	5			},	// I2C INSTANCE 0
  { 	2					,	2			},	// I2C INSTANCE 1
  { 	5					,	5			}		// I2C INSTANCE 2
};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void i2cMasterInit(i2c_id_t id, uint32_t baudRate)
{
  if (!i2cInstances[id].alreadyInit)
  {
    PORT_Type* ports[] = PORT_BASE_PTRS;

    // Clock gating of the I2C peripheral
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK | SIM_SCGC4_I2C1_MASK; 
    SIM->SCGC1 |= SIM_SCGC1_I2C2_MASK;

    // Clock gating of the PORT peripheral
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    // Enabling the PCR alternative for the pin out selected in the instance and Open Drain output
    for (uint8_t i = 0 ; i < I2C_PINOUT_COUNT ; i++)
    {
      ports[PIN2PORT(i2cPinout[id][i])]->PCR[PIN2NUM(i2cPinout[id][i])] = PORT_PCR_MUX(i2cAlternative[id][i]) | PORT_PCR_ODE(1);
    }

    // Configure the frequency of the peripheral to get the desired baud rate
    baudrate_cfg_t setting = computeBaudRateSettings(baudRate);
    i2cPointers[id]->F = I2C_F_MULT(setting.mul) | I2C_F_ICR(setting.icr);

    // Enable the NVIC for the I2C peripheral
    NVIC_EnableIRQ(i2cIrqs[id]);

    // Enable I2C module and interrupts
    i2cPointers[id]->S = I2C_S_TCF_MASK | I2C_S_IICIF_MASK;
    i2cPointers[id]->C1 = I2C_C1_IICEN(1) | I2C_C1_IICIE(1);

    // Raise the already initialized flag
    i2cInstances[id].alreadyInit = true;
  }
}

void i2cStartTransaction(i2c_id_t id, uint8_t address, uint8_t* writeBuffer, size_t bytesToWrite, uint8_t* readBuffer, size_t bytesToRead)
{
  if ((bytesToWrite && writeBuffer) || (bytesToRead && readBuffer))
  {  // Clear the current status of the I2C instance
	  i2cInstances[id].currentState = I2C_STATE_IN_PROGRESS;

	  // Initializing transaction variables.
	  i2cInstances[id].writeBuffer = writeBuffer;
	  i2cInstances[id].readBuffer = readBuffer;
	  i2cInstances[id].bytesToWrite = bytesToWrite;
	  i2cInstances[id].bytesToRead = bytesToRead;
	  i2cInstances[id].address = address;
	  i2cInstances[id].bytesWritten = 0;
	  i2cInstances[id].bytesRead = 0;
	  i2cInstances[id].isReadMode = bytesToWrite == 0 && bytesToRead > 0;

	  // Sets the transfer direction to transmit, and starts the communication, taking control
	  // of the I2C bus as Master, sending the address + R/W to check if slave found
	  i2cPointers[id]->C1 = (i2cPointers[id]->C1 & ~I2C_C1_TXAK_MASK) | I2C_C1_TXAK(0);
	  i2cPointers[id]->C1 = (i2cPointers[id]->C1 & ~I2C_C1_TX_MASK) | I2C_C1_TX(1);
	  i2cPointers[id]->C1 = (i2cPointers[id]->C1 & ~I2C_C1_MST_MASK) | I2C_C1_MST(1);
	  i2cPointers[id]->D = ((address & 0x0000007F) << 1) | 0x00000000;
  }
  else
  {
    i2cInstances[id].currentState = I2C_STATE_ERROR;
  }
}

i2c_state_t i2cQueryTransaction(i2c_id_t id)
{
  i2c_state_t result = i2cInstances[id].currentState;
  if (result == I2C_STATE_FINISHED || result == I2C_STATE_ERROR)
  {
    i2cInstances[id].currentState = I2C_STATE_IDLE;
  }
  return result;
}

void i2cOnFinished(i2c_id_t id, i2c_callback callback)
{
  i2cInstances[id].onFinished = callback;
}

void i2cOnError(i2c_id_t id, i2c_callback callback)
{
  i2cInstances[id].onError = callback;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static baudrate_cfg_t computeBaudRateSettings(uint32_t desiredBaudRate)
{
  baudrate_cfg_t setting = {.mul = 0, .icr = 0};
  uint32_t bestBaudRate = 0;
  uint32_t bestError = 0;
  uint32_t currentBaudRate = 0;
  uint32_t currentError = 0;

  for(uint8_t mul = 1 ; mul <= 4 ; mul = mul << 1)
  {
	for(uint8_t icr = 0 ; icr <= 0x3F ; icr++)
	{
	  currentBaudRate = I2C_BUS_CLOCK / (mul * ICR2SCLDivider[icr]);
	  currentError = currentBaudRate > desiredBaudRate ? currentBaudRate - desiredBaudRate : desiredBaudRate - currentBaudRate;
	  if (bestBaudRate == 0 || bestError > currentError)
	  {
		  bestBaudRate = currentBaudRate;
		  bestError = currentError;
		  setting.mul = mul;
		  setting.icr = icr;
		  setting.targetBaudRate = currentBaudRate;
	  }
	}
  }

  return setting;
}

/*******************************************************************************
 *******************************************************************************
						            INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

void I2C_IRQDispatcher(i2c_id_t id)
{
  i2c_instance_t* instance = &(i2cInstances[id]);
  I2C_Type* pointer = i2cPointers[id];

  // Save current state of flags and clear interrupt flags
  uint8_t status = pointer->S;
  pointer->S = I2C_S_TCF_MASK | I2C_S_IICIF_MASK;

  // If transfer completed
  if (status & I2C_S_TCF_MASK)
  {
    // If transmission mode
    if (pointer->C1 & I2C_C1_TX_MASK)
    {
      if (instance->bytesWritten == instance->bytesToWrite)
      {
        // If was the last byte, verify if has to read
        if (instance->bytesToRead)
        {
          // Verify if the slave answered with ACK
          if (pointer->S & I2C_S_RXAK_MASK)
          {
            // An error occurred and the slave didn't answer, so the transaction
            // is stopped and the status of the instance is changed.
        	  // Release the I2C bus.
            instance->currentState = I2C_STATE_ERROR;
            pointer->C1 = (pointer->C1 & ~I2C_C1_MST_MASK) | I2C_C1_MST(0);
            if (instance->onError)
            {
              instance->onError();
            }
          }
          else
          {
            if (instance->isReadMode)
            {
              // Execution reaches this point when a transfer was completed, with transmitter
              // enabled, and the transfer was the repeated start changing the direction
              // of the data bus to read the slave, start reading the first byte
              pointer->C1 = (pointer->C1 & ~I2C_C1_TX_MASK) | I2C_C1_TX(0);
              if (instance->bytesToRead == 1)
              {
                pointer->C1 = (pointer->C1 & ~I2C_C1_TXAK_MASK) | I2C_C1_TXAK(1);
              }
              (void)pointer->D;   // Dummy read
            }
            else
            {
              // Transmission finished and there are some bytes to be read,
              // so the master generates a repeated start to change the data direction
              // of the bus, without releasing it...
              pointer->C1 = (pointer->C1 & ~I2C_C1_RSTA_MASK) | I2C_C1_RSTA(1);
              pointer->D = ((instance->address & 0x0000007F) << 1) | 0x00000001;
              instance->isReadMode = true;
            }
          }
        }
        else
        {
          // There were no bytes to read, so the master releases the I2C bus
          // and the communication stops, the instance status is updated
          instance->currentState = I2C_STATE_FINISHED;
          pointer->C1 = (pointer->C1 & ~I2C_C1_MST_MASK) | I2C_C1_MST(0);
          if (instance->onFinished)
          {
            instance->onFinished();
          }
        }
      }
      else if (instance->bytesWritten < instance->bytesToWrite)
      {
        // Verify if the slave answered with ACK
        if (pointer->S & I2C_S_RXAK_MASK)
        {
          // An error occurred and the slave didn't answer, so the transaction
          // is stopped and the status of the instance is changed
          instance->currentState = I2C_STATE_ERROR;
          pointer->C1 = (pointer->C1 & ~I2C_C1_MST_MASK) | I2C_C1_MST(0);
          if (instance->onError)
          {
            instance->onError();
          }
        }
        else
        {
          // Keep sending bytes
          pointer->D = instance->writeBuffer[instance->bytesWritten++];
        }
      }
    }
    else
    {
      if (instance->bytesRead == instance->bytesToRead - 1)
      {
        // The current byte received is the last one, so after
        // reading it from the peripheral, the master releases the bus.
        // Also the current state of the instance is updated.
        pointer->C1 = (pointer->C1 & ~I2C_C1_MST_MASK) | I2C_C1_MST(0);
        instance->readBuffer[instance->bytesRead++] = pointer->D;
        instance->currentState = I2C_STATE_FINISHED;
        if (instance->onFinished)
        {
          instance->onFinished();
        }
      }
      else if (instance->bytesRead < instance->bytesToRead - 1)
      {
        // If this is the 2nd to last byte, set the ACK to stop slave from
        // continue sending data frames, and then read the current data
        if (instance->bytesRead == instance->bytesToRead - 2)
        {
          pointer->C1 = (pointer->C1 & ~I2C_C1_TXAK_MASK) | I2C_C1_TXAK(1);
        }
        instance->readBuffer[instance->bytesRead++] = pointer->D;
      }
    }
  }
}

__ISR__ I2C0_IRQHandler(void)
{
  I2C_IRQDispatcher(I2C_INSTANCE_0);
}

__ISR__ I2C1_IRQHandler(void)
{
  I2C_IRQDispatcher(I2C_INSTANCE_1);
}

__ISR__ I2C2_IRQHandler(void)
{
  I2C_IRQDispatcher(I2C_INSTANCE_2);
}

/******************************************************************************/
