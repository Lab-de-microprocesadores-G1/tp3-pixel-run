/***************************************************************************//**
  @file     adc.c
  @brief    ADC peripheral driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdio.h>
#include "adc.h"
#include "board.h"
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// IMPORTANT!
// The driver does not use the ping pong functionality of the adc peripheral,
// by default it always uses one harcoded SC1 register
#define SC1_REG_DEFAULT     0

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the ADC instance id's
typedef enum {
  ADC_0,
  ADC_1,
  ADC_COUNT
} adc_id_t;

// Declaring the adc instance data structure
typedef struct {  
  const pin_t           pin;                      // Pin used
  const uint8_t         alt;                      // Pin PCR alternative ued
  const uint8_t         adcId;                    // ADC Peripheral used
  const uint8_t         channel;                  // ADC Channel used
  adc_cfg_t             config;                   // Instance configuration
  int16_t              lastConversion;           // Last conversion done on this instance
  conversion_callback_t onConversionCompleted;    // Callback on conversion completed
  bool                  conversionCompleted;      // Flag of conversion completed
} adc_instance_t; 

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

// ISR handler for each ADC
void ADC0_IRQHandler(void);
void ADC1_IRQHandler(void);

// Dispatcher for IRQs, in callback mode, calls the onConversionCompleted of id.
static void adcIRQDispatcher(adc_instance_id_t id);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static adc_instance_t adcInstances[] = {
    //    PIN      ALT  ADC_ID  ADC_CHANNEL
    { 	PIN_ADC_0, 0,	ADC_0, 		1  	}  // ADC_INSTANCE_0
};

// ADC registers pointers
static ADC_Type * adcPointers[] = ADC_BASE_PTRS;

// Current channel being used in each ADC
static adc_instance_id_t ADC0CurrentID;
static adc_instance_id_t ADC1CurrentID;

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
*******************************************************************************
******************************************************************************/
void adcInit()
{
  // Clock gating for ADC peripherals 
  SIM->SCGC6 |= SIM_SCGC6_ADC0(1);
  SIM->SCGC3 |= SIM_SCGC3_ADC1(1);
  
  // Clock gating of the PORT peripheral
  SIM->SCGC5 |= SIM_SCGC5_PORTA(1);
  SIM->SCGC5 |= SIM_SCGC5_PORTB(1);
  SIM->SCGC5 |= SIM_SCGC5_PORTC(1);
  SIM->SCGC5 |= SIM_SCGC5_PORTD(1);
  SIM->SCGC5 |= SIM_SCGC5_PORTE(1);

  // Set PCR alternative for ADC instances
  for(uint8_t i = 0 ; i < ADC_INSTANCE_COUNT ; i++)
  {
    PORT_Type * ports[] = PORT_BASE_PTRS;
    const pin_t pin = adcInstances[i].pin;
    const uint8_t alt = adcInstances[i].alt;
    
    ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = PORT_PCR_MUX(alt);
  }
  
  // Enable interrupts on NVIC
  NVIC_EnableIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC1_IRQn);
}

void adcConfig(adc_instance_id_t id, adc_cfg_t config)
{
  if (id < ADC_INSTANCE_COUNT)
  {
    adcInstances[id].config = config;
  }
}

bool adcStartConversion(adc_instance_id_t id)
{
  adc_instance_t instance = adcInstances[id];
  ADC_Type* pointer = adcPointers[instance.adcId];
  adc_cfg_t config = instance.config;
  bool available = adcAvailable(id);

  if (available)
  {
    // Resolution configuration
    pointer->CFG1 = (pointer->CFG1 & ~ADC_CFG1_MODE_MASK) | ADC_CFG1_MODE(config.resolution);

    // Averaging configuration
    pointer->SC3 = ADC_SC3_AVGE(config.usingAverage) | ADC_SC3_AVGS(config.averageSamples);

    // Mux select which SC1 to use
    pointer->CFG2 = (pointer->CFG2 & ~ADC_CFG2_MUXSEL_MASK) | ADC_CFG2_MUXSEL(SC1_REG_DEFAULT);

    // Write to SC1 starts conversion
    pointer->SC1[SC1_REG_DEFAULT] = ADC_SC1_AIEN(1) | ADC_SC1_DIFF(config.diff) | ADC_SC1_ADCH(adcInstances[id].channel);

    // Set current ID being used
    if (instance.adcId == ADC_0)
    {
      ADC0CurrentID = id;
    }
    else
    {
      ADC1CurrentID = id;
    }
    
    // Clear conversion completed flag if set
    instance.conversionCompleted = false;
  }

  return available;
}

bool adcConversionCompleted(adc_instance_id_t id)
{
  adc_instance_t* instance = &(adcInstances[id]);
  bool result = instance->conversionCompleted;
  if (result)
  {
    instance->conversionCompleted = false;
  }
  return result;
}

int16_t adcGetConversion(adc_instance_id_t id)
{
  return adcInstances[id].lastConversion;
}

bool adcAvailable(adc_instance_id_t id)
{
  adc_instance_t* instance = &(adcInstances[id]);
  ADC_Type* pointer = adcPointers[instance->adcId];
  return (pointer->SC2 & ADC_SC2_ADACT_MASK) == 0;
}

int16_t adcBlockingConversion(adc_instance_id_t id)
{
  if (adcStartConversion(id))
  {
    while(!adcConversionCompleted(id));
    return adcGetConversion(id);
  }
  return INVALID_CONVERSION;
}

void adcOnConversion(adc_instance_id_t id, conversion_callback_t callback)
{
  adcInstances[id].onConversionCompleted = callback;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
*******************************************************************************
******************************************************************************/

void adcIRQDispatcher(adc_instance_id_t id)
{
  adc_id_t adc = adcInstances[id].adcId;
  adcInstances[id].lastConversion = adcPointers[adc]->R[SC1_REG_DEFAULT];
  
  if (adcInstances[id].onConversionCompleted == NULL)
  {
    adcInstances[id].conversionCompleted = true;
  }
  else
  {
    adcInstances[id].onConversionCompleted(adcInstances[id].lastConversion);
  }
}

/*******************************************************************************
 *******************************************************************************
                        INTERRUPT SERVICE ROUTINES
*******************************************************************************
******************************************************************************/

void ADC0_IRQHandler(void)
{
  adcIRQDispatcher(ADC0CurrentID);
}

void ADC1_IRQHandler(void)
{
  adcIRQDispatcher(ADC1CurrentID);
}


/******************************************************************************/
