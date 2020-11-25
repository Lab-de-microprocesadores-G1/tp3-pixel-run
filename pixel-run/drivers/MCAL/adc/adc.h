/*******************************************************************************
  @file     adc.h
  @brief    ADC peripheral driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef MCAL_ADC_ADC_H_
#define MCAL_ADC_ADC_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define INVALID_CONVERSION  0x0

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the conversion completed callback
typedef void (*conversion_callback_t)(int16_t sample);

// Declaring the single-ended resolution configuration
typedef enum {
  ADC_8_BIT_SINGLE_CONV,
  ADC_12_BIT_SINGLE_CONV,
  ADC_10_BIT_SINGLE_CONV,
  ADC_16_BIT_SINGLE_CONV
} adc_single_res_t;

// Declaring the differential resolution configuration
typedef enum {
  ADC_9_BIT_DIFF_CONV,
  ADC_13_BIT_DIFF_CONV,
  ADC_11_BIT_DIFF_CONV,
  ADC_16_BIT_DIFF_CONV
} adc_diff_res_t;

// Declaring the average samples configuration
typedef enum {
  ADC_4_SAMPLES_AVERAGE,
  ADC_8_SAMPLES_AVERAGE,
  ADC_16_SAMPLES_AVERAGE,
  ADC_32_SAMPLES_AVERAGE
} adc_average_samples_t;

// Declaring the ADC configuration structure
typedef struct {
  uint8_t diff            : 1;      // Diff = 1 for differential mode
  uint8_t resolution      : 2;      // adc_single_res_t or adc_diff_res_t (ADC bytes resolution)
  uint8_t usingAverage    : 1;      // usingAverage = 1 for averaging mode
  uint8_t averageSamples  : 2;      //
} adc_cfg_t;

// Declaring the ADC channels configured
typedef enum {
  ADC_POTENTIOMETER,
  ADC_INSTANCE_COUNT
} adc_instance_id_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialises the adc driver
 */
void adcInit(void);

/**
 * @brief Changes the configuration of the adc driver whenever you want
 * @param id        ADC id (which refers to a combination of ADC hardware and Channel)
 * @param config    ADC configuration
 */
void adcConfig(adc_instance_id_t id, adc_cfg_t config);

/*************************
 * NON-BLOCKING SERVICES *
 ************************/

/**
 * @brief Starts the conversion
 * @param id     ADC id (which refers to a combination of ADC hardware and Channel)
 */
bool adcStartConversion(adc_instance_id_t id);

/**
 * @brief Returns whether the conversion is completed or not
 * @param id     ADC id (which refers to a combination of ADC hardware and Channel)
 */
bool adcConversionCompleted(adc_instance_id_t id);

/**
 * @brief Returns the last conversion done on the given channel
 * @param id     ADC id (which refers to a combination of ADC hardware and Channel)
 */
int16_t adcGetConversion(adc_instance_id_t id);

/*********************
 * BLOCKING SERVICES *
 ********************/

/**
 * @brief Returns whether the adc is available for conversion or not
 * @param id     ADC id (which refers to a combination of ADC hardware and Channel)
 */
bool adcAvailable(adc_instance_id_t id);

/**
 * @brief Performs a blocking conversion with the adc
 * @param id     ADC id (which refers to a combination of ADC hardware and Channel)
 */
int16_t adcBlockingConversion(adc_instance_id_t id);

/***************************
 * EVENT-ORIENTED SERVICES *
 **************************/

/**
 * @brief Subscribes to the conversion completed event
 * @param id     ADC id (which refers to a combination of ADC hardware and Channel)
 * @param callback    Callback to be called when the conversion is completed
 */
void adcOnConversion(adc_instance_id_t id, conversion_callback_t callback);

/*******************************************************************************
 ******************************************************************************/


#endif /* MCAL_ADC_ADC_H_ */
