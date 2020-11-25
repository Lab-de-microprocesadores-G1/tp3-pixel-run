/*******************************************************************************
  @file     FXOS8700.h
  @brief    FXOS8700 accelerometer driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef HAL_FXOS8700_FXOS8700_H_
#define HAL_FXOS8700_FXOS8700_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring accelerometer callback
typedef void (*fxos_callback_t)(void);

// For portrait-landscape detection, z-lockout angle determines the limit angle
// required to detect the orientation.
typedef enum {
  FXOS_ZLOCK_13_DEG,
  FXOS_ZLOCK_17_DEG,
  FXOS_ZLOCK_20_DEG,
  FXOS_ZLOCK_24_DEG,
  FXOS_ZLOCK_28_DEG,
  FXOS_ZLOCK_32_DEG,
  FXOS_ZLOCK_36_DEG,
  FXOS_ZLOCK_40_DEG,
} fxos_zlock_t;

// The back-front trip angle determines when the orientation changes from back to front
// or vice versa.
typedef enum {			  // Back → front transition	Front → back transition
  FXOS_BKFR_THRESHOLD_0, // Z < 80° or Z > 280°		Z > 100° and Z < 260°
  FXOS_BKFR_THRESHOLD_1, // Z < 75° or Z > 285°		Z > 105° and Z < 255°
  FXOS_BKFR_THRESHOLD_2, // Z < 70° or Z > 290°		Z > 110° and Z < 250°
  FXOS_BKFR_THRESHOLD_3  // Z < 65° or Z > 295°		Z > 115° and Z < 245°
} fxos_bkfr_ths_t;

// The landscape-portrait trip angle determines when the orientation changes from
// landscape to portrait or vice versa.
typedef enum {
  FXOS_THRESHOLD_15_DEG = 0x07,
  FXOS_THRESHOLD_20_DEG = 0x09,
  FXOS_THRESHOLD_30_DEG = 0x0C,
  FXOS_THRESHOLD_35_DEG = 0x0D,
  FXOS_THRESHOLD_40_DEG = 0x0F,
  FXOS_THRESHOLD_45_DEG = 0x10,
  FXOS_THRESHOLD_55_DEG = 0x13,
  FXOS_THRESHOLD_60_DEG = 0x14,
  FXOS_THRESHOLD_70_DEG = 0x17,
  FXOS_THRESHOLD_75_DEG = 0x19,
} fxos_threshold_t;

// Add hysteresis to the portrait-landscape orientation change.
typedef enum {
  FXOS_HYSTERESIS_4_DEG,
  FXOS_HYSTERESIS_7_DEG,
  FXOS_HYSTERESIS_11_DEG,
  FXOS_HYSTERESIS_14_DEG,
  FXOS_HYSTERESIS_17_DEG,
  FXOS_HYSTERESIS_21_DEG,
  FXOS_HYSTERESIS_24_DEG
} fxos_hysteresis_t;

// Declaring landscape and portrait orientations
typedef enum {
  ACC_PORTRAIT_UP,
  ACC_PORTRAIT_DOWN,
  ACC_LANDSCAPE_RIGHT,
  ACC_LANDSCAPE_LEFT,
} acc_lp_orientations_t;

// Declaring back and front orientations
typedef enum {
  ACC_FRONT,
  ACC_BACK
} acc_bf_orientations_t;

// Declaring the acceleration data structure
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} acc_vector_t;

// Declaring the orientation data structure
typedef struct {
  uint8_t   landscapePortrait   : 2;    // Returns a acc_lp_orientations_t value
  uint8_t   backFront           : 1;    // Returns a acc_bf_orientations_t value
} acc_orientation_t;

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define FXOS_ZLOCK_DEFAULT				FXOS_ZLOCK_40_DEG
#define FXOS_BKFR_DEFAULT				FXOS_BKFR_THRESHOLD_0
#define FXOS_THRESHOLD_DEFAULT			FXOS_THRESHOLD_15_DEG
#define FXOS_HYSTERESIS_DEFAULT			FXOS_HYSTERESIS_7_DEG
#define FXOS_INIT_DEFAULT				FXOS_ZLOCK_DEFAULT, FXOS_BKFR_DEFAULT, FXOS_THRESHOLD_DEFAULT, FXOS_HYSTERESIS_DEFAULT

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*
 * @brief Initialized the FXOS8700 accelerometer
 * @returns True if successful init
 */
bool FXOSInit(fxos_zlock_t zlock, fxos_bkfr_ths_t bkfr, fxos_threshold_t ths, fxos_hysteresis_t hyst);

/*
 * @brief Returns whether the driver is running
 */
bool FXOSIsRunning(void);

/**
 * @brief Returns when a new measurement is available
 */
bool FXOSMeasurementAvailable(void);

/**
 * @brief Returns whether the orientation has changed or not
 */
bool FXOSOrientationChanged(void);

/*
 * @brief Registers a callback to be called when the orientation has changed
 * @param callback 		Callback to be registered
 */
void FXOSSubscribeOrientationChanged(fxos_callback_t callback);

/*
 * @brief Instantaneous acceleration vector getter
 * @param coord         Pointer for acceleration in x, y, z axes
 * @returns status, false on error
 */
bool FXOSGetAcceleration(acc_vector_t* vector);

/*
 * @brief Instantaneous orientation getter
 * @param orientation   Pointer for orientation
 * @returns status, false on error 
 */
bool FXOSGetOrientation(acc_orientation_t* orientation);

/*******************************************************************************
 ******************************************************************************/

#endif /* HAL_FXOS8700_FXOS8700_H_ */
