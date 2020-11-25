/*******************************************************************************
  @file     i2c_master.h
  @brief    I2C peripheral driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef MCAL_I2C_I2C_H_
#define MCAL_I2C_I2C_H_

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

// Declaring the callback data type
typedef void (*i2c_callback)(void);

// Declaring I2C instance id's, used to refer to the
// MCU I2C instance used
typedef enum {
  I2C_INSTANCE_0,
  I2C_INSTANCE_1,
  I2C_INSTANCE_2,
  I2C_INSTANCE_COUNT
} i2c_id_t;

// Declaring I2C peripheral state to get from query
typedef enum {
  I2C_STATE_IDLE,			// Idle
  I2C_STATE_IN_PROGRESS,	// Currently transmitting or receiving
  I2C_STATE_FINISHED,		// Already finished communication
  I2C_STATE_ERROR			// Some error occurred
} i2c_state_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initializes the I2C peripheral driver
 * @param id            I2C Instance Id
 * @param baudRate      I2C Baud Rate
 */
void i2cMasterInit(i2c_id_t id, uint32_t baudRate);

/**
 * @brief Starts a transaction on the I2C bus
 * @param id            I2C Instance id
 * @param address       Slave address
 * @param writeBuffer   Data to be sent
 * @param bytesToWrite  Number of bytes to be sent
 * @param readBuffer    Buffer to place read data
 * @param bytesToRead   Number of bytes to be sent
 */
void i2cStartTransaction(i2c_id_t id, uint8_t address, uint8_t* writeBuffer, size_t bytesToWrite, uint8_t* readBuffer, size_t bytesToRead);

/********************
 * POLLING SERVICES *
 ********************/

/**
 * @brief Returns the current status of the transaction
 * @param id            I2C Instance Id
 * @return  Returns i2c_state_t
 */
i2c_state_t i2cQueryTransaction(i2c_id_t id);

/***************************
 * EVENT ORIENTED SERVICES *
 **************************/

/**
 * @brief Registers a callback to be called when the i2c transaction finishes
 * @param id            I2C Instance Id
 * @param callback      Callback being registed
 */
void i2cOnFinished(i2c_id_t id, i2c_callback callback);

/** 
 * @brief Registers a callback to be called when an error occurrs during a i2c transaction
 * @param id            I2C Instance Id
 * @param callback      Callback being registered
 */
void i2cOnError(i2c_id_t id, i2c_callback callback);

/*******************************************************************************
 ******************************************************************************/

#endif /* MCAL_I2C_I2C_H_ */
