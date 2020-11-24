/*******************************************************************************
  @file     general.h
  @brief    General Library
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef GENERAL_GENERAL_H_
#define GENERAL_GENERAL_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Returns the length of a number, that is the amount of non-zero digits
 * @param number	Number to calculate its length
 */
uint32_t getNumberLength(uint32_t number);

/**
 * @brief Converts the number to its ASCII representation
 * 		number 2 -> character '2'
 * @param number		Number to be converted
 */
uint8_t number2ASCII(uint8_t number);

/**
 * @brief Converts from an array of digits to its number representation
 * 		  array [1, 2, 3, 4] -> number 1234
 * @param arrayNumber   Array
 * @param length        Current length of the number
 */
uint32_t array2Number(uint8_t *arrayNumber, size_t length);

/**
 * @brief Converts a number to an array of digits
 * 		  number 1234 -> array [1, 2, 3, 4]
 * @param number		Number to be converted
 * @param arrayNumber	Array where each digit is saved
 * @param length		Amount of digits
 * @param transform		Callback used to transform each digit if needed, if not used NULL
 */
void number2Array(uint32_t number, uint8_t *arrayNumber, uint8_t length, uint8_t (*transform)(uint8_t));

/*******************************************************************************
 ******************************************************************************/


#endif /* GENERAL_GENERAL_H_ */
