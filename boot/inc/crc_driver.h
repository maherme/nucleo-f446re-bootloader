/*****************************************************************************************************
* FILENAME :        crc_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the CRC peripheral.
*
* PUBLIC FUNCTIONS :
*       void        CRC_Init(void)
*       void        CRC_DeInit(void)
*       uint32_t    CRC_Accumulate(uint32_t* pBuffer, uint32_t length)
*       uint32_t    CRC_Calculate(uint32_t* pBuffer, uint32_t length)
*
**/

#ifndef CRC_DRIVER_H
#define CRC_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn CRC_Init
 *
 * @brief function to initialize CRC peripheral.
 *
 * @return void
 */
void CRC_Init(void);

/**
 * @fn CRC_DeInit
 *
 * @brief function to reset all registers of CRC peripheral.
 *
 * @return void
 */
void CRC_DeInit(void);

/**
 * @fn CRC_Accumulate
 *
 * @brief function to calculate the 32bit CRC of 32bit data buffer using the previous CRC value
 *        and the new one.
 *
 * @param[in] pBuffer is the pointer to the buffer containing the data to be computed.
 * @param[in] length is the length of the buffer to be computed.
 *
 * @return void
 */
uint32_t CRC_Accumulate(uint32_t* pBuffer, uint32_t length);

/**
 * @fn CRC_Calculate
 *
 * @brief function to calculate the 32 bit CRC of 32 bit data buffer independently of the
 *        previous CRC value.
 *
 * @param[in] pBuffer is the pointer to the buffer containing the data to be computed.
 * @param[in] length is the length of the buffer to be computed.
 *
 * @return void
 */
uint32_t CRC_Calculate(uint32_t* pBuffer, uint32_t length);

#endif
