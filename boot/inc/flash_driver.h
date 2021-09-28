/*****************************************************************************************************
* FILENAME :        flash_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the FLASH peripheral.
*
* PUBLIC FUNCTIONS :
*       uint8_t Flash_EraseSector(uint8_t sector)
*
**/

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

#define MAX_NUM_SECTOR  7

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn Flash_EraseSector
 *
 * @brief function to erase a sector of the FLASH.
 *
 * @param[in] sector is the selected sector of the flash to be erased.
 *
 * @return 0 is sucess.
 *         1 is fail.
 */
uint8_t Flash_EraseSector(uint8_t sector);

#endif
