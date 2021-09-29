/*****************************************************************************************************
* FILENAME :        flash_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the FLASH peripheral.
*
* PUBLIC FUNCTIONS :
*       uint8_t Flash_EraseSector(uint8_t sector, uint8_t num_sectors)
*
**/

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

#define MAX_NUM_SECTOR  8

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn Flash_Erase
 *
 * @brief function to erase the FLASH memory.
 *
 * @param[in] sector is the selected sector of the flash to start the erase, 0xFF means mass erase.
 * @param[in] num_sectors is the number of consecutive sectors of the flash to be erased.
 *
 * @return 0 is sucess.
 *         1 is fail.
 */
uint8_t Flash_Erase(uint8_t sector, uint8_t num_sectors);

#endif
