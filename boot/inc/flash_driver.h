/*****************************************************************************************************
* FILENAME :        flash_driver.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the APIs for configuring the FLASH peripheral.
*
* PUBLIC FUNCTIONS :
*       uint8_t Flash_Erase(uint8_t sector, uint8_t num_sectors)
*       uint8_t Flash_WriteMemoryByte(uint32_t address, uint8_t data)
*       uint8_t Flash_WriteMemoryHalfWord(uint32_t address, uint16_t data)
*       uint8_t Flash_WriteMemoryWord(uint32_t address, uint32_t data)
*       uint8_t Flash_WriteMemoryDoubleWord(uint32_t address, uint64_t data)
*       void    Flash_Unlock(void)
*       void    Flash_Lock(void)
*       void    Flash_SetPSIZE(flash_psize_t psize)
*
**/

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stdint.h>
#include "stm32f446xx.h"

#define MAX_NUM_SECTOR          8

/**
 * List of allowed PSIZE values for FLASH CR register.
 */
typedef enum{
    FLASH_PSIZE_BYTE        = 0x00, /* Supply voltage from 1.7V to 3.6V */
    FLASH_PSIZE_HALFWORD    = 0x01, /* Supply voltage from 2.1V to 3.6V */
    FLASH_PSIZE_WORD        = 0x02, /* Supply voltage from 2.7V to 3.6V */
    FLASH_PSIZE_DOUBLEWORD  = 0x03  /* Supply voltage from 2.7V to 3.6V with external Vpp 8V to 9V */
}flash_psize_t;

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn Flash_EraseSector
 *
 * @brief function to erase a sector of the FLASH.
 *
 * @param[in] sector is the selected sector of the flash to be erased, 0xFF means mass erase.
 *
 * @return 0 is sucess.
 *         1 is fail.
 */
uint8_t Flash_EraseSector(uint8_t sector);

/**
 * @fn Flash_WriteMemoryByte
 *
 * @brief function to program a byte in a flash memory address.
 *
 * @param[in] address is the memory address to be programmed.
 * @param[in] data is the data to be stored in address.
 *
 * @return 0 is sucess.
 *         1 is fail.
 *
 * @note this function must be used when the device voltage range is from 1.7V to 3.6V.
 */
uint8_t Flash_WriteMemoryByte(uint32_t address, uint8_t data);

/**
 * @fn Flash_WriteMemoryHalfWord
 *
 * @brief function to program a half word in a flash memory address.
 *
 * @param[in] address is the memory address to be programmed.
 * @param[in] data is the data to be stored in address.
 *
 * @return 0 is sucess.
 *         1 is fail.
 *
 * @note this function must be used when the device voltage range is from 2.1V to 3.6V.
 */
uint8_t Flash_WriteMemoryHalfWord(uint32_t address, uint16_t data);

/**
 * @fn Flash_WriteMemoryWord
 *
 * @brief function to program a word in a flash memory address.
 *
 * @param[in] address is the memory address to be programmed.
 * @param[in] data is the data to be stored in address.
 *
 * @return 0 is sucess.
 *         1 is fail.
 *
 * @note this function must be used when the device voltage range is from 2.7V to 3.6V.
 */
uint8_t Flash_WriteMemoryWord(uint32_t address, uint32_t data);

/**
 * @fn Flash_WriteMemoryDoubleWord
 *
 * @brief function to program a double word in a flash memory address.
 *
 * @param[in] address is the memory address to be programmed.
 * @param[in] data is the data to be stored in address.
 *
 * @return 0 is sucess.
 *         1 is fail.
 *
 * @note this function must be used when the device voltage range is from 2.7V to 3.6V
 *       with external Vpp in the range 8V to 9V.
 */
uint8_t Flash_WriteMemoryDoubleWord(uint32_t address, uint64_t data);

/**
 * @fn Flash_Unlock
 *
 * @brief function to unlock the flash programming/erase protection.
 *
 * @return void.
 */
void Flash_Unlock(void);

/**
 * @fn Flash_Lock
 *
 * @brief function to lock up the flash programming/erase protection.
 *
 * @return void.
 */
void Flash_Lock(void);

/**
 * @fn Flash_SetPSIZE
 *
 * @brief function to set the PSIZE according to the supply voltage.
 *
 * @param[in] psize is the value to store in the PSIZE bit in FLASH CR.
 *
 * @return void.
 *
 * @note: be aware that the PSIZE need to be set according to the supply voltage range.
 */
void Flash_SetPSIZE(flash_psize_t psize);

/**
 * @fn Flash_Busy
 *
 * @brief function to check if a flash operation is ongoing.
 *
 * @return 0 is Flash is not busy.
 *         1 if Flash is busy.
 */
uint8_t Flash_Busy(void);

#endif