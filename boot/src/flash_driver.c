/*****************************************************************************************************
* FILENAME :        flash_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the FLASH peripheral.
*
* PUBLIC FUNCTIONS :
*       uint8_t Flash_EraseSector(uint8_t sector)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include "flash_driver.h"

/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

uint8_t Flash_EraseSector(uint8_t sector){

    if(sector >= MAX_NUM_SECTOR){
        return 1;
    }

    /* Unlock Flash Control Register */
    FLASHINTR->KEYR = 0x45670123;
    FLASHINTR->KEYR = 0xCDEF89AB;

    /* Check no flash memory operation is ongoing */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /* Set Sector Erase bit in Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_SER);
    /* Select the sector to be erased in the SNB bit in Flash Control Register */
    FLASHINTR->CR &= ~(0x0F << FLASH_CR_SNB); /* Clear the SNB bits */
    FLASHINTR->CR |= ((0x07 & sector) << FLASH_CR_SNB);
    /* Set the STRT bit in Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_STRT);

    /* Wait for flash memory operation is finished */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /* Lock Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_LOCK);

    return 0;
}
