/*****************************************************************************************************
* FILENAME :        flash_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the FLASH peripheral.
*
* PUBLIC FUNCTIONS :
*       uint8_t Flash_Erase(uint8_t sector, uint8_t num_sectors)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include "flash_driver.h"

/*****************************************************************************************************/
/*                                       Static Function Prototypes                                  */
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


/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

uint8_t Flash_Erase(uint8_t sector, uint8_t num_sectors){

    uint8_t ret = 1;
    uint8_t i = 0;

    if((num_sectors == 0) || ((sector != 0xFF) && ((sector + num_sectors) > MAX_NUM_SECTOR))){
        return 1;
    }

    if(sector == 0xFF){
        ret = Flash_EraseSector(0xFF);
    }
    else{
        for(i = sector; i < (sector + num_sectors); i++){
            ret = Flash_EraseSector(i);
            if(ret != 0){
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

static uint8_t Flash_EraseSector(uint8_t sector){

    /* Unlock Flash Control Register */
    FLASHINTR->KEYR = 0x45670123;
    FLASHINTR->KEYR = 0xCDEF89AB;

    /* Check no flash memory operation is ongoing */
    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    if(sector != 0xFF){
        /* Set Sector Erase bit in Flash Control Register */
        FLASHINTR->CR |= (1 << FLASH_CR_SER);
        /* Select the sector to be erased in the SNB bit in Flash Control Register */
        FLASHINTR->CR &= ~(0x0F << FLASH_CR_SNB); /* Clear the SNB bits */
        FLASHINTR->CR |= ((0x07 & sector) << FLASH_CR_SNB);
    }
    else{
        /* Set Mass Erase bit in Flash Control Register */
        FLASHINTR->CR |= (1 << FLASH_CR_MER);
    }
    /* Set the STRT bit in Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_STRT);

    /* Wait for flash memory operation is finished */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /* Lock Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_LOCK);

    return 0;
}
