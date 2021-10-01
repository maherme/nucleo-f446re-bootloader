/*****************************************************************************************************
* FILENAME :        flash_driver.c
*
* DESCRIPTION :
*       File containing the APIs for configuring the FLASH peripheral.
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
*       uint8_t Flash_Busy(void)
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

    /* Check no flash memory operation is ongoing */
    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    /* Unlock Flash to perform erase operation */
    Flash_Unlock();

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
    Flash_Lock();

    return 0;
}

uint8_t Flash_WriteMemoryByte(uint32_t address, uint8_t data){

    /* check no flash memory operation is ongoing */
    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    /* Set PSIZE bits in the Flash Control Register */
    Flash_SetPSIZE(FLASH_PSIZE_BYTE);

    /* Set Programming bit in the Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_PG);

    /* Write data in flash memory */
    *(uint8_t*)address = data;

    /* Wait for flash memory operation is finished */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /*Check for any error */
    if(FLASHINTR->SR & ((1 << FLASH_SR_PGSERR) | (1 << FLASH_SR_PGPERR) | 
      (1 << FLASH_SR_PGAERR) | (1 << FLASH_SR_WRPERR))){
        return 1;
    }

    return 0;
}

uint8_t Flash_WriteMemoryHalfWord(uint32_t address, uint16_t data){

    /* check no flash memory operation is ongoing */
    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    /* Set PSIZE bits in the Flash Control Register */
    Flash_SetPSIZE(FLASH_PSIZE_HALFWORD);

    /* Set Programming bit in the Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_PG);

    /* Write data in flash memory */
    *(uint16_t*)address = data;

    /* Wait for flash memory operation is finished */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /*Check for any error */
    if(FLASHINTR->SR & ((1 << FLASH_SR_PGSERR) | (1 << FLASH_SR_PGPERR) | 
      (1 << FLASH_SR_PGAERR) | (1 << FLASH_SR_WRPERR))){
        return 1;
    }

    return 0;
}

uint8_t Flash_WriteMemoryWord(uint32_t address, uint32_t data){

    /* check no flash memory operation is ongoing */
    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    /* Set PSIZE bits in the Flash Control Register */
    Flash_SetPSIZE(FLASH_PSIZE_WORD);

    /* Set Programming bit in the Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_PG);

    /* Write data in flash memory */
    *(uint32_t*)address = data;

    /* Wait for flash memory operation is finished */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /*Check for any error */
    if(FLASHINTR->SR & ((1 << FLASH_SR_PGSERR) | (1 << FLASH_SR_PGPERR) | 
      (1 << FLASH_SR_PGAERR) | (1 << FLASH_SR_WRPERR))){
        return 1;
    }

    return 0;
}

uint8_t Flash_WriteMemoryDoubleWord(uint32_t address, uint64_t data){

    /* check no flash memory operation is ongoing */
    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    /* Set PSIZE bits in the Flash Control Register */
    Flash_SetPSIZE(FLASH_PSIZE_DOUBLEWORD);

    /* Set Programming bit in the Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_PG);

    /* Write data in flash memory */
    *(uint32_t*)address = (uint32_t)data;
    *(uint32_t*)(address + 4) = (uint32_t)(data >> 32);

    /* Wait for flash memory operation is finished */
    while(FLASHINTR->SR & (1 << FLASH_SR_BSY));

    /*Check for any error */
    if(FLASHINTR->SR & ((1 << FLASH_SR_PGSERR) | (1 << FLASH_SR_PGPERR) | 
      (1 << FLASH_SR_PGAERR) | (1 << FLASH_SR_WRPERR))){
        return 1;
    }

    return 0;
}

void Flash_Unlock(void){

    /* Write KEY1 and KEY2 in Flash Key Register */
    FLASHINTR->KEYR = 0x45670123;
    FLASHINTR->KEYR = 0xCDEF89AB;
}

void Flash_Lock(void){

    /* Set Lock bit in Flash Control Register */
    FLASHINTR->CR |= (1 << FLASH_CR_LOCK);
}

void Flash_SetPSIZE(flash_psize_t psize){

    /* Set PSIZE bits in the Flash Control Register */
    FLASHINTR->CR &= (0x07 << FLASH_CR_PSIZE);
    FLASHINTR->CR |= (psize << FLASH_CR_PSIZE);
}

uint8_t Flash_Busy(void){

    if(FLASHINTR->SR & (1 << FLASH_SR_BSY)){
        return 1;
    }

    return 0;
}
