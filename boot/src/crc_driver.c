/**
* @file crc_driver.c
*
* @brief File containing the APIs for configuring the CRC peripheral.
*
* Public Functions:
*       - void     CRC_Init(void)
*       - void     CRC_DeInit(void)
*       - uint32_t CRC_Accumulate(uint32_t* pBuffer, uint32_t length)
*       - uint32_t CRC_Calculate(uint32_t* pBuffer, uint32_t length)
*
* @note
*       For further information about functions refer to the corresponding header file.
*/

#include <stdint.h>
#include "crc_driver.h"

/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

void CRC_Init(void){
    CRC_PCLK_EN();
}

void CRC_DeInit(void){
    CRC_REG_RESET();
}

uint32_t CRC_Accumulate(uint32_t* pBuffer, uint32_t length){

    for(uint32_t i = 0; i < length; i++){
        CRC->DR = pBuffer[i];
    }

    return CRC->DR;
}

uint32_t CRC_Calculate(uint32_t* pBuffer, uint32_t length){

    /* Reset CRC unit */
    CRC->CR |= (1 << CRC_CR_RESET);

    for(uint32_t i = 0; i < length; i++){
        CRC->DR = pBuffer[i];
    }

    return CRC->DR;
}
