/*****************************************************************************************************
* FILENAME :        btl_functions.h
*
* DESCRIPTION :
*       Header file containing the prototypes of the needed functions for performing the bootloader.
*
* PUBLIC FUNCTIONS :
*       void uart_read_data(USART_Handle_t* pUSART_Handle)
*       void jump_to_app(void)
*
**/

#ifndef BTL_FUNCTIONS_H
#define BTL_FUNCTIONS_H

#include <stdint.h>
#include "usart_driver.h"

#define BL_ACK      0xA5
#define BL_NACK     0x7F
#define CRC_LEN     4

/**
 * Starting address for the user application code in flash.
 */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

/**
 * List of allowed commands for bootloader
 */
typedef enum{
    BL_GET_VER              = 0x51,
    BL_GET_HELP             = 0x52,
    BL_GET_CID              = 0x53,
    BL_GET_RDP_STATUS       = 0x54,
    BL_GO_TO_ADDR           = 0x55,
    BL_FLASH_ERASE          = 0x56,
    BL_MEM_WRITE            = 0x57,
    BL_EN_RW_PROTECT        = 0x58,
    BL_MEM_READ             = 0x59,
    BL_READ_SECTOR_P_STATUS = 0x5A,
    BL_OTP_READ             = 0x5B,
    BL_DIS_R_W_PROTECT      = 0x5C
}btl_cmd_t;

/*****************************************************************************************************/
/*                                 Core Function Access                                              */
/*****************************************************************************************************/

/**
 * @fn __set_MSP
 *
 * @brief function for assigning the given value to the Main Stack Pointer (MSP).
 *
 * @param[in] topOfMainStack is the Main Stack Pointer value to set.
 *
 * @return void
 */
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack){
    __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack));
}

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @fn uart_read_data
 *
 * @brief function for reading bootloader commands using the USART peripheral.
 *
 * @param[in] pUSART_Handle pointer to the handle structure for the USART peripheral.
 *
 * @return void
 */
void uart_read_data(USART_Handle_t* pUSART_Handle);

/**
 * @fn jump_to_app
 *
 * @brief function for jumping to the user application code in flash.
 *
 * @return void
 */
void jump_to_app(void);

#endif
