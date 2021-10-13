/**
* @file btl_functions.h
*
* @brief Header file containing the prototypes of the needed functions for performing the bootloader.
*
* Public Functions:
*       - void uart_read_data(USART_Handle_t* pUSART_Handle)
*       - void jump_to_app(void)
*/

#ifndef BTL_FUNCTIONS_H
#define BTL_FUNCTIONS_H

#include <stdint.h>
#include "usart_driver.h"

/** ACK value used for communicating with the host */
#define BL_ACK      0xA5
/** NACK value used for communicating with the host */
#define BL_NACK     0x7F
/** Length of the CRC used for communicating with the host */
#define CRC_LEN     4
/** Valid Flash address flag */
#define ADDR_VALID      0x00
/** Invalid Flash address flag */
#define ADDR_INVALID    0x01
/** Starting address for the user application code in flash. */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

/**
 * @brief List of allowed commands for bootloader
 */
typedef enum{
    BL_GET_VER              = 0x51, /**< Get Version Command */
    BL_GET_HELP             = 0x52, /**< Get Supported Command Command */
    BL_GET_CID              = 0x53, /**< Get Chip Identifier Command */
    BL_GET_RDP_STATUS       = 0x54, /**< Get Read Protection Status Command */
    BL_GO_TO_ADDR           = 0x55, /**< Go to Address Command */
    BL_FLASH_ERASE          = 0x56, /**< Flash Erase Command */
    BL_MEM_WRITE            = 0x57, /**< Memory Write Commmand */
    BL_EN_RW_PROTECT        = 0x58, /**< Enable Read/Write Protection Sector Command */
    BL_MEM_READ             = 0x59, /**< Memory Read Command */
    BL_READ_SECTOR_P_STATUS = 0x5A, /**< Read Protection Sector Status Command */
    BL_OTP_READ             = 0x5B, /**< Read One Time Programmable Address Command */
    BL_DIS_R_W_PROTECT      = 0x5C  /**< Disable Read/Write Protection for All SectorS Command */
}btl_cmd_t;

/*****************************************************************************************************/
/*                                 Core Function Access                                              */
/*****************************************************************************************************/

/**
 * @brief Function for assigning the given value to the Main Stack Pointer (MSP).
 * @param[in] topOfMainStack: is the Main Stack Pointer value to set.
 * @return void
 */
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack){
    __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack));
}

/*****************************************************************************************************/
/*                                       APIs Supported                                              */
/*****************************************************************************************************/

/**
 * @brief Function for reading bootloader commands using the USART peripheral.
 * @param[in] pUSART_Handle: pointer to the handle structure for the USART peripheral.
 * @return void
 */
void uart_read_data(USART_Handle_t* pUSART_Handle);

/**
 * @brief Function for jumping to the user application code in flash.
 * @return void
 */
void jump_to_app(void);

#endif
