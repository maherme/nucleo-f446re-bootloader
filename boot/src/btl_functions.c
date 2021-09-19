/*****************************************************************************************************
* FILENAME :        btl_functions.c
*
* DESCRIPTION :
*       File containing the needed functions for performing the bootloader.
*
* PUBLIC FUNCTIONS :
*       void bootloader_uart_read_data(USART_Handle_t* pUSART_Handle)
*       void bootloader_jump_to_app(void)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include <string.h>
#include "btl_functions.h"
#include "usart_driver.h"

/*****************************************************************************************************/
/*                                       Static Function Prototypes                                  */
/*****************************************************************************************************/

static void bootloader_handle_getver_cmd(uint8_t* buffer);
static void bootloader_handle_gethelp_cmd(uint8_t* buffer);
static void bootloader_handle_getcid_cmd(uint8_t* buffer);
static void bootloader_handle_getrdp_cmd(uint8_t* buffer);
static void bootloader_handle_go_cmd(uint8_t* buffer);
static void bootloader_handle_flash_erase_cmd(uint8_t* buffer);
static void bootloader_handle_mem_write_cmd(uint8_t* buffer);
static void bootloader_handle_en_rw_protect(uint8_t* buffer);
static void bootloader_handle_mem_read(uint8_t* buffer);
static void bootloader_handle_read_sector_protection_status(uint8_t* buffer);
static void bootloader_handle_read_otp(uint8_t* buffer);
static void bootloader_handle_dis_rw_protect(uint8_t* buffer);

/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

void bootloader_jump_to_app(void){

    void (*app_reset_handler)(void);

    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
    __set_MSP(msp_value);

    uint32_t reset_handler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*)reset_handler_address;

    app_reset_handler();
}

void bootloader_uart_read_data(USART_Handle_t* pUSART_Handle){

    uint8_t len_rx = 0;
    uint8_t rx_buffer[200] = {0};

    while(1){
        memset(rx_buffer, 0, sizeof(rx_buffer));
        USART_ReceiveData(pUSART_Handle, rx_buffer, 1);
        len_rx = rx_buffer[0];
        USART_ReceiveData(pUSART_Handle, &rx_buffer[1], len_rx);
        switch((btl_cmd_t)rx_buffer[1]){
            case BL_GET_VER:
                bootloader_handle_getver_cmd(rx_buffer);
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(rx_buffer);
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(rx_buffer);
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(rx_buffer);
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(rx_buffer);
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(rx_buffer);
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(rx_buffer);
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(rx_buffer);
                break;
            case BL_MEM_READ:
                bootloader_handle_mem_read(rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(rx_buffer);
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(rx_buffer);
                break;
            case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(rx_buffer);
                break;
             default:
                break;
        }
    }
}

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

static void bootloader_handle_getver_cmd(uint8_t* buffer){
}

static void bootloader_handle_gethelp_cmd(uint8_t* buffer){
}

static void bootloader_handle_getcid_cmd(uint8_t* buffer){
}

static void bootloader_handle_getrdp_cmd(uint8_t* buffer){
}

static void bootloader_handle_go_cmd(uint8_t* buffer){
}

static void bootloader_handle_flash_erase_cmd(uint8_t* buffer){
}

static void bootloader_handle_mem_write_cmd(uint8_t* buffer){
}

static void bootloader_handle_en_rw_protect(uint8_t* buffer){
}

static void bootloader_handle_mem_read(uint8_t* buffer){
}

static void bootloader_handle_read_sector_protection_status(uint8_t* buffer){
}

static void bootloader_handle_read_otp(uint8_t* buffer){
}

static void bootloader_handle_dis_rw_protect(uint8_t* buffer){
}
