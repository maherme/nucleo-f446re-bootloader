/*****************************************************************************************************
* FILENAME :        btl_functions.c
*
* DESCRIPTION :
*       File containing the needed functions for performing the bootloader.
*
* PUBLIC FUNCTIONS :
*       void uart_read_data(USART_Handle_t* pUSART_Handle)
*       void jump_to_app(void)
*
* NOTES :
*       For further information about functions refer to the corresponding header file.
*
**/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "btl_functions.h"
#include "usart_driver.h"
#include "crc_driver.h"
#include "flash_driver.h"

/*****************************************************************************************************/
/*                                       Static Function Prototypes                                  */
/*****************************************************************************************************/

/**
 * @fn handle_getver_cmd
 *
 * @brief function for handling the get version command, which sends to the host the bootloader version.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_getver_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);

/**
 * @fn handle_gethelp_cmd
 *
 * @brief function for handling the get help command, which sends to the host the supported command.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_gethelp_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);

/**
 * @fn handle_getcid_cmd
 *
 * @brief function for handling the get cid command, which sends to the host the chip identifier.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_getcid_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);

/**
 * @fn handle_getrdp_cmd
 *
 * @brief function for handling the get rdp command, which sends to the host the read protection
 *        option byte.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_getrdp_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);

/**
 * @fn handle_go_cmd
 *
 * @brief function for handling the go to address command, which order to the bootloader jump to
 *        an specific memory address.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_go_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);

/**
 * @fn handle_flash_erase_cmd
 *
 * @brief function for handling the flash erase command, which order to the bootloader to erase an
 *        specific sector of the flash or a mass erase.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_flash_erase_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);

/**
 * @fn handle_mem_write_cmd
 *
 * @brief function for handling the memory write command, which order to the bootloader to write an
 *        specific memory address of the flash.
 *
 * @param[in] buffer is a pointer to the command frame received.
 * @param[in] pUSART_Handle is the handle structure for the UART peripheral used for receiving and
 *            sending commands.
 *
 * @return void
 */
static void handle_mem_write_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle);
static void handle_en_rw_protect(uint8_t* buffer);
static void handle_mem_read(uint8_t* buffer);
static void handle_read_sector_protection_status(uint8_t* buffer);
static void handle_read_otp(uint8_t* buffer);
static void handle_dis_rw_protect(uint8_t* buffer);

/**
 * @fn verify_cmd_crc
 *
 * @brief function for checking if the CRC received in the command frame is correct.
 *
 * @param[in] pData is a pointer to the data frame received (command).
 * @param[in] length is the length of the data frame received without the CRC.
 * @param[in] crc_host is the CRC received in the data frame with the command sent by the host.
 *
 * @return 0 if crc received is OK.
 *         1 if crc received in NOK.
 */
static uint8_t verify_cmd_crc(uint8_t* pData, uint32_t length, uint32_t crc_host);

/**
 * @fn send_ack
 *
 * @brief function for sending an ACK to the host.
 *
 * @param[in] pUSART_Handle is the handle structure for the USART peripheral used for sending the ACK.
 * @param[in] follow_len is the length of the reply to the received command.
 *
 * @return void
 */
static void send_ack(USART_Handle_t* pUSART_Handle, uint8_t follow_len);

/**
 * @fn send_nack
 *
 * @brief function for sending an NACK to the host.
 *
 * @param[in] pUSART_Handle is the handle structure for the USART peripheral used for sending the NACK.
 *
 * @return void
 */
static void send_nack(USART_Handle_t* pUSART_Handle);

/**
 * @fn verify_address
 *
 * @brief function to verify is an address is suitable for a bootloader action (jump, program...).
 *
 * @param[in] address is the flash address to be verified.
 *
 * @return 0 if address is not allowed for bootloader.
 *         1 if address is allowed for bootloader.
 */
static uint8_t verify_address(uint32_t address);

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
static uint8_t Flash_Erase(uint8_t sector, uint8_t num_sectors);

/**
 * @fn Flash_Write
 *
 * @brief function to program a flash memory section byte by byte.
 *
 * @param[in] address is the starting memory address of the flash to start the programming.
 * @param[in] buffer is an byte array with the data to be programmed.
 * @param[in] length is the number of data to be programmed.
 *
 * @return 0 is sucess.
 *         1 is fail.
 */
static uint8_t Flash_Write(uint32_t address, uint8_t* buffer, uint8_t length);

/*****************************************************************************************************/
/*                                       Public API Definitions                                      */
/*****************************************************************************************************/

void jump_to_app(void){

    void (*app_reset_handler)(void);

    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
    __set_MSP(msp_value);

    uint32_t reset_handler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*)reset_handler_address;

    app_reset_handler();
}

void uart_read_data(USART_Handle_t* pUSART_Handle){

    uint8_t len_rx = 0;
    uint8_t rx_buffer[200] = {0};

    while(1){
        memset(rx_buffer, 0, sizeof(rx_buffer));
        USART_ReceiveData(pUSART_Handle, rx_buffer, 1);
        len_rx = rx_buffer[0];
        USART_ReceiveData(pUSART_Handle, &rx_buffer[1], len_rx);
        switch((btl_cmd_t)rx_buffer[1]){
            case BL_GET_VER:
                handle_getver_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_GET_HELP:
                handle_gethelp_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_GET_CID:
                handle_getcid_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_GET_RDP_STATUS:
                handle_getrdp_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_GO_TO_ADDR:
                handle_go_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_FLASH_ERASE:
                handle_flash_erase_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_MEM_WRITE:
                handle_mem_write_cmd(rx_buffer, pUSART_Handle);
                break;
            case BL_EN_RW_PROTECT:
                handle_en_rw_protect(rx_buffer);
                break;
            case BL_MEM_READ:
                handle_mem_read(rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                handle_read_sector_protection_status(rx_buffer);
                break;
            case BL_OTP_READ:
                handle_read_otp(rx_buffer);
                break;
            case BL_DIS_R_W_PROTECT:
                handle_dis_rw_protect(rx_buffer);
                break;
             default:
                break;
        }
    }
}

/*****************************************************************************************************/
/*                                       Static Function Definitions                                 */
/*****************************************************************************************************/

static void handle_getver_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    uint8_t bl_version = 0x10;
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Get Version received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(bl_version));
        USART_SendData(pUSART_Handle, &bl_version, sizeof(bl_version));
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_gethelp_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    /* List of supported commands */
    uint8_t cmd_list[] = {BL_GET_VER,
                          BL_GET_HELP,
                          BL_GET_CID,
                          BL_GET_RDP_STATUS,
                          BL_GO_TO_ADDR,
                          BL_FLASH_ERASE,
                          BL_MEM_WRITE,
                          BL_EN_RW_PROTECT,
                          BL_MEM_READ,
                          BL_READ_SECTOR_P_STATUS,
                          BL_OTP_READ,
                          BL_DIS_R_W_PROTECT};
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Get Help received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(cmd_list));
        USART_SendData(pUSART_Handle, cmd_list, sizeof(cmd_list));
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_getcid_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    /* Chip identifier */
    uint16_t cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Get Chip ID received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(cid));
        USART_SendData(pUSART_Handle, (uint8_t*)&cid, sizeof(cid));
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_getrdp_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    /* Option byte register address */
    volatile uint32_t* pOB_addr = (uint32_t*)0x1FFFC000;
    /* RDP bytes */
    uint8_t rdp_status = (uint8_t)(*pOB_addr >> 8);
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Get Read Protection OB received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(rdp_status));
        USART_SendData(pUSART_Handle, &rdp_status, sizeof(rdp_status));
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_go_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    uint32_t go_addr = 0;
    uint8_t addr_check = ADDR_INVALID;
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Go Address received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(addr_check));
        /* Get the address to jump */
        go_addr = *((uint32_t*)&buffer[2]);
        /* Check if the address to jump is valid */
        if(verify_address(go_addr) == 1){
            /* Send the address to jump is valid to the host */
            addr_check = ADDR_VALID;
            USART_SendData(pUSART_Handle, &addr_check, sizeof(addr_check));
            /* Jump to the address */
            go_addr += 1; /* Plus 1 for setting the T bit */
            void (*jump_addr)(void) = (void*)go_addr;
            jump_addr();
        }
        else{
            /* Send invalid address to the host */
            USART_SendData(pUSART_Handle, &addr_check, sizeof(addr_check));
        }
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_flash_erase_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    uint8_t erase_status = 0;
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Flash Erase received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(erase_status));
        /* Erase selected sector */
        erase_status = Flash_Erase(buffer[2], buffer[3]);
        /* Send the flash erase result to the host */
        USART_SendData(pUSART_Handle, &erase_status, sizeof(erase_status));
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_mem_write_cmd(uint8_t* buffer, USART_Handle_t* pUSART_Handle){

    uint8_t write_status = 0;
    /* Total length of the cmd packet */
    uint32_t cmd_packet_len = buffer[0] + 1;
    /* Extract the CRC32 sent by the host */
    uint32_t host_crc = *((uint32_t*)(buffer + cmd_packet_len - CRC_LEN));

    printf("CMD Flash Write received\r\n");

    /* Verify checksum */
    if(!verify_cmd_crc(&buffer[0], cmd_packet_len - CRC_LEN, host_crc)){
        send_ack(pUSART_Handle, sizeof(write_status));
        /* Write selected memory address */
        write_status = Flash_Write(*(uint32_t*)(&buffer[2]), &buffer[7], buffer[6]);
        /* Send the flash erase result to the host */
        USART_SendData(pUSART_Handle, &write_status, sizeof(write_status));
    }
    else{
        send_nack(pUSART_Handle);
    }
}

static void handle_en_rw_protect(uint8_t* buffer){
}

static void handle_mem_read(uint8_t* buffer){
}

static void handle_read_sector_protection_status(uint8_t* buffer){
}

static void handle_read_otp(uint8_t* buffer){
}

static void handle_dis_rw_protect(uint8_t* buffer){
}

static uint8_t verify_cmd_crc(uint8_t* pData, uint32_t length, uint32_t crc_host){

    uint32_t crc_value = 0xFF;

    crc_value = CRC_Calculate((uint32_t*)pData, length);

    if(crc_value == crc_host){
        printf("CRC cmd NOK\r\n");
        return 1;
    }

    printf("CRC cmd OK\r\n");

    return 0;
}

static void send_ack(USART_Handle_t* pUSART_Handle, uint8_t follow_len){

    uint8_t ack_buf[2] = {0};

    ack_buf[0] = BL_ACK;
    ack_buf[1] = follow_len;
    USART_SendData(pUSART_Handle, ack_buf, sizeof(ack_buf));
}

static void send_nack(USART_Handle_t* pUSART_Handle){

    uint8_t nack = BL_NACK;

    USART_SendData(pUSART_Handle, &nack, 1);
}

static uint8_t verify_address(uint32_t address){

    if((address >= SRAM1_BASEADDR && address <= SRAM1_ENDADDR) ||
       (address >= SRAM2_BASEADDR && address <= SRAM2_ENDADDR) ||
       (address >= FLASH_BASEADDR && address <= FLASH_ENDADDR) ||
       (address >= BKPSRAM_BASEADDR && address <= BKPSRAM_ENDADDR)){
        return 1;
    }

    return 0;
}

static uint8_t Flash_Erase(uint8_t sector, uint8_t num_sectors){

    uint8_t ret = 1;
    uint8_t i = 0;

    /* Check arguments are valid */
    if((num_sectors == 0) || ((sector != 0xFF) && ((sector + num_sectors) > MAX_NUM_SECTOR))){
        return 1;
    }

    /* Check no flash memory operation is ongoing */
    if(Flash_Busy() == 1){
        return 1;
    }

    /* Check the PSIZE bit in FLASH CR according to the supply voltage */
    Flash_SetPSIZE(FLASH_PSIZE_WORD);

    /* Check for a mass erase request */
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

static uint8_t Flash_Write(uint32_t address, uint8_t* buffer, uint8_t length){

    uint8_t i = 0;
    uint8_t ret = 1;

    /* Check no flash memory operation is ongoing */
    if(Flash_Busy() == 1){
        return 1;
    }

    /* Unlock flash control register */
    Flash_Unlock();

    for(i = 0; i < length; i++){
        ret = Flash_WriteMemoryByte(address + i, buffer[i]);
        if(ret != 0){
            break;
        }
    }

    /* Lock Flash Control Register */
    Flash_Lock();

    return 0;
}
