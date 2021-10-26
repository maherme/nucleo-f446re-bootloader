#!/usr/bin/env python3
"""! @brief Bootloader commands.

This is a module for managing the commands to be sent to the bootloader application via serial port.

This modue requires that the boot_serial module was imported.

This module contains the following functions:

    * word_to_byt - Extracts a byte from a word.
    * get_crc - Calculates the crc of a number of bytes.
"""

##
# @file boot_cmd.py
#

import boot_serial
from typing import BinaryIO

# Ids and legths of supported commands
## ID for get version command
CMD_GET_VER             = 0x51
## Length of get version command frame
CMD_GET_VER_LEN         = 6
## ID for get help command
CMD_GET_HELP            = 0x52
## Length of get help command frame
CMD_GET_HELP_LEN        = 6
## ID for get chip identifier command
CMD_GET_CID             = 0x53
## Length of get chip identifier command frame
CMD_GET_CID_LEN         = 6
## ID for get read protection command
CMD_GET_RDP             = 0x54
## Length of get read protection command frame
CMD_GET_RDP_LEN         = 6
## ID for go address command
CMD_GO                  = 0x55
## Length of go address command frame
CMD_GO_LEN              = 10
## ID for erase command
CMD_ERASE               = 0x56
## Length of erase command frame
CMD_ERASE_LEN           = 8
## ID for write command
CMD_WRITE               = 0x57
## Length of write command frame
CMD_WRITE_LEN           = 11
## ID for enable read/write flash protection command
CMD_EN_RW_PROTECT       = 0x58
## Length of enable read/write flash protection command frame
CMD_EN_RW_PROTECT_LEN   = 8
## ID for memory read command
CMD_MEM_READ            = 0x59
## Length of memory read command frame
CMD_MEM_READ_LEN        = 11
## ID for read flash sector status command
CMD_READ_SECTOR_ST      = 0x5A
## Length of read flash sector status command frame
CMD_READ_SECTOR_ST_LEN  = 6
## ID for disable read/write flash protection command
CMD_DIS_RW_PROTECT      = 0x5C
## Length of disable read/write flash protection command frame
CMD_DIS_RW_PROTECT_LEN  = 6

def word_to_byte(word : int, index : int) -> int:
    """! Extract a byte from a word.

    @param word Is the word.
    @param index Is the byte number to be extracted.

    @return The byte extracted from the word.
    """

    return (word >> (8 * (index - 1)) & 0x000000FF)

def get_crc(buff : list, length : int) -> int:
    """! Calculate the crc of a number of bytes.

    @param buff Is the array of bytes to calculate the crc.
    @param length Is the number of bytes for calculating the crc.

    @return The calculated crc.
    """

    crc = 0xFFFFFFFF

    for data in buff[0:length]:
        crc = crc ^ data
        for i in range(32):
            if(crc & 0x80000000):
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
    return crc

def cmd_ver() -> bytearray:
    """! Send version command using the serial port.

    @return The bootloader version.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_VER_LEN - 1)
    data_buf.append(CMD_GET_VER)
    crc32 = get_crc(data_buf, CMD_GET_VER_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_VER_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_help() -> bytearray:
    """! Send help command using the serial port.

    @return The IDs of the supported commands by the bootloader.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_HELP_LEN - 1)
    data_buf.append(CMD_GET_HELP)
    crc32 = get_crc(data_buf, CMD_GET_HELP_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_HELP_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_cid() -> bytearray:
    """! Send chip identifier command using the serial port.

    @return The chip identifier of the microcontroller.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_CID_LEN - 1)
    data_buf.append(CMD_GET_CID)
    crc32 = get_crc(data_buf, CMD_GET_CID_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_CID_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_rdp() -> bytearray:
    """! Send read protection command using the serial port.

    @return The read protection status of the flash memory.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_RDP_LEN - 1)
    data_buf.append(CMD_GET_RDP)
    crc32 = get_crc(data_buf, CMD_GET_RDP_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_RDP_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_go(addr : int) -> bytearray:
    """! Send go command using the serial port.

    @param addr Is the address to jump.
    
    @return 1 if address for jumping is invalid.
    """

    data_buf = []
    go_address = int(addr, 16)

    boot_serial.purge_serial()

    data_buf.append(CMD_GO_LEN - 1)
    data_buf.append(CMD_GO)
    data_buf.append(word_to_byte(go_address, 1))
    data_buf.append(word_to_byte(go_address, 2))
    data_buf.append(word_to_byte(go_address, 3))
    data_buf.append(word_to_byte(go_address, 4))
    crc32 = get_crc(data_buf, CMD_GO_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GO_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_erase(sector : int, num_sectors : int) -> bytearray:
    """! Send erase memory command using the serial port.

    @param sector The first sector to be erased. If it is set to 0xFF means a mass erase.
    @param num_sectors The number of consecutive sectors to be erased.

    @return 0 if success or 1 if failure.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_ERASE_LEN - 1)
    data_buf.append(CMD_ERASE)
    data_buf.append(sector)
    data_buf.append(num_sectors)
    crc32 = get_crc(data_buf, CMD_ERASE - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_ERASE_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_write(len_to_read : int, address : int, bin_file : BinaryIO) -> None:
    """! Send write command using the serial port.

    @param len_to_read Is the number of bytes to read from the binary file. They will be written in memory.
    @param address Is the starting flash address to write.
    @param bin_file Is the descriptor of a file open as binary io.
    """

    data_buf = []

    boot_serial.purge_serial()

    cmd_total_len = CMD_WRITE_LEN + len_to_read

    data_buf.append(cmd_total_len - 1)
    data_buf.append(CMD_WRITE)
    data_buf.append(word_to_byte(address, 1))
    data_buf.append(word_to_byte(address, 2))
    data_buf.append(word_to_byte(address, 3))
    data_buf.append(word_to_byte(address, 4))
    data_buf.append(len_to_read)

    for _ in range(len_to_read):
        file_read_value = bytearray(bin_file.read(1))
        data_buf.append(int(file_read_value[0]))

    crc32 = get_crc(data_buf, cmd_total_len - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:cmd_total_len]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    ack = bytearray(ack)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)

def cmd_read_sector_st() -> bytearray:
    """! Send read sector protection status command using the serial port.

    @return the value of the nWRP from Option Byte configuration.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_READ_SECTOR_ST_LEN - 1)
    data_buf.append(CMD_READ_SECTOR_ST)
    crc32 = get_crc(data_buf, CMD_READ_SECTOR_ST_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_READ_SECTOR_ST_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    ack = bytearray(ack)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_en_rw_protect(sectors : int, mode : int) -> bytearray:
    """! Send enable read/write protection command using the serial port.

    @param sectors Is a byte with the selected sectors for enabling the protection.
    @param mode Is the protection mode.

    @return 0 if success or 1 if failure.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_EN_RW_PROTECT_LEN - 1)
    data_buf.append(CMD_EN_RW_PROTECT)
    data_buf.append(sectors)
    data_buf.append(mode)
    crc32 = get_crc(data_buf, CMD_EN_RW_PROTECT_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_EN_RW_PROTECT_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    ack = bytearray(ack)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_dis_rw_protect() -> bytearray:
    """! Send disable read/write protection command using the serial port.

    @return 0 if success or 1 if failure.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_DIS_RW_PROTECT_LEN - 1)
    data_buf.append(CMD_DIS_RW_PROTECT)
    crc32 = get_crc(data_buf, CMD_DIS_RW_PROTECT_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_DIS_RW_PROTECT_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    ack = bytearray(ack)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_mem_read(address : int, len_to_read : int) -> bytearray:
    """! Send memory read command using the serial port.

    @param address Is the base address for reading.
    @param len_to_read Is the number of addres for reading.

    @return The content of the read memory address.
    """

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_MEM_READ_LEN - 1)
    data_buf.append(CMD_MEM_READ)
    data_buf.append(word_to_byte(address, 1))
    data_buf.append(word_to_byte(address, 2))
    data_buf.append(word_to_byte(address, 3))
    data_buf.append(word_to_byte(address, 4))
    data_buf.append(len_to_read)
    crc32 = get_crc(data_buf, CMD_MEM_READ_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1))
    data_buf.append(word_to_byte(crc32, 2))
    data_buf.append(word_to_byte(crc32, 3))
    data_buf.append(word_to_byte(crc32, 4))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_MEM_READ_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    ack = bytearray(ack)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value
