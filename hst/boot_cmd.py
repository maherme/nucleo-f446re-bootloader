import boot_serial

CMD_GET_VER         = 0x51
CMD_GET_VER_LEN     = 6
CMD_GET_HELP        = 0x52
CMD_GET_HELP_LEN    = 6
CMD_GET_CID         = 0x53
CMD_GET_CID_LEN     = 6

def word_to_byte(addr, index, lowerfirst):
    return (addr >> (8 * (index - 1)) & 0x000000FF)

def get_crc(buff, length):

    crc = 0xffffffff

    for data in buff[0:length]:
        crc = crc ^ data
        for i in range(32):
            if(crc & 0x80000000):
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
    return crc

def cmd_ver():

    data_buf = []

    data_buf.append(CMD_GET_VER_LEN - 1)
    data_buf.append(CMD_GET_VER)
    crc32 = get_crc(data_buf, CMD_GET_VER_LEN - 4)
    crc32 = crc32 & 0xFFFFFFFF
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_VER_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)

    recv = boot_serial.read_serial(1)
    value = bytearray(recv)

    return value[0]

def cmd_help():

    data_buf = []

    data_buf.append(CMD_GET_HELP_LEN - 1)
    data_buf.append(CMD_GET_HELP)
    crc32 = get_crc(data_buf, CMD_GET_HELP_LEN - 4)
    crc32 = crc32 & 0xFFFFFFFF
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_HELP_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_cid():

    data_buf = []

    data_buf.append(CMD_GET_CID_LEN - 1)
    data_buf.append(CMD_GET_CID)
    crc32 = get_crc(data_buf, CMD_GET_CID_LEN - 4)
    crc32 = crc32 & 0xFFFFFFFF
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_CID_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

