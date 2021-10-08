import boot_serial

CMD_GET_VER         = 0x51
CMD_GET_VER_LEN     = 6
CMD_GET_HELP        = 0x52
CMD_GET_HELP_LEN    = 6
CMD_GET_CID         = 0x53
CMD_GET_CID_LEN     = 6
CMD_GET_RDP         = 0x54
CMD_GET_RDP_LEN     = 6
CMD_GO              = 0x55
CMD_GO_LEN          = 10
CMD_ERASE           = 0x56
CMD_ERASE_LEN       = 8
CMD_WRITE           = 0x57
CMD_WRITE_LEN       = 11

def word_to_byte(addr, index, lowerfirst):
    return (addr >> (8 * (index - 1)) & 0x000000FF)

def get_crc(buff, length):

    crc = 0xFFFFFFFF

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

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_VER_LEN - 1)
    data_buf.append(CMD_GET_VER)
    crc32 = get_crc(data_buf, CMD_GET_VER_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_VER_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_help():

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_HELP_LEN - 1)
    data_buf.append(CMD_GET_HELP)
    crc32 = get_crc(data_buf, CMD_GET_HELP_LEN - 4)
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

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_CID_LEN - 1)
    data_buf.append(CMD_GET_CID)
    crc32 = get_crc(data_buf, CMD_GET_CID_LEN - 4)
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

def cmd_rdp():

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_GET_RDP_LEN - 1)
    data_buf.append(CMD_GET_RDP)
    crc32 = get_crc(data_buf, CMD_GET_RDP_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GET_RDP_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_go(addr):

    data_buf = []
    go_address = int(addr, 16)

    boot_serial.purge_serial()

    data_buf.append(CMD_GO_LEN - 1)
    data_buf.append(CMD_GO)
    data_buf.append(word_to_byte(go_address, 1, 1))
    data_buf.append(word_to_byte(go_address, 2, 1))
    data_buf.append(word_to_byte(go_address, 3, 1))
    data_buf.append(word_to_byte(go_address, 4, 1))
    crc32 = get_crc(data_buf, CMD_GO_LEN - 4)
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_GO_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_erase(sector, num_sectors):

    data_buf = []

    boot_serial.purge_serial()

    data_buf.append(CMD_ERASE_LEN - 1)
    data_buf.append(CMD_ERASE)
    data_buf.append(sector)
    data_buf.append(num_sectors)
    crc32 = get_crc(data_buf, CMD_ERASE - 4)
    data_buf.append(word_to_byte(crc32, 1, 1))
    data_buf.append(word_to_byte(crc32, 2, 1))
    data_buf.append(word_to_byte(crc32, 3, 1))
    data_buf.append(word_to_byte(crc32, 4, 1))

    boot_serial.write_serial(data_buf[0])
    for i in data_buf[1:CMD_ERASE_LEN]:
        boot_serial.write_serial(i)

    ack = boot_serial.read_serial(2)
    len_recv = (bytearray(ack))[1]

    recv = boot_serial.read_serial(len_recv)
    value = bytearray(recv)

    return value

def cmd_write(file_name, file_size, address):

    data_buf = []
    bytes_remaining = 0
    bytes_to_sent = 0
    len_to_read = 0

    bytes_remaining = file_size - bytes_to_sent

    bin_file = open(file_name,'rb')

    boot_serial.purge_serial()

    while(bytes_remaining):
        if(bytes_remaining >= 128):
            len_to_read = 128
        else:
            len_to_read = bytes_remaining

        cmd_total_len = CMD_WRITE_LEN + len_to_read

        data_buf.append(cmd_total_len - 1)
        data_buf.append(CMD_WRITE)
        data_buf.append(word_to_byte(address, 1, 1))
        data_buf.append(word_to_byte(address, 2, 1))
        data_buf.append(word_to_byte(address, 3, 1))
        data_buf.append(word_to_byte(address, 4, 1))
        data_buf.append(len_to_read)

        for _ in range(len_to_read):
            file_read_value = bytearray(bin_file.read(1))
            data_buf.append(int(file_read_value[0]))

        crc32 = get_crc(data_buf, cmd_total_len - 4)
        data_buf.append(word_to_byte(crc32, 1, 1))
        data_buf.append(word_to_byte(crc32, 2, 1))
        data_buf.append(word_to_byte(crc32, 3, 1))
        data_buf.append(word_to_byte(crc32, 4, 1))

        boot_serial.write_serial(data_buf[0])
        for i in data_buf[1:cmd_total_len]:
            boot_serial.write_serial(i)

        ack = boot_serial.read_serial(2)
        ack = bytearray(ack)
        len_recv = (bytearray(ack))[1]

        recv = boot_serial.read_serial(len_recv)

        address += len_to_read
        bytes_to_sent += len_to_read
        bytes_remaining = file_size - bytes_to_sent

        data_buf.clear()
