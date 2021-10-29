#!/usr/bin/env python3
"""! @brief Serial module.

This is a module for managing the serial communication between the host computer and the bootloader.

This modue requires that pyserial be installed within the Python environment you are running this module in.

This module contains the following functions:

    * serial_ports - Scan all active USB connected ports.
    * connect_serial - Open a serial port with a baudrate of 115200 and a timeout of 2 seconds.
    * read_serial - Read a number of bytes passed as argument from the serial port.
    * write_serial - Send a byte passed as argument through the serial port.
    * purge_serial - Reset the input buffer of the serial port.
"""

##
# @file boot_serial.py
#

import serial
import sys
import glob
import struct

def serial_ports() -> list:
    """! Scan all active USB connected ports.

    @return result A list of strings with the file descriptor of each USB port connected.
    """

    if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/ttyUSB*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []

    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass

    return result

def connect_serial(port : str) -> bool:
    """! Open a serial port configuring the baudrate to 115200 and a recpetion timeout of 2 seconds.

    @param port Is an string with the file descriptor of the USB port.

    @return True is port could be oppened or False is failed.
    """

    global ser

    try:
        ser = serial.Serial(port, 115200, timeout = 2)
    except:
        return False

    if ser.is_open:
        pass
    else:
        return False

    return True

def read_serial(length : int) -> bytes:
    """! Read a number of bytes from the serial port.

    @param length Is the number of bytes to read.

    @return An instance of bytes with the bytes read from the serial port.
    """

    return ser.read(length)

def write_serial(value : bytes):
    """! Write a byte in the serial port for sending.

    @param value Is the byte to be sent.
    """

    data = struct.pack('>B', value)
    ser.write(data)

def purge_serial():
    """! Purge the serial input buffer."""

    ser.reset_input_buffer()

def timeout_serial(new_timeout : int):
    """! Change the configured timeout value.

    @param new_timeout Is the value of the timeout in seconds.
    """

    ser.timeout = new_timeout
