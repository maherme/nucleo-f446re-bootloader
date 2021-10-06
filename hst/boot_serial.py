import serial
import sys
import glob
import struct

def serial_ports():

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

def connect_serial(port):

    global ser

    try:
        ser = serial.Serial(port,115200,timeout=2)
    except:
        return 0

    if ser.is_open:
        pass
    else:
        return 0

    return 1

def read_serial(length):
    return ser.read(length)

def write_serial(value):

    data = struct.pack('>B', value)
    ser.write(data)

def purge_serial():
    ser.reset_input_buffer()
