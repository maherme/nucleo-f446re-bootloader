# NUCLEO-F446RE Bootloader from Scratch
Embedded project for a bootloader from scratch using [NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board, based on [STM32F446RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html) microcontroller.

## OpenOCD
You can use OpenOCD (Open On-Chip Debugger) for programming or debugging this project. You can starting OpenOCD typing:
```console
openocd -f board/st_nucleo_f4.cfg
```
Or using the Makefile:
```console
make load
```
You can use a telnet connection for connecting to the OpenOCD server:
```console
telnet 127.0.0.1 4444
```
You can program the microcontroller using:
```console
reset init
flash write_image erase your_bootloader_app.hex
flash write_image your_user_app.hex
reset
```
Remember you must enable semihosting in the telnet session if you compile the project for using this feature (you can see printf outputs):
```console
arm semihosting enable
```
## Flash Memory Layout
| Address Range              | Content          | Size  |
|:--------------------------:|:----------------:|:-----:|
| 0x0800-0000 to 0x0800-7FFF | Bootloader       | 32KB  |
| 0x0800-8000 to 0x0807-FFFF | User Application | 480KB |

## Bootloader command frames

- **Get Version:**

  *Command Frame:*
  | Length to Follow | Command Code | CRC    |
  | :--------------: | :----------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte |
  | 0x05             | 0x51         | TBC    |

  *Bootloader Reply:*
  | Bootloader Version Number |
  | :-----------------------: |
  | 1 Byte                    |
  
- **Get Supported Commands**
  
  *Command Frame:*
  | Length to Follow | Command Code | CRC    |
  | :--------------: | :----------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte |
  | 0x05             | 0x52         | TBC    |
  
  *Bootloader Reply:*
  | Supported Command Codes         |
  | :-----------------------------: |
  | N Byte (N = number of commands) |

- **Get Chip identifier**

  *Command Frame:*
  | Length to Follow | Command Code | CRC    |
  | :--------------: | :----------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte |
  | 0x05             | 0x53         | TBC    |

  *Bootloader Reply:*
  | MCU Chip ID (LSB) | MCU Chip ID (MSB) |
  | :---------------: | :---------------: |
  | 1 Byte            | 1 Byte            |

- **Get Read Protection Status**

  *Command Frame:*
  | Length to Follow | Command Code | CRC    |
  | :--------------: | :----------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte |
  | 0x05             | 0x54         | TBC    |

  *Bootloader Reply:*
  | RDP Status |
  | :--------: |
  | 1 Byte     |

- **Go to Address**

  *Command Frame:*
  | Length to Follow | Command Code | Memory Address            | CRC    |
  | :--------------: | :----------: | :-----------------------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte (Little Endian)    | 4 Byte |
  | 0x09             | 0x55         | TBC                       | TBC    |

  *Bootloader Reply:*
  | Status |
  | :----: |
  | 1 Byte |

- **Flash Erase**

  *Command Frame:*
  | Length to Follow | Command Code | Sector Number | Number of Sectors | CRC    |
  | :--------------: | :----------: | :-----------: | :---------------: | :----: |
  | 1 Byte           | 1 Byte       | 1 Byte        | 1 Byte            | 4 Byte |
  | 0x07             | 0x56         | TBC           | TBC               | TBC    |

  *Bootloader Reply:*
  | Status |
  | :----: |
  | 1 Byte |

- **Memory Write**

  *Command Frame:*
  | Length to Follow | Command Code | Base Memory Addr  | Payload Length | Payload | CRC    |
  | :--------------: | :----------: | :---------------: | :------------: | :-----: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte (L Endian) | 1 Byte         | X Byte  | 4 Byte |
  | 0x0A + X         | 0x57         | TBC               | TBC            | TBC     | TBC    |

  *Bootloader Reply:*
  | Status |
  | :----: |
  | 1 Byte |

- **Enable Read/Write Flash Sector Protection**

  *Command Frame:*
  | Length to Follow | Command Code | Sector Details | Protection Mode | CRC    |
  | :--------------: | :----------: | :------------: | :-------------: | :----: |
  | 1 Byte           | 1 Byte       | 1 Byte         | 1 Byte          | 4 Byte |
  | 0x07             | 0x58         | TBC            | TBC             | TBC    |

  *Bootloader Reply:*
  | Status |
  | :----: |
  | 1 Byte |

- **Memory Read**

  *Command Frame:*
  | Length to Follow | Command Code | Base Memory Addr  | Length | CRC    |
  | :--------------: | :----------: | :---------------: | :----: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte (L Entian) | 1 Byte | 4 Byte |
  | 0x0A             | 0x59         | TBC               | TBC    | TBC    |

  *Bootloader Reply:*
  | Status | Read Value           |
  | :----: | :------------------: |
  | 1 Byte | L Byte (L = Length)  |

- **Read Flash Sector Protection Status**

  *Command Frame:*
  | Length to Follow | Command Code | CRC    |
  | :--------------: | :----------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte |
  | 0x05             | 0x5A         | TBC    |

  *Bootloader Reply*
  | Sector Status |
  | :-----------: |
  | 2 Byte        |

- **Read One-time Protection Bytes**

  *Command Frame:*
  | Length to Follow | Command Code | OPT Sector | CRC    |
  | :--------------: | :----------: | :--------: | :----: |
  | 1 Byte           | 1 Byte       | 1 Byte     | 4 Byte |
  | 0x06             | 0x5B         | TBC        | TBC    |

  *Bootloader Reply*
  | Status | Read Value |
  | :----: | :--------: |
  | 1 Byte | 32 Byte    |

- **Disable Read/Write Flash Protection**

  *Command Frame:*
  | Length to Follow | Command Code | CRC    |
  | :--------------: | :----------: | :----: |
  | 1 Byte           | 1 Byte       | 4 Byte |
  | 0x05             | 0x5C         | TBC    |

  *Bootloader Reply*
  | Status |
  | :----: |
  | 1 Byte |
