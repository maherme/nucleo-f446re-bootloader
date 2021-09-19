# nucleo-f446re-bootloader
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
