## Breath4Life software controller ##

This controller is supposed to run on an Arduino Mega2560.
This project requires the following tools and librairies to be installed:
- avr-gcc
- avrdude
- avr-libc

### Compilation & flashing
To compile the project, use `make`. The project can be cleaned using `make clean`.

To flash the Arduino after compilation, use `tools/flash.sh [<DEV>]` where <DEV> is an optional argument indicating the device path (default: /dev/ttyACM0). The UART0 bus displaying the debug messages uses a baud rate of 19200.
