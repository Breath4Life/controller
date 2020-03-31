## Breath4Life software controller ##

**Please read this entire README before writing code.** All commits will be peer-reviewed, commits that do not respect the architectural organization or the coding style will be rejected!

### Requirements

This controller is supposed to run on an Arduino Mega2560.
This project requires the following tools and librairies to be installed:
- avr-gcc
- avrdude
- avr-libc

### Compilation & flashing
To compile the project, use `make`. The project can be cleaned using `make clean`.

To flash the Arduino after compilation, use `tools/flash.sh [<DEV>]` where `<DEV>` is an optional argument indicating the device path (default: `/dev/ttyACM0`). The UART0 bus displaying the debug messages uses a baud rate of 19200.

Note: An easy way to read the UART from a linux machine: `screen /dev/ttyACM0 19200`.

### Hierarchy organization

All source files have to be written in C11. The `src/` folder is organized as follows:

* `main.c`: contains the main function which initializes the different FreeRTOS tasks. At this level, this must be the only C file!
* `core/`: contains all the functions implementing the several tasks and their related helper functions. These functions must be high-level and hardware-independent. They may however call functions from the HAL (*Hardware Abstraction Layer*), explained here under. 
* `hal/`: implements the *Hardware Abstraction Layer*. This layer acts as a proxy between the core functions and the MCU. 

It is important to respect this organization, as providing a separate HAL allows to easily change of MCU in the future if needs be. If your feature requires the usage of a new library, please include it into the `lib/` folder and update the Makefile.

### Coding style

This section presents some rules regarding the coding style used in this project. Their enforcement is **mandatory**.

* *Indentation*: 4 spaces per level of indentation, no tabs.
* *Variable names*: camel-case, starting with a lower-case letter.
* *Function names*: camel-case, prefix the name with a pseudo-namespace/unit name, e.g.: `uart_init()`, `uart_transmit()`.
* *Macros & enums*: all `#defines` and values of enums are all-capitales, with words separated by an underscore.
* *Braces*: curly braces go on their own lines. Use curly braces even for one-line blocks, and use parentheses even when precedence of operator applies.
* *Abbreviations*: use `ptr` for denoting a pointer, and `cbf` for a callback function.
* *Documentation*: please respect the Doxygen syntax (http://www.doxygen.nl/manual/docblocks.html). For every function declared in a .h file, there **MUST** be a description of the function.

* Use `stdint.h` integer types.

### Architecture

Several real times tasks are spawned in the main function. They are listed by descending order of priority:

1. `MotorControlTask`: control the motor. 
1. `UserInterfaceTask`: handle the peripherals related to the user interface (buttons, LEDs, ...), except the LCD screen.
1. `LCDDisplayTask`: handle the display of the LCD screen.
1. `AlarmsTask`: handle the different alarms.
1. `LEDTask`: blink a simple LED.

### Technical note

enum dio_pin and struct io_pin_config DIO_PIN_CONFIG[] should stay aligned as the dio_pin is used to index the array.

Do not forget to init your ios in the initHardware() in main.c.
In the future, we could add the dio_mode into io_pin_conf, so the digital io setup could be done automatically (a loop on DIO_PIN_CONFIG).