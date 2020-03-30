#include "hal/pins.h"

/* ==== PHYSICAL PIN MAPPING  ==== */
// See: https://www.arduino.cc/en/Hacking/PinMapping2560 */ 

struct io_pin_config DIO_PIN_CONFIG[] = {
    {DIO_PIN_STARTSTOP, IO_PORTA, 0},
    {DIO_PIN_MAINPOWER, IO_PORTA, 1},
    {DIO_PIN_AUXPOWER,  IO_PORTA, 2},
    {DIO_PIN_DEBUGLED,  IO_PORTB, 7},
    {DIO_PIN_READ_IO_TEST, IO_PORTA, 0},
};

// The order of declaration in this array needs to be the same as for aio_pin

struct aio_pin_config AIO_PIN_CONFIG[] = {
    {AIO_PIN_PRESSURE, 0},  // ANALOG IN PIN 0
};

