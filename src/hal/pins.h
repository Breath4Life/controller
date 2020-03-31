#include <stdint.h>

// Simple structure to hold all pin mappings
struct io_pin_config {
    uint8_t io_pin;
    uint8_t io_port;
    uint8_t pos;
};

/* ==== IO PORTS DECLARATION ==== */
enum io_port {
    IO_PORTA,
    IO_PORTB,
    IO_PORTC,
    IO_PORTD,
    IO_PORTE,
    IO_PORTF,
    IO_PORTG,
    IO_PORTH
};

/* ==== IO PINS DECLARATION  ==== */
enum dio_pin {
    DIO_PIN_STARTSTOP,
    DIO_PIN_MAINPOWER,
    DIO_PIN_AUXPOWER,
    DIO_PIN_DEBUGLED,
    DIO_PIN_READ_IO_TEST
    // TODO: to complete ...
};

enum aio_pin {
    AIO_PIN_PRESSURE
};

// The declaration of the mappings are contained in hal/pins.c.
extern struct io_pin_config DIO_PIN_CONFIG[];
extern struct io_pin_config AIO_PIN_CONFIG[];
