#include <stdint.h>
#include <avr/io.h>

#include "hal/pins.h"
#include "hal/io.h"

static uint8_t * PORTS [] = {
    &PORTA,
    &PORTB,
    &PORTC,
    &PORTD,
    &PORTE,
    &PORTF,
    &PORTG,
    &PORTH,
    };

void dio_init(uint8_t pin, uint8_t mode)
{
    DDRB = 0xff; // for quick test !
    // TODO
    return;
}

uint8_t dio_read(uint8_t pin)
{
    // TODO
    return DIO_LOW;
}


void dio_write(uint8_t pin, uint8_t level)
{
    const uint8_t io_port = DIO_PIN_CONFIG[pin].io_port;
    const uint8_t bit_pos = DIO_PIN_CONFIG[pin].pos;

    if (level == 1) {
        *PORTS[io_port] |= (1 << bit_pos);
    }
    else {
        *PORTS[io_port] &= ~(1 << bit_pos);
    }
}
