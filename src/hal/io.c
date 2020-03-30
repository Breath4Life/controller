#include <stdint.h>
#include <avr/io.h>

#include "hal/pins.h"
#include "hal/io.h"

static volatile uint8_t * PORTS [] = {
    &PORTA,
    &PORTB,
    &PORTC,
    &PORTD,
    &PORTE,
    &PORTF,
    &PORTG,
    &PORTH,
};

static volatile uint8_t * DDRS [] = {
    &DDRA,
    &DDRB,
    &DDRC,
    &DDRD,
    &DDRE,
    &DDRF,
    &DDRG,
    &DDRH
};

void dio_init(uint8_t pin, uint8_t mode)
{
    const uint8_t io_port = DIO_PIN_CONFIG[pin].io_port;
    const uint8_t bit_pos = DIO_PIN_CONFIG[pin].pos;

    switch(mode) {
        case DIO_INPUT:
            *DDRS[io_port] &= ~(1 << bit_pos);
            break;
        case DIO_OUTPUT:
            *DDRS[io_port] |= (1 << bit_pos);
            break;
        case DIO_DISABLED:
            *DDRS[io_port] &= ~(1 << bit_pos);
            break;
    }
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

    if (level == DIO_HIGH) {
        *PORTS[io_port] |= (1 << bit_pos);
    }
    else {
        *PORTS[io_port] &= ~(1 << bit_pos);
    }
}
