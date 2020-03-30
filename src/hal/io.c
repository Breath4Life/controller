#include <stdint.h>
#include <avr/io.h>

#include "hal/pins.h"
#include "hal/io.h"
#include "../core/debug.h"

static volatile uint8_t * PORTS [] =
{
    &PORTA,
    &PORTB,
    &PORTC,
    &PORTD,
    &PORTE,
    &PORTF,
    &PORTG,
    &PORTH,
};

static volatile uint8_t * DDRS [] =
{
    &DDRA,
    &DDRB,
    &DDRC,
    &DDRD,
    &DDRE,
    &DDRF,
    &DDRG,
    &DDRH
};

static volatile uint8_t * PINS [] =
{
    &PINA,
    &PINB,
    &PINC,
    &PIND,
    &PINE,
    &PINF,
    &PING,
    &PINH
};

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void dio_init(uint8_t pin, uint8_t mode)
{
    const uint8_t io_port = DIO_PIN_CONFIG[pin].io_port;
    const uint8_t bit_pos = DIO_PIN_CONFIG[pin].pos;

    switch(mode)
    {
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
    const uint8_t io_port = DIO_PIN_CONFIG[pin].io_port;
    const uint8_t bit_pos = DIO_PIN_CONFIG[pin].pos;

    volatile uint8_t *pin_reg = PINS[io_port];
    const uint8_t mask = 1 << bit_pos;

    return (*pin_reg & mask) == 0 ? DIO_LOW : DIO_HIGH;
}


void dio_write(uint8_t pin, uint8_t level)
{
    const uint8_t io_port = DIO_PIN_CONFIG[pin].io_port;
    const uint8_t bit_pos = DIO_PIN_CONFIG[pin].pos;

    if (level == DIO_HIGH)
    {
        *PORTS[io_port] |= (1 << bit_pos);
    }
    else
    {
        *PORTS[io_port] &= ~(1 << bit_pos);
    }
}


uint16_t aio_read(uint8_t pin) {
    // Very expérimental here !!!
    ADMUX = 64;
    ADCSRB = 0;
    ADCSRA = 151;
    char a, b, c;
    a = ADMUX;
    b = ADCSRB;

    ADCSRA |= (1<<ADSC);

    c = ADCSRA;

    // Wait conversion
    while (bit_is_set(ADCSRA, ADSC));

    debug_print("%u %u %u\r\n", a, b, c);

    // Order is important here !
    const uint8_t low  = ADCL;
    const uint8_t high = ADCH;

    return (high << 8) | low;
}

