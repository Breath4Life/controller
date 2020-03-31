#include <stdint.h>
#include <avr/io.h>

#include "hal/pins.h"
#include "hal/io.h"

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

//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

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

    const uint8_t refs1 = 0 << 7;
    const uint8_t refs0 = 1 << 6;
    const uint8_t adlar = 0 << 5;

    ADMUX = refs1 | refs0 | adlar | AIO_PIN_CONFIG[pin].pos;

    const uint8_t aden  = 1 << 7;
    const uint8_t adsc  = 0 << 6;
    const uint8_t adate = 0 << 5;
    const uint8_t adif  = 1 << 4;
    const uint8_t adie  = 0 << 3;
    const uint8_t adsp2 = 1 << 2;
    const uint8_t adsp1 = 1 << 1;
    const uint8_t adsp0 = 1 << 0;

    ADCSRA = aden | adsc | adate | adif | adie | adsp2 | adsp1 | adsp0;

    ADCSRA |= 1 << ADSC;

    // Wait conversion
    while (bit_is_set(ADCSRA, ADSC));

    // Order is important here !
    const uint8_t low  = ADCL;
    const uint8_t high = ADCH;

    return (high << 8) | low;
}

