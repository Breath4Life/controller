#include <stdint.h>
#include <avr/io.h>

#include "hal/pins.h"
#include "hal/io.h"

// From ATmega 2560 datasheet, section 13.2.1:
// > The DDxn bit in the DDRx Register selects the direction of this pin.  If DDxn
// > is written logic one, Pxn is configured as an output pin.  If DDxn is written
// > logic zero, Pxn is configured as an input pin.  If PORTxn is written logic one
// > when the pin is configured as an input pin, the
// > pull-up resistor is activated. To switch the pull-up resistor off, PORTxn has
// > to be written logic zero or the pin has to be configured as an output pin. The
// > port pins are tri-stated when reset condition becomes active, even if no clocks
// > are running.  If PORTxn is written logic one when the pin is configured as an
// > output pin, the port pin is driven high (one). If PORTxn is written logic zero
// > when the pin is configured as an output pin, the port pin is driven low (zero).

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
    &PORTJ,
    &PORTK,
    &PORTL,
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
    &DDRH,
    &DDRJ,
    &DDRK,
    &DDRL
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
    &PINH,
    &PINK,
    &PINL,
};

//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void dio_init(uint8_t pin, uint8_t mode)
{
    const uint8_t io_port = DIO_PIN_CONFIG[pin].io_port;
    const uint8_t bit_pos = DIO_PIN_CONFIG[pin].pos;

    switch(mode)
    {
        case DIO_INPUT:
            *PORTS[io_port] &= ~(1 << bit_pos);
            *DDRS[io_port] &= ~(1 << bit_pos);
            break;
        case DIO_INPUT_PULLUP:
            *PORTS[io_port] |= (1 << bit_pos);
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
    //debug_print("port:%d bit:%d\r\n", io_port, bit_pos);
    if (level == DIO_HIGH)
    {
        *PORTS[io_port] |= (1 << bit_pos);
    }
    else
    {
        *PORTS[io_port] &= ~(1 << bit_pos);
    }
}

void aio_read_start(uint8_t pin) {

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

    const uint8_t mask5 = 1 << 3;
    if (AIO_PIN_CONFIG[pin].io_port == IO_PORTF)
    {
        ADCSRB = ~ mask5;
    }
    else if (AIO_PIN_CONFIG[pin].io_port == IO_PORTK)
    {
        ADCSRB |= mask5;
    }

    ADCSRA |= 1 << ADSC;
}

uint8_t aio_ready(){
    return ~bit_is_set(ADCSRA, ADSC);
}

uint16_t aio_read_result() {
    // Wait conversion
    while (bit_is_set(ADCSRA, ADSC));

    // Order is important here !
    const uint8_t low  = ADCL;
    const uint8_t high = ADCH;

    return (high << 8) | low;
}

