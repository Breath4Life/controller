
#include <stdbool.h>
#include <stdint.h>

#include "hal/pins.h"
#include "hal/io.h"
#include "hal/tone.h"

#define TONE_MULT F_CPU/256/2

void tone_init()
{
    tone_stop();
    dio_init(DIO_PIN_ALARM_SOUND, DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_SOUND_BIS, DIO_OUTPUT);
    dio_write(DIO_PIN_ALARM_SOUND, DIO_HIGH);
    dio_write(DIO_PIN_ALARM_SOUND_BIS, DIO_HIGH);
}


void tone_start(uint16_t freq, bool high_volume)
{
    tone_stop();
    // For frequencies of the alarm : 260 - 523
    // OCR0A will take 58 -> 119
    uint8_t period =(TONE_MULT/freq)-1;
    OCR0A = period;
    OCR0B = period;
    // configure mode : CTC mode, COM0A0
    TCCR0A = _BV(WGM01);
    // set both outputs to zero
    TCCR0A |= _BV(COM0A1); // set clear on compare
    TCCR0B |= _BV(FOC0A); // trigger compare
    TCCR0A &= ~_BV(COM0A1); // disable clear on compare
    // configure toggle on compare
    TCCR0A |= _BV(COM0A0);
    if (high_volume) {
        TCCR0A |= _BV(COM0B1) | _BV(COM0B0); // set set on compare
        TCCR0B |= _BV(FOC0B); // trigger compare
        TCCR0A &= ~(_BV(COM0B1) | _BV(COM0B0)); // disable set on compare
        // configure toggle on compare
        TCCR0A |= _BV(COM0B0);
    }
    TCCR0B = _BV(CS02); // with 256 prescaler
}

void tone_stop()
{
    TCCR0A = 0;
    TCCR0B = 0;
}
