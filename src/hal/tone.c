#include "hal/pins.h"
#include "hal/io.h"
#include "hal/tone.h"
#define TONE_MULT F_CPU/256/2

void tone_init()
{
    dio_init(DIO_PIN_ALARM_SOUND, DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_SOUND_BIS, DIO_OUTPUT);
    dio_write(DIO_PIN_ALARM_SOUND, DIO_LOW);
    dio_write(DIO_PIN_ALARM_SOUND_BIS, DIO_LOW);
    TCCR0A = _BV(WGM01); // WGM01 : CTC mode, COM0A0 : Output toggle OC0A
    TCCR0B = _BV(CS02); // with 256 prescaler
}

void tone_start(uint16_t freq)
{
    // For frequencies of the alarm : 260 - 523
    // OCR0A will take 58 -> 119
    uint8_t period =(TONE_MULT/freq)-1;
    OCR0A = period;
    OCR0B = period;
    // WGM01 : CTC mode, COM0A0 | COM0B0 : Output toggle OC0A and OC0B
    TCCR0A = _BV(COM0A0) | _BV(COM0B0) | _BV(WGM01);
}

void tone_stop()
{
    // WGM01 : CTC mode, with COMA and COMB outputs in 'normal' DIO mode
    TCCR0A = _BV(WGM01);
}
