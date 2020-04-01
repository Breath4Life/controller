#include "hal/tone.h"
#define TONE_MULT F_CPU/256/2

void tone_init()
{
    TCCR0A = (1<<WGM01); // WGM01 : CTC mode, COM0A0 : Output toggle OC0A
    TCCR0B = 1<<CS02; // with 256 prescler
}

void tone_start(uint16_t freq)
{
    // For frequencies of the alarm : 260 - 523
    // OCR0A will take 58 -> 119
    OCR0A = (TONE_MULT/freq)-1;
    TCCR0A = (1<<COM0A0)|(1<<WGM01); // WGM01 : CTC mode, COM0A0 : Output toggle OC0A
}

void tone_stop()
{
    TCCR0A = (1<<WGM01); // WGM01 : CTC mode, COM0A0 : Output toggle OC0A
}
