/** Generation of tone signals for a buzzer.
 * This is based on Timer0, used as a frequency-controlled PWM.
 * The signal is output of PB6.
 */
#include <avr/io.h>
#include <avr/interrupt.h>

/** @tone_init Initialize the hardware for the tone.
 */
void tone_init();

/** @tone_start Generate a square wave of 50% duty cycle. The tone plays
 * indefinitely.
 *
 * @param frequency The frequency of the tone in Hz.
 */
void tone_start(uint16_t frequency);

/** @tone_stop Stop the playing of the tone.
 */
void tone_stop();
