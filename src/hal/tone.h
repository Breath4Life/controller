/** Generation of tone signals for a buzzer.
 * This is based on Timer0, used as a frequency-controlled PWM.
 * The signal is either:
 * - (for low volume) a square wave with duty cycle 50% on PB6, with PG5 at ground.
 * - (for high volume) a square wave with duty cycle 50% on both PB6 and PG5 with 180° dephasing.
 */
#ifndef TONE_H_
#define TONE_H_

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/** @tone_init Initialize the hardware for the tone.
 */
void tone_init();

/** @tone_start Generate a square wave of 50% duty cycle. The tone plays
 * indefinitely.
 *
 * @param frequency The frequency of the tone in Hz.
 * @param high_volume true for high volume, false for low volume (approx. 6 dB difference)
 */
void tone_start(uint16_t frequency, bool high_volume);

/** @tone_stop Stop the playing of the tone.
 */
void tone_stop();

#endif // TONE_H_
