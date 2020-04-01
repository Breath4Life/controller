#include <avr/io.h>
#include <avr/interrupt.h>

/* @tone_init
 * Prepare timer0
 */
void tone_init();

/* @tone_start
 * Define a tone using timer 0 on PB6. The tone plays indefinitely.
 * freq : tone frequency
 */
void tone_start(uint16_t frequency);

/* @tone_stop
 * Stop the playing of the tone
 */
void tone_stop();
