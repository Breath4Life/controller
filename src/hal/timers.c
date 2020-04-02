#include <avr/io.h>             /// Controller specific header file
#include <avr/interrupt.h>      /// Controller specific header file

#include "timers.h"

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)


volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

void clock_init()
{
	cli();//stop interrupts

	//set timer4 interrupt at 1Hz
	TCCR4A = 0;// set entire TCCR1A register to 0
	TCCR4B = 0;// same for TCCR1B
	TCNT4  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR4A = 15; //56/1;// = (16*10^6) / (1*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR4B |= (1 << WGM12);
	// Set CS12 and CS10 bits for 1024 prescaler
	TCCR4B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK4 |= (1 << OCIE4A);


	sei(); // enable interrupts
}

ISR(TIMER4_COMPA_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long clock_millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;
    sei();
	return m;
}
