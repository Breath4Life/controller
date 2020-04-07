
#include <avr/io.h>
#include <avr/interrupt.h>
#include "core/debug.h"

#include "time.h"

volatile uint32_t timer4_overflow_count = 0;

void init_time()
{
	cli();//stop interrupts

        // Set timer4 in normal mode (0)
	TCNT4  = 0;
	// timer overflow interrupt
	TIMSK4 = _BV(TOIE4);
	TCCR4A = 0;
	TCCR4B = _BV(CS41); // 1/8 clk divider

	sei(); // enable interrupts
}

ISR(TIMER4_OVF_vect)
{
    timer4_overflow_count += 1;
    //debug_print_FromISR("ovf_cnt: %u\r\n", timer4_overflow_count);

}

uint32_t time_us() {
    uint32_t ovf_cnt0;
    uint16_t cnt;
    uint8_t ifr;
    // Loop to prevent the case of being interrupted by the overflow routine
    do {
        ovf_cnt0 = timer4_overflow_count;
        cnt = TCNT4;
        ifr = TIFR4;
    } while (ovf_cnt0 != timer4_overflow_count);
    uint32_t res = (cnt >> 1); // Timer 4 counts halfs of us
    res += ((uint32_t) (((ifr >> TOV5) & 0x1) << 15)); // In case overflow is set but ISR not yet executed
    res += (ovf_cnt0 << 15);
    return res;
}

