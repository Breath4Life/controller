#include <stdint.h>
#include <avr/interrupt.h>
#include "core/volume.h"
#include "hal/sfm3000.h"
#include "hal/time.h"

// Total volume in standard ml
volatile int32_t volume;

// Last poll time
uint32_t last_poll_time;

void init_volume() {
    sfm3000_init(1);
    volume = 0;
    last_poll_time = time_us();
}

uint8_t poll_volume() {
    if (sfm3000_poll() == 0) {
        uint32_t curr_time = time_us();
        // TODO divisions by powers of 2 ideally...
        int32_t t_delta = (curr_time - last_poll_time)/1000;
        int32_t volume_inc = flow * t_delta / 1000;
        cli();
        volume += volume_inc;
        last_poll_time = curr_time;
        sei();
        return 0;
    } else {
        return 1;
    }
}

void reset_volume() {
    uint32_t curr_time = time_us();
    cli();
    volume = 0;
    last_poll_time = curr_time;
    sei();
}
