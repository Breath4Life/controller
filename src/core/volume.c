#include <stdint.h>
#include <avr/interrupt.h>
#include "core/volume.h"
#include "core/debug.h"
#include "hal/sfm3000.h"
#include "hal/time.h"

// Total volume in standard ml
volatile int32_t volume;

// Last poll time
uint32_t last_poll_time;

#define DEBUG_FLOW 0
#define SEND_FLOW_TO_SERIAL 0
#define SEND_VOLUME_TO_SERIAL 0

void init_volume() {
#if DEBUG_FLOW
    debug_print("[FLOW] Init.\r\n");
#endif
    sfm3000_init(1);
    volume = 0;
    last_poll_time = time_us();
}

uint8_t poll_volume() {
    if (sfm3000_poll() == 0) {
        uint32_t curr_time = time_us();
        // TODO divisions by powers of 2 ideally...
        // t_delta is in ms
        int32_t t_delta = (curr_time - last_poll_time)/1000;
        // flow is in sml/min and is inverted
        int32_t volume_inc = -flow/60 * t_delta/1000;
        cli();
        volume += volume_inc;
        last_poll_time = curr_time;
        sei();
#if SEND_FLOW_TO_SERIAL
        debug_print("%li:%li\n", curr_time/1000, -flow/1000);
#elif SEND_VOLUME_TO_SERIAL
        debug_print("%li:%li\n", curr_time/1000, volume);
#endif
        return 0;
    } else {
        return 1;
    }
}

void reset_volume() {
#if DEBUG_FLOW
    debug_print("[FLOW] Reset.\r\n");
#endif
    uint32_t curr_time = time_us();
    cli();
    volume = 0;
    last_poll_time = curr_time;
    sei();
}
