#include <stdint.h>
#include <avr/interrupt.h>
#include "core/volume.h"
#include "core/debug.h"
#include "hal/sfm3000.h"
#include "hal/time.h"

// Total volume in standard µl
volatile int32_t volume;

// Last poll time
uint32_t last_poll_time;

#define DEBUG_FLOW 0
#define SEND_TO_SERIAL 0

void init_volume() {
#if DEBUG_FLOW
    debug_print("[FLOW] Init.\r\n");
#endif
    sfm3000_init(1);
    volume = 0;
    last_poll_time = time_us();
}

/*
 * Parameters giving rise to the smallest flow:
 * - respiratory frequency: 10/min -> TCT = 6s
 * - I:E ratio: 1:1 -> Ti = 3s
 * - tidal volume of 200ml -> 0.2l/3s * 60s/min = 4l/min
 *
 * Target accuracy: 1%m.v.
 * -> 0.04l/min ~ 0.0005l/s = 500µl/s = 0.5µl/ms
 *
 * Over a sampling period of ~20ms, this gives a volume
 * increment accuracy of 10µl:
 * -> volume_inc and volume should be in µl
 * -> 32 bits give a max value of 4m3 -> OK
 *
 * reading_sfm3300 (offset removed) : [-32768, 32767]
 * -> fits on 16 bits
 * t_delta (sampling period) : [0, 1e6] µs
 * -> doesn't fit on 16 bits -> 32 bits
 * reading_sfm3300 * t_delta -> 64 bits
 */

uint8_t poll_volume() {
    if (sfm3000_poll() == 0) {
        uint32_t curr_time = time_us();
        uint32_t t_delta = curr_time - last_poll_time;
        int64_t scale = SCALE_SFM3000 * 60;
        int32_t volume_inc = (((int64_t) reading_sfm3300) * ((int64_t) t_delta)) / scale;

        debug_print("%lu:", last_poll_time);
        cli();
        volume += volume_inc;
        last_poll_time = curr_time;
        sei();
#if SEND_TO_SERIAL
        // Send curr_time in µs, flow in L/min, volume in mL
        debug_print("%lu:%lu:%li:%li\r\n", curr_time, t_delta, volume, volume_inc);
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
