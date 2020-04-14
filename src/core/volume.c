#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include "core/volume.h"
#include "core/debug.h"
#include "hal/sfm3000.h"
#include "hal/time.h"

// Total volume in standard µl
volatile int32_t volume;

// Has there been an error since the last reset ?
static volatile bool error;

// Last poll time
static volatile uint32_t last_poll_time;

#define DEBUG_FLOW 1
#define SEND_TO_SERIAL 0

void init_volume() {
#if DEBUG_FLOW
    debug_print("[FLOW] Init.\r\n");
#endif
    sfm3000_init(1);
    reset_volume();
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
    switch (sfm3000_poll()) {
        case 0: {
                    uint32_t curr_time = time_us();
                    uint32_t t_delta = curr_time - last_poll_time;
                    int64_t scale = SCALE_SFM3000 * 60;
                    int32_t volume_inc = (((int64_t) reading_sfm3300) * ((int64_t) t_delta)) / scale;

#if SEND_TO_SERIAL
                    // Send last_poll_time in µs
                    debug_print("%lu:", last_poll_time);
#endif
                    cli();
                    volume += volume_inc;
                    last_poll_time = curr_time;
                    sei();
#if SEND_TO_SERIAL
                    // Send curr_time and t_delta in µs, volume and volume_inc in µl
                    debug_print("%lu:%lu:%li:%li\r\n", curr_time, t_delta, volume, volume_inc);
#endif
                    return 0;
                }
        case 1:
            return 1;
        default:
#if DEBUG_FLOW
            debug_print("[FLOW] Error.\r\n");
#endif
            error = true;
            return 2;
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
    error = false;
    sei();
}

uint8_t get_volume(int32_t *vol) {
#if DEBUG_FLOW
    debug_print("[FLOW] Get volume: %i %li.\r\n", error, volume);
#endif
    cli();
    bool tmp_err = error;
    int32_t tmp_vol = volume;
    sei();
    if (tmp_err) {
        return 1;
    } else {
        *vol = tmp_vol;
        return 0;
    }
}
