/** The limit switches are used to detect the end of course of the
 * bag-compressing lever.
 * There are two limit switches: one corresponding to the lever Up position and
 * one corresponding to the lever Down position.
 * The switches are closed when the lever is not touching them and open if they
 * are touched. They are connected to the Ground and to pins configured as
 * input with internal pull-up resistors.
 *
 * An interrupt is raised when one of the switch states is changed, which in
 * turn notifies the SWITCH_HANDLING_TASK with the LIM_SWITCH_UP and
 * LIM_SWITCH_DOWN notifications when the switches are pushed down.
 */

#ifndef LIMIT_SWITCH_H_
#define LIMIT_SWITCH_H_

#include <stdint.h>

#include "hal/pins.h"
#include "core/system.h"
#include "core/motor_control.h"

#define SWITCH_HANDLING_TASK motorControlTaskHandle

// Duration [us] for the debouncing algorithm.
// Typical bouncing total duration is 500 us to 1 ms, with max observed 3.5 ms
// (three switches sample).
// To have some margin, we take 10 ms
#define BOUNCE_THRESHOLD 10000L

#define LIM_DOWN 0
#define LIM_UP 1

#define N_SWITCHES 2
static const enum dio_pin switchPins[N_SWITCHES] = {
    DIO_PIN_LIM_SWITCH_DOWN_MONITORING,
    DIO_PIN_LIM_SWITCH_UP_MONITORING
};
static const uint32_t switchNotifs[N_SWITCHES] = {
    MOTOR_NOTIF_LIM_DOWN,
    MOTOR_NOTIF_LIM_UP,
};

typedef enum {
    LimSwitchNotPressed,
    LimSwitchPressed,
} LimSwitchState_t;

/** @init_limit_switch Initialize the limit switch handling
 * Sets up the pins config and the ISR.
 */
void init_limit_switch();

LimSwitchState_t get_lim_v(uint8_t sw);

#endif // LIMIT_SWITCH_H_
