/** Limit switch:
 *
 * Low-level details (see limit_switch.h for high-level information):
 * The toggling of limit switches trigger the PCINT0 interrupt. The state of
 * the switches is stored in the lim_switch_down_lvl and lim_switch_up_lvl.
 * Then the level rises (lever coming in contact with the switch), we notify
 * the appropriate task.
 * When switches toggle, the signal bounces, hence we apply a simple debouncing algorithm:
 * when a signal edge is detected, a notification is sent if needed, and then
 * the sending of notifications is disabled for a fixed amount of time.
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/limit_switch.h"
#include "hal/time.h"
#include "hal/pins.h"
#include "hal/io.h"

#include "core/debug.h"
#include "core/system.h"

#if DEBUG_LIM_SWITCH
#define DEBUG_PRINT debug_print
#define DEBUG_PUTS_FROM_ISR puts_FromISR
#define DEBUG_PINT_FROM_ISR print_c_FromISR
#else
#define DEBUG_PRINT fake_debug_print
#define DEBUG_PUTS_FROM_ISR(x) do {} while (0)
#define DEBUG_PINT_FROM_ISR do {} while (0)
#endif // DEBUG_LIM_SWITCH

typedef struct {
    // Last read state of the switches
    LimSwitchState_t last_value;
    // Currently exposed state
    LimSwitchState_t exposed_value;
    // Last time [us] of trigger of the debouncing algorithm.
    uint32_t last_bouncing;
} LimSwitchMachine_t;

volatile LimSwitchMachine_t switchMachines[N_SWITCHES];

void init_limit_switch() {
    // initialize pins and state machines
    uint8_t sw;
    for (sw=0; sw<N_SWITCHES; sw++) {
        dio_init(switchPins[sw], DIO_INPUT_PULLUP);
        switchMachines[sw].last_value = dio_read(switchPins[sw]);
        switchMachines[sw].exposed_value = switchMachines[sw].last_value;
        switchMachines[sw].last_bouncing = 0;
    }
    // enable level change interrupt (on PCINT7:0)
    PCMSK0 =  _BV(PCINT0) | _BV(PCINT1);
    PCICR |= _BV(PCIE0);
}

LimSwitchState_t get_lim_v(uint8_t sw) {
    uint8_t b;
    cli();
    LimSwitchState_t new_value = dio_read(switchPins[sw]);
    LimSwitchState_t old_value = switchMachines[sw].last_value;
    sei();
    if (new_value != old_value) {
        DEBUG_PRINT("LS%x ERR %x %x\r\n", sw, new_value, new_value, old_value);
    }
    cli();
    if (time_us() - switchMachines[sw].last_bouncing > BOUNCE_THRESHOLD) {
        b=0;
        // not bouncing anymore
        switchMachines[sw].exposed_value = switchMachines[sw].last_value;
    } else {
        b=1;
        // bouncing, we keep the value we exposed, that is:
        // for rising edge start exposing 1, for falling edge, keep exposing 1
    }
    sei();
    DEBUG_PRINT("S%x %x %x\r\n", sw, switchMachines[sw].exposed_value, b);
    return switchMachines[sw].exposed_value;
}

ISR(PCINT0_vect) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    uint32_t current_time_irq = time_us();
    uint8_t sw;
    for (sw=0; sw<N_SWITCHES; sw++) {
        LimSwitchState_t new_value = dio_read(switchPins[sw]);
        LimSwitchState_t old_value = switchMachines[sw].last_value;
        DEBUG_PUTS_FROM_ISR("IS");
        DEBUG_PINT_FROM_ISR(sw);
        DEBUG_PINT_FROM_ISR(new_value);
        DEBUG_PINT_FROM_ISR(old_value);
        DEBUG_PUTS_FROM_ISR("\r\n");
        if (new_value != old_value) {
            if (current_time_irq - switchMachines[sw].last_bouncing > BOUNCE_THRESHOLD) {
                // We are not bouncing, let's send notification if we have a rising edge.
                if (new_value == LimSwitchPressed) {
                    switchMachines[sw].exposed_value = LimSwitchPressed;
                    xTaskNotifyFromISR(
                            SWITCH_HANDLING_TASK,
                            switchNotifs[sw],
                            eSetBits,
                            &higherPriorityTaskWoken);
                }
                switchMachines[sw].last_bouncing = current_time_irq;
                switchMachines[sw].last_value = new_value;
            }
        }
    }
    if (higherPriorityTaskWoken) {
        taskYIELD();
    }
}
