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

#define DEBUG_LIM_SWITCH 0

// Last read state of the switches
static volatile uint8_t lim_switch_down_lvl;
static volatile uint8_t lim_switch_up_lvl;

// Duration [us] for the debouncing algorithm.
const uint32_t threshold_time_irq = 10000;
// Last time [us] of trigger of the debouncing algorithm.
static uint32_t last_start_bouncing = 0;

void init_limit_switch() {
    // init pins
    dio_init(DIO_PIN_LIM_SWITCH_DOWN_MONITORING, DIO_INPUT_PULLUP);
    dio_init(DIO_PIN_LIM_SWITCH_UP_MONITORING, DIO_INPUT_PULLUP);
    // current levels
    lim_switch_down_lvl = dio_read(DIO_PIN_LIM_SWITCH_DOWN_MONITORING);
    lim_switch_up_lvl = dio_read(DIO_PIN_LIM_SWITCH_UP_MONITORING);
    // enable level change interrupt (on PCINT7:0)
    PCMSK0 =  _BV(PCINT0) | _BV(PCINT1);
    PCICR |= _BV(PCIE0);
}

// We need to read the levels of the pins in the ISR since the interrupt is non-specific:
// we don't know which pin toggled and in which direction.
uint8_t get_lim_down_v() {
    if (time_us() - last_start_bouncing > threshold_time_irq) {
        return dio_read(DIO_PIN_LIM_SWITCH_DOWN_MONITORING);
    } else {
        return lim_switch_down_lvl;  
    }
}

uint8_t get_lim_up_v() {
    if (time_us() - last_start_bouncing > threshold_time_irq) {
        return dio_read(DIO_PIN_LIM_SWITCH_UP_MONITORING);
    } else {
        return lim_switch_up_lvl;  
    }
}

ISR(PCINT0_vect) {
    uint8_t l_down = dio_read(DIO_PIN_LIM_SWITCH_DOWN_MONITORING);
    uint8_t l_up = dio_read(DIO_PIN_LIM_SWITCH_UP_MONITORING);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    uint32_t current_time_irq = time_us();
    uint8_t flagToggle;
    flagToggle = (l_up == lim_switch_up_lvl && l_down == lim_switch_down_lvl);
    if (flagToggle) {
        return;
    }
    if (current_time_irq - last_start_bouncing > threshold_time_irq) {
        if (l_down && !lim_switch_down_lvl) {
            xTaskNotifyFromISR(SWITCH_HANDLING_TASK, LIM_SWITCH_DOWN, eSetBits, &higherPriorityTaskWoken);
#if DEBUG_LIM_SWITCH
            debug_print_FromISR("switch down pressed\r\n");
#endif // DEBUG_LIM_SWITCH
        }
        if (l_up && !lim_switch_up_lvl) {
            xTaskNotifyFromISR(SWITCH_HANDLING_TASK, LIM_SWITCH_UP, eSetBits, &higherPriorityTaskWoken);
#if DEBUG_LIM_SWITCH
            debug_print_FromISR("switch up pressed\r\n");
#endif // DEBUG_LIM_SWITCH
        }
        //last_start_bouncing = current_time_irq;
    }
    if(!flagToggle) {
        last_start_bouncing = current_time_irq;
    }
    lim_switch_down_lvl = l_down;
    lim_switch_up_lvl = l_up;
    if (higherPriorityTaskWoken) {
        taskYIELD();
    }
}
