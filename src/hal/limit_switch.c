
// Limit switches handling
// Limit switches are closed when not pressed and open when pressed.
// We connect them to io pins with pullup, and detect a rising edge.
// This is done through the PCINT (level change interrupt), and we notify
// the motor control task.
// We need to read the levels of the pins in the ISR since the interrupt is non-specific:
// we don't know which pin toggled and in which direction.

#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "time.h"
#include "hal/pins.h"
#include "hal/io.h"
#include "core/debug.h"
#include "core/system.h"
#include "core/motor_control.h"

#define DEBUG_LIM_SWITCH 0

static volatile uint8_t lim_switch_down_lvl;
static volatile uint8_t lim_switch_up_lvl;

const uint32_t threshold_time_irq = 10000;
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


ISR(PCINT0_vect) {
    uint8_t l_down = dio_read(DIO_PIN_LIM_SWITCH_DOWN_MONITORING);
    uint8_t l_up = dio_read(DIO_PIN_LIM_SWITCH_UP_MONITORING);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    uint32_t current_time_irq = time_us();
    if (current_time_irq - last_start_bouncing > threshold_time_irq) {
        if (l_down && !lim_switch_down_lvl) {
            xTaskNotifyFromISR(motorControlTaskHandle, MOTOR_NOTIF_LIM_DOWN, eSetBits, &higherPriorityTaskWoken);
#if DEBUG_LIM_SWITCH
            debug_print_FromISR("switch0 pressed\r\n");
#endif // DEBUG_LIM_SWITCH
        }
        if (l_up && !lim_switch_up_lvl) {
            xTaskNotifyFromISR(motorControlTaskHandle, MOTOR_NOTIF_LIM_UP, eSetBits, &higherPriorityTaskWoken);
#if DEBUG_LIM_SWITCH
            debug_print_FromISR("switch1 pressed\r\n");
#endif // DEBUG_LIM_SWITCH
        }
        last_start_bouncing = current_time_irq;
    }
    lim_switch_down_lvl = l_down;
    lim_switch_up_lvl = l_up;
    if (higherPriorityTaskWoken) {
        taskYIELD();
    }
}
