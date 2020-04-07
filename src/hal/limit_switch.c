
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

static volatile uint8_t lim_switch0_lvl;
static volatile uint8_t lim_switch1_lvl;

const uint32_t threshold_time_irq = 2000000;
static uint32_t deadline_time_irq;
static uint32_t current_time_irq;

void init_limit_switch() {
    // init pins
    dio_init(DIO_PIN_LIM_SWITCH_0_MONITORING, DIO_INPUT_PULLUP);
    dio_init(DIO_PIN_LIM_SWITCH_1_MONITORING, DIO_INPUT_PULLUP);
    // current levels
    lim_switch0_lvl = dio_read(DIO_PIN_LIM_SWITCH_0_MONITORING);
    lim_switch1_lvl = dio_read(DIO_PIN_LIM_SWITCH_1_MONITORING);
    // enable level change interrupt (on PCINT7:0)
    PCMSK0 =  _BV(PCINT5) | _BV(PCINT6);
    PCICR |= _BV(PCIE0);
    current_time_irq = 0;
    deadline_time_irq = current_time_irq+ threshold_time_irq; 
     
}


ISR(PCINT0_vect) {
    uint8_t l0 = dio_read(DIO_PIN_LIM_SWITCH_0_MONITORING);
    uint8_t l1 = dio_read(DIO_PIN_LIM_SWITCH_1_MONITORING);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    current_time_irq = time_us();
    if (l0 && !lim_switch0_lvl) {
        if((deadline_time_irq-current_time_irq) & 0x80000000){
            xTaskNotifyFromISR(motorControlTaskHandle, MOTOR_NOTIF_LIM_DOWN, eSetBits, &higherPriorityTaskWoken);
            deadline_time_irq = current_time_irq + threshold_time_irq;
        }
#if DEBUG_LIM_SWITCH
        debug_print_FromISR("switch0 pressed\r\n");
#endif // DEBUG_LIM_SWITCH
    }
    if (l1 && !lim_switch1_lvl) {
        if((deadline_time_irq-current_time_irq) & 0x80000000){
            xTaskNotifyFromISR(motorControlTaskHandle, MOTOR_NOTIF_LIM_UP, eSetBits, &higherPriorityTaskWoken);
            deadline_time_irq += current_time_irq + threshold_time_irq;
        }
#if DEBUG_LIM_SWITCH
        debug_print_FromISR("switch1 pressed\r\n");
#endif // DEBUG_LIM_SWITCH
    }
    lim_switch0_lvl = l0;
    lim_switch1_lvl = l1;
    if (higherPriorityTaskWoken) {
        taskYIELD();
    }
}
