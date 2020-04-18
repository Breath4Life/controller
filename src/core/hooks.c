/** Hooks for FreeRTOS */

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "core/system.h"
#include "hal/motor.h"
#include "hal/tone.h"

static void uart_transmit_char(char data) {
    while ((UCSR0A & (1 << UDRE0)) == 0);
    UDR0 = data;
}
static void uart_transmit_str(const char *data) {
    while (*data != '\0') {
        uart_transmit_char(*data);
        data++;
    }
}
 
void vApplicationStackOverflowHook( TaskHandle_t xTask __attribute__((unused)), char *pcTaskName)
{
    /*---------------------------------------------------------------------------*\
    Usage:
       called by task system when a stack overflow is noticed
    Description:
       Stack overflow handler Shut down all interrupts, send serious complaint
        to command port. SLOW Blink.
    Arguments:
       pxTask - pointer to task handle
       pcTaskName - pointer to task name
    Results:
       <none>
    Notes:
       This routine will never return.
       This routine is referenced in the task.c file of FreeRTOS as an extern.
    \*---------------------------------------------------------------------------*/
    cli();

    // Shutting down everything
    motor_disable();

    // warn everybody
    tone_start(565, true);
    uart_transmit_str("\r\nSTACK OVERFLOW ");
    uart_transmit_str(pcTaskName);
    uart_transmit_str("\r\n");

    // Slow blink LED - from libminiavrfreertos port
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.
    for(;;)
    {
        _delay_ms(2000);
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.
    }
}
