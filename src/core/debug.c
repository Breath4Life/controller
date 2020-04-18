#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/uart.h"
#include "hal/io.h"
#include "hal/pins.h"
#include "hal/i2c.h"
#include "hal/time.h"
#include "core/debug.h"
#include "core/volume.h"

static const char *debugPrefixStr[] = {
    "[MOTOR] ",
    "[MAIN] ",
    "[ANALOG] ",
    "[BUZZER] ",
    "[VOL] ",
};

void _debug_print_FromISR(const char *fmt, ...)
{
    char buffer[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    uart_transmit(buffer);
}

void _debug_print(const char *fmt, ...)
{
    char buffer[32];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    cli();
    uart_transmit(buffer);
    sei();
}

void _debug_print_prefix(DebugPrefix_t prefix, const char *fmt, ...)
{
    char buffer[32];
    va_list args;
    va_start(args, fmt);

    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    cli();
    uart_transmit(debugPrefixStr[prefix]);
    uart_transmit(buffer);
    uart_transmit("\r\n");
    sei();
}
