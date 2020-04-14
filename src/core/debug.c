#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/uart.h"
#include "hal/io.h"
#include "hal/pins.h"
#include "hal/i2c.h"
#include "hal/time.h"
#include "core/debug.h"
#include "core/volume.h"


void debug_print_FromISR(const char *fmt, ...)
{
    char buffer[128];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    uart_transmit(buffer);
}

void debug_print(const char *fmt, ...)
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

void fake_debug_print(const char *_fmt, ...)
{
}

