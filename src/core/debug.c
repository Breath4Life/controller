#include <stdarg.h>
#include <stdio.h>
#include "hal/uart.h"

void debug_print(const char *fmt, ...)
{
    char buffer[32];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    char *c = buffer;
    while (*c != '\0')
    {
        uart_transmit(*c);
	c++;
    }
}
