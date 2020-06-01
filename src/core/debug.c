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
    "[MTR] ",
    "[MAIN] ",
    "[ANALOG] ",
    "[BUZZER] ",
    "[VOL] ",
    "[PARAM] ",
};

static const char hex_chars[]={
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

static void c2hex(const char data, char *buf) {
    if (data >= 16) {
        buf[0]=hex_chars[data >> 4];
        buf+=1;
    }
    buf[0] = hex_chars[data & 0xF];
    buf[1] = 0;
}

void puts_FromISR(const char *str) {
    uart_transmit(str);
}

void print_c_FromISR(const char data) {
    char buf[3];
    c2hex(data, buf);
    uart_transmit(buf);
}


/*
void _debug_print_FromISR(const char *fmt, ...)
{
    char buffer[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    uart_transmit(buffer);
}
*/

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
