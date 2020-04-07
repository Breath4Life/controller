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
#include "core/volume.h"


void debug_print_fromISR(const char *fmt, ...)
{
    char buffer[32];

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

void SFM3000Task(void *pvParameters)
{
    init_volume();

    vTaskDelay(100 / portTICK_PERIOD_MS);


    while (1)
    {

        TickType_t xLastWakeTime;
        const TickType_t xFrequency = pdMS_TO_TICKS(200); // which is around 38 ms on scope

        poll_volume();
        debug_print("vol: %lu\r\n", volume);

        //debug_print("current time [us] %lu\r\n", time_us());

        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
