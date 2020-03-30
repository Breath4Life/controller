#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/uart.h"
#include "hal/io.h"
#include "hal/pins.h"

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
	// TODO: put mutex on uart_transmit
        uart_transmit(*c);
	c++;
    }
}

void LEDTask(void *pvParameters)
{
    unsigned char n = 0;

    dio_init(DIO_PIN_DEBUGLED, DIO_OUTPUT);

    while (1)
    {
        debug_print("LED Task: %d!\r\n", n); // TODO: currently present for debug purposes, to remove later on
        dio_write(DIO_PIN_DEBUGLED, n & 1); // toggle debug LED

        vTaskDelay(100 / portTICK_PERIOD_MS); // sleep 1s
        n++;
    }
}

