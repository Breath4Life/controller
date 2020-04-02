#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "hal/uart.h"
#include "hal/io.h"
#include "hal/pins.h"
#include "hal/i2c.h"


SemaphoreHandle_t debug_print_semaphore;

void init_debug_print_sem() {
    debug_print_semaphore = xSemaphoreCreateMutex();
}

void debug_print(const char *fmt, ...)
{
    char buffer[32];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    char *c = buffer;
    const int timeout_ticks = 100;
    if (xSemaphoreTake(debug_print_semaphore, timeout_ticks) == pdTRUE)
    {
        while (*c != '\0')
        {
            // TODO: put mutex on uart_transmit
            uart_transmit(*c);
            c++;
        }
        xSemaphoreGive(debug_print_semaphore);
    }
    else
    {
        // What to do here ?
    }
}

void TestI2CTask(void *pvParameters)
{
    const uint8_t i2c_address = 64;
    uint32_t counter = 0;

    i2c_begin();
    i2c_beginTransmission(i2c_address);
    i2c_write(0x10);
    i2c_write(0x00);
    i2c_endTransmission(0);

    vTaskDelay(100 / portTICK_PERIOD_MS);


    while (1)
    {
        i2c_requestFrom(i2c_address, 3);
        uint8_t a = i2c_read();
        uint8_t b = i2c_read();
        uint8_t c = i2c_read();
        (void) c;
        i2c_endTransmission(0);

        if (counter++ % 5 == 0)
        {
            debug_print("sfm3000:%d %d\r\n",  configTICK_RATE_HZ, (a << 8) | b);
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}