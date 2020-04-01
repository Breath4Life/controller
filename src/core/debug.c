#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/uart.h"
#include "hal/io.h"
#include "hal/pins.h"
#include "hal/i2c.h"

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

    while (1)
    {
        debug_print("LED Task: %d!\r\n", n); // TODO: currently present for debug purposes, to remove later on
        dio_write(DIO_PIN_DEBUGLED, n & 1); // toggle debug LED

        vTaskDelay(1000 / portTICK_PERIOD_MS); // sleep 1s
        n++;
    }
}

void ReadIOTask(void *pvParameters)
{
    while (1)
    {
        debug_print("Read IO Task: %d!\r\n", dio_read(DIO_PIN_MAIN_POWER_MONITORING)); // for testing to remove later on

        vTaskDelay(1000 / portTICK_PERIOD_MS); // sleep 1s
    }
}


void ReadAnalogTask(void *pvParameters)
{
    while (1)
    {
        //debug_print("Read Analog Task: %d!\r\n", aio_read(AIO_PIN_TEST_1)); // for testing to remove later on
        //debug_print("Read Analog Task: %d!\r\n", aio_read(AIO_PIN_TEST_2)); // for testing to remove later on

        //debug_print("\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // sleep 1s
    }
}

void TestI2CTask(void *pvParameters)
{
    const uint8_t i2c_address = 64;

    i2c_begin();

    while (1)
    {
        i2c_beginTransmission(i2c_address);
        i2c_write(0x10);
        i2c_write(0x00);
        i2c_endTransmission(0);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        i2c_requestFrom(i2c_address, 3);
        uint8_t a = i2c_read();
        uint8_t b = i2c_read();
        uint8_t c = i2c_read();
        (void) c;
        i2c_endTransmission(0);

        debug_print("sfm3000:%d\r\n",  (a << 8) | b);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}