#include <stdio.h>
#include <avr/io.h>

// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "core/debug.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/uart.h"

void initHardware(void)
{
    uart_init();
    dio_init(DIO_PIN_DEBUGLED, DIO_OUTPUT);
}

int main(void)
{
    initHardware();

    // Create the different tasks
    xTaskCreate(LEDTask, (const char *) "LEDTask", 256, NULL, 1, NULL);

    // Run the OS
    vTaskStartScheduler();
    for(;;);

    return 0;
}


