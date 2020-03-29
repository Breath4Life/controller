#include <stdio.h>
#include <avr/io.h>

// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "core/debug.h"

#include "hal/uart.h"

void UART_Task(void *pvParameters)
{
    int n = 0;
    while (1) {
        for(int i=0; i <1000; i++)
	{
            for(int j=0; j <1000; j++);
	}
        debug_print("Hello world: %d!\r\n", n);
        n++;
    }
}

int main(void)
{
    // Initialize the hardware
    uart_init();

    // Create the different tasks
    xTaskCreate(    UART_Task,
                    (const char *) "UART",
                    256,
                    NULL,
                    10,
                    NULL);

    // Run the OS
    vTaskStartScheduler();
    for(;;);

    return 0;
}


