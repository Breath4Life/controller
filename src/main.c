#include <stdio.h>
#include <avr/io.h>

// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "core/debug.h"
#include "core/motor_control.h"
#include "core/display.h"
#include "core/ui.h"
#include "core/alarms.h"

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
    xTaskCreate(MotorControlTask,  (const char *) "MotorControlTask",  256, NULL, 10, NULL);
    xTaskCreate(UserInterfaceTask, (const char *) "UserInterfaceTask", 64,  NULL,  8, NULL);
    xTaskCreate(LCDDisplayTask,    (const char *) "LCDDisplayTask",    64,  NULL,  5, NULL);
    xTaskCreate(AlarmsTask,        (const char *) "AlarmsTask",        64,  NULL,  4, NULL);
    xTaskCreate(LEDTask,           (const char *) "LEDTask",           128, NULL,  1, NULL);
    xTaskCreate(ReadIOTask,        (const char *) "ReadIOTask",        128, NULL,  1, NULL);

    // Run the OS
    vTaskStartScheduler();
    for(;;);

    return 0;
}


