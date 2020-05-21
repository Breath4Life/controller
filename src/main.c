#include <stdio.h>
#include <avr/io.h>

// FreeRTOS header files
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

#include "core/system.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "core/display.h"
#include "core/buzzer.h"
#include "core/main_task.h"
#include "core/analog_read.h"
#include "core/volume.h"
#include "core/eeprom.h"
#include "core/alarm.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/uart.h"
#include "hal/lcd.h"
#include "hal/time.h"
#include "hal/power_monitoring.h"
#include "hal/door_open.h"

TaskHandle_t mainTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t buzzerTaskHandle;
TaskHandle_t lcdDisplayTaskHandle;
TaskHandle_t analogReadTaskHandle;

void initHardware(void)
{
    init_time();

    uart_init();

    dio_init(DIO_PIN_DEBUGLED,                DIO_OUTPUT);

    dio_init(DIO_PIN_I2C_FLOW_SENSOR_DATA,    DIO_OUTPUT);
    dio_init(DIO_PIN_I2C_FLOW_SENSOR_CLOCK,   DIO_OUTPUT);

    //dio_init(DIO_PIN_AUX_ALARM_GATE_PIN,      DIO_INPUT);

    dio_init(DIO_PIN_LCD_EN,                  DIO_OUTPUT);
    dio_init(DIO_PIN_LCD_RS,                  DIO_OUTPUT);
    dio_init(DIO_PIN_LCD_RW,                  DIO_OUTPUT);
    dio_init(DIO_PIN_LCD_D4,                  DIO_OUTPUT);
    dio_init(DIO_PIN_LCD_D5,                  DIO_OUTPUT);
    dio_init(DIO_PIN_LCD_D6,                  DIO_OUTPUT);
    dio_init(DIO_PIN_LCD_D7,                  DIO_OUTPUT);
    dio_write(DIO_PIN_LCD_RW,                 DIO_LOW);

    lcd_initLCD();

    init_motor();

    initMainTask();

    init_buzzer();

    init_power_monitoring();

    init_volume();

    door_open_init();

    initAlarm();

    init_eeprom();
}

int main(void)
{
    initHardware();

    // Create the different tasks
    xTaskCreate(MainTask, "MainTask", 512, NULL, 12, &mainTaskHandle);
    xTaskCreate(MotorControlTask, "MotorControl", 1024, NULL, 10, &motorControlTaskHandle);
    xTaskCreate(BuzzerTask, "Buzzer", 256, NULL, 4, &buzzerTaskHandle);
    xTaskCreate(LCDDisplayTask, "LCDDisplay", 512, NULL, 3, &lcdDisplayTaskHandle);
    xTaskCreate(AnalogReadTask, "ReadAnalog", 256, NULL, 2, &analogReadTaskHandle);

    // Run the OS
    vTaskStartScheduler();
    for(;;);

    return 0;
}
