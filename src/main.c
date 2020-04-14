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
#include "core/alarms.h"
#include "core/main_task.h"
#include "core/analog_read.h"
#include "core/volume.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/uart.h"
#include "hal/lcd.h"
#include "hal/time.h"
#include "hal/power_monitoring.h"

TaskHandle_t mainTaskHandle;
TaskHandle_t motorControlTaskHandle;
TaskHandle_t lcdDisplayTaskHandle;
TaskHandle_t analogReadTaskHandle;
TaskHandle_t alarmsTaskHandle;
TaskHandle_t sfm3000TaskHandle;

void initHardware(void)
{
    init_time();

    uart_init();

    dio_init(DIO_PIN_MOTOR_UART_TX,           DIO_INPUT);
    dio_init(DIO_PIN_MOTOR_UART_RX,           DIO_OUTPUT);

    dio_init(DIO_PIN_MOTOR_STEP,              DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_STEP_BIS,          DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_ENABLE,            DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_DIRECTION,         DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_ERROR,              DIO_OUTPUT);

    dio_init(DIO_PIN_DEBUGLED,                DIO_OUTPUT);

    dio_init(DIO_PIN_I2C_FLOW_SENSOR_DATA,    DIO_OUTPUT);
    dio_init(DIO_PIN_I2C_FLOW_SENSOR_CLOCK,   DIO_OUTPUT);

    dio_init(DIO_PIN_AUX_ALARM_GATE_PIN,      DIO_INPUT);

    dio_init(DIO_PIN_ALARM_LED_HPA,           DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_LED_LPA,           DIO_OUTPUT);
    dio_init(DIO_PIN_STEP_COUNTER_TN,         DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_LED_PAUSED,        DIO_OUTPUT);
    dio_init(DIO_PIN_LED_NORMAL_STATE,        DIO_OUTPUT);

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

    init_alarm();

    init_power_monitoring();

    init_volume();
}

int main(void)
{
    initHardware();

    // Create the different tasks
    xTaskCreate(MotorControlTask,  (const char *) "MotorControlTask",  1024, NULL, 10, &motorControlTaskHandle);
    xTaskCreate(MainTask,  (const char *) "MainTask",  512, NULL, 12, &mainTaskHandle);
    xTaskCreate(LCDDisplayTask,    (const char *) "LCDDisplayTask",    512,  NULL,  3, &lcdDisplayTaskHandle);
    xTaskCreate(AlarmsTask,        (const char *) "AlarmsTask",        128,  NULL,  4, &alarmsTaskHandle);
    //xTaskCreate(LEDTask,           (const char *) "LEDTask",           128, NULL,  1, NULL);
    //xTaskCreate(ReadIOTask,        (const char *) "ReadIOTask",        128, NULL,  1, NULL);
    xTaskCreate(AnalogReadTask,    (const char *) "ReadAnalogTask",    128, NULL,  1, &analogReadTaskHandle);

    // Run the OS
    vTaskStartScheduler();
    for(;;);

    return 0;
}
