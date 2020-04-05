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
#include "core/ui.h"
#include "core/alarms.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/uart.h"
#include "hal/lcd.h"
#include "hal/motor.h"

TaskHandle_t motorControlTaskHandle;
TaskHandle_t userInterfaceTaskHandle;
TaskHandle_t lcdDisplayTaskHandle;
TaskHandle_t alarmsTaskHandle;
TaskHandle_t sfm3000TaskHandle;

#include "hal/timers.h"

void initHardware(void)
{

    init_debug_print_sem();

    uart_init();

    dio_init(DIO_PIN_MOTOR_UART_TX,           DIO_INPUT);
    dio_init(DIO_PIN_MOTOR_UART_RX,           DIO_OUTPUT);

    dio_init(DIO_PIN_MOTOR_STEP,              DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_STEP_BIS,          DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_ENABLE,            DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_DIRECTION,         DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_PDN,               DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_SPRD,              DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_INDEX,             DIO_OUTPUT);
    dio_init(DIO_PIN_MOTOR_DIAG,              DIO_OUTPUT);

    dio_init(DIO_PIN_AUX_POWER_MONITORING,    DIO_INPUT);
    dio_init(DIO_PIN_MAIN_POWER_MONITORING,   DIO_INPUT);

    dio_init(DIO_PIN_LIM_SWITCH_0_MONITORING, DIO_INPUT);
    dio_init(DIO_PIN_LIM_SWITCH_1_MONITORING, DIO_INPUT);

    dio_init(DIO_PIN_DEBUGLED,                DIO_OUTPUT);

    dio_init(DIO_PIN_I2C_FLOW_SENSOR_DATA,    DIO_OUTPUT);
    dio_init(DIO_PIN_I2C_FLOW_SENSOR_CLOCK,   DIO_OUTPUT);

    dio_init(DIO_PIN_BUTTON_RIGHT,            DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_LEFT,             DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_UP,               DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_DOWN,             DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_VTIDAL_UP,        DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_VTIDAL_DOWN,      DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_FREQ_RESPI_DOWN,  DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_FREQ_RESPI_UP,    DIO_INPUT);

    dio_init(DIO_PIN_AUX_ALARM_GATE_PIN,      DIO_INPUT);

    dio_init(DIO_PIN_BUTTON_ALARM_MUTE,       DIO_INPUT);
    dio_init(DIO_PIN_BUTTON_STARTSTOP,        DIO_INPUT);

    dio_init(DIO_PIN_ALARM_SOUND,             DIO_OUTPUT);
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

    clock_init();

    lcd_initLCD();

    setup_motor();
}

int main(void)
{
    initHardware();

    // Create the different tasks

    xTaskCreate(MotorControlTask,  (const char *) "MotorControlTask",  256, NULL, 10, &motorControlTaskHandle);
    xTaskCreate(UserInterfaceTask, (const char *) "UserInterfaceTask", 64,  NULL,  8, &userInterfaceTaskHandle);
    xTaskCreate(LCDDisplayTask,    (const char *) "LCDDisplayTask",    64,  NULL,  5, &lcdDisplayTaskHandle);
    xTaskCreate(AlarmsTask,        (const char *) "AlarmsTask",        64,  NULL,  4, &alarmsTaskHandle);
    //xTaskCreate(LEDTask,           (const char *) "LEDTask",           128, NULL,  1, NULL);
    //xTaskCreate(ReadIOTask,        (const char *) "ReadIOTask",        128, NULL,  1, NULL);
    //xTaskCreate(ReadAnalogTask,    (const char *) "ReadAnalogTask",    128, NULL,  1, NULL);
    xTaskCreate(SFM3000Task,       (const char *) "SFM3000Task",       200, NULL,  1, &sfm3000TaskHandle);

    // Run the OS
    vTaskStartScheduler();
    for(;;);

    return 0;
}
