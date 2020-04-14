#include "hal/pins.h"

/* ==== PHYSICAL PIN MAPPING  ==== */
// See: https://www.arduino.cc/en/Hacking/PinMapping2560 */

struct io_pin_config DIO_PIN_CONFIG[] = {

    {DIO_PIN_MOTOR_UART_TX,           IO_PORTE, 0},
    {DIO_PIN_MOTOR_UART_RX,           IO_PORTE, 1},

    {DIO_PIN_MOTOR_STEP,              IO_PORTE, 4},
    {DIO_PIN_MOTOR_STEP_BIS,          IO_PORTE, 5},
    {DIO_PIN_MOTOR_ENABLE,            IO_PORTH, 3},
    {DIO_PIN_MOTOR_DIRECTION,         IO_PORTE, 3},
    {DIO_PIN_MOTOR_ERROR,             IO_PORTH, 4},

    {DIO_PIN_AUX_POWER_MONITORING,    IO_PORTH, 6},
    {DIO_PIN_MAIN_POWER_MONITORING,   IO_PORTB, 4},

    {DIO_PIN_LIM_SWITCH_DOWN_MONITORING, IO_PORTB, 0},
    {DIO_PIN_LIM_SWITCH_UP_MONITORING, IO_PORTB, 1},

    {DIO_PIN_DOOR_OPEN,                IO_PORTB, 2},

    {DIO_PIN_DEBUGLED,                IO_PORTJ, 0},

    {DIO_PIN_I2C_FLOW_SENSOR_DATA,    IO_PORTD, 0},
    {DIO_PIN_I2C_FLOW_SENSOR_CLOCK,   IO_PORTD, 1},

    {DIO_PIN_BUTTON_RIGHT,            IO_PORTA, 0},
    {DIO_PIN_BUTTON_LEFT,             IO_PORTA, 1},
    {DIO_PIN_BUTTON_UP,               IO_PORTA, 2},
    {DIO_PIN_BUTTON_DOWN,             IO_PORTA, 3},
    {DIO_PIN_BUTTON_VTIDAL_UP,        IO_PORTA, 4},
    {DIO_PIN_BUTTON_VTIDAL_DOWN,      IO_PORTA, 7},
    {DIO_PIN_BUTTON_FREQ_RESPI_DOWN,  IO_PORTA, 5},
    {DIO_PIN_BUTTON_FREQ_RESPI_UP,    IO_PORTA, 6},
    {DIO_PIN_BUTTON_ALARM_ACK,        IO_PORTG, 0},

    {DIO_PIN_LCD_EN,                  IO_PORTC, 4},
    {DIO_PIN_LCD_RS,                  IO_PORTC, 5},
    {DIO_PIN_LCD_RW,                  IO_PORTC, 6},
    {DIO_PIN_LCD_D4,                  IO_PORTC, 0},
    {DIO_PIN_LCD_D5,                  IO_PORTC, 1},
    {DIO_PIN_LCD_D6,                  IO_PORTC, 2},
    {DIO_PIN_LCD_D7,                  IO_PORTC, 3},

    {DIO_PIN_AUX_ALARM_GATE_PIN,      IO_PORTG, 1},

    {DIO_PIN_BUTTON_ALARM_MUTE,       IO_PORTD, 7},
    {DIO_PIN_BUTTON_STARTSTOP,        IO_PORTG, 2},

    {DIO_PIN_ALARM_SOUND,             IO_PORTB, 7},
    {DIO_PIN_ALARM_SOUND_BIS,         IO_PORTG, 5},
    {DIO_PIN_ALARM_LED_HPA,           IO_PORTL, 4},
    {DIO_PIN_ALARM_LED_LPA,           IO_PORTL, 3},
    {DIO_PIN_STEP_COUNTER_TN,         IO_PORTL, 2},
    {DIO_PIN_ALARM_LED_PAUSED,        IO_PORTL, 1},
    {DIO_PIN_LED_NORMAL_STATE,        IO_PORTL, 0},
};

// The order of declaration in this array needs to be the same as for aio_pin
struct io_pin_config AIO_PIN_CONFIG[] = {
    {AIO_PIN_PRESSURE_SENSOR_0, IO_PORTF, 0},
    {AIO_PIN_PRESSURE_SENSOR_1, IO_PORTF, 3},
    {AIO_PIN_PRESSURE_SENSOR_2, IO_PORTF, 4},
    {AIO_PIN_TEMP_SENSOR_0,     IO_PORTF, 1},
    {AIO_PIN_TEMP_SENSOR_1,     IO_PORTF, 2},
    {AIO_PIN_TEMP_SENSOR_2,     IO_PORTF, 5},

    {AIO_PIN_TEST_1,            IO_PORTF, 6},
    {AIO_PIN_TEST_2,            IO_PORTK, 0},
};
