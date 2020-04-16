/** Pinout of the system.
 */
#ifndef PINS_H_H
#define PINS_H_H

#include <stdint.h>

// Simple structure to hold all pin mappings
struct io_pin_config {
    uint8_t io_pin;
    uint8_t io_port;
    uint8_t pos;
};

/* ==== IO PORTS DECLARATION ==== */
enum io_port {
    IO_PORTA,
    IO_PORTB,
    IO_PORTC,
    IO_PORTD,
    IO_PORTE,
    IO_PORTF,
    IO_PORTG,
    IO_PORTH,
    IO_PORTJ,
    IO_PORTK,
    IO_PORTL
};

/* ==== IO PINS DECLARATION  ==== */
enum dio_pin {
    DIO_PIN_MOTOR_UART_TX,
    DIO_PIN_MOTOR_UART_RX,

    DIO_PIN_MOTOR_STEP,
    DIO_PIN_MOTOR_STEP_BIS,
    DIO_PIN_MOTOR_ENABLE,
    DIO_PIN_MOTOR_DIRECTION,
    DIO_PIN_MOTOR_ERROR,

    DIO_PIN_AUX_POWER_MONITORING,
    DIO_PIN_MAIN_POWER_MONITORING,

    DIO_PIN_LIM_SWITCH_DOWN_MONITORING,
    DIO_PIN_LIM_SWITCH_UP_MONITORING,

    DIO_PIN_DOOR_OPEN,

    DIO_PIN_DEBUGLED,

    DIO_PIN_I2C_FLOW_SENSOR_DATA,
    DIO_PIN_I2C_FLOW_SENSOR_CLOCK,

    DIO_PIN_BUTTON_NEXT,
    DIO_PIN_BUTTON_CONFIRM,
    DIO_PIN_BUTTON_UP,
    DIO_PIN_BUTTON_DOWN,
    DIO_PIN_BUTTON_VTIDAL_UP,
    DIO_PIN_BUTTON_VTIDAL_DOWN,
    DIO_PIN_BUTTON_FREQ_RESPI_DOWN,
    DIO_PIN_BUTTON_FREQ_RESPI_UP,
    DIO_PIN_BUTTON_ALARM_ACK,

    DIO_PIN_LCD_EN,
    DIO_PIN_LCD_RS,
    DIO_PIN_LCD_RW,
    DIO_PIN_LCD_D4,
    DIO_PIN_LCD_D5,
    DIO_PIN_LCD_D6,
    DIO_PIN_LCD_D7,

    DIO_PIN_AUX_ALARM_GATE_PIN,

    DIO_PIN_BUTTON_ALARM_MUTE,
    DIO_PIN_BUTTON_STARTSTOP,

    DIO_PIN_ALARM_SOUND,
    DIO_PIN_ALARM_SOUND_BIS,
    DIO_PIN_ALARM_LED_HPA,
    DIO_PIN_ALARM_LED_LPA,
    DIO_PIN_STEP_COUNTER_TN,
    DIO_PIN_ALARM_LED_PAUSED,
    DIO_PIN_LED_NORMAL_STATE,
};

enum aio_pin {
    AIO_PIN_PRESSURE_SENSOR_0,
    AIO_PIN_PRESSURE_SENSOR_1,
    AIO_PIN_PRESSURE_SENSOR_2,
    AIO_PIN_TEMP_SENSOR_0,
    AIO_PIN_TEMP_SENSOR_1,
    AIO_PIN_TEMP_SENSOR_2,
    AIO_PIN_TEST_1,
    AIO_PIN_TEST_2,
};

// The declaration of the mappings are contained in hal/pins.c.
extern struct io_pin_config DIO_PIN_CONFIG[];
extern struct io_pin_config AIO_PIN_CONFIG[];

#endif // PINS_H_H
