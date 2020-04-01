#include <stdint.h>

enum dio_mode {
    DIO_INPUT,
    DIO_INPUT_PULLUP,
    DIO_OUTPUT,
    DIO_DISABLED
};

enum dio_value {
    DIO_LOW,
    DIO_HIGH
};

/**
 * @dio_init Initialize a digital IO pin.
 * If the pin is set in output mode, the level is set to DIO_LOW.
 *
 * @param pin A digital IO pin (e.g. DIO_PIN_STARTSTOP).
 * @param mode DIO_INPUT / DIO_OUTPUT / DIO_DISABLED
 */
void dio_init(uint8_t pin, uint8_t mode);

/**
 * @dio_read Read from a digital IO pin in input mode.
 *
 * @param pin A digital IO pin (e.g. DIO_PIN_STARTSTOP).
 * @return Return either DIO_LOW or DIO_HIGH.
 */
uint8_t dio_read(uint8_t pin);

/**
 * @dio_write Set the level of a digital IO pin in output mode.
 *
 * @param pin A digital IO pin (e.g. DIO_PIN_STARTSTOP).
 * @param level DIO_LOW or DIO_HIGH
 */
void dio_write(uint8_t pin, uint8_t level);

/**
 * @aio_read Read from an analog input pin.
 *
 * @param pin An analog input pin.
 * @return Return the analog reading (0 for 0V - 1023 for 5V)
 */
uint16_t aio_read(uint8_t pin);

