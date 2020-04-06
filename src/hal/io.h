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
 * @aio_read_start Prepare from an analog input pin.
 *  Starts the ADC.
 *
 * @param pin An analog input pin.
 */
void aio_read_start(uint8_t pin);

/**
 * @aio_ready Test if the analog reading is finished.
 *
 * @return Return 1 if the ADC has finished conversion, otherwise 0.
 */
uint8_t aio_ready();

/**
 * @aio_read_result Read the result from the ADC
 *
 * @return Return the analog reading (0 for 0V - 1023 for 5V)
 */
uint16_t aio_read_result();

