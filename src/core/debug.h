#include "FreeRTOS.h"


/**
 * @debug_print Print a debug message on the UART.
 *
 * Print a message to UART0. The message, after formatting, cannot exceed 32 bytes.
 * This function supports the same arguments as printf.
 * Disables interrupts in the critical part of the function.
 */
void debug_print(const char *fmt, ...);

/**
 * @debug_print Print a debug message on the UART.
 *
 * Print a message to UART0. The message, after formatting, cannot exceed 32 bytes.
 * This function supports the same arguments as printf.
 * Called when iterrupts are already disabled
 */
void debug_print_FromISR(const char *fmt, ...);

/**
 * @fake_debug_print Do nothing with varargs
 *
 * Useful to enable debug with pre-processor flags
 */
void fake_debug_print(const char *, ...);

/**
 * @LEDTask Blink a LED every second.
 *
 * @param pvParameters Set to NULL.
 */
void LEDTask(void *pvParameters);

/**
 * @ReadIOTask Read a digital io every second (for testing purpose)
 *
 * @param pvParameters Set to NULL.
 */
void ReadIOTask(void *pvParameters);

/**
 * @ReadAnalogTask Read an analog input every second (for testing purpose)
 *
 * @param pvParameters Set to NULL.
 */
void ReadAnalogTask(void *pvParameters);

/**
 * @SFM3000Task Test SFM3000 (with I2C communication for testing purpose)
 *
 * @param pvParameters Set to NULL.
 */
void SFM3000Task(void *pvParameters);
