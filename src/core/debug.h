/** Debug print functions.
 */
#ifndef DEBUG_H_
#define DEBUG_H_
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
//#define fake_debug_print(...) do (__VA_ARGS__)
void fake_debug_print(const char *, ...);

#endif //DEBUG_H_
