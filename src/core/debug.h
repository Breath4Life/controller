/** Debug print functions.
 */
#ifndef DEBUG_H_
#define DEBUG_H_

#include <avr/pgmspace.h>

#include "FreeRTOS.h"

typedef enum {
    motor,
    mainTask,
    analogRead,
    buzzerTsk,
    volumeDbg,
    parametersDbg,
} DebugPrefix_t;


/**
 * @debug_print Print a debug message on the UART.
 *
 * Print a message to UART0. The message, after formatting, cannot exceed 32 bytes.
 * This function supports the same arguments as printf.
 * Disables interrupts in the critical part of the function.
 */
#define debug_print(fmt, ...) do { _debug_print(fmt, ##__VA_ARGS__); } while (0)
void _debug_print(const char *fmt, ...);

/**
 * @debug_print Print a debug message on the UART.
 *
 * Print a message to UART0. The message, after formatting, cannot exceed 32 bytes.
 * This function supports the same arguments as printf.
 * Called when iterrupts are already disabled
 */
#define debug_print_FromISR(fmt, ...) do { _debug_print_FromISR(fmt, ##__VA_ARGS__  ); } while (0)
void _debug_print_FromISR(const char *fmt, ...);

/**
 * @fake_debug_print Do nothing with varargs
 *
 * Useful to enable debug with pre-processor flags
 */
//#define fake_debug_print(...) do (__VA_ARGS__)
#define fake_debug_print(fmt, ...) do {} while (0)
//void _fake_debug_print(const char *, ...);

#define debug_print_prefix(...) do { _debug_print_prefix(CURR_DEBUG_PREFIX, __VA_ARGS__); } while (0)
void _debug_print_prefix(DebugPrefix_t prefix, const char *fmt, ...);


#endif //DEBUG_H_
