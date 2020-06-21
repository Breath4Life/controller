#include <stdint.h>

#include <avr/interrupt.h>

#include "core/utils.h"
#include "core/system.h"
#include "core/analog_read.h"
#include "core/motor_control.h"
#include "hal/time.h"

#define CURR_DEBUG_PREFIX measurePEEP
#include "core/debug.h"

#if DEBUG_MEASURE_PEEP
#define DEBUG_PRINT debug_print_prefix
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_MEASURE_PEEP

// Flow thresholds used to detect entry/exit of V-shaped
// flow curve of the expiration phase [ml/min]
#define FLOW_THRESHOLD_ENTRY -5000L
#define FLOW_THRESHOLD_EXIT -3000L

/*
 * Possible states of the PEEPMeasurement machine
 * - idle: PEEP measurement has been done for the current cycle
 *   (also the initial state)
 * - wait_high_flow: expiration phase has started, tracking flow measuremements
 *   to check where we are in V-shaped region of the flow curve
 * - wait_low_flow: entered the V-shaped region of the flow curve, tracking
 *   flow measurements to check when to measure PEEP (i.e., at exit
 *   of the V-shaped region of the flow curve)
 * - wait_low_p: after peep detection, wait for inspiration detection
 */
typedef enum {
    idle,
    wait_high_flow,
    wait_low_flow,
    wait_low_p,
} PEEPMeasurement_t;

// State of the PEEPMeasurement machine
static volatile PEEPMeasurement_t PEEPMeasurementState;
// Measured flow in ml/min
static int32_t last_flow;

/*
 * Initialize the PEEPMeasurement machine.
 */
void initPEEPMeasurement() {
    DEBUG_PRINT("Init");
    PEEPMeasurementState = idle;
}

/*
 * Called by MotorControl when entering the expiration
 * phase. Sets the PEEPMeasurementState to wait_high_flow.
 */
void armPEEPMeasurement() {
    DEBUG_PRINT("wait_high_flow");
    PEEPMeasurementState = wait_high_flow;
}

void resetPEEPMeasurement() {
    DEBUG_PRINT("idle");
    PEEPMeasurementState = idle;
}

/*
 * Periodically called by MainTask.
 * - if PEEPMeasurementState is idle: nothing to do
 * - if PEEPMeasurementState is wait_high_flow:
 *   - if last_flow < FLOW_THRESHOLD_ENTRY [ml/min]:
 *      set PEEPMeasurementState to wait_low_flow
 * - if PEEPMeasurementState is wait_low_flow:
 *   - if last_flow > FLOW_THRESHOLD_EXIT [ml/min]:
 *      measure_peep()
 *      set PEEPMeasurementState to idle
 */
void pollPEEPMeasurement() {
    switch (PEEPMeasurementState) {
        case idle:
            break;
        case wait_high_flow:
            get_flow(&last_flow);
            // Entering the V-shaped expiration flow curve
            if (last_flow < FLOW_THRESHOLD_ENTRY) {
                DEBUG_PRINT("Flow %li", last_flow);
                DEBUG_PRINT("wait_low_flow");
                PEEPMeasurementState = wait_low_flow;
            }
            break;
        case wait_low_flow:
            get_flow(&last_flow);
            // Exiting the V-shaped expiration flow curve
            // -> end of expiration, time to measure the PEEP
            if (last_flow > FLOW_THRESHOLD_EXIT) {
                DEBUG_PRINT("Flow %li", last_flow);
                DEBUG_PRINT("Measuring");
                measure_peep();
                PEEPMeasurementState = wait_low_p;
                DEBUG_PRINT("wait_low_p");
            }
            break;
        case wait_low_p:
            cli();
            int16_t p_loc = p;
            sei();
            int16_t dp = p_loc - peep;
            if (dp < DP_THRESH_INSP) {
                DEBUG_PRINT("Detected");
#if SEND_TO_SERIAL
                debug_print(":dp:%lu:%i\r\n", time_us(), dp);
#endif
                motorStartInsp();
                PEEPMeasurementState = idle;
            }
            break;
    }
}
