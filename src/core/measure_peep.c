#include <stdint.h>
#include "core/analog_read.h"

#include "core/utils.h"
#include "core/system.h"
#include "core/analog_read.h"

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
 * - armed: expiration phase has started, tracking flow measuremements
 *   to check where we are in V-shaped region of the flow curve
 * - waiting: entered the V-shaped region of the flow curve, tracking
 *   flow measurements to check when to measure PEEP (i.e., at exit
 *   of the V-shaped region of the flow curve)
 */
typedef enum {
    idle,
    armed,
    waiting
} PEEPMeasurement_t;

// State of the PEEPMeasurement machine
static volatile PEEPMeasurement_t PEEPMeasurementState;
// Measured flow in ml/min
static int32_t flow;

/*
 * Initialize the PEEPMeasurement machine.
 */
void initPEEPMeasurement() {
    DEBUG_PRINT("Init.");
    PEEPMeasurementState = idle;
}

/*
 * Called by MotorControl when entering the expiration
 * phase. Sets the PEEPMeasurementState to armed.
 */
void armPEEPMeasurement() {
    DEBUG_PRINT("Armed.");
    PEEPMeasurementState = armed;
}

/*
 * Periodically called by MainTask.
 * - if PEEPMeasurementState is idle: nothing to do
 * - if PEEPMeasurementState is armed:
 *   - if flow < FLOW_THRESHOLD_ENTRY [ml/min]:
 *      set PEEPMeasurementState to waiting
 * - if PEEPMeasurementState is waiting:
 *   - if flow > FLOW_THRESHOLD_EXIT [ml/min]:
 *      measure_peep()
 *      set PEEPMeasurementState to idle
 */
void pollPEEPMeasurement() {
    switch (PEEPMeasurementState) {
        case idle:
            break;
        case armed:
            get_flow(&flow);
            DEBUG_PRINT("Flow: %li", flow);
            // Entering the V-shaped expiration flow curve
            if (flow < FLOW_THRESHOLD_ENTRY) {
                DEBUG_PRINT("Waiting.");
                PEEPMeasurementState = waiting;
            }
            break;
        case waiting:
            get_flow(&flow);
            DEBUG_PRINT("Flow: %li", flow);
            // Exiting the V-shaped expiration flow curve
            // -> end of expiration, time to measure the PEEP
            if (flow > FLOW_THRESHOLD_EXIT) {
                DEBUG_PRINT("Measuring.");
                measure_peep();
                PEEPMeasurementState = idle;
                DEBUG_PRINT("Idle.");
            }
            break;
    }
}
