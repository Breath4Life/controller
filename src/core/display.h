/** Task in charge of handling the LCD display.
 * The display depends the the global state variables of the system.  This task
 * should be notified with the adequate notification when a parameter changes.
 */
#include "FreeRTOS.h"

#define WELCOME_MSG1            "Breath4Life v1.0"
#define WELCOME_MSG2            "Starting...     "
#define WAIT_CALI_MSG1          "Press START     "
#define WAIT_CALI_MSG2          "to calibrate.   "
#define PAT_CONNECTED_MSG       "Remove patient !"
#define INC_FLOW_MSG            "Flow-check fail."
#define RETRY_CALI_MSG          "START to re-try."
#define CALI_MSG1               "Calibrating...  "
#define CALI_MSG2               "Please wait...  "
#define CRITICAL_FAILURE_MSG1   "CRITICAL FAILURE"
#define CRITICAL_FAILURE_MSG2   "RESTART REQUIRED"

// The alaram state changed.
#define DISP_NOTIF_ALARM 0x01
// A parameter changed.
#define DISP_NOTIF_PARAM 0x02
// The plateau pressure changed.
#define DISP_NOTIF_PLATEAU_P 0x04
// The peak pressure changed.
#define DISP_NOTIF_PEAK_P 0x08
// The global state changed.
#define DISP_NOTIF_STATE 0x10

/**
 * @LCDDisplayTask Handle the display of the LCD screen.
 *
 * @param pvParameters Set to NULL.
 */
void LCDDisplayTask(void *pvParameters);
