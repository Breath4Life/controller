/** Task in charge of handling the LCD display.
 * The display depends the the global state variables of the system.  This task
 * should be notified with the adequate notification when a parameter changes.
 */
#include "FreeRTOS.h"

// Welcome message
#define WELCOME_MSG1            "Breath4Life v1.0"
#define WELCOME_MSG2            "Starting...     "

// Waiting calibration message
#define WAIT_CALI_MSG1          "Press START     "
#define WAIT_CALI_MSG2          "to calibrate.   "

// Calibration error messages
#define PAT_CONNECTED_MSG       "Remove patient !"
#define INC_FLOW_MSG            "Flow-check fail."
#define UNKNOWN_ERROR_MSG       "Unknown error..."
#define RETRY_CALI_MSG          "START to re-try."

// Calibration message
#define CALI_MSG1               "Calibrating...  "
#define CALI_MSG2               "Please wait...  "

// Critical failure messages
#define DOOR_OPEN_MSG           "! DOOR IS OPEN !"
#define MOTOR_ERROR_MSG         "! MOTOR ERROR  !"
#define POWER_ERROR_MSG         "! POWER ERROR  !"
#define CRITICAL_FAILURE_MSG    "RESTART REQUIRED"

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
