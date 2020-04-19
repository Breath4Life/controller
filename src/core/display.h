/** Task in charge of handling the LCD display.
 * The display depends the the global state variables of the system.  This task
 * should be notified with the adequate notification when a parameter changes.
 */
#include "FreeRTOS.h"

// Welcome message
#define WELCOME_MSG1            "BREATH4LIFE V1.0"
#define WELCOME_MSG2            "SN:00001 UP:0042"

// Waiting calibration message
#define WAIT_CALI_MSG1          "START to "
#define WAIT_CALI_MSG2          "self-calibrate  "

// Calibration message
#define CALI_MSG1               "Wait...  "
#define CALI_MSG2               "Self-calibrating"

// Calibration error messages
#define CALI_ERROR_MSG1         "ABORTED: "
#define CALI_ERROR_MSG2         "START to retry  "

// Stop and run state messages
#define RUNNING_MSG             "Running"
#define STOPPED_MSG             "Stopped"
#define BLANK_MSG               "       "
#define MUTED_MSG               " MUTED "

// Critical failure messages
#define CRITICAL_FAILURE_MSG1   "CRITICAL:"
#define CRITICAL_FAILURE_MSG2   "RESTART REQUIRED"

// The alarm state changed.
#define DISP_NOTIF_ALARM 0x01
// A parameter changed.
#define DISP_NOTIF_PARAM 0x02
// The plateau pressure changed.
#define DISP_NOTIF_PLATEAU_P 0x04
// The peak pressure changed.
#define DISP_NOTIF_PEAK_P 0x08
// The global state changed.
#define DISP_NOTIF_STATE 0x10
// The mute button has been pressed
#define DISP_NOTIF_MUTE 0x20
// Refresh aggregated notif
#define DISP_NOTIF_REFRESH (DISP_NOTIF_ALARM | DISP_NOTIF_PARAM | DISP_NOTIF_PLATEAU_P | DISP_NOTIF_PEAK_P | DISP_NOTIF_STATE)

/**
 * @LCDDisplayTask Handle the display of the LCD screen.
 *
 * @param pvParameters Set to NULL.
 */
void LCDDisplayTask(void *pvParameters);
