#include "FreeRTOS.h"

#define WELCOME_MSG1            "Breath4Life v1.0"
#define WELCOME_MSG2            "Starting...     "
#define WAIT_CALI_MSG1          "Press START     "
#define WAIT_CALI_MSG2          "to calibrate.   "
#define CALI_MSG1               "Calibrating...  "
#define CALI_MSG2               "Please wait...  "
#define CRITICAL_FAILURE_MSG1   "CRITICAL FAILURE"
#define CRITICAL_FAILURE_MSG2   "RESTART REQUIRED"

#define DISP_NOTIF_ALARM 0x01
#define DISP_NOTIF_PARAM 0x02
#define DISP_NOTIF_INST_P 0x04
#define DISP_NOTIF_PEAK_P 0x08
#define DISP_NOTIF_STATE 0x10

/**
 * @LCDDisplayTask Handle the display of the LCD screen.
 *
 * @param pvParameters Set to NULL.
 */
void LCDDisplayTask(void *pvParameters);
