#include "FreeRTOS.h"

#define WELCOME_MSG1 "welcome1 d    "
#define WELCOME_MSG2 "welcome2     "
#define WAIT_CALI_MSG1 "walibration1 "
#define WAIT_CALI_MSG2 "walibration2 "
#define CALI_MSG1 "calibration1  "
#define CALI_MSG2 "calibration2  "

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
