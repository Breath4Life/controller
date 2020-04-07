#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/lcd.h"
#include "hal/time.h"
#include "core/system.h"
#include "core/display.h"
#include "core/main_task.h"
#include "core/debug.h"
#include "core/analog_read.h"

static void disp_alarm();
static void disp_muted();
static void disp_param();
static void disp_inst_p();
static void disp_peak_p();
static void disp_state();

#define DEBUG_LCD 0

uint32_t muted_switch_time = 0;
uint8_t muted_msg_on = 0;

void LCDDisplayTask(void *pvParameters)
{
#if DEBUG_LCD
    debug_print("[LCD] Starting.\r\n");
#endif

    // Wait 100ms (otherwhise welcome message not printed)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    lcd_initLCD();
    lcd_write_string(WELCOME_MSG1, 1, 1, NO_CR_LF);
    lcd_write_string(WELCOME_MSG2, 2, 1, NO_CR_LF);


    while (1) {
        uint32_t notification;
        uint32_t curr_time = time_us();

        // Display "Muted" and the current state/alarm alternatively for 2 seconds if mute_on
        if (mute_on) {
            if (curr_time - muted_switch_time > 2000000L) {
                if (!muted_msg_on) {
                    disp_muted();
                } else {
                    if (alarmState == noAlarm) {
                        disp_state();
                    } else {
                        disp_alarm();
                    }
                }

                muted_msg_on = !muted_msg_on;
                muted_switch_time = curr_time;
            }
        } else {
            if (muted_msg_on) {
                muted_msg_on = !muted_msg_on;
                if (alarmState == noAlarm) {
                    disp_state();
                } else {
                    disp_alarm();
                }
            }
        }

        // FIXME: max wait has been changed to implement alternate mute/state/alarm display
        BaseType_t notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, pdMS_TO_TICKS(1000));
        if (notification & DISP_NOTIF_ALARM) {
#if DEBUG_LCD
            debug_print("[LCD] rcvd notif ALARM.\r\n");
#endif
            disp_alarm();
        }
        if (notification & DISP_NOTIF_PARAM) {
#if DEBUG_LCD
            debug_print("[LCD] rcvd notif param.\r\n");
#endif
            disp_param();
        }
        if (notification & DISP_NOTIF_INST_P) {
#if DEBUG_LCD
            debug_print("[LCD] rcvd notif inst p.\r\n");
#endif
            disp_inst_p();
        }
        if (notification & DISP_NOTIF_PEAK_P) {
#if DEBUG_LCD
            debug_print("[LCD] rcvd notif peak p.\r\n");
#endif
            disp_peak_p();
        }
        if (notification & DISP_NOTIF_STATE) {
#if DEBUG_LCD
            debug_print("[LCD] rcvd notif state.\r\n");
#endif
            disp_state();
        }


        lcd_refreshLCD();
    }
}

static void disp_alarm() {
    char alarm_buffer[9];

    switch(errorCode) {
        case overPressure:
            sprintf(alarm_buffer, " MXPSR ");
            break;
        case noPressure:
            sprintf(alarm_buffer, " NOPSR ");
            break;
        case highPressure:
            sprintf(alarm_buffer, " HOPSR ");
            break;
        case highTemperature:
            sprintf(alarm_buffer, " MXTPM ");
            break;
        case lowPressure:
            sprintf(alarm_buffer, " LOPSR ");
            break;
        case abnVolume:
            sprintf(alarm_buffer, " TIDALV");
            break;
        case abnFreq:
            sprintf(alarm_buffer, " RESPR ");
            break;
        default:
            sprintf(alarm_buffer, " ALARM ");
    }

    lcd_write_string(alarm_buffer, 1, 10, NO_CR_LF);
}

static void disp_muted() {
    char buffer[9];

    sprintf(buffer, " MUTED ");
    lcd_write_string(buffer, 1, 10, NO_CR_LF);
}

char param_buffer[17];
static void disp_param() {
    if (extra_param == 0) {
        sprintf(param_buffer, "%2u0 %2u I:E = 1:%1u", tidal_vol, bpm, ie);
    } else {
        sprintf(param_buffer, "%2u0 %2u Pmax = %2u", tidal_vol, bpm, p_max);
    }
    lcd_write_string(param_buffer,2,1,NO_CR_LF);
}

char inst_p_buffer[5];
static void disp_inst_p() {
    sprintf(inst_p_buffer, "P%2i ", p);
    lcd_write_string(inst_p_buffer, 1, 1, NO_CR_LF);
}

char peak_p_buffer[6];
static void disp_peak_p() {
    sprintf(peak_p_buffer, "PIC%2i", p_peak);
    lcd_write_string(peak_p_buffer, 1, 5, NO_CR_LF);
}

static void disp_state() {
    if (globalState == welcome_wait_cal) {
        lcd_write_string(WAIT_CALI_MSG1, 1, 1, NO_CR_LF);
        lcd_write_string(WAIT_CALI_MSG2, 2, 1, NO_CR_LF);
    } else if(globalState == calibration) {
        lcd_write_string(CALI_MSG1, 1, 1, NO_CR_LF);
        lcd_write_string(CALI_MSG2, 2, 1, NO_CR_LF);
    } else if (globalState == stop) {
        lcd_write_string(" STOP ",1,10,NO_CR_LF);
    } else if (globalState == run) {
        lcd_write_string(" RUN  ",1,10,NO_CR_LF);
    }
}
