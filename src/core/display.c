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
static void disp_tidal_vol(uint8_t force);
static void disp_bpm(uint8_t force);
static void disp_extra_param(uint8_t force);
static void disp_plateau_p();
static void disp_peak_p();
static void disp_state();

#define DEBUG_LCD 1
#if DEBUG_LCD
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_LCD

uint32_t muted_switch_time = 0;
uint8_t muted_msg_on = 0;

uint32_t param_switch_time = 0;
uint8_t tidal_vol_on = 0;
uint8_t bpm_on = 0;
uint8_t extra_param_on = 0;

void LCDDisplayTask(void *pvParameters)
{
    DEBUG_PRINT("[LCD] Starting.\r\n");

    // Wait 100ms (otherwhise welcome message not printed)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    lcd_initLCD();
    lcd_write_string(WELCOME_MSG1, 1, 1, NO_CR_LF);
    lcd_write_string(WELCOME_MSG2, 2, 1, NO_CR_LF);

    while (1) {
        uint32_t notification;
        uint32_t curr_time = time_us();

        // Display "Muted" and the current state or alarm alternatively for 2 seconds if mute_on
        if (mute_on && (globalState == stop || globalState == run)) {
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

        if (curr_time - param_switch_time > 250000L) {
            if (new_tidal_vol != tidal_vol) {
                disp_tidal_vol(0);
            } else if (!tidal_vol_on) {
                disp_tidal_vol(1);
            }

            if (new_bpm != bpm) {
                disp_bpm(0);
            } else if (!bpm_on) {
                disp_bpm(1);
            }

            if (new_ie != ie || new_p_max != p_max) {
                disp_extra_param(0);
            } else if (!extra_param_on) {
                disp_extra_param(1);
            }

            param_switch_time = curr_time;
        }

        // FIXME: max wait has been changed to implement alternate mute/state/alarm display
        BaseType_t notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, pdMS_TO_TICKS(100));
        if (notif_recv == pdTRUE) {
            if (notification & DISP_NOTIF_PARAM) {
                DEBUG_PRINT("[LCD] rcvd notif param.\r\n");
                // FIXME: not clean, for a single param change, we re-write everything
                disp_tidal_vol(1);
                disp_bpm(1);
                disp_extra_param(1);

                param_switch_time = time_us();
            }
            if (notification & DISP_NOTIF_PLATEAU_P) {
                DEBUG_PRINT("[LCD] rcvd notif plateau p.\r\n");
                disp_plateau_p();
            }
            if (notification & DISP_NOTIF_PEAK_P) {
                DEBUG_PRINT("[LCD] rcvd notif peak p.\r\n");
                disp_peak_p();
            }
            if (notification & DISP_NOTIF_STATE) {
                DEBUG_PRINT("[LCD] rcvd notif state.\r\n");
                // Only write the state info if either
                // - there is no pending alarm (otherwhise alarm info is overwritten)
                // - globalState is critical_failure FIXME: why?
                if (alarmState == noAlarm || globalState == critical_failure) {
                    disp_state();
                }
            }
            if (notification & DISP_NOTIF_ALARM) {
                DEBUG_PRINT("[LCD] rcvd notif ALARM.\r\n");
                disp_alarm();
            }
        }

        lcd_refreshLCD();
    }
}

static void disp_alarm() {
    // FIXME: buffer useless here
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
        case auxPower:
            sprintf(alarm_buffer, " AUXPWR");
            break;
        case noError:
            // FIXME: should never happen if above code well design
            disp_state();
            break;
        default:
            sprintf(alarm_buffer, " ALARM ");
    }

    lcd_write_string(alarm_buffer, 1, 10, NO_CR_LF);
}

static void disp_muted() {
    lcd_write_string(" MUTED ", 1, 10, NO_CR_LF);
}

// FIXME: better buffer usage
char tidal_vol_buffer[8];
static void disp_tidal_vol(uint8_t force) {
    if (force) {
        sprintf(tidal_vol_buffer, "%2u0 ", new_tidal_vol);
        tidal_vol_on = 1;
    } else {
        if (tidal_vol_on) {
            sprintf(tidal_vol_buffer, "    ");
            tidal_vol_on = 0;
        } else {
            sprintf(tidal_vol_buffer, "%2u0 ", new_tidal_vol);
            tidal_vol_on = 1;
        }
    }

    lcd_write_string(tidal_vol_buffer, 2, 1, NO_CR_LF);
}

// FIXME: better buffer usage
char bpm_buffer[8];
static void disp_bpm(uint8_t force) {
    if (force) {
        sprintf(bpm_buffer, "%2u ", new_bpm);
        bpm_on = 1;
    } else {
        if (bpm_on) {
            sprintf(bpm_buffer, "   ");
            bpm_on = 0;
        } else {
            sprintf(bpm_buffer, "%2u ", new_bpm);
            bpm_on = 1;
        }
    }

    lcd_write_string(bpm_buffer, 2, 5, NO_CR_LF);
}

// FIXME: better buffer usage
char extra_param_buffer[12];
static void disp_extra_param(uint8_t force) {
    if (extra_param == 0) {
        if (force) {
            sprintf(extra_param_buffer, "I:E = 1:%1u", new_ie);
            extra_param_on = 1;
        } else {
            if (extra_param_on) {
                sprintf(extra_param_buffer, "         ");
                extra_param_on = 0;
            } else {
                sprintf(extra_param_buffer, "I:E = 1:%1u", new_ie);
                extra_param_on = 1;
            }
        }
    } else if (extra_param == 1) {
        if (force) {
            sprintf(extra_param_buffer, "Pmax = %2u", new_p_max);
            extra_param_on = 1;
         } else {
            if (extra_param_on) {
                sprintf(extra_param_buffer, "         ");
                extra_param_on = 0;
            } else {
                sprintf(extra_param_buffer, "Pmax = %2u", new_p_max);
                extra_param_on = 1;
            }
         }
    } else if (extra_param == 2) {
        sprintf(extra_param_buffer, "PEEP = %2u", peep);
    }

    lcd_write_string(extra_param_buffer, 2, 8, NO_CR_LF);
}

char plateau_p_buffer[5];
static void disp_plateau_p() {
    sprintf(plateau_p_buffer, "PL%2i", p_plateau);
    lcd_write_string(plateau_p_buffer, 1, 1, NO_CR_LF);
}

char peak_p_buffer[6];
static void disp_peak_p() {
    sprintf(peak_p_buffer, "PIC%2i", p_peak);
    lcd_write_string(peak_p_buffer, 1, 5, NO_CR_LF);
}

static void disp_state() {
    if (globalState == critical_failure) {
        DEBUG_PRINT("[LCD] rcvd crit_fail.\r\n");
        switch (criticalFailureCause) {
            case doorOpen:
                lcd_write_string(DOOR_OPEN_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
                break;
            case cfMotorError:
                lcd_write_string(MOTOR_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
                break;
            case powerError:
                lcd_write_string(POWER_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
                break;
             default:
                // Should never happen
                DEBUG_PRINT("[LCD] Unknown critical failure.\r\n");
                lcd_write_string(UNKNOWN_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(CRITICAL_FAILURE_MSG, 2, 1, NO_CR_LF);
        }
    } else if (globalState == welcome_wait_cal) {
        switch (calibError) {
            case calibNoError:
                lcd_write_string(WAIT_CALI_MSG1, 1, 1, NO_CR_LF);
                lcd_write_string(WAIT_CALI_MSG2, 2, 1, NO_CR_LF);
                break;
            case patientConnected:
                lcd_write_string(PAT_CONNECTED_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(RETRY_CALI_MSG, 2, 1, NO_CR_LF);
                break;
            case incorrectFlow:
                lcd_write_string(INC_FLOW_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(RETRY_CALI_MSG, 2, 1, NO_CR_LF);
                break;
            default:
                // Should never happen
                DEBUG_PRINT("[LCD] Unknown calib error.\r\n");
                lcd_write_string(UNKNOWN_ERROR_MSG, 1, 1, NO_CR_LF);
                lcd_write_string(WAIT_CALI_MSG2, 2, 1, NO_CR_LF);
        }
    } else if(globalState == calibration) {
        lcd_write_string(CALI_MSG1, 1, 1, NO_CR_LF);
        lcd_write_string(CALI_MSG2, 2, 1, NO_CR_LF);
    } else if (globalState == stop) {
        lcd_write_string(" STOP  ",1,10,NO_CR_LF);
    } else if (globalState == run) {
        lcd_write_string(" RUN   ",1,10,NO_CR_LF);
    }
}
