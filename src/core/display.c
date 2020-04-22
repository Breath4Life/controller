#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/lcd.h"
#include "core/system.h"
#include "core/display.h"
#include "core/main_task.h"
#include "core/debug.h"
#include "core/analog_read.h"
#include "core/alarm.h"
#include "core/utils.h"
#include "core/parameters.h"

#if DEBUG_DISPLAY
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_DISPLAY

// Time in ticks
#define MUTE_MSG_PERIOD pdMS_TO_TICKS(2000L)
#define PARAM_BLINK_PERIOD pdMS_TO_TICKS(250)

static void lcdWriteTwoLines(const char * firstLine, const char * secondLine);

static void initDisplay();

// Pressure measurements display
static void displayPlateauPressure();
static void displayPeakPressure();

// Information zone display (state, error, mute)
static void refreshInfoZone();
static void displayState();
static void displayErrorCode();

// Used for the alternating muted indication
static void pollAlarmMutedDisplay();
static void delayAlarmMutedDisplay();
static void initAlarmMutedDisplay();
static TimeOut_t muteMsgToggleTimeOut;
static TickType_t muteMsgToggleRemTime;
static bool muteMsgVisible = false;

// Parameters display
static void refreshParametersZone();
static void toggleUnsavedParameters();
static void displayTidalVolume(bool visible);
static void displayRespFreq(bool visible);
static void displayExtraParam(bool visible);

// Used for unconfirmed parameters blinking
static void pollUnsavedParamDisplay();
static TimeOut_t paramToggleTimeOut;
static TickType_t paramToggleRemTime;
static bool paramVisible = true;

static void processDisplayNotification(uint32_t notification);

static void welcomeScreen();
static void welcomeWaitCalScreen();
static void calibrationScreen();
static void normalScreen(uint32_t notification);
static void criticalFailureScreen();
static void refreshLCD(uint32_t notification);

void LCDDisplayTask(void *pvParameters)
{
    DEBUG_PRINT("[LCD] Starting.\r\n");
    initDisplay();
    displayState();

    uint32_t notification = 0;
    BaseType_t notif_recv = pdFALSE;

    while (true) {
        pollAlarmMutedDisplay();
        pollUnsavedParamDisplay();

        if (notif_recv == pdTRUE) {
            processDisplayNotification(notification);
        }

        lcd_refreshLCD();

        notification = 0;
        notif_recv = xTaskNotifyWait( 0x0, ALL_NOTIF_BITS, &notification,
                                      MIN(muteMsgToggleRemTime, paramToggleRemTime));

    }
}

/*
 * Initializes the display.
 */
static void initDisplay() {
    // Note: this delay is needed for correct operation of the LCD afterwards
    vTaskDelay(pdMS_TO_TICKS(100));
}

/*
 * If alarm is muted, LCD information zone must alternate between
 * a muted alarm indication and the current state/error information
 */
static void pollAlarmMutedDisplay() {
    if (alarmMuted) {
        if (xTaskCheckForTimeOut(&muteMsgToggleTimeOut, &muteMsgToggleRemTime) == pdTRUE) {
            if (muteMsgVisible) {
                refreshInfoZone();
            } else {
                lcd_write_string(MUTED_MSG, 1, 10, NO_CR_LF);
            }

            vTaskSetTimeOutState(&muteMsgToggleTimeOut);
            muteMsgToggleRemTime = MUTE_MSG_PERIOD;
            muteMsgVisible = !muteMsgVisible;
        }
    } else {
        if (muteMsgVisible) {
            // Ensures the state/error information is visible when alarm has been unmuted
            refreshInfoZone();
            muteMsgVisible = false;
        }

        muteMsgToggleRemTime = portMAX_DELAY;
    }
}

/*
 * Delay mute message timeout (if any) to make new
 * error code/new state appears first for MUTE_MSG_PERIOD
 */
static void delayAlarmMutedDisplay() {
    vTaskSetTimeOutState(&muteMsgToggleTimeOut);
    muteMsgToggleRemTime = MAX(muteMsgToggleRemTime, MUTE_MSG_PERIOD);
    muteMsgVisible = false;
}

/*
 * Initialize mute message timeout.
 */
static void initAlarmMutedDisplay() {
    // Initialize timeout for the mute indicator
    vTaskSetTimeOutState(&muteMsgToggleTimeOut);
    muteMsgToggleRemTime = 0;
    muteMsgVisible = false;
}

/*
 * Displays either the current state or the current
 * alarm error code, depending on the alarm state.
 */
static void refreshInfoZone() {
    if (alarmCause == noError) {
        displayState();
    } else {
        displayErrorCode();
    }
}

/*
 * Unsaved parameters must blink while unconfirmed
 */
static void pollUnsavedParamDisplay() {
    if (unsaved_parameters()) {
        if (xTaskCheckForTimeOut(&paramToggleTimeOut, &paramToggleRemTime) == pdTRUE) {
            toggleUnsavedParameters();

            vTaskSetTimeOutState(&paramToggleTimeOut);
            paramToggleRemTime = PARAM_BLINK_PERIOD;
        }
    } else {
        if (!paramVisible) {
            // Ensure parameters are visible when confirmed
            refreshParametersZone();
        }

        paramToggleRemTime = portMAX_DELAY;
    }
}

static void toggleUnsavedParameters() {
    if (unsaved_tidal_vol()) {
        displayTidalVolume(!paramVisible);
    }

    if (unsaved_bpm()) {
        displayRespFreq(!paramVisible);
    }

    if (unsaved_extra_param()) {
        displayExtraParam(!paramVisible);
    }

    paramVisible = !paramVisible;
}


/*
 * Displays all parameters and update the paramLastToggle
 */
static void refreshParametersZone() {
    displayTidalVolume(true);
    displayRespFreq(true);
    displayExtraParam(true);

    vTaskSetTimeOutState(&paramToggleTimeOut);
    paramToggleRemTime = PARAM_BLINK_PERIOD;
    paramVisible = true;
}


/*
 * Display tidal volume if visible is true, a blank
 * space of the corresponding size otherwise.
 */
static void displayTidalVolume(bool visible) {
    char tidalVolumeBuffer[8];
    if (!visible) {
        sprintf(tidalVolumeBuffer, "    ");
    } else {
        sprintf(tidalVolumeBuffer, "%2u0 ", tidal_vol);
    }

    lcd_write_string(tidalVolumeBuffer, 2, 1, NO_CR_LF);
}

/*
 * Display respiratory frequency if visible is true, a blank
 * space of the corresponding size otherwise.
 */
static void displayRespFreq(bool visible) {
    char respFreqBuffer[8];

    if (!visible) {
        sprintf(respFreqBuffer, "   ");
    } else {
        sprintf(respFreqBuffer, "%2u ", bpm);
    }

    lcd_write_string(respFreqBuffer, 2, 5, NO_CR_LF);
}

/*
 * Display extra parameter if visible is true, a blank
 * space of the corresponding size otherwise.
 */
static void displayExtraParam(bool visible) {
    char extraParamBuffer[12];

    if (!visible) {
        sprintf(extraParamBuffer, "         ");
    } else {
        switch (extra_param) {
            case 0:
                sprintf(extraParamBuffer, "I:E = 1:%1u", ie);
                break;
            case 1:
                sprintf(extraParamBuffer, "Pmax = %2u", p_max);
                break;
            case 2:
                sprintf(extraParamBuffer, "PEEP = %2i", peep);
                break;
        }
    }

    lcd_write_string(extraParamBuffer, 2, 8, NO_CR_LF);
}

/*
 * Display plateau pressure.
 */
static void displayPlateauPressure() {
    char plateauPressureBuffer[5];
    sprintf(plateauPressureBuffer, "PL%2i", p_plateau);
    lcd_write_string(plateauPressureBuffer, 1, 1, NO_CR_LF);
}

/*
 * Display peak pressure.
 */
static void displayPeakPressure() {
    char peakPressureBuffer[6];
    sprintf(peakPressureBuffer, "PK%2i ", p_peak);
    lcd_write_string(peakPressureBuffer, 1, 5, NO_CR_LF);
}

/*
 * Display current state (run or stop) in the LCD information zone.
 */
static void displayState() {
    if (globalState == stop) {
        lcd_write_string(STOPPED_MSG, 1, 10, NO_CR_LF);
    } else if (globalState == run) {
        lcd_write_string(RUNNING_MSG, 1, 10, NO_CR_LF);
    } else {
        lcd_write_string(BLANK_MSG, 1, 10, NO_CR_LF);
    }
}

/*
 * Displays alarm error code in the LCD information zone.
 */
static void displayErrorCode() {
    lcd_write_string(alarmCodes[alarmCause], 1, 10, NO_CR_LF);
}

// LCD welcome screen
static void welcomeScreen() {
    lcdWriteTwoLines(WELCOME_MSG1, WELCOME_MSG2);
}

// LCD welcome_wait_cal screen
static void welcomeWaitCalScreen() {
    if (alarmCause == noError) {
        DEBUG_PRINT("[LCD] Normal.\r\n");
        lcdWriteTwoLines(WAIT_CALI_MSG1, WAIT_CALI_MSG2);
        displayState();
    } else {
        DEBUG_PRINT("[LCD] Error.\r\n");
        lcdWriteTwoLines(CALI_ERROR_MSG1, CALI_ERROR_MSG2);
        displayErrorCode();
        delayAlarmMutedDisplay();
    }
}

// LCD calibration screen
static void calibrationScreen() {
    lcdWriteTwoLines(CALI_MSG1, CALI_MSG2);
}

// LCD "normal" (i.e. run or stop) screen
static void normalScreen(uint32_t notification) {
    if (notification & DISP_NOTIF_PLATEAU_P) {
        displayPlateauPressure();
    }

    if (notification & DISP_NOTIF_PEAK_P) {
        displayPeakPressure();
    }

    if (alarmCause == noError) {
        if (notification & DISP_NOTIF_STATE) {
            displayState();
            delayAlarmMutedDisplay();
         }
    } else {
        if (notification & DISP_NOTIF_ALARM) {
            displayErrorCode();
            delayAlarmMutedDisplay();
        }
    }

    if (notification & DISP_NOTIF_PARAM) {
        refreshParametersZone();
    }
}

// LCD critical failure screen
static void criticalFailureScreen() {
    lcdWriteTwoLines(CRITICAL_FAILURE_MSG1, CRITICAL_FAILURE_MSG2);
    displayErrorCode();
}

static void refreshLCD(uint32_t notification) {
   switch (globalState) {
        case welcome:
            DEBUG_PRINT("[LCD] gS = welcome.\r\n");
            welcomeScreen();
            break;
        case welcome_wait_cal:
            DEBUG_PRINT("[LCD] gS = welcome_wait_cal.\r\n");
            welcomeWaitCalScreen();
            break;
        case calibration:
            DEBUG_PRINT("[LCD] gS = calibration.\r\n");
            calibrationScreen();
            break;
        case stop:
        case run:
            DEBUG_PRINT("[LCD] gS = stop|run.\r\n");
            normalScreen(notification);
            break;
        case critical_failure:
            DEBUG_PRINT("[LCD] gS = critical_failure.\r\n");
            criticalFailureScreen();
            break;
    }
}

static void processDisplayNotification(uint32_t notification) {
    if (notification & DISP_NOTIF_ALARM) {
        DEBUG_PRINT("[LCD] NOTIF_ALARM.\r\n");
    }
    if (notification & DISP_NOTIF_PARAM) {
        DEBUG_PRINT("[LCD] NOTIF_PARAM.\r\n");
    }
    if (notification & DISP_NOTIF_PLATEAU_P) {
        DEBUG_PRINT("[LCD] NOTIF_PLATEAU_P.\r\n");
    }
    if (notification & DISP_NOTIF_PEAK_P) {
        DEBUG_PRINT("[LCD] NOTIF_PEAK_P.\r\n");
    }
    if (notification & DISP_NOTIF_STATE) {
        DEBUG_PRINT("[LCD] NOTIF_STATE.\r\n");
    }
    if (notification & DISP_NOTIF_MUTE) {
        DEBUG_PRINT("[LCD] NOTIF_MUTE.\r\n");
    }

    if (notification & DISP_NOTIF_REFRESH) {
        refreshLCD(notification);
    } else if (notification & DISP_NOTIF_MUTE) {
        initAlarmMutedDisplay();
    }
}

static void lcdWriteTwoLines(const char * firstLine, const char * secondLine) {
    lcd_write_string(firstLine, 1, 1, NO_CR_LF);
    lcd_write_string(secondLine, 2, 1, NO_CR_LF);
}
