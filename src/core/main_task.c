#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#include "core/system.h"
#include "core/buttons.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/utils.h"
#include "core/display.h"
#include "core/volume.h"
#include "core/buzzer.h"
#include "core/alarm.h"
#include "core/parameters.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/power_monitoring.h"
#include "hal/door_open.h"

#define CURR_DEBUG_PREFIX mainTask
#include "core/debug.h"

#if DEBUG_MAIN
#define DEBUG_PRINT debug_print_prefix
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_MAIN

volatile GlobalState_t globalState;

// Used to measure the actual respiratory frequency
static uint32_t lastCycleCount;
static TickType_t startBPMCheckTime;

static const char *globalStateDescr[] = {
    "welcome",
    "welcome_wait_cal",
    "calibration",
    "stop",
    "run",
    "critical_failure"
};

static void setGlobalState(GlobalState_t newState, bool notifyMotor, bool populateDisplay);
static void resetBPMCheck();
static void checkBPM();

void initMainTask()
{
    globalState = welcome;
    initButtons();
    initParameters();
}

void MainTask(void *pvParameters)
{
    setGlobalState(welcome, false, false);

    uint32_t notification = 0;

    while (true) {
        // Move to critical_failure if alarmLevel is criticalPriorityAlarm
        if (alarmLevel == criticalPriorityAlarm && globalState != critical_failure) {
            setGlobalState(critical_failure, true, false);
        }

        // Read buttons state
        ButtonsState buttons_pressed = poll_buttons();

        // MUTE button pressed
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_mute)) {
            mutePressed();
        }

        // ACK button pressed
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_ack)) {
            ackAlarm();
        }

        switch (globalState) {
            case welcome:
                // TODO SPEC and adjust this
                play_tone(440, 500, false);

                vTaskDelay(WELCOME_MSG_DUR);

                setGlobalState(welcome_wait_cal, false, false);
                break;
            case welcome_wait_cal:
                // Move to calibration if START pressed
                if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                    // Acknowledge alarm if any (useful if a previous calibration
                    // attempt triggered an alarm)
                    ackAlarm();
                    setGlobalState(calibration, true, false);
                }
                break;
            case calibration:
                // If an error occured during calibration, go back to
                // welcome_wait_cal for an eventual other attempt
                if (alarmLevel == highPriorityAlarm) {
                    setGlobalState(welcome_wait_cal, true, false);
                }

                if (motorState == motorStopped) {
                    setGlobalState(stop, false, true);
                } else if (motorState == motorCalibrating) {
                    if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                        setGlobalState(welcome_wait_cal, true, false);
                    }
                }
                break;
            case stop:
                upd_params(buttons_pressed);
                if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                    setGlobalState(run, true, false);
                    // Reset actual BPM measurement when moving from stop to run
                    resetBPMCheck();
                }

                break;
            case run:
                upd_params(buttons_pressed);
                if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                    setGlobalState(stop, true, false);
                }

                checkBPM();

               break;
            case critical_failure:
                break;
        }

        pollAlarm();

#if POWER_AUX_CHECK
        if (error_power_aux()) {
            sendNewAlarm(auxPower);
        }
#endif

#if POWER_MAIN_CHECK
        if (error_power_main()) {
            sendNewAlarm(powerError);
        }
#endif

#if DOOR_CHECK
        if (is_door_open()) {
            sendNewAlarm(doorOpen);
        }
#endif

        poll_volume();

        xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, pdMS_TO_TICKS(10));
    }
}

static void setGlobalState( GlobalState_t newState,
                            bool notifyMotor,
                            bool populateDisplay) {
    globalState = newState;
    DEBUG_PRINT("-> %s", globalStateDescr[newState]);

    if (populateDisplay) {
        // Populate display when moving from calibration to stop
        xTaskNotify(  lcdDisplayTaskHandle,
                        DISP_NOTIF_STATE |
                        DISP_NOTIF_PARAM |
                        DISP_NOTIF_PLATEAU_P |
                        DISP_NOTIF_PEAK_P, eSetBits);
        DEBUG_PRINT("NOTIF_STATE|PARAM|PLATEAU_P|PEAK_P -> LCD");
    } else {
        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
        DEBUG_PRINT("NOTIF_STATE -> LCD");
    }

    if (notifyMotor) {
        xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
        DEBUG_PRINT("NOTIF_STATE -> MOTOR");
    }
}

static void resetBPMCheck() {
    DEBUG_PRINT("Reset BPMCheck");
    lastCycleCount = cycleCount;
    startBPMCheckTime = xTaskGetTickCount();
}

static void checkBPM() {
    TickType_t elapsedTime = xTaskGetTickCount() - startBPMCheckTime;
    if (elapsedTime > pdMS_TO_TICKS(BPM_CHECK_PERIOD_MS)) {
        uint8_t measuredBPM = ((uint8_t) (cycleCount - lastCycleCount)) / BPM_CHECK_PERIOD_PER_MIN;
        DEBUG_PRINT("Measured BPM: %u", measuredBPM);
        if (ABS(measuredBPM - bpm) > BPM_TOL) {
            sendNewAlarm(abnFreq);
        }

        resetBPMCheck();
    }
}

// TODO: move, nothing to do here
void check_volume(uint32_t actual_vol) {
    // Convert target tidal volume to tens µl
    int32_t target_vol = tidal_vol * 1000L;

    // Check that measured volume is within +-10% of the target volume
    if((actual_vol > 11*target_vol) || (actual_vol < 9*target_vol)) {
        sendNewAlarm(abnVolume);
    }
}

