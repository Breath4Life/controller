#include "FreeRTOS.h"
#include "task.h"

#include "core/debug.h"
#include "core/system.h"
#include "core/buttons.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/utils.h"
#include "core/display.h"
#include "core/volume.h"
#include "core/buzzer.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/power_monitoring.h"
#include "hal/door_open.h"

volatile GlobalState_t globalState;
volatile AlarmState_t alarmState;
volatile ErrorCode_t errorCode;
volatile CalibError_t calibError;
volatile CriticalFailureCause_t criticalFailureCause;

volatile uint8_t mute_on;
TickType_t mute_time;

uint8_t tidal_vol; // tens of mL
uint8_t bpm;
uint8_t ie;
uint8_t p_max;
uint8_t extra_param;

static void process_alarm(uint32_t notification);
static void process_calib_error(uint32_t notification);
static void set_critical_failure(CriticalFailureCause_t cause);

// debug print
#define DEBUG_MAIN 1

#if DEBUG_MAIN
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_MAIN

#define SIM_MOTOR 0             // "simulate" motor to debug the rest
#define ALARM_CHECK 1           // active/deactivate alarm check for debug
#define CALIB_ERROR_CHECK 1     // active/deactivate calib error check during calib for debug
#define POWER_AUX_CHECK 0       // active/deactivate power aux check for debug
#define POWER_MAIN_CHECK 1      // active/deactivate power main check for debug
#define DOOR_CHECK 1            // active/deactivate door check for debug

void initMainTask()
{
    globalState = welcome;

    dio_write(DIO_PIN_LED_NORMAL_STATE, 1);
    dio_write(DIO_PIN_ALARM_LED_LPA, 0);
    dio_write(DIO_PIN_ALARM_LED_HPA, 0);

    alarmState = noAlarm;
    errorCode = noError;
    calibError = calibNoError;
    mute_on = 0;

    // TODO notify Buzzer task of Welcome

    tidal_vol = DEFAULT_TIDAL_VOL;
    bpm = DEFAULT_BPM;
    ie = DEFAULT_IE;
    p_max = DEFAULT_PMAX;
    extra_param = 0;

    initButtons();
}

void MainTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    DEBUG_PRINT("[MAIN] -> welcome.\r\n");

    // TODO SPEC and adjust this
    play_tone(440, 500);

    vTaskDelayUntil(&xLastWakeTime, WELCOME_MSG_DUR);
    /*
     * 4. If globalState is welcome and WELCOME_MSG_DUR s have elapsed,
     * set globalState to welcome_wait_cal
     */
    globalState = welcome_wait_cal;
    DEBUG_PRINT("[MAIN] -> welcome_wait_cal.\r\n");
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
    DEBUG_PRINT("[MAIN] NOTIF_STATE -> LCD.\r\n");

    // Indicate if the state has changed
    uint8_t updated_state;
    // Indicate if settings have changed
    uint8_t updated_setting;
    // Received notifications from other tasks
    uint32_t notification = 0;
    BaseType_t notif_recv = pdFALSE;

#if SIM_MOTOR
    uint32_t calib_start = 0;
#endif

    while (1) {
        /*
         * 0. If globalState is critical failure, full restart required, do nothing.
         */
        if (globalState == critical_failure) {
            DEBUG_PRINT("[MAIN] Critically failed.\r\n");
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
            continue;
        }

        updated_state = 0;
        updated_setting = 0;

        /*
         * TODO:
         * - notif_LCD_to_send variable that is |=ed at different point
         *   in the code, with a single xTaskNotify at the end of the loop
         */

        /*
         * 1. Read buttons state
         */
        ButtonsState buttons_pressed = poll_buttons();

        /*
         * 2. If the MUTE button was pressed down: toggle mute_on and record time
         */
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_mute)) {
            DEBUG_PRINT("[MAIN] MUTE pressed.\r\n");
            mute_on = !mute_on;
            dio_write(DIO_PIN_ALARM_LED_PAUSED, mute_on);
            mute_time = xTaskGetTickCount();
        }

        /*
         * 3. If the ACK button was pressed down: set alarmState to noAlarm,
         * set errorCode to noError, turn on green LED and turn off yellow
         * and red LEDs
         */
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_ack)) {
            DEBUG_PRINT("[MAIN] ACK pressed.\r\n");
            alarmState = noAlarm;
            errorCode = noError;
            calibError = calibNoError;
            notify_buzzer();

            dio_write(DIO_PIN_LED_NORMAL_STATE, 1);
            dio_write(DIO_PIN_ALARM_LED_LPA, 0);
            dio_write(DIO_PIN_ALARM_LED_HPA, 0);
            updated_state = 1;
        }

        /*
         * 5. If globalState is welcome_wait_cal and start/stop button was
         * pressed down: set globalState to calibration and notify MotorTask
         * to start calibration
         */
        if (globalState == welcome_wait_cal) {
           if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                globalState = calibration;
                if (calibError != calibNoError) {
                    // Reset calibError if coming from a failed calibration
                    calibError = calibNoError;
                    dio_write(DIO_PIN_LED_NORMAL_STATE, 1);
                    dio_write(DIO_PIN_ALARM_LED_HPA, 0);
                }

                DEBUG_PRINT("[MAIN] -> calibration.\r\n");
                updated_state = 1;

                xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);

#if SIM_MOTOR
                calib_start = xTaskGetTickCount();
#endif
            }
        }
        /*
         * 6. If globalState is calibration:
         */
        else if (globalState == calibration) {
            /*
             * 6a. If motorState is motorStopped, set globalState to stop and notify
             * LCD that calibration is done (display settings + measured parameters
             * + state).
             */

#if SIM_MOTOR
            if (xTaskGetTickCount() - calib_start > 2*WELCOME_MSG_DUR) {
#else
            if (motorState == motorStopped) {
#endif
                globalState = stop;
                DEBUG_PRINT("[MAIN] -> stop.\r\n");

                xTaskNotify(lcdDisplayTaskHandle,
                        DISP_NOTIF_STATE | DISP_NOTIF_PARAM |
                        DISP_NOTIF_PLATEAU_P | DISP_NOTIF_PEAK_P, eSetBits);
            }
            /*
             * 6b. If motorState is calibrating and the start/stop button was pressed
             * down, set globalState to welcome_wait_cal and notify motor of HALT
             */

#if SIM_MOTOR
            else {
#else
            else if (motorState == motorCalibrating) {
#endif
                if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                    globalState = welcome_wait_cal;
                    DEBUG_PRINT("[MAIN] -> welcome_wait_cal.\r\n");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                }
            }
        }
       /*
        * 7. If globalState is stop or run:
        */
        else if (globalState == stop || globalState == run) {
            /*
             * 7a. If any setting button was pressed down, update the corresponding
             * state and notify LCD
             */
            if (BUTTON_PRESSED(buttons_pressed, button_vtidal_up)) {
                tidal_vol = MIN(MAX_TIDAL_VOL, tidal_vol + INC_TIDAL_VOL);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_vtidal_down)) {
                tidal_vol = MAX(MIN_TIDAL_VOL, tidal_vol - INC_TIDAL_VOL);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_freq_respi_up)) {
                bpm = MIN(MAX_BPM, bpm + INC_BPM);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_freq_respi_down)) {
                bpm = MAX(MIN_BPM, bpm - INC_BPM);
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_right)) {
                extra_param = (extra_param + 1) % N_EXTRA;
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_left)) {
                extra_param = (extra_param - 1) % N_EXTRA;
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_up)) {
                if (extra_param == 0) {
                    ie = MIN(MAX_IE, ie + INC_IE);
                    updated_setting = 1;
                } else if (extra_param == 1) {
                    p_max = MIN(MAX_PMAX, p_max + INC_PMAX);
                    updated_setting = 1;
                } else if (extra_param == 2) {
                    // PEEP, not settable
                }
            }
            if (BUTTON_PRESSED(buttons_pressed, button_down)) {
                if (extra_param == 0) {
                    ie = MAX(MIN_IE, ie - INC_IE);
                    updated_setting = 1;
                } else if (extra_param == 1) {
                    p_max = MAX(MIN_PMAX, p_max - INC_PMAX);
                    updated_setting = 1;
                } else if (extra_param == 2) {
                    // PEEP, not settable
                }
             }

            if (updated_setting) {
                DEBUG_PRINT("[MAIN] NOTIF_PARAM -> LCD.\r\n");
                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
            }

            /*
             * 7b. If stop/start button was pressed down, update globalState
             * accordingly
             */
            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                if (globalState == stop) {
                    globalState = run;
                    DEBUG_PRINT("[MAIN] -> run.\r\n");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                } else if (globalState == run) {
                    globalState = stop;
                    DEBUG_PRINT("[MAIN] -> stop.\r\n");
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
                }
            }
        }

        /*
         * 8. If ALARM_AUTO_UNMUTE_SEC have elapsed since last MUTE button press
         * and mute is active, deactivate it
         */
        if (mute_on && xTaskGetTickCount() - mute_time > pdMS_TO_TICKS(ALARM_AUTO_UNMUTE_SEC)) {
            mute_on = 0;
            DEBUG_PRINT("[MAIN] Auto unmute.\r\n");
            dio_write(DIO_PIN_ALARM_LED_PAUSED, 0);
        }

        /*
         * Common to any changing state, notify LCD of a state change
         * FIXME: && errorCode == noError
         */
        if (updated_state == 1 && errorCode == noError) {
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
            DEBUG_PRINT("[MAIN] NOTIF_STATE -> LCD. \r\n");
        }

        /*
         * 9. If globalState is run, process received alarm notifications (if any)
         */
        if (globalState == run) {
            if (notif_recv == pdTRUE) {
                DEBUG_PRINT("[MAIN] run rcvd notif.\r\n");
#if ALARM_CHECK
                if (notification & NOTIF_MOTOR_ERROR) {
                    CriticalFailureCause_t cause = cfMotorError;
                    set_critical_failure(cause);
                } else {
                    process_alarm(notification);
                }
#endif
            }
        }

        /*
         * 10. If globalState is calibration, process received calib error
         * notifications (if any)
         */
        if (globalState == calibration) {
            if (notif_recv == pdTRUE) {
                DEBUG_PRINT("[MAIN] calib rcvd notif.\r\n");
#if CALIB_ERROR_CHECK
                if (notification & NOTIF_MOTOR_ERROR) {
                    CriticalFailureCause_t cause = cfMotorError;
                    set_critical_failure(cause);
                } else {
                    process_calib_error(notification);
                }
#endif
            }
        }

        /*
         * 11. Check BATTERY_LOW signal
         */
        if (error_power_aux()) {
#if POWER_AUX_CHECK
            DEBUG_PRINT("[MAIN] POWER AUX ERROR.\r\n");
            process_alarm(ALARM_NOTIF_POWER_AUX);
#endif
        }

        /*
         * 12. Check POWER_FAIL signal
         */
        if (error_power_main()) {
#if POWER_MAIN_CHECK
            DEBUG_PRINT("[MAIN] POWER MAIN ERROR.\r\n");
            CriticalFailureCause_t cause = powerError;
            set_critical_failure(cause);
#endif
        }

        /*
         * 13. Check if casing door is open
         */
        if (is_door_open()) {
#if DOOR_CHECK
            DEBUG_PRINT("[MAIN] DOOR OPEN.\r\n");
            CriticalFailureCause_t cause = doorOpen;
            set_critical_failure(cause);
#endif
        }

        /*
         * 14. EEPROM total operating writing
         */

        // TODO

        /*
         * 15. Poll volume sensing.
         */
        poll_volume();

        /*
         * 16. Bounded wait for notification (10ms)
         */
        notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, pdMS_TO_TICKS(10));
    }
}

void process_alarm(uint32_t notification)
{
    if (alarmState == noAlarm) {
        if (notification >= 0x10) {
            DEBUG_PRINT("[MAIN] noAlarm -> HPA.\r\n");
            alarmState = highPriorityAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
            dio_write(DIO_PIN_ALARM_LED_HPA, 1);
        } else {
            DEBUG_PRINT("[MAIN] noAlarm -> MPA.\r\n");
            alarmState = mediumPriorityAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
            dio_write(DIO_PIN_ALARM_LED_LPA, 1);
        }

        if (notification & ALARM_NOTIF_OVERPRESSURE) {
            errorCode = overPressure;
            DEBUG_PRINT("[MAIN] OVERPSR.\r\n");
        } else if (notification & ALARM_NOTIF_NO_PRESSURE) {
            errorCode = noPressure;
            DEBUG_PRINT("[MAIN] NOPSR.\r\n");
        } else if (notification & ALARM_NOTIF_HIGH_PRESSURE) {
            errorCode = highPressure;
            DEBUG_PRINT("[MAIN] HIGHPSR.\r\n");
        } else if (notification & ALARM_NOTIF_HIGH_TEMP) {
            errorCode = highTemperature;
            DEBUG_PRINT("[MAIN] HIGHTEMP.\r\n");
        } else if (notification & ALARM_NOTIF_LOW_PRESSURE) {
            errorCode = lowPressure;
            DEBUG_PRINT("[MAIN] NOPSR.\r\n");
        } else if (notification & ALARM_NOTIF_ABN_VOLUME) {
            errorCode = abnVolume;
            DEBUG_PRINT("[MAIN] ABNVOL.\r\n");
        } else if (notification & ALARM_NOTIF_ABN_FREQ) {
            errorCode = abnFreq;
            DEBUG_PRINT("[MAIN] ABNFREQ.\r\n");
        } else if (notification & ALARM_NOTIF_POWER_AUX) {
            errorCode = auxPower;
            DEBUG_PRINT("[MAIN] AUXPWR.\r\n");
        }
        // TODO: errorCode?

        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
        notify_buzzer();
        DEBUG_PRINT("[MAIN] NOT_ALARM -> LCD.\r\n");
    } else if (alarmState == mediumPriorityAlarm && notification >= 0x10) {
        DEBUG_PRINT("[MAIN] MPA -> HPA.\r\n");
        alarmState = highPriorityAlarm;
        dio_write(DIO_PIN_ALARM_LED_LPA, 0);
        dio_write(DIO_PIN_ALARM_LED_HPA, 1);

        if (notification & ALARM_NOTIF_OVERPRESSURE) {
            errorCode = overPressure;
            DEBUG_PRINT("[MAIN] OVERPSR.\r\n");
        } else if (notification & ALARM_NOTIF_NO_PRESSURE) {
            errorCode = noPressure;
            DEBUG_PRINT("[MAIN] NOPSR.\r\n");
        } else if (notification & ALARM_NOTIF_HIGH_PRESSURE) {
            errorCode = highPressure;
            DEBUG_PRINT("[MAIN] HIGHPSR.\r\n");
        } else if (notification & ALARM_NOTIF_HIGH_TEMP) {
            errorCode = highTemperature;
            DEBUG_PRINT("[MAIN] HIGHTEMP.\r\n");
        }

        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
        notify_buzzer();
        DEBUG_PRINT("[MAIN] NOT_ALARM -> LCD. \r\n");
    } else {
        // Nothing to do here, already at highest priority
    }
}

void process_calib_error(uint32_t notification) {
    // TODO: buzzer beep
    dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
    dio_write(DIO_PIN_ALARM_LED_HPA, 1);

    if (notification & NOTIF_PATIENT_CONNECTED) {
        DEBUG_PRINT("[MAIN] PATIENT CONNECTED.\r\n");
        calibError = patientConnected;
    } else if (notification & NOTIF_INCORRECT_FLOW) {
        DEBUG_PRINT("[MAIN] INCORRECT FLOW.\r\n");
        calibError = incorrectFlow;
    } else {
        DEBUG_PRINT("[MAIN] Unknown notif.\r\n");
    }

    globalState = welcome_wait_cal;
    DEBUG_PRINT("[MAIN] -> welcome_wait_cal.\r\n");

    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
    DEBUG_PRINT("[MAIN] NOT_STATE -> LCD. \r\n");
}

void set_critical_failure(CriticalFailureCause_t cause) {
    alarmState = highPriorityAlarm;
    globalState = critical_failure;
    criticalFailureCause = cause;
    DEBUG_PRINT("[MAIN] -> critical_failure.\r\n");

    dio_write(DIO_PIN_ALARM_LED_LPA, 0);
    dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
    dio_write(DIO_PIN_ALARM_LED_HPA, 1);

    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_GLOBAL_STATE, eSetBits);
    notify_buzzer();
    DEBUG_PRINT("[MAIN] NOT_STATE -> LCD. \r\n");
}

uint8_t stoppedOrRunning() {
    return (globalState == stop || globalState == run);
}

// TODO: move
void check_volume(uint32_t actual_vol) {
    // Convert target tidal volume to tens µl
    int32_t target_vol = tidal_vol * 1000L;

    // Check that measured volume is within +-10% of the target volume
    if((actual_vol > 11*target_vol) || (actual_vol < 9*target_vol)) {
        process_alarm(ALARM_NOTIF_ABN_VOLUME);
    }
}
