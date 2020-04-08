#include "FreeRTOS.h"
#include "task.h"

#include "core/debug.h"
#include "core/system.h"
#include "core/buttons.h"
#include "core/main_task.h"
#include "core/motor_control.h"
#include "core/utils.h"
#include "core/display.h"

#include "hal/io.h"
#include "hal/pins.h"

volatile GlobalState_t globalState;
volatile AlarmState_t alarmState;
volatile ErrorCode_t errorCode;

volatile uint8_t mute_on;
TickType_t mute_time;

uint8_t tidal_vol; // tens of mL
uint8_t bpm;
uint8_t ie;
uint8_t p_max;
uint8_t extra_param;

static void process_alarm(uint32_t notification);

#define DEBUG_MAIN 1
#define SIM_MOTOR 1

void initMainTask()
{
    globalState = welcome;

    dio_write(DIO_PIN_LED_NORMAL_STATE, 1);
    dio_write(DIO_PIN_ALARM_LED_LPA, 0);
    dio_write(DIO_PIN_ALARM_LED_HPA, 0);

    alarmState = noAlarm;
    errorCode = noError;
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

#if DEBUG_MAIN
    debug_print("[MAIN] -> welcome.\r\n");
#endif

    vTaskDelayUntil(&xLastWakeTime, WELCOME_MSG_DUR);
    /*
     * 4. If globalState is welcome and WELCOME_MSG_DUR s have elapsed,
     * set globalState to welcome_wait_cal
     */
    globalState = welcome_wait_cal;
#if DEBUG_MAIN
    debug_print("[MAIN] -> welcome_wait_cal.\r\n");
#endif
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
#if DEBUG_MAIN
    debug_print("[MAIN] NOT_STATE -> LCD.\r\n");
#endif

    // Indicate if the state has changed
    uint8_t updated_state;
    // Indicate if settings have changed
    uint8_t updated_setting;
    // Received notifications from other tasks
    uint32_t notification = 0;
    BaseType_t notif_recv = pdFALSE;

#if SIM_MOTOR
    // MOTOR SIMULATON TODO: REMOVE
    uint32_t calib_start = 0;
#endif

    while (1) {
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
#if DEBUG_MAIN
            debug_print("[MAIN] MUTE pressed.\r\n");
#endif
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
#if DEBUG_MAIN
            debug_print("[MAIN] ACK pressed.\r\n");
#endif
            alarmState = noAlarm;
            errorCode = noError;

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
#if DEBUG_MAIN
                debug_print("[MAIN] -> calibration.\r\n");
#endif
                updated_state = 1;

                xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_START_CALIBRATION, eSetBits);

#if SIM_MOTOR
                // MOTOR SIMULATION TODO: remove
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
            // MOTOR SIMULATION TODO: replace with commented condition
            if (xTaskGetTickCount() - calib_start > 2*WELCOME_MSG_DUR) {
#else
            if (motorState == motorStopped) {
#endif
                globalState = stop;
#if DEBUG_MAIN
                debug_print("[MAIN] -> stop.\r\n");
#endif

                xTaskNotify(lcdDisplayTaskHandle,
                        DISP_NOTIF_STATE | DISP_NOTIF_PARAM |
                        DISP_NOTIF_INST_P | DISP_NOTIF_PEAK_P, eSetBits);
            }
            /*
             * 6b. If motorState is calibrating and the start/stop button was pressed
             * down, set globalState to welcome_wait_cal and notify motor of HALT
             */

#if SIM_MOTOR
            // MOTOR SIMULATION TODO: replace with commented condition
            else {
#else
            else if (motorState == motorCalibrating) {
#endif
                if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                    globalState = welcome_wait_cal;
#if DEBUG_MAIN
                    debug_print("[MAIN] -> welcome_wait_cal.\r\n");
#endif
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_HALT, eSetBits);
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
                extra_param = (extra_param + 1) % 2;
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_left)) {
                extra_param = (extra_param - 1) % 2;
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_up)) {
                if(extra_param == 0) {
                    ie = MIN(MAX_IE, ie + INC_IE);
                } else {
                    p_max = MIN(MAX_PMAX, p_max + INC_PMAX);
                }
                updated_setting = 1;
            }
            if (BUTTON_PRESSED(buttons_pressed, button_down)) {
                if(extra_param == 0) {
                    ie = MAX(MIN_IE, ie - INC_IE);
                } else {
                    p_max = MAX(MIN_PMAX, p_max - INC_PMAX);
                }
                updated_setting = 1;
             }

            if (updated_setting) {
#if DEBUG_MAIN
                debug_print("[MAIN] NOT_PARA -> LCD.\r\n");
#endif
                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
            }

            /*
             * 7b. If stop/start button was pressed down, update globalState
             * accordingly
             */
            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                if (globalState == stop) {
                    globalState = run;
#if DEBUG_MAIN
                    debug_print("[MAIN] -> run.\r\n");
#endif
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_START, eSetBits);
                } else if (globalState == run) {
                    globalState = stop;
#if DEBUG_MAIN
                    debug_print("[MAIN] -> stop.\r\n");
#endif
                    updated_state = 1;

                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_HALT, eSetBits);
                }
            }
        }

        /*
         * 8. If ALARM_AUTO_UNMUTE_SEC have elapsed since last MUTE button press
         * and mute is active, deactivate it
         */
        if (mute_on && xTaskGetTickCount() - mute_time > pdMS_TO_TICKS(ALARM_AUTO_UNMUTE_SEC)) {
            mute_on = 0;
#if DEBUG_MAIN
            debug_print("[MAIN] Auto unmute.\r\n");
#endif
            dio_write(DIO_PIN_ALARM_LED_PAUSED, 0);
        }

        /*
         * Common to any changing state, notify LCD of a state change
         * FIXME: && errorCode == noError
         */
        if (updated_state == 1 && errorCode == noError) {
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
#if DEBUG_MAIN
            debug_print("[MAIN] NOT_STATE -> LCD. \r\n");
#endif
        }

        /*
         * 9. If globalState is run, process received alarm notifications (if any)
         */
        if (globalState == run) {
            if (notif_recv == pdTRUE) {
#if DEBUG_MAIN
                debug_print("[MAIN] Rcvd notif.\r\n");
#endif
                process_alarm(notification);
            }
        }

        /*
         * 10. If globalState is calibration, process received critical failure
         * notifications (if any)
         */

        // TODO

        /*
         * 11. Check BATTERY_LOW signal
         */

        // TODO

        /*
         * 12. Check POWER_FAIL signal
         */

        // TODO

        /*
         * 13. EEPROM total operating writing
         */

        // TODO

        /*
         * 14. Bounded wait for notification (10ms)
         */
        notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification, pdMS_TO_TICKS(10));
    }
}

void process_alarm(uint32_t notification)
{
    if (alarmState == noAlarm) {
        if (notification >= 0x08) {
#if DEBUG_MAIN
            debug_print("[MAIN] noAlarm -> HPA.\r\n");
#endif
            alarmState = highPriorityAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
            dio_write(DIO_PIN_ALARM_LED_HPA, 1);
        } else {
#if DEBUG_MAIN
            debug_print("[MAIN] noAlarm -> MPA.\r\n");
#endif
            alarmState = mediumPriorityAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
            dio_write(DIO_PIN_ALARM_LED_LPA, 1);
        }

        if (notification & ALARM_NOTIF_OVERPRESSURE) {
            errorCode = overPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] OVERPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_NO_PRESSURE) {
            errorCode = noPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] NOPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_HIGH_PRESSURE) {
            errorCode = highPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] HIGHPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_HIGH_TEMP) {
            errorCode = highTemperature;
#if DEBUG_MAIN
            debug_print("[MAIN] HIGHTEMP.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_LOW_PRESSURE) {
            errorCode = lowPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] NOPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_ABN_VOLUME) {
            errorCode = abnVolume;
#if DEBUG_MAIN
            debug_print("[MAIN] ABNVOL.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_ABN_FREQ) {
            errorCode = abnFreq;
#if DEBUG_MAIN
            debug_print("[MAIN] Rcvd ABNFREQ.\r\n");
#endif
        }

        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
#if DEBUG_MAIN
        debug_print("[MAIN] NOT_ALARM -> LCD.\r\n");
#endif
    } else if (alarmState == mediumPriorityAlarm && notification >= 0x08) {
#if DEBUG_MAIN
        debug_print("[MAIN] MPA -> HPA.\r\n");
#endif
        alarmState = highPriorityAlarm;
        dio_write(DIO_PIN_ALARM_LED_LPA, 0);
        dio_write(DIO_PIN_ALARM_LED_HPA, 1);

        if (notification & ALARM_NOTIF_OVERPRESSURE) {
            errorCode = overPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] OVERPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_NO_PRESSURE) {
            errorCode = noPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] NOPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_HIGH_PRESSURE) {
            errorCode = highPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] HIGHPSR.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_HIGH_TEMP) {
            errorCode = highTemperature;
#if DEBUG_MAIN
            debug_print("[MAIN] HIGHTEMP.\r\n");
#endif
        }

        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
#if DEBUG_MAIN
        debug_print("[MAIN] NOT_ALARM -> LCD. \r\n");
#endif
    } else {
        // Nothing to do here, already at highest priority
    }
}

uint8_t stoppedOrRunning() {
    return (globalState == stop || globalState == run);
}

