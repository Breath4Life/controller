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
// TODO: remove, used to simulate calibration
TickType_t start_calib;

uint8_t tidal_vol; // tens of mL
uint8_t bpm;
uint8_t ie;
uint8_t p_max;
uint8_t extra_param;

static void process_alarm(uint32_t notification);

#define DEBUG_MAIN 1

void initMainTask()
{
    globalState = welcome;

    dio_write(DIO_PIN_LED_NORMAL_STATE, 1);
    dio_write(DIO_PIN_ALARM_LED_LPA, 0);
    dio_write(DIO_PIN_ALARM_LED_HPA, 0);

    alarmState = noAlarm;
    errorCode = noError;
    mute_on = 0;

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
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    debug_print("[MAIN] Starting.\r\n");

    // TODO notify Buzzer task of Welcome
    vTaskDelayUntil(&xLastWakeTime, WELCOME_MSG_DUR);
    globalState = welcome_wait_cal;
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);

    while (1) {
        uint8_t updated_state = 0;
        uint8_t updated_setting = 0;

        // 1. Read buttons
        ButtonsState buttons_pressed = poll_buttons();

        // 2. If the MUTE button was pressed down
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_mute)) {
            mute_on = !mute_on;
            dio_write(DIO_PIN_ALARM_LED_PAUSED, mute_on);
            mute_time = xTaskGetTickCount();
            // TODO notify buzzer task of mute
        }

        // 3. If the ACK button was pressed down
        if (BUTTON_PRESSED(buttons_pressed, button_alarm_ack)) {
            alarmState = noAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 1);
            dio_write(DIO_PIN_ALARM_LED_LPA, 0);
            dio_write(DIO_PIN_ALARM_LED_HPA, 0);
            errorCode = noError;
            updated_state = 1;
            // TODO notify buzzer task of ack
        }

        // 5. Welcome wait calibration
        if (globalState == welcome_wait_cal && BUTTON_PRESSED(buttons_pressed, button_startstop)) {
            globalState = calibration;
            // TODO remove, used to simulate calibration
            start_calib = xTaskGetTickCount();

            updated_state = 1;
            xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_START_CALIBRATION, eSetBits);
        }
        // 6. Calibration state
        else if ((globalState == calibration)) {
            // FIXME: to be replaced by motorState == motorStopped condition, used to simulate calibration delay
            if (xTaskGetTickCount() - start_calib > 2*WELCOME_MSG_DUR) { //motorState == motorStopped) {
                globalState = stop;
                // Notify LCD with every possible notifications to intialize display
                xTaskNotify(lcdDisplayTaskHandle,
                        DISP_NOTIF_STATE | DISP_NOTIF_PARAM |
                        DISP_NOTIF_INST_P | DISP_NOTIF_PEAK_P, eSetBits);

#if DEBUG_MAIN
                debug_print("[MAIN] Told LCD calib done. \r\n");
#endif
            }

            // If START/STOP button pressed during calibration
            // TODO: add motorState condition currently commented
            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {// && motorState == motorRunning) {
                debug_print("[MAIN] Calibration stopped.\r\n");
                globalState = welcome_wait_cal;
                updated_state = 1;
                xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_HALT, eSetBits);
            }
        }
        // 7. Settings & Start/Stop
        // TODO: other part of the condition needed?
        else if (globalState == stop || globalState == run) { // motorState == motorRunning) {
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
                debug_print("[MAIN] Sending NO_PAR LCD. \r\n");
#endif
                xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_PARAM, eSetBits);
            }

            if (BUTTON_PRESSED(buttons_pressed, button_startstop)) {
                if (globalState == stop) {
                    globalState = run;
                    updated_state = 1;
                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_START, eSetBits);
                } else if (globalState == run) {
                    globalState = stop;
                    updated_state = 1;
                    xTaskNotify(motorControlTaskHandle, MOTOR_NOTIF_HALT, eSetBits);
                }
            }
        }

        if (updated_state == 1 && errorCode == noError) {
            xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
#if DEBUG_MAIN
            debug_print("[MAIN] Sent NOTIF_STATE to LCD. \r\n");
#endif
        }

        // 8. Buzzer auto-unmute
        if (mute_on && xLastWakeTime - mute_time > pdMS_TO_TICKS(ALARM_AUTO_UNMUTE_SEC)) {
            mute_on = 0;
            dio_write(DIO_PIN_ALARM_LED_PAUSED, 0);
            // TODO notify buzzer task of mute
        }

        // 9. Alarm wait for run state
        if (globalState == run) {
            uint32_t notification;
            // TODO: determine timeout here
            BaseType_t notif_recv = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification,
                                                    pdMS_TO_TICKS(50));

            if (notif_recv == pdTRUE) {
                process_alarm(notification);
            }
        }

        // 10. Critical failure during calibration
        if (globalState == calibration) {
            uint32_t notification_cf;
            // TODO: determine timeout here
            BaseType_t notif_recv_cf = xTaskNotifyWait(0x0, ALL_NOTIF_BITS, &notification_cf,
                                                    pdMS_TO_TICKS(50));

            if (notif_recv_cf == pdTRUE) {
                // TODO
            }
        }

        // 11. Check BATTERY_LOW signal

        // 12. Check POWER_FAIL

        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}

void process_alarm(uint32_t notification)
{
    if (alarmState == noAlarm) {
#if DEBUG_MAIN
        debug_print("Rcvd alarm from noAlarm state.\r\n");
#endif
        if (notification >= 0x08) {
#if DEBUG_MAIN
            debug_print("High priority alarm.\r\n");
#endif
            alarmState = highPriorityAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
            dio_write(DIO_PIN_ALARM_LED_HPA, 1);
        } else {
#if DEBUG_MAIN
            debug_print("Low priority alarm.\r\n");
#endif
            alarmState = mediumPriorityAlarm;
            dio_write(DIO_PIN_LED_NORMAL_STATE, 0);
            dio_write(DIO_PIN_ALARM_LED_LPA, 1);
        }

        if (notification & ALARM_NOTIF_OVERPRESSURE) {
            errorCode = overPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: Overpressure.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_NO_PRESSURE) {
            errorCode = noPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: No pressure.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_HIGH_PRESSURE) {
            errorCode = highPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: High pressure.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_HIGH_TEMP) {
            errorCode = highTemperature;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: High temp.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_LOW_PRESSURE) {
            errorCode = lowPressure;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: low pressure.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_ABN_VOLUME) {
            errorCode = abnVolume;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: volume.\r\n");
#endif
        } else if (notification & ALARM_NOTIF_ABN_FREQ) {
            errorCode = abnFreq;
#if DEBUG_MAIN
            debug_print("[MAIN] ALARM: frequency.\r\n");
#endif
        }

        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
#if DEBUG_MAIN
        debug_print("[MAIN] Sent NOTIF_ALARM to LCD. \r\n");
#endif
    } else if (alarmState == mediumPriorityAlarm && notification >= 0x08) {
        alarmState = highPriorityAlarm;
        dio_write(DIO_PIN_ALARM_LED_LPA, 0);
        dio_write(DIO_PIN_ALARM_LED_HPA, 1);

        if (notification & ALARM_NOTIF_OVERPRESSURE) {
            errorCode = overPressure;
        } else if (notification & ALARM_NOTIF_NO_PRESSURE) {
            errorCode = noPressure;
        } else if (notification & ALARM_NOTIF_HIGH_PRESSURE) {
            errorCode = highPressure;
        } else if (notification & ALARM_NOTIF_HIGH_TEMP) {
            errorCode = highTemperature;
        }

        xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
#if DEBUG_MAIN
        debug_print("[MAIN] Sent NOTIF_ALARM to LCD. \r\n");
#endif
    } else {
        // Nothing to do here, already at highest priority
    }
}

uint8_t stoppedOrRunning() {
    return (globalState == stop || globalState == run);
}

