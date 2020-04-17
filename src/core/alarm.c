
#include <avr/interrupt.h>

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"

#include "core/alarm.h"
#include "core/system.h"
#include "core/display.h"
#include "core/main_task.h"
#include "core/debug.h"
#include "core/buzzer.h"

// debug print
#if DEBUG_ALARM
#define DEBUG_PRINT debug_print
#else
#define DEBUG_PRINT fake_debug_print
#endif // DEBUG_ALARM


volatile AlarmLevel_t alarmLevel;
volatile AlarmCause_t alarmCause;
volatile bool alarmMuted;

static volatile AlarmCause_t newAlarmCause;
static TickType_t mute_time;

static const AlarmLevel_t alarmLevels[] = {
 noAlarm, // noError
 highPriorityAlarm, // overPressure
 highPriorityAlarm, // noPressure
 highPriorityAlarm, // highTemperature
 mediumPriorityAlarm, // lowPressure
 mediumPriorityAlarm, // abnVolume
 mediumPriorityAlarm, // abnFreq
 mediumPriorityAlarm, // auxPower
 criticalPriorityAlarm, // calibPatientConnected
 criticalPriorityAlarm, // calibIncorrectFlow
 criticalPriorityAlarm, // doorOpen
 criticalPriorityAlarm, // cfMotorError
 criticalPriorityAlarm // powerError
};

static const char *alarmDescr[] = {
    "noError",
    "overPressure",
    "noPressure",
    "highPressure",
    "highTemperature",
    "lowPressure",
    "abnVolume",
    "abnFreq",
    "auxPower",
    "calibPatientConnected",
    "calibIncorrectFlow",
    "doorOpen",
    "cfMotorError",
    "powerError"
};


static void setMuteState(bool mute);
static void alarmChangeNotify();
static void processNewAlarm();


void initAlarm() {
    dio_init(DIO_PIN_LED_NORMAL_STATE, DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_LED_MPA, DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_LED_HPA, DIO_OUTPUT);
    dio_init(DIO_PIN_ALARM_LED_PAUSED, DIO_OUTPUT);

    alarmLevel = noAlarm;
    alarmCause = noError;
    newAlarmCause = noError;
    setMuteState(false);
    set_alarm_led(alarmLevel);
}

static void setMuteState(bool mute) {
    alarmMuted = mute;
    dio_write(DIO_PIN_ALARM_LED_PAUSED, alarmMuted);
}

void mutePressed() {
    DEBUG_PRINT("[MAIN] MUTE pressed.\r\n");
    setMuteState(!alarmMuted);
    mute_time = xTaskGetTickCount();
}

static void alarmChangeNotify() {
    set_alarm_led(alarmLevel);
    notify_buzzer();
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_ALARM, eSetBits);
}

// Do NOT call from context where interrupts are disabled
void sendNewAlarm(AlarmCause_t alarm) {
    cli();
    if (alarmLevels[alarm] > alarmLevels[newAlarmCause]) {
        newAlarmCause = alarm;
        sei();
        xTaskNotify(mainTaskHandle, MAIN_NOTIF_ALARM, eSetBits);
    } else {
        sei();
    }
}

// Do NOT call from context where interrupts are disabled
void ackAlarm() {
    cli();
    newAlarmCause = noError;
    sei();
    xTaskNotify(mainTaskHandle, MAIN_NOTIF_ALARM, eSetBits);
}

static void processNewAlarm() {
    if (newAlarmCause != alarmCause) {
        DEBUG_PRINT("[ALARM] new %s\r\n", alarmDescr[newAlarmCause]);
        cli();
        alarmCause = newAlarmCause;
        alarmLevel = alarmLevels[alarmCause];
        sei();
        alarmChangeNotify();
    }
}

void set_alarm_led(AlarmLevel_t state) {
    dio_write(DIO_PIN_LED_NORMAL_STATE, DIO_LOW);
    dio_write(DIO_PIN_ALARM_LED_MPA, DIO_LOW);
    dio_write(DIO_PIN_ALARM_LED_HPA, DIO_LOW);
    switch (state) {
        case noAlarm:
            dio_write(DIO_PIN_LED_NORMAL_STATE, DIO_HIGH);
            break;
        case mediumPriorityAlarm:
            dio_write(DIO_PIN_ALARM_LED_MPA, DIO_HIGH);
            break;
        case highPriorityAlarm:
        case criticalPriorityAlarm:
            dio_write(DIO_PIN_ALARM_LED_HPA, DIO_HIGH);
            break;
    }
}

void pollAlarm() {
    if (alarmMuted && xTaskGetTickCount() - mute_time > pdMS_TO_TICKS(ALARM_AUTO_UNMUTE_DELAY)) {
        DEBUG_PRINT("[MAIN] Auto unmute.\r\n");
        setMuteState(false);
    }
    processNewAlarm();
}
