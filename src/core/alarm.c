
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

#define X(a, b, c) c,
static const AlarmLevel_t alarmLevels[] = {
    ERROR_TABLE
};
#undef X

#define X(a, b, c) b,
const char *alarmCodes[] = {
    ERROR_TABLE
};
#undef X


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
    alarmMuted = false;
    dio_write(DIO_PIN_ALARM_LED_PAUSED, DIO_LOW);
    set_alarm_led(alarmLevel);
}

static void setMuteState(bool mute) {
    alarmMuted = mute;
    dio_write(DIO_PIN_ALARM_LED_PAUSED, alarmMuted);
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_MUTE, eSetBits);
}

void mutePressed() {
    DEBUG_PRINT("[MAIN] MUTE pressed.\r\n");
    setMuteState(!alarmMuted);
    mute_time = xTaskGetTickCount();
}

static void alarmChangeNotify() {
    set_alarm_led(alarmLevel);

    // Automatic unmute if new alarmLevel is critical priority
    if (alarmLevel == criticalPriorityAlarm) {
        setMuteState(false);
    }

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
    if (alarmLevels[newAlarmCause] == criticalPriorityAlarm) {
        // Cannot ACK critical failures
        sei();
    } else {
        newAlarmCause = noError;
        sei();
        xTaskNotify(mainTaskHandle, MAIN_NOTIF_ALARM, eSetBits);
    xTaskNotify(lcdDisplayTaskHandle, DISP_NOTIF_STATE, eSetBits);
    }
}

static void processNewAlarm() {
    if (newAlarmCause != alarmCause) {
        DEBUG_PRINT("[ALARM] new %s\r\n", alarmCodes[newAlarmCause]);
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
