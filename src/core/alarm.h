/** Alarm management
 */
#ifndef ALARM_H_
#define ALARM_H_

#include <stdbool.h>

#define ALARM_CHECK 1           // active/deactivate alarm check for debug
#define CALIB_ERROR_CHECK 1     // active/deactivate calib error check during calib for debug

#define POWER_AUX_CHECK 0       // active/deactivate power aux check for debug
#define POWER_MAIN_CHECK 0      // active/deactivate power main check for debug
#define DOOR_CHECK 1            // active/deactivate door check for debug

typedef enum {
    noAlarm,
    mediumPriorityAlarm,
    highPriorityAlarm,
    criticalPriorityAlarm
} AlarmLevel_t;

typedef enum {
    noError,
    overPressure,
    noPressure,
    highTemperature,
    lowPressure,
    abnVolume,
    abnFreq,
    auxPower,
    calibPatientConnected,
    calibIncorrectFlow,
    doorOpen,
    cfMotorError,
    powerError
} AlarmCause_t;

extern volatile AlarmLevel_t alarmLevel;
extern volatile AlarmCause_t alarmCause;
extern volatile bool alarmMuted;

void initAlarm();
void mutePressed();
void pollAlarm();
void sendNewAlarm(AlarmCause_t alarm);
void ackAlarm();
void set_alarm_led(AlarmLevel_t state);


#endif // ALARM_H_
