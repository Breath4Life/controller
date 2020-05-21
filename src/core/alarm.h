/** Alarm management
 */
#ifndef ALARM_H_
#define ALARM_H_

#include <stdbool.h>

#define ALARM_CHECK 1           // active/deactivate alarm check for debug
#define CALIB_ERROR_CHECK 1     // active/deactivate calib error check during calib for debug

#define POWER_AUX_CHECK 1       // active/deactivate power aux check for debug
#define POWER_MAIN_CHECK 1      // active/deactivate power main check for debug
#define DOOR_CHECK 1            // active/deactivate door check for debug

typedef enum {
    noAlarm,
    mediumPriorityAlarm,
    highPriorityAlarm,
    criticalPriorityAlarm
} AlarmLevel_t;

#define ERROR_TABLE \
    X(noError, "       ", noAlarm) \
    X(overPressure, "MXPSR11", highPriorityAlarm) \
    X(noPressure, "NOPSR12", highPriorityAlarm) \
    X(highTemperature, "HITMP13", highPriorityAlarm) \
    X(motorHot, "HITMP14", criticalPriorityAlarm) \
    X(lowPressure, "LOPSR21", mediumPriorityAlarm) \
    X(abnVolume, "VOLUM22", mediumPriorityAlarm) \
    X(abnFreq, "RESPR23", mediumPriorityAlarm) \
    X(auxPower, "LOBAT24", mediumPriorityAlarm) \
    X(calibPatientConnected, "SENSO14", highPriorityAlarm) \
    X(calibIncorrectFlow, "PATCO15", highPriorityAlarm) \
    X(doorOpen, "ODOOR03", criticalPriorityAlarm) \
    X(cfMotorError, "MOTOR02", criticalPriorityAlarm) \
    X(powerError, "POWER01", criticalPriorityAlarm)

#define X(a, b, c) a,
typedef enum {
    ERROR_TABLE
} AlarmCause_t;
#undef X

extern const char *alarmCodes[];

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
