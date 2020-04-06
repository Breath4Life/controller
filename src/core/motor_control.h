#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "FreeRTOS.h"

#define MOTOR_NOTIF_SWITCH_UP 0x01
#define MOTOR_NOTIF_SWITCH_DOWN 0x02
#define MOTOR_NOTIF_HALT 0x04
#define MOTOR_NOTIF_MOVEMENT_FINISHED 0x08
#define MOTOR_NOTIF_START_CALIBRATION 0x10
#define MOTOR_NOTIF_START 0x20
#define MOTOR_NOTIF_EMERGENCY_STOP 0x40
#define MOTOR_NOTIF_OVER_PRESSURE 0x80

#define MOTOR_FULL_BITS 0xffffffff
#define MOTOR_USTEPS 16

typedef enum {
    motorStopped,
    motorRunning,
} MotorState_t;

typedef enum {
    insp,
    plateau,
    exp,
    cycleEnd,
    startNewCycle
} BreathState_t;

extern MotorState_t motorState;
extern BreathState_t breathState;


void initMotorControlTask();

/**
 * @MotorControlTask Control the motor.
 *
 * @param pvParameters Set to NULL.
 */
void MotorControlTask(void *pvParameters);


#endif // MOTOR_CONTROL_H_
