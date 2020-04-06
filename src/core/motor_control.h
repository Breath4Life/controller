#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "FreeRTOS.h"

#define MOTOR_NOTIF_LIM_DOWN 0x01
#define MOTOR_NOTIF_LIM_UP 0x02

typedef enum {
    motorStopped,
    motorRunning,
} MotorState_t;

extern MotorState_t motorState;

void init_motor();

/**
 * @MotorControlTask Control the motor.
 *
 * @param pvParameters Set to NULL.
 */
void MotorControlTask(void *pvParameters);

#endif // MOTOR_CONTROL_H_
