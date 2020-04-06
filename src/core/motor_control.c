#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "hal/pins.h"
#include "hal/motor.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "hal/limit_switch.h"

/////////////////////////////////////
uint32_t TCT;
uint32_t Ti;
uint32_t Te;

uint32_t n_steps;
uint32_t tot_pulses;  

uint32_t plateau_pulses;
uint32_t insp_pulses;
uint32_t exp_pulses;

uint32_t T_tot_plateau;
uint32_t T_tot_Ti;
uint32_t T_tot_Te;

uint32_t tmp_f_insp;
uint32_t tmp_f_plateau;
uint32_t tmp_f_exp;

uint32_t f_insp;
uint32_t f_plateau;
uint32_t f_exp;

/////////////////////////////////////


volatile MotorState_t motorState;
volatile BreathState_t breathState;

unsigned long homePosition;
uint32_t notif_recv;

TickType_t previousWakeTime;

void init_motor() {
    init_limit_switch();
    setup_motor();

    motorState = motorStopped;
    breathState = startNewCycle;
    homePosition = 0;

    // Breathing cycles parameter
    TCT = 40 * 100; // in ms
    Ti = 20 * 100; // in ms
    Te = TCT - Ti;

    n_steps = 600UL;
    tot_pulses = n_steps * MOTOR_USTEPS;  

    plateau_pulses = tot_pulses/20;
    insp_pulses = tot_pulses - plateau_pulses;
    exp_pulses = tot_pulses;

    T_tot_plateau = Ti / 10;
    T_tot_Ti = Ti - T_tot_plateau;
    T_tot_Te = Te / 2;

    tmp_f_insp = insp_pulses * 1000;
    tmp_f_plateau = plateau_pulses * 1000;
    tmp_f_exp = tot_pulses * 1000;

    f_insp = tmp_f_insp / T_tot_Ti;
    f_plateau = tmp_f_plateau / T_tot_plateau;
    f_exp = tmp_f_exp / T_tot_Te;  

}


void MotorControlTask(void *pvParameters)
{
    while (1)
    {

        switch (motorState){
            case motorStopped:
                
                // Non bounded wait for notif 
                xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,portMAX_DELAY);

                // if notif == START: update motor state
                if (notif_recv & MOTOR_NOTIF_START) {
                    motorState = motorRunning;
                    breathState = startNewCycle;
                }
                break;

            case motorRunning:
                switch (breathState){
                    case insp:
                        // BOUNDED Wait notification
                        xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(3000));
                        if (notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                            breathState = plateau;
                            set_motor_goto_position_accel_exec(homePosition+insp_pulses+plateau, f_plateau, 2, 200);
                        }

                        // TODO update state based on the notification value 
			break;

                    case plateau:
                        // BOUNDED Wait notification
                        xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(2000));

                        if (notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                            breathState = exp;
                            set_motor_goto_position_accel_exec(homePosition, f_exp, 2, 200);
                        }

                        // TODO update state based on the notification value 
			break;  

                    case exp:
                        // BOUNDED Wait notification
                        xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(3000));

                        if (notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                            breathState = cycleEnd;
                        }

                        // Update states regarding the received notification
                        // TODO update state based on the notification value
                        // TODO disable motor
                        breathState = cycleEnd;
                        //debug_print("to cycleEnd \r\n");
			break;

                    // Add RECALIBRATION step

                    case cycleEnd:
                        vTaskDelayUntil(&previousWakeTime,pdMS_TO_TICKS(TCT));
                        breathState = startNewCycle;
        
                    case startNewCycle:
                        breathState = insp;
                        previousWakeTime = xTaskGetTickCount();

                        set_motor_goto_position_accel_exec(homePosition+insp_pulses, f_insp, 2, 200); 
                        
                        break;
                }

                break;
        }


   }
}

