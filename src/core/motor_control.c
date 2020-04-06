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

const uint32_t vol_ml = 700;

///////////////////////////////////


volatile MotorState_t motorState;
volatile BreathState_t breathState;
volatile CalibState_t calibState;
volatile FlowState_t flowState;

int32_t currentPosition;
int32_t homePosition;
uint32_t notif_recv;

TickType_t previousWakeTime;

// TODO reset proper value! 
const uint32_t f_home = 4000; // steps/s (not µsteps/s)

const int32_t steps_calib_down = 2000;
const int32_t steps_calib_up = 1000;
const int32_t steps_calib_end = 60;
const int32_t steps_caliv_vol = 600;

const uint32_t thresh_calib_vol_mil = 600;

// Threshold for volume calibraiton 


void init_motor() {
    init_limit_switch();
    setup_motor();

    motorState = motorInit;
    breathState = startNewCycle;
    calibState = calibDown; 
    flowState = flowVol;
    currentPosition = 0;
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
        BaseType_t n_wait_recv;

        switch (motorState){
            case motorInit:
                // NON BOUNDED wait for calibrating notification
                xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,portMAX_DELAY);

                if (notif_recv & MOTOR_NOTIF_START_CALIBRATION) {
                    motorState = motorCalibrating;
                    calibState = calibDown;
                    currentPosition = MOTOR_USTEPS*steps_calib_down;
                    motor_enable();
                    set_motor_goto_position_accel_exec(currentPosition, f_home, 2, 200);
                    debug_print("to calib down\r\n");
                }
                break;

            case motorCalibrating:
                switch (calibState) {
                    case calibDown:
                        // BOUNDED wait for limit switch down 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(600000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if (notif_recv & MOTOR_NOTIF_LIM_UP) {
                                calibState = calibDown;
                            //} else if(notif_recv & MOTOR_NOTIF_LIM_DOWN) {
                            // TODO: set the condition above
                            } else if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                calibState = calibUp;
                                int32_t rem_usteps = motor_remaining_distance();
                                currentPosition = currentPosition-rem_usteps-MOTOR_USTEPS*steps_calib_up; 
                                set_motor_goto_position_accel_exec(currentPosition, f_home, 2, 200);
                                debug_print("to calib up\r\n");
                            } else {
                                debug_print("DOWN calib SEND ERROR1\r\n");
                                //TODO Notify MOTOR_ERROR
                            }
                        } else {
                            // TODO Notify MOTOR_ERROR     
                            debug_print("DOWN calib SEND ERROR2\r\n"); 
                        }
                        break;
                    
                    case calibUp:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(300000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            //if(notif_recv & MOTOR_NOTIF_LIM_DOWN) {
                            //set the condition above
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                calibState = calibPosEnd;
                                currentPosition = currentPosition + MOTOR_USTEPS*steps_calib_end;
                                set_motor_goto_position_accel_exec(currentPosition, f_home, 2, 200);
                                debug_print("to calib pos end\r\n");
                            } else {
                                debug_print("UP calib SEND ERROR1\r\n");
                                //TODO Notify MOTOR_ERROR
                            }
                        } else {
                            debug_print("UP calib SEND ERROR1\r\n");
                            // TODO Notify MOTOR_ERROR     
                        }
                        break;

                    case calibPosEnd:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(5000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                homePosition = currentPosition;
                                currentPosition = currentPosition + MOTOR_USTEPS*steps_caliv_vol;
                                motorState = motorFlowCheck;
                                flowState = flowVol;
                                set_motor_goto_position_accel_exec(currentPosition, MOTOR_USTEPS*steps_caliv_vol, 2, 200);
                                debug_print("to flow vol\r\n");
                            } else {
                                //TODO Notify MOTOR_ERROR
                            }
                        } else {
                            // TODO Notify MOTOR_ERROR     
                        }
                        break;
                    }
                    break;

            case motorFlowCheck: 
                switch (flowState){
                    case flowVol:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(5000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                //TODO add volume check!
                                flowState = flowVolEnd;
                                currentPosition = homePosition;
                                set_motor_goto_position_accel_exec(currentPosition, MOTOR_USTEPS*steps_caliv_vol, 2, 200);
                                debug_print("to flow vol end\r\n");
                            } else {
                                // TODO Notify MOTOR_ERROR
                            }
                        } else {
                            // TODO Notify MOTOR_ERROR     
                        }
                        break;

                    case flowVolEnd:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(5000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                motor_disable();
                                motorState = motorStopped;
                                debug_print("END INIT PHASE\r\n");
                            } else {
                                // TODO Notify MOTOR_ERROR
                            }
                        } else {
                            // TODO Notify MOTOR_ERROR     
                        }
                        break;

                }
                break;
                
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

