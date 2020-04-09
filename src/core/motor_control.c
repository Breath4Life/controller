#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "core/main_task.h"
#include "core/system.h"
#include "hal/pins.h"
#include "hal/motor.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "hal/limit_switch.h"

#define DEBUG_MOTOR 1

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

uint32_t currentPosition;
uint32_t targetPosition;
uint32_t homePosition;
uint32_t notif_recv;
uint32_t posOffset;

TickType_t previousWakeTime;
uint32_t ticksTctTime;

// TODO reset proper value! 
const uint32_t f_home = 200; // steps/s (not µsteps/s)

const int32_t steps_calib_down = 2000;
const int32_t steps_calib_up = 1000;
const int32_t steps_calib_end = 10;
const int32_t steps_caliv_vol = 500;

const uint32_t thresh_calib_vol_mil = 600;

// Threshold for volume calibraiton 

// 0 replaces the whole task by an empty loop for testing purposes
#define MOTOR_ACTIVE 1

uint32_t vol2steps(uint8_t  tidal_vol){
    return 10*tidal_vol;
} 

void compute_config(){

    // Breathing cycles parameter
    TCT = (60000L / bpm); // in ms
    // TODO check for each possibilities
    Ti = (TCT/(1+ie)); // in ms
    Te = TCT - Ti;

    ticksTctTime = pdMS_TO_TICKS(TCT);

    n_steps = vol2steps(tidal_vol);
    tot_pulses = n_steps * MOTOR_USTEPS;  

    plateau_pulses = MOTOR_USTEPS*5; //tot_pulses/20;
    insp_pulses = tot_pulses - plateau_pulses;
    exp_pulses = tot_pulses;

    T_tot_plateau = Ti / 5;
    T_tot_Ti = Ti - T_tot_plateau;
    T_tot_Te = 750;

    tmp_f_insp = insp_pulses * 1000;
    tmp_f_plateau = plateau_pulses * 1000;
    tmp_f_exp = tot_pulses * 1000;

    f_insp = tmp_f_insp / T_tot_Ti;
    f_plateau = tmp_f_plateau / T_tot_plateau;
    f_exp = tmp_f_exp / T_tot_Te;

#if DEBUG_MOTOR     
    debug_print("ie %u \r\n",ie);
    debug_print("bpm %u \r\n",bpm);
    debug_print("TCT %u \r\n",TCT);
    debug_print("Ti %u \r\n",Ti);
    debug_print("nsteps %u \r\n",n_steps);
    debug_print("pulse_lateau %u \r\n",plateau_pulses);
    debug_print("Ttot plate %u \r\n",T_tot_plateau);
#endif
}

void init_motor() {
    init_limit_switch();
    setup_motor();

    motorState = motorInit;
    breathState = startNewCycle;
    calibState = calibDown; 
    currentPosition = 0;
    targetPosition = 0;
    homePosition = 0;
    posOffset = 0;

    // Setup config  
    compute_config();
}


void MotorControlTask(void *pvParameters)
{
    while (1)
    {
#if MOTOR_ACTIVE
        BaseType_t n_wait_recv;

        switch (motorState){
            case motorInit:
                // NON BOUNDED wait for calibrating notification
                xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,portMAX_DELAY);

                if (notif_recv & MOTOR_NOTIF_START_CALIBRATION) {
                    compute_config();
                    motorState = motorCalibrating;
                    calibState = calibDown;
                    currentPosition = 0;
                    targetPosition = MOTOR_USTEPS*steps_calib_down;
                    motor_enable();
                    set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*f_home, 2, 200);
#if DEBUG_MOTOR
                    debug_print("to calib down\r\n");
#endif
                }
                break;

            case motorCalibrating:
                switch (calibState) {
                    case calibDown:
                        // BOUNDED wait for limit switch down 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(10000));

                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if (notif_recv & MOTOR_NOTIF_LIM_UP) {
                                calibState = calibDown;
                            } else if(notif_recv & MOTOR_NOTIF_LIM_DOWN) {
                                // Stop motor
                                motor_anticipated_stop();
                                // Compute actual position when notified
                                currentPosition = motor_current_position();
                                // Compute next position   
                                posOffset = MOTOR_USTEPS*steps_calib_up;

                                if (currentPosition > posOffset){
                                    targetPosition = currentPosition - posOffset;
                                } else {
                                    // Manually set the absolute position of the motor 
                                    set_motor_current_position_value(posOffset);
                                    targetPosition = 0;
                                }
                                calibState = calibUp;
                                set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*f_home, 2, 200);
#if DEBUG_MOTOR
                                debug_print("to calib up\r\n");
#endif
                            } else if (notif_recv & MOTOR_NOTIF_HALT){
                                motor_anticipated_stop();
                                motor_disable();
                                motorState = motorInit;
#if DEBUG_MOTOR
                                debug_print("to motor INIT \r\n");
#endif
                            } else {
                                xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
#if DEBUG_MOTOR
                                debug_print("DOWN calib SEND ERROR1\r\n");
#endif
                                
                            }
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
#if DEBUG_MOTOR
                            debug_print("DOWN calib SEND ERROR2\r\n"); 
#endif
                        }
                        break;
                    
                    case calibUp:
#if DEBUG_MOTOR
                        debug_print("In calibUp\r\n");
#endif
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(10000));
                        
                        // Verify deadline
                        if (n_wait_recv){
#if DEBUG_MOTOR
                            debug_print("received notif %d\r\n",notif_recv);
#endif
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_LIM_UP) {
                                // Stop motor
                                motor_anticipated_stop();
                                // Compute actual position when notified
                                currentPosition = motor_current_position();
                                // Compute next position   
                                posOffset = MOTOR_USTEPS*steps_calib_end;
                                targetPosition = currentPosition + posOffset;

                                calibState = calibPosEnd;
                                set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*f_home, 2, 200);
#if DEBUG_MOTOR
                                debug_print("to calib pos end\r\n");
#endif
                            } else if (notif_recv & MOTOR_NOTIF_HALT){
                                motor_anticipated_stop();
                                motor_disable();
                                motorState = motorInit;
#if DEBUG_MOTOR
                                debug_print("to motor INIT \r\n");
#endif
                            } else {
                                xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
#if DEBUG_MOTOR
                                debug_print("UP calib SEND ERROR1\r\n");
#endif
                            }
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
#if DEBUG_MOTOR
                            debug_print("UP calib SEND ERROR1\r\n");
#endif
                        }
                        break;

                    case calibPosEnd:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(10000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                homePosition = motor_current_position();
                                posOffset = MOTOR_USTEPS*steps_caliv_vol;

                                targetPosition = homePosition + posOffset;
                                calibState = calibVol;
                                set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*steps_caliv_vol, 2, 200);
#if DEBUG_MOTOR
                                debug_print("to flow vol\r\n");
#endif
                            } else if (notif_recv & MOTOR_NOTIF_HALT){
                                motor_anticipated_stop();
                                motor_disable();
                                motorState = motorInit;
#if DEBUG_MOTOR
                                debug_print("to motor INIT \r\n");
#endif
                            } else {
                                xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
#if DEBUG_MOTOR
                                debug_print("notif received %d\r\t",notif_recv);
                                debug_print("pos end ERROR1\r\n");
#endif
                            }
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
#if DEBUG_MOTOR
                            debug_print("pos end ERROR2\r\n");
#endif
                        }
                        break;
                    

                    case calibVol:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(5000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                //TODO add volume check!
                                calibState = calibVolEnd;
                                targetPosition = homePosition;
                                set_motor_goto_position_accel_exec(targetPosition, MOTOR_USTEPS*steps_caliv_vol, 2, 200);
#if DEBUG_MOTOR
                                debug_print("to flow vol end\r\n");
#endif
                            } else if (notif_recv & MOTOR_NOTIF_HALT){
                                motor_anticipated_stop();
                                motor_disable();
                                motorState = motorInit;
#if DEBUG_MOTOR
                                debug_print("to motor INIT \r\n");
#endif
                            } else {
                                xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                            }
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                        }
                        break;

                    case calibVolEnd:
                        // BOUNDED wait for limit switch up 
                        n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(5000));
                        
                        // Verify deadline
                        if (n_wait_recv){
                            // Check for undesirable notification
                            if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                motor_disable();
                                motorState = motorStopped;
#if DEBUG_MOTOR
                                debug_print("END INIT PHASE\r\n");
#endif
                            } else if (notif_recv & MOTOR_NOTIF_HALT){
                                motor_anticipated_stop();
                                motor_disable();
                                motorState = motorInit;
#if DEBUG_MOTOR
                                debug_print("to motor INIT \r\n");
#endif
                            } else {
                                xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                            }
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
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
                    previousWakeTime = xTaskGetTickCount();
#if DEBUG_MOTOR
                    debug_print("TCT %u \r\n",TCT);
                    debug_print("Ti %u \r\n",Ti);
                    debug_print("tot_pulses %u \r\n",tot_pulses);
                    debug_print("plat pulses %u \r\n",plateau_pulses);
                    debug_print("Ti pulses %u \r\n",insp_pulses);
                    debug_print("TtotTi %u \r\n",T_tot_Ti);
                    debug_print("TtotTe %u \r\n",T_tot_Te);
                    debug_print("f_insp %u \r\n",f_insp);
                    debug_print("f_exp %u \r\n",f_exp);
#endif
                }
                break;

            case motorRunning:
                switch (breathState){
                    case insp:
                        // BOUNDED Wait notification
                        xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(3000));
                        if (notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                            breathState = plateau;
                            targetPosition = homePosition + insp_pulses + plateau_pulses; 
                            set_motor_goto_position(targetPosition, f_plateau);
#if DEBUG_MOTOR
                            debug_print("to plateau \r\n");
#endif
                        } else if (notif_recv & MOTOR_NOTIF_HALT) {
                            motor_anticipated_stop();
                            breathState = stopping;
                            targetPosition = homePosition;
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("to motor STOPPING \r\n");
#endif
                        // TODO update state based on the notification value 

                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);    
                        }
                        break;

                    case plateau:
                        // BOUNDED Wait notification
                        xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(2000));

                        if (notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                            breathState = exp;
                            targetPosition = homePosition;
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("to exp \r\n");
#endif
                        } else if (notif_recv & MOTOR_NOTIF_HALT) {
                            motor_anticipated_stop();
                            breathState = stopping;
                            targetPosition = homePosition;
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("to motor STOPPING \r\n");
#endif
                            // TODO update state based on the notification value
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                        }

                         
			break;  

                    case exp:
                        // BOUNDED Wait notification
                        xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(3000));

                        if (notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                            breathState = cycleEnd;
                            motor_disable();
#if DEBUG_MOTOR
                            debug_print("to wait cycle end \r\n");
#endif
                        } else if (notif_recv & MOTOR_NOTIF_HALT) {
                            motor_anticipated_stop();
                            breathState = stopping;
                            targetPosition = homePosition;
                            set_motor_goto_position_accel_exec(targetPosition, f_exp, 2, 200);
#if DEBUG_MOTOR
                            debug_print("to motor STOPPING \r\n");
#endif
                            // TODO update state based on the notification value
                        } else {
                            xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                        }

                        
			break;

                    // TODO: Add RECALIBRATION step

                    case cycleEnd:
                        vTaskDelayUntil(&previousWakeTime,ticksTctTime);
                        breathState = startNewCycle;
#if DEBUG_MOTOR
                        debug_print("to Start new cycle \r\n");
#endif
                        break;
        
                    case startNewCycle:
                        compute_config();
                        motor_enable();
                        breathState = insp;
                        targetPosition = homePosition + insp_pulses;
                        set_motor_goto_position_accel_exec(targetPosition, f_insp, 2, 200); 
#if DEBUG_MOTOR
                        debug_print("Ti pulses used %u \r\n",targetPosition);
                        debug_print("=> target %u \r\n",targetPosition);
                        debug_print("cur pos %u \r\n",motor_current_position());
                        debug_print("home %u \r\n",homePosition);
                        debug_print("to insp \r\n");
#endif
                        break;
                

                    case stopping:
                        // BOUNDED wait for limit switch up 
                        if(motor_moving()){
                            n_wait_recv = xTaskNotifyWait(0x0,MOTOR_FULL_BITS,&notif_recv,pdMS_TO_TICKS(5000));

                            // Verify deadline
                            if (n_wait_recv){
                                // Check for undesirable notification
                                if(notif_recv & MOTOR_NOTIF_MOVEMENT_FINISHED) {
                                    //TODO add volume check!
                                    motor_disable();
                                    motorState = motorStopped;
#if DEBUG_MOTOR
                                    debug_print("to motor stopped\r\n");
#endif
                                } else {
                                    xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
                                }
                            } else {
                                xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);  
                            }
                        } else {
                            motor_disable();
                            motorState = motorStopped;
#if DEBUG_MOTOR
                            debug_print("to motor stopped\r\n");
#endif
                        }
                        break;
                }
                break;

        }
#else
    vTaskDelay(pdMS_TO_TICKS(100));
#endif
   }
}

