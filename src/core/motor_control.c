#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal/io.h"
#include "core/main_task.h"
#include "core/system.h"
#include "core/utils.h"
#include "hal/pins.h"
#include "hal/motor.h"
#include "core/debug.h"
#include "core/motor_control.h"
#include "core/volume.h"
#include "hal/limit_switch.h"


// TODO
// * delay motor disable to prevent overrunning
// * fix timeout values

#define DEBUG_MOTOR 1
#if DEBUG_MOTOR
#define MOTOR_DEBUG_PRINT debug_print
#else
#define MOTOR_DEBUG_PRINT fake_debug_print
#endif // DEBUG_MOTOR
#define MOTOR_ERROR_PRINT debug_print

#define MOTOR_NOTIF_LIM (MOTOR_NOTIF_LIM_UP | MOTOR_NOTIF_LIM_DOWN)
#define MOTOR_NOTIF_CYCLE (MOTOR_NOTIF_LIM | MOTOR_NOTIF_GLOBAL_STATE | MOTOR_NOTIF_OVER_PRESSURE | MOTOR_NOTIF_MOVEMENT_FINISHED)

#define MOTOR_UP_POSITION (2000*MOTOR_USTEPS)

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
uint32_t posOffset;

TickType_t cycleStartTime;
uint32_t ticksTctTime;

// TODO reset proper value!
const uint32_t f_home = 100*MOTOR_USTEPS; // steps/s (not µsteps/s)

const int32_t steps_calib_down = 2000;
const int32_t steps_calib_up = 1000;
const int32_t steps_calib_end = 50;
const int32_t steps_calib_vol = 500;
const int32_t steps_recalib = 1000;

const uint32_t thresh_calib_vol_mil = 600;

static uint8_t recalibrateFlag;
static uint8_t sleep_finished;
static uint32_t notif = 0;


static const char *state_names[] = {
    "motorInit",
    "motorCalibrating",
    "motorStopped",
    "motorRunning",
    "motorError"
};

static const char *bstate_names[] = {
    "insp",
    "plateau",
    "exp",
    "cycleEnd",
    "startNewCycle",
    "stopping",
    "reCalibUp",
    "reCalibUpWaitStop",
    "reCalibHome",
    "expStopping",
    "reCalibDown",
};

static const char *cstate_names[] = {
    "calibStart",
    "calibDown",
    "calibDownWaitStop",
    "calibUp",
    "calibUpWaitStop",
    "calibPosEnd",
    "calibVol",
    "calibVolEnd"
};

static const char *notif_names[] = {
    "MOTOR_NOTIF_LIM_DOWN",
    "MOTOR_NOTIF_LIM_UP",
    "",
    "MOTOR_NOTIF_MOVEMENT_FINISHED",
    "MOTOR_NOTIF_GLOBAL_STATE",
    "",
    "",
    "MOTOR_NOTIF_OVER_PRESSURE",
};

// Threshold for volume calibraiton

// 0 replaces the whole task by an empty loop for testing purposes
#define MOTOR_ACTIVE 1

static void genMotorError(char *msg);
static void motorStChCalib(char *state);
static void motorGlStCh(char *state);
static void motorUnimplementedCase(char *state);
static void resumeBoundedWaitNotification();
static void boundedWaitNotification(uint32_t admissible_notifications, uint32_t ignored_notifications, TickType_t ticksToWait);
static uint8_t sleepWaitNotif(TickType_t endOfWait, uint32_t *notif);
static uint32_t startCycleEnd();
static void startRecalib();
static void unboundedWaitNotification(uint32_t admissible_notifications);
static void move_and_wait(uint32_t targetPosition, uint32_t max_duration, uint32_t admissible_notifications);
static void doExpiration();
static uint8_t test_notif(uint32_t tested_notif);

// TODO something real
uint32_t vol2steps(uint8_t  tidal_vol){
    return 10*tidal_vol;
}

void compute_config() {
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

    T_tot_plateau = Ti / 8;
    T_tot_Ti = Ti - T_tot_plateau;
    // TODO make this depend on number of pulses ? / "as fast as possible" means speed should not depend on volume
    T_tot_Te = 750; // As fast as possible. Should be less than 1s (for 30 BPM + I/E+1 case)

    // 1000 factor since time in ms and freq in Hz
    tmp_f_insp = insp_pulses * 1000;
    tmp_f_plateau = plateau_pulses * 1000;
    tmp_f_exp = tot_pulses * 1000;

    f_insp = tmp_f_insp / T_tot_Ti;
    f_plateau = tmp_f_plateau / T_tot_plateau;
    f_exp = tmp_f_exp / T_tot_Te;

    MOTOR_DEBUG_PRINT("ie %u \r\n",ie);
    MOTOR_DEBUG_PRINT("bpm %u \r\n",bpm);
    MOTOR_DEBUG_PRINT("TCT %u \r\n",TCT);
    MOTOR_DEBUG_PRINT("Ti %u \r\n",Ti);
    MOTOR_DEBUG_PRINT("nsteps %u \r\n",n_steps);
    MOTOR_DEBUG_PRINT("pulse_plateau %u \r\n",plateau_pulses);
    MOTOR_DEBUG_PRINT("Ttot plateau %u \r\n",T_tot_plateau);
    MOTOR_DEBUG_PRINT("tmp_f_plateau %u \r\n",tmp_f_plateau);
    MOTOR_DEBUG_PRINT("f_plateau %u \r\n",f_plateau);
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
    recalibrateFlag = 0;
}

static void genMotorError(char *msg) {
    MOTOR_DEBUG_PRINT("[MOTOR] genMotorError: %s\r\n", msg);
    motor_anticipated_stop();
    motor_disable();
    motorState = motorError;
    xTaskNotify(mainTaskHandle, NOTIF_MOTOR_ERROR, eSetBits);
}

static void motorStChCalib(char *state) {
    motor_anticipated_stop();
    motor_disable();
    motorState = motorInit;
    MOTOR_DEBUG_PRINT("[MOTOR] halting in %s\r\n", state);
}

static void motorGlStCh(char *state) {
    // TODO handle not only stopping
    motor_anticipated_stop();
    breathState = stopping;
    MOTOR_DEBUG_PRINT("[MOTOR] halting in %s\r\n", state);
    // TODO cleanup this
    set_motor_goto_position_accel_exec(homePosition, f_exp, 2, 200*MOTOR_USTEPS);
    boundedWaitNotification(MOTOR_NOTIF_CYCLE, 0, pdMS_TO_TICKS(10000));
}

static void motorUnimplementedCase(char *state) {
    MOTOR_ERROR_PRINT("[MOTOR] Unimplemnted %s\r\n", state);
    genMotorError("unimplemented");
}

static TickType_t endOfBoundedWait;
static uint32_t allowedNotifications;
static uint32_t ignoredNotifications;

static void resumeBoundedWaitNotification() {
    uint32_t notif_recv;
    TickType_t start_wait_time = xTaskGetTickCount();
    while (xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,endOfBoundedWait-start_wait_time) == pdTRUE) {
        // received notification
        if (notif_recv & ~allowedNotifications) {
            // invalid notification
            genMotorError("invalid notif");
            MOTOR_DEBUG_PRINT("[MOTOR] Unexp notif %x\r\n", notif_recv);
            notif = notif_recv;
            return;
        } else if (notif_recv & ~ignoredNotifications) {
            // non-ignored notification
            notif = notif_recv;
            return;
        } else {
            // ignored notification, continue
        }
    }
    // no notification received, timeout
    genMotorError("TIMEOUT");
    notif = notif_recv;
}

static void boundedWaitNotification(uint32_t admissible_notifications, uint32_t ignored_notifications, TickType_t ticksToWait) {
    TickType_t start_wait_time = xTaskGetTickCount();
    endOfBoundedWait = start_wait_time + ticksToWait;
    allowedNotifications = admissible_notifications | ignored_notifications;
    ignoredNotifications = ignored_notifications;
    resumeBoundedWaitNotification();
}

// return 1 if non-interrupted sleep
static uint8_t sleepWaitNotif(TickType_t endOfWait, uint32_t *notif) {
    TickType_t start_wait_time = xTaskGetTickCount();
    if (endOfWait <= start_wait_time) {
        MOTOR_DEBUG_PRINT("[MOTOR] sleepwaitnotify 0\r\n");
        return 1;
    } else {
        MOTOR_DEBUG_PRINT("[MOTOR] sleepwaitnotify for %i\r\n", endOfWait-start_wait_time);
        return !xTaskNotifyWait(0x0,ALL_NOTIF_BITS,notif,endOfWait-start_wait_time);
    }
}

// return 1 and clear the notification flag if the notification is present
static uint8_t test_notif(uint32_t tested_notif) {
    if (notif & tested_notif) {
        notif &= ~tested_notif;
        int8_t i;
        for (i=0; i < 8; i++) {
            if ((tested_notif >>i) & 0x1) {
                MOTOR_DEBUG_PRINT("NOTIF %s\r\n", notif_names[i]);
            }
        }
        return 1;
    } else {
        return 0;
    }
}

static uint32_t startCycleEnd() {
    motor_disable();
    breathState = cycleEnd;
    cycleStartTime += ticksTctTime;
    uint32_t notif;
    MOTOR_DEBUG_PRINT("[MOTOR] startCycleEnd\r\n");
    sleep_finished = sleepWaitNotif(cycleStartTime, &notif);
    return notif;
}


static void startRecalib() {
    breathState = reCalibUp;
    MOTOR_DEBUG_PRINT("[MOTOR] reCalibUp pos %u\r\n", motor_current_position());
    move_and_wait(0, f_home, MOTOR_NOTIF_CYCLE);
}

static void unboundedWaitNotification(uint32_t admissible_notifications) {
    uint32_t notif_recv;
    xTaskNotifyWait(0x0,ALL_NOTIF_BITS,&notif_recv,portMAX_DELAY);
    // received notification
    if (notif_recv & ~admissible_notifications) {
        // invalid notification
        genMotorError("invalid notif");
        MOTOR_DEBUG_PRINT("[MOTOR] Unexp notif %x\r\n", notif_recv);
    }
    notif = notif_recv;
}

static void move_and_wait(uint32_t targetPosition, uint32_t max_freq, uint32_t admissible_notifications) {
    // max speed: 1200 steps/s. Acceleration: 200 steps/s/10ms
    // -> max 3*120 = 360 ms deceleration + acceleration + deceleration (we take 400 to have a margin)
    // total time is thus bounded by 360 ms + 1000*abs(currentPosition-targetPosition) / (max_freq)
    uint32_t max_duration = 400 + (1000*ABS(((int32_t) targetPosition) - ((int32_t) motor_current_position())))/max_freq;
    set_motor_goto_position_accel_exec(targetPosition, max_freq, 2, 200*MOTOR_USTEPS);
    MOTOR_DEBUG_PRINT("[MOTOR] move_and_wait curr:");
    MOTOR_DEBUG_PRINT("%lu target: %lu max_freq:", motor_current_position(), targetPosition);
    MOTOR_DEBUG_PRINT("%lu max_dur: %lu\r\n", max_freq, max_duration);
    boundedWaitNotification(admissible_notifications, 0, pdMS_TO_TICKS(max_duration));

}

static void doExpiration() {
    MOTOR_DEBUG_PRINT("[MOTOR] doExpiration\r\n");
    breathState = exp;
    targetPosition = homePosition;
    return move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
}

#if MOTOR_ACTIVE
void MotorControlTask(void *pvParameters)
{
    while (1)
    {
        MOTOR_DEBUG_PRINT("[MOTOR] In ");
        MOTOR_DEBUG_PRINT("%s / %s", state_names[motorState], bstate_names[breathState]);
        MOTOR_DEBUG_PRINT(" (%s)\r\n", cstate_names[calibState]);
        switch (motorState) {
            case motorInit:
                if (globalState == calibration) {
                    motorState = motorCalibrating;
                    calibState = calibStart;
                    MOTOR_DEBUG_PRINT("[MOTOR] Start calibration\r\n");
                    notif = 0;
                } else {
                    // NON BOUNDED wait for calibrating notification
                    unboundedWaitNotification(MOTOR_NOTIF_GLOBAL_STATE);
                }
                break;

            case motorCalibrating:
                if (test_notif(MOTOR_NOTIF_OVER_PRESSURE)) {
                    motorUnimplementedCase("calib & OP");
                } else if (test_notif(MOTOR_NOTIF_GLOBAL_STATE)) {
                    motorStChCalib("");
                } else if (test_notif(MOTOR_NOTIF_MOVEMENT_FINISHED)) {
                    switch (calibState) {
                        case calibStart:
                            genMotorError("URCH calibStart");
                            break;

                        case calibDown:
                            genMotorError("Not touched calibDown");
                            break;

                        case calibDownWaitStop:
                            // Compute next position
                            posOffset = MOTOR_USTEPS*steps_calib_up;
                            // Manually set the absolute position of the motor
                            // Use this trick to avoid negative absolute positions
                            set_motor_current_position_value(posOffset);
                            targetPosition = 0;
                            calibState = calibUp;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibDownWaitStop\r\n");
                            move_and_wait(targetPosition, f_home, MOTOR_NOTIF_CYCLE);
                            break;

                        case calibUp:
                            genMotorError("Not touched calibUp");
                            break;

                        case calibUpWaitStop:
                            // Compute next position   
                            posOffset = MOTOR_UP_POSITION + MOTOR_USTEPS*steps_calib_end;
                            set_motor_current_position_value(MOTOR_UP_POSITION);
                            calibState = calibPosEnd;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibUpWaitStop\r\n");
                            move_and_wait(posOffset, f_home, MOTOR_NOTIF_CYCLE);
                            break;

                        case calibPosEnd:
                            homePosition = motor_current_position();
                            // Start volume calibration
                            posOffset = MOTOR_USTEPS*steps_calib_vol;
                            targetPosition = homePosition + posOffset;
                            calibState = calibVol;
                            // Reset volume prior to flow check
                            reset_volume();
                            MOTOR_DEBUG_PRINT("[MOTOR] Start flow check.\r\n");
                            move_and_wait(targetPosition, steps_calib_vol*MOTOR_USTEPS, MOTOR_NOTIF_CYCLE);
                            break;

                        case calibVol:
                            if (MOCK_VOLUME_SENSOR || volume > VOLUME_CHECK_THRESHOLD) {
                                // Sufficient volume insufflated.
                                calibState = calibVolEnd;
                                MOTOR_DEBUG_PRINT("[MOTOR] Flow check: OK\r\n");
                                move_and_wait(homePosition, steps_calib_vol*MOTOR_USTEPS, MOTOR_NOTIF_CYCLE);
                            } else {
                                MOTOR_DEBUG_PRINT("[MOTOR] Flow check: FAIL\r\n");
                                xTaskNotify(mainTaskHandle, NOTIF_INCORRECT_FLOW, eSetBits);
                                motor_disable();
                                motorState = motorError;
                            }
                            break;

                        case calibVolEnd:
                            // case MOTOR_NOTIF_MOVEMENT_FINISHED
                            motor_disable();
                            motorState = motorStopped;
                            MOTOR_DEBUG_PRINT("[MOTOR] END CALIB\r\n");
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_UP)) {
                    switch (calibState) {
                        case calibDown:
                            // We were unrolling the rope, we will soon start
                            // enrolling it in the correct direction.
                            // Continue in this state.
                            resumeBoundedWaitNotification();
                            break;

                        case calibUp:
                            // Stop motor
                            motor_anticipated_stop();
                            calibState = calibUpWaitStop;
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibUp\r\n");
                            boundedWaitNotification(MOTOR_NOTIF_CYCLE, MOTOR_NOTIF_LIM, pdMS_TO_TICKS(2*1000/f_home));
                            break;

                        default:
                            genMotorError("unexpected notif");
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_DOWN)) {
                    switch (calibState) {
                        case calibDown:
                            // Finished going down, let's now stop the motor and go up.
                            // Stop motor
                            motor_anticipated_stop();
                            MOTOR_DEBUG_PRINT("[MOTOR] finished calibDown\r\n");
                            calibState = calibDownWaitStop;
                            boundedWaitNotification(MOTOR_NOTIF_CYCLE, MOTOR_NOTIF_LIM, pdMS_TO_TICKS(2*1000/f_home));
                            break;

                        default:
                            genMotorError("unexpected notif");
                    }
                } else {
                    switch (calibState) {
                        case calibStart:
                            compute_config();
                            calibState = calibDown;
                            set_motor_current_position_value(0);
                            targetPosition = MOTOR_USTEPS*steps_calib_down;
                            motor_enable();
                            MOTOR_DEBUG_PRINT("to calib down\r\n");
                            move_and_wait(targetPosition, f_home, MOTOR_NOTIF_CYCLE);
                            break;

                        default:
                            genMotorError("URCH");
                    }
                }
                break;

            case motorStopped:
                // if notif == START: update motor state
                if (globalState == run) {
                    motorState = motorRunning;
                    breathState = startNewCycle;
                    cycleStartTime = xTaskGetTickCount();
                    MOTOR_DEBUG_PRINT("TCT %u \r\n",TCT);
                    MOTOR_DEBUG_PRINT("Ti %u \r\n",Ti);
                    MOTOR_DEBUG_PRINT("tot_pulses %u \r\n",tot_pulses);
                    MOTOR_DEBUG_PRINT("plat pulses %u \r\n",plateau_pulses);
                    MOTOR_DEBUG_PRINT("Ti pulses %u \r\n",insp_pulses);
                    MOTOR_DEBUG_PRINT("TtotTi %u \r\n",T_tot_Ti);
                    MOTOR_DEBUG_PRINT("TtotTe %u \r\n",T_tot_Te);
                    MOTOR_DEBUG_PRINT("f_insp %u \r\n",f_insp);
                    MOTOR_DEBUG_PRINT("f_exp %u \r\n",f_exp);
                } else {
                    // Non bounded wait for notif
                    unboundedWaitNotification(MOTOR_NOTIF_GLOBAL_STATE);
                }
                break;

            case motorRunning:
                if (test_notif(MOTOR_NOTIF_OVER_PRESSURE)) {
                    motorUnimplementedCase("running & OP");
                } else if (test_notif(MOTOR_NOTIF_GLOBAL_STATE)) {
                    motorGlStCh("");
                } else if (test_notif(MOTOR_NOTIF_MOVEMENT_FINISHED)) {
                    switch (breathState) {
                        case insp:
                            breathState = plateau;
                            targetPosition = homePosition + insp_pulses + plateau_pulses;
                            move_and_wait(targetPosition, f_plateau, MOTOR_NOTIF_CYCLE);
                            MOTOR_DEBUG_PRINT("[MOTOR] to plateau %lu %lu\r\n", plateau_pulses, f_plateau);
                            break;
                        case plateau:
                            doExpiration();
                            break;
                        case exp:
                            if (recalibrateFlag) {
                                startRecalib();
                            } else {
                                MOTOR_DEBUG_PRINT("[MOTOR] to wait cycle end\r\n");
                                notif = startCycleEnd();
                            }
                            break;
                        case expStopping:
                            // Move a bit down to escape from limit switch
                            breathState = reCalibDown;
                            move_and_wait(motor_current_position()+steps_calib_end*MOTOR_USTEPS, f_home, MOTOR_NOTIF_CYCLE);
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("[MOTOR] MF while cycleEnd\r\n");
                            sleep_finished = sleepWaitNotif(cycleStartTime, &notif);
                            break;
                        case stopping:
                                motor_disable();
                                motorState = motorStopped;
                                MOTOR_DEBUG_PRINT("[MOTOR] to motor stopped\r\n");
                            break;
                        case reCalibUpWaitStop:
                            // Compute actual position when notified
                            set_motor_current_position_value(MOTOR_UP_POSITION);
                            // Compute next position
                            targetPosition = MOTOR_UP_POSITION + MOTOR_USTEPS*steps_calib_end;
                            breathState = reCalibHome;
                            MOTOR_DEBUG_PRINT("[MOTOR] to reCalibHome\r\n");
                            move_and_wait(targetPosition, f_exp, MOTOR_NOTIF_CYCLE);
                            break;
                        case reCalibHome:
                            recalibrateFlag = 0;
                            notif = startCycleEnd();
                            break;
                        case reCalibDown:
                            startRecalib();
                            break;
                        case startNewCycle:
                        case reCalibUp:
                            genMotorError("URCH");
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_UP)) {
                    switch (breathState) {
                        case insp:
                        case plateau:
                            recalibrateFlag = 1;
                            resumeBoundedWaitNotification();
                            break;
                        case exp:
                            motor_anticipated_stop();
                            breathState = expStopping;
                            boundedWaitNotification(MOTOR_NOTIF_CYCLE, MOTOR_NOTIF_LIM, pdMS_TO_TICKS(2*1000/f_home));
                            break;
                        case expStopping:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while expStopping\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("[MOTOR] LIM while cycleEnd\r\n");
                            sleep_finished = sleepWaitNotif(cycleStartTime, &notif);
                            break;
                        case stopping:
                            recalibrateFlag = 1;
                            motor_anticipated_stop();
                            motor_disable();
                            motorState = motorStopped;
                            MOTOR_ERROR_PRINT("[MOTOR] UP while stopping\r\n");
                            break;
                        case reCalibUp:
                            // Stop motor
                            motor_anticipated_stop();
                            breathState = reCalibUpWaitStop;
                            MOTOR_DEBUG_PRINT("[MOTOR] to reCalibUpWaitStop\r\n");
                            boundedWaitNotification(MOTOR_NOTIF_CYCLE, MOTOR_NOTIF_LIM, pdMS_TO_TICKS(2*1000/f_home));
                            break;
                        case reCalibHome:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while reCalibHome\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case startNewCycle:
                        case reCalibUpWaitStop:
                        case reCalibDown:
                            genMotorError("URCH");
                            break;
                    }
                } else if (test_notif(MOTOR_NOTIF_LIM_DOWN)) {
                    switch (breathState) {
                        case insp:
                        case plateau:
                            motor_anticipated_stop();
                            recalibrateFlag = 1;
                            MOTOR_DEBUG_PRINT("recalibrate INSP asked \r\n");
                            // TODO wait a bit... to respect Ti
                            doExpiration();
                            break;
                        case exp:
                            MOTOR_ERROR_PRINT("[MOTOR] LIM_DOWN while EXP\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case expStopping:
                            MOTOR_ERROR_PRINT("[MOTOR] UP while expStopping\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case cycleEnd:
                            MOTOR_ERROR_PRINT("[MOTOR] LIM while cycleEnd\r\n");
                            sleep_finished = sleepWaitNotif(cycleStartTime, &notif);
                            break;
                        case stopping:
                                recalibrateFlag = 1;
                                motor_anticipated_stop();
                                motor_disable();
                                motorState = motorStopped;
                                MOTOR_ERROR_PRINT("[MOTOR] UP while stopping\r\n");
                            break;
                        case reCalibUp:
                            MOTOR_DEBUG_PRINT("[MOTOR] reCalibUp wait\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case reCalibHome:
                            MOTOR_ERROR_PRINT("[MOTOR] DOWN while reCalibHome\r\n");
                            resumeBoundedWaitNotification();
                            break;
                        case startNewCycle:
                        case reCalibUpWaitStop:
                        case reCalibDown:
                            genMotorError("URCH");
                            break;
                    }
                } else {
                    switch (breathState) {
                        case cycleEnd:
                            if (sleep_finished) {
                                breathState = startNewCycle;
                                MOTOR_DEBUG_PRINT("to Start new cycle \r\n");
                            } else {
                                // Unreachable
                                genMotorError("cycleEnd URCH");
                            }
                            break;
                        case startNewCycle:
                            compute_config();
                            motor_enable();
                            breathState = insp;
                            targetPosition = homePosition + insp_pulses;
                            MOTOR_DEBUG_PRINT("Ti pulses used %u \r\n",targetPosition);
                            MOTOR_DEBUG_PRINT("=> target %u \r\n",targetPosition);
                            MOTOR_DEBUG_PRINT("cur pos %u \r\n",motor_current_position());
                            MOTOR_DEBUG_PRINT("home %u \r\n",homePosition);
                            MOTOR_DEBUG_PRINT("to insp \r\n");
                            move_and_wait(targetPosition, f_insp, MOTOR_NOTIF_CYCLE);
                            break;
                        default:
                            genMotorError("URCH");
                    }
                }
                break;

            case motorError:
                // TODO
                vTaskDelayUntil(&cycleStartTime,pdMS_TO_TICKS(10000));
                break;
        }
    }
}

#else // MOTOR_ACTIVE

void MotorControlTask(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif // MOTOR_ACTIVE

