#ifndef PWM_BOUND_H_
#define PWM_BOUND_H_

#include <stdbool.h>

/**
 * @setup_motor Setup the low-level motor hardware and state.
 */
void setup_motor();

/**
 * @motor_enable Enable the motor driving (hold to fixed position and allow movement).
 */
void motor_enable();

/**
 * @motor_disable Disable the motor driving (freewheel and no current consumption).
 */
void motor_disable();

/**
 * @set_motor_goto_position_accel_exec Move motor to given position with acceleration ramps.
 * This function is non-blocking, movement state be checked with the motor_moving() function.
 * Additionnally, the motorControlTask is notified when the movement finishes.
 *
 * @param target_position_abs Position in steps to be reached at the end of the movement.
 * @param target_speed Maximum speed in steps per second at which the movement
 * is done. This speed might not be reached if the distance is too small to
 * allow for full acceleration and deceleration.
 * @param step_num_base Number of steps for the first speed level of the
 * acceleration ramp. Each level of the acceleration ramp lasts for
 * step_num_base/step_freq_base. Acceleration levels last for successive
 * multiples of step_num_base, with frequency being successive multiples of
 * step_freq_base until target_speed is reached.
 * Deceleration has a symmetric profile to acceleration.
 * @param step_freq_base Frequency in steps per second of the first acceleration level.
 */
void set_motor_goto_position_accel_exec(
        uint32_t target_position_abs,
        uint16_t target_speed,
        uint16_t step_num_base,
        uint16_t step_freq_base);

/**
 * @set_motor_goto_position Move at constant speed to the given position.
 * This function has no acceleration ramps, therefore speed should be low,
 * otherwise the motor will not start.
 * This function is non-blocking, movement state be checked with the motor_moving() function.
 * Additionnally, the motorControlTask is notified when the movement finishes.
 *
 * @param position Position in steps to be reached at the end of the movement.
 * @param speed Speed in steps per second at which the movement shoud occur.
 */
void set_motor_goto_position(const unsigned long position, const unsigned int speed);

/**
 * @motor_moving Check if the movement is moving.
 *
 * @return true if the motor is moving, false otherwise
 */
bool motor_moving();

/**
 * @motor_current_position Compute the current absolute position of the motor.
 *
 * @return The current absolute position of the motor.
 */
uint32_t motor_current_position();

/**
 * @motor_remaining_distance Compute the distance that remains until the end of
 * the movement.
 *
 * @return The absolute value of the difference between the current position
 * and the targent position for the end of the movement.
 */
uint32_t motor_remaining_distance();

/* @set_motor_current_position_value Reset the current absolute position of the
 * motor, while not moving it.
 * 
 * @param new_abs_position New absolute position of the motor.
 */
void set_motor_current_position_value(int32_t new_abs_position);

/** @motor_anticipated_stop Stops the motor as fast as possible.
 * The does not respect the acceleration ramps, so the motor may mis steps.
 * This function is non-blocking, movement state be checked with the motor_moving() function.
 * Additionnally, the motorControlTask is notified when the movement finishes.
 */
void motor_anticipated_stop();

extern volatile bool motor_inmotion;

#endif // PWM_BOUND_H_
