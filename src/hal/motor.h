#ifndef PWM_BOUND_H_
#define PWM_BOUND_H_

void setup_motor();
void enable_motor();
void disable_motor();
void set_motor_goto_position_accel_exec(
        uint32_t target_position_abs,
        const uint16_t target_speed,
        const uint16_t step_num_base,
        const uint16_t step_freq_base);
void set_motor_goto_position(const unsigned long position, const unsigned int speed);
uint8_t motor_moving();
int32_t motor_remaining_distance();
extern volatile uint8_t motor_inmotion;

#endif // PWM_BOUND_H_
