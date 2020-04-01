#ifndef PWM_BOUND_H_
#define PWM_BOUND_H_

void setup_motor();
void set_motor_goto_position(const unsigned long position, const unsigned int speed);
uint8_t motor_moving();
int32_t motor_remaining_distance();

#endif // PWM_BOUND_H_
