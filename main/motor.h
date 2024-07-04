#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
#include "hal/gpio_types.h"

#define MOTOR_CW 1
#define MOTOR_CCW -1

typedef struct
{
  float kp;
  float ki;
  float kd;

  int clock_wise;

  float integrator_limit;
  float accumulated_error;
  float previous_error;
  float max_output_value;

} motor_pid_t;

void motor_vibrate(uint32_t duty_cycle, uint8_t channel);
void init_motor();
void esp_init_pwm();

void pin_set_level(gpio_num_t gpio_num, int level);
void pin_init(gpio_num_t gpio_num);
#endif // define MOTOR_H