#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <string.h>
#include "wifi.h"
#include "motor.h"
#include "as5600.h"
#include "button.h"
#include <stdio.h>

static const char *TAG = "ESP";
bool flag_pause = false;

motor_pid_t motor_pid;

void button_vEventCallback(int pin)
{
    if (pin == GPIO_NUM_13)
    {
        if (flag_pause)
            motor_vibrate(0, LEDC_CHANNEL_1);
        flag_pause = !flag_pause;
        return;
    }
}

void app_main(void)
{
    button_vCreateInput(GPIO_NUM_13, HIGH_TO_LOW);
    button_vSetCallback(button_vEventCallback);

    pin_init(GPIO_NUM_18);
    pin_set_level(GPIO_NUM_18, 1);

    pin_init(GPIO_NUM_4);
    pin_set_level(GPIO_NUM_4, 0);

    init_motor();
    init_as5600();
    init_task_tcp();

    motor_pid.kp = 5.0f;
    motor_pid.ki = 0.5f;
    motor_pid.kd = 0.002f;

    motor_pid.clock_wise = MOTOR_CW;
}