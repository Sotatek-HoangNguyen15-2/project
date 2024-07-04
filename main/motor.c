#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "motor.h"

void pin_init(gpio_num_t gpio_num)
{
  esp_rom_gpio_pad_select_gpio(gpio_num);
  // Set the GPIO as a push/pull output
  gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
}

void pin_set_level(gpio_num_t gpio_num, int level)
{
  gpio_set_level(gpio_num, level);
}

static void esp_config_io(gpio_config_t *io_conf)
{
  // Khởi tạo cấu hình GPIO
  io_conf->intr_type = GPIO_INTR_DISABLE;       // Vô hiệu hóa ngắt
  io_conf->mode = GPIO_MODE_OUTPUT;             // Chế độ output
  io_conf->pin_bit_mask = (1ULL << GPIO_NUM_2); // Sử dụng chân GPIO đã được định nghĩa
  io_conf->pull_down_en = GPIO_PULLDOWN_ENABLE; // kích hoạt pull-down resistor
  io_conf->pull_up_en = GPIO_PULLDOWN_ENABLE;   // kích hoạt pull-up resistor
}

void motor_vibrate(uint32_t duty_cycle, uint8_t channel)
{
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty_cycle);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

void init_motor()
{
  gpio_config_t io_conf;
  // Cấu hình GPIO
  esp_config_io(&io_conf);

  esp_init_pwm();
}

void esp_init_pwm()
{
  // Initialize LEDC PWM
  ledc_timer_config_t timer_conf;
  timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  timer_conf.timer_num = LEDC_TIMER_0;
  timer_conf.duty_resolution = LEDC_TIMER_12_BIT;
  timer_conf.freq_hz = 5000;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ledc_conf1;
  ledc_channel_config_t ledc_conf2;

  ledc_conf1.gpio_num = GPIO_NUM_4;
  ledc_conf1.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf1.channel = LEDC_CHANNEL_1;
  ledc_conf1.intr_type = LEDC_INTR_DISABLE;
  ledc_conf1.timer_sel = LEDC_TIMER_0;
  ledc_conf1.duty = 0;
  ledc_conf1.hpoint = 0;
  ledc_channel_config(&ledc_conf1);

  ledc_conf2.gpio_num = GPIO_NUM_2;
  ledc_conf2.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_conf2.channel = LEDC_CHANNEL_0;
  ledc_conf2.intr_type = LEDC_INTR_DISABLE;
  ledc_conf2.timer_sel = LEDC_TIMER_0;
  ledc_conf2.duty = 0;
  ledc_conf2.hpoint = 0;
  ledc_channel_config(&ledc_conf2);
}
