#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "motor.h"

#include "as5600.h"

double set_point = 90.0f;

static const char *TAG = "AS5600";
static uint16_t data_sensor;
static uint16_t root_point = 0;
static bool flag_find_pos = false;
extern uint16_t cur_pos;
extern double veclocity;
extern QueueHandle_t myQueue;
extern motor_pid_t motor_pid;
extern bool flag_pause;

// PID control variables
float integral = 0;
float previous_error = 0;

uint16_t get_root_point()
{
  return root_point;
}

static esp_err_t i2c_master_init(void)
{
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK)
  {
    return err;
  }
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void pid_control(double current_value)
{
  static double pwm;
  double error = (set_point - current_value) * motor_pid.clock_wise;
  integral += error * 0.01;
  double derivative = error - previous_error;
  double output = motor_pid.kp * error + motor_pid.ki * integral + (motor_pid.kd * derivative) * 100;
  previous_error = error;

  // Clamp max_output_value
  if (output > 1200.0f)
  {
    output = 1200.0f; // Max max_output_value
  }
  if (flag_find_pos == false)
    pwm = output;
  if (as5600_get_data_sensor() > 1000 && as5600_get_data_sensor() < 1100)
  {
    flag_find_pos = true;
    motor_vibrate(0, LEDC_CHANNEL_1);
  }
  if (flag_find_pos == false)
  {
    motor_vibrate(output, LEDC_CHANNEL_1);
  }
  else
  {
    motor_vibrate(0, LEDC_CHANNEL_1);
  }
}

static void read_as5600_data(void *pvParameters)
{
  static TickType_t last_time_read_position = 0;
  while (1)
  {
    uint8_t write_buffer = AS5600_REG_DATA;
    uint8_t read_buffer[2] = {0, 0};
    esp_err_t err;

    do
    {
      err = i2c_master_write_read_device(I2C_MASTER_NUM,
                                         AS5600_SENSOR_ADDR,
                                         &write_buffer,
                                         1,
                                         read_buffer,
                                         2,
                                         portMAX_DELAY);
      if (err != ESP_OK)
      {
        ESP_LOGI(TAG, "error I2C is %d", err);
      }
    } while (err != ESP_OK);

    data_sensor = ((read_buffer[0] << 8) | read_buffer[1]) & AS5600_READING_MASK;

    if (data_sensor > AS5600_PULSES_PER_REVOLUTION - 1)
    {
      data_sensor = AS5600_PULSES_PER_REVOLUTION - 1;
    }

    if (data_sensor > 1000 && data_sensor < 1100)
    {
      motor_vibrate(0, LEDC_CHANNEL_1);
    }

    if (xTaskGetTickCount() - last_time_read_position >= pdMS_TO_TICKS(10))
    {
      cur_pos = data_sensor;
      // Send the message to the queue
      if (xQueueSend(myQueue, &cur_pos, pdMS_TO_TICKS(1000)) != pdPASS)
      {
        ESP_LOGE(TAG, "Failed to send to queue");
      }
      else
      {
        last_time_read_position = xTaskGetTickCount();
        if (flag_pause == false && flag_find_pos == false)
          pid_control(veclocity);
      }
    }

    // ESP_LOGI(TAG, "Data sensor AS5600 is %d", data_sensor);
    root_point = data_sensor;
    vTaskDelay(pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  }
}

uint16_t as5600_get_data_sensor()
{
  return data_sensor;
}

void init_as5600()
{
  esp_err_t ret = i2c_master_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "I2C master initialization failed: %s", esp_err_to_name(ret));
    return;
  }

  xTaskCreate(read_as5600_data, "read_as5600_data", 8192, NULL, 2, NULL);
}