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

#define UART_TX GPIO_NUM_17
#define UART_RX GPIO_NUM_16

#define EX_UART_NUM UART_NUM_1

#define LOG_POSITION_OFF
// #define LOG_VECLOCITY_OFF

QueueHandle_t queue_message_response;

// Define the queue handle
QueueHandle_t myQueue;

static const char *TAG = "ESP";
bool flag_pause = false;

uint16_t pre_pos;
uint16_t cur_pos;
double veclocity;

motor_pid_t motor_pid;
static bool flag_first = false;

void button_vEventCallback(int pin);

void uart_vCreate()
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, 2560 * 2, 2560 * 2, 30, &queue_message_response, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    // Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void task_log_uart()
{
    uint8_t buffer_pos[2];
    uint8_t buffer_vec[4];

    uint16_t current_pos;
    while (1)
    {
        // Receive a message from the queue
        if (xQueueReceive(myQueue, &current_pos, portMAX_DELAY))
        {
            if (flag_first == false)
            {
                pre_pos = current_pos;
                flag_first = true;
            }

            // printf("\n current_pos is %d, pre_pos is %d\n", current_pos, pre_pos);
#ifndef LOG_POSITION_OFF
            buffer_pos[0] = current_pos >> 8;
            buffer_pos[1] = current_pos;
            uart_write_bytes(EX_UART_NUM, buffer_pos, 2);
#endif

#ifndef LOG_VECLOCITY_OFF
            caculate_vec(current_pos, pre_pos);
            uint32_t vec = (uint32_t)veclocity;

            buffer_vec[0] = vec >> 24;
            buffer_vec[1] = vec >> 16;
            buffer_vec[2] = vec >> 8;
            buffer_vec[3] = vec;
            // printf("\n vec is %ld, veclocity is %lf\n", vec, veclocity);
            uart_write_bytes(EX_UART_NUM, buffer_vec, 4);
#endif
            pre_pos = current_pos;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

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

    uart_vCreate();
    pin_init(GPIO_NUM_27);
    pin_set_level(GPIO_NUM_27, 1);

    // Create a queue capable of containing 10 integers
    myQueue = xQueueCreate(10, sizeof(uint16_t));

    if (myQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

    init_motor();
    init_as5600();
    // init_task_tcp();

    motor_pid.kp = 9.0f;
    motor_pid.ki = 0.5f;
    motor_pid.kd = 0.05f;

    motor_pid.clock_wise = MOTOR_CW;
    xTaskCreate(task_log_uart, "task_log_uart", 8092, NULL, 3, NULL);
}