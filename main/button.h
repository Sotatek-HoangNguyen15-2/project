#ifndef BUTTON_H
#define BUTTON_H
#include "esp_err.h"
#include "hal/gpio_types.h"

typedef enum
{
  LOW_TO_HIGH = 1,
  HIGH_TO_LOW,
  ANY_EDGE
} interrupt_type_edge_t;

typedef void (*input_callback_t)(int);

void button_vCreateInput(gpio_num_t gpio_num, interrupt_type_edge_t type);
void button_vSetCallback(void *callbackk);
#endif