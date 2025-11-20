#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H
#include "../bsw/gpio_driver.h"
#include "esp_err.h"

typedef enum { BUTTON_EVENT_NONE, BUTTON_EVENT_CLICK, BUTTON_EVENT_LONG_PRESS } button_event_t;
typedef struct { bsw_gpio_num_t pin; uint32_t press_start_time; bool is_pressed; bool long_press_handled; } button_t;

esp_err_t button_init(button_t* btn, bsw_gpio_num_t pin);
button_event_t button_get_event(button_t* btn);
#endif
