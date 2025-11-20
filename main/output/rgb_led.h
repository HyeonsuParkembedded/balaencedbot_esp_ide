#ifndef RGB_LED_H
#define RGB_LED_H
#include "../bsw/gpio_driver.h"
typedef enum { LED_COLOR_OFF, LED_COLOR_RED, LED_COLOR_GREEN, LED_COLOR_BLUE, LED_COLOR_YELLOW } led_color_t;
void rgb_led_init(bsw_gpio_num_t r, bsw_gpio_num_t g, bsw_gpio_num_t b);
void rgb_led_set_color(led_color_t color);
#endif
