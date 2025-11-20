#include "rgb_led.h"
static bsw_gpio_num_t pr, pg, pb;
void rgb_led_init(bsw_gpio_num_t r, bsw_gpio_num_t g, bsw_gpio_num_t b) {
    pr=r; pg=g; pb=b;
    bsw_gpio_config_pin(r, BSW_GPIO_MODE_OUTPUT, 0, 0);
    bsw_gpio_config_pin(g, BSW_GPIO_MODE_OUTPUT, 0, 0);
    bsw_gpio_config_pin(b, BSW_GPIO_MODE_OUTPUT, 0, 0);
    rgb_led_set_color(LED_COLOR_OFF);
}
void rgb_led_set_color(led_color_t c) {
    int r=0, g=0, b=0;
    switch(c) {
        case LED_COLOR_RED: r=1; break; case LED_COLOR_GREEN: g=1; break;
        case LED_COLOR_BLUE: b=1; break; case LED_COLOR_YELLOW: r=1; g=1; break;
        default: break;
    }
    bsw_gpio_set_level(pr, r); bsw_gpio_set_level(pg, g); bsw_gpio_set_level(pb, b);
}
