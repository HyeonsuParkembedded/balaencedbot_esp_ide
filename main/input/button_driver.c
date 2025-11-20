#include "button_driver.h"
#include "../bsw/system_services.h"
#define LONG_PRESS_MS 1000
#define DEBOUNCE_MS 50

esp_err_t button_init(button_t* btn, bsw_gpio_num_t pin) {
    btn->pin = pin; btn->press_start_time = 0; btn->is_pressed = false; btn->long_press_handled = false;
    return bsw_gpio_config_pin(pin, BSW_GPIO_MODE_INPUT, BSW_GPIO_PULLUP_ENABLE, BSW_GPIO_PULLDOWN_DISABLE);
}

button_event_t button_get_event(button_t* btn) {
    int level = bsw_gpio_get_level(btn->pin);
    uint32_t now = bsw_get_time_ms();
    if (level == 0) { // Pressed
        if (!btn->is_pressed) { btn->is_pressed = true; btn->press_start_time = now; btn->long_press_handled = false; }
        else if (!btn->long_press_handled && (now - btn->press_start_time > LONG_PRESS_MS)) {
            btn->long_press_handled = true; return BUTTON_EVENT_LONG_PRESS;
        }
    } else { // Released
        if (btn->is_pressed) {
            btn->is_pressed = false;
            if (!btn->long_press_handled && (now - btn->press_start_time > DEBOUNCE_MS)) return BUTTON_EVENT_CLICK;
        }
    }
    return BUTTON_EVENT_NONE;
}
