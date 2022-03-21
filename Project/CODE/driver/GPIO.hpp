#ifndef _GPIO_hpp
#define _GPIO_hpp

#include <cstdint>

extern "C" {
#include "zf_gpio.h"
}

class GPIO {
    const PIN_enum _pin;

 public:
    GPIO(PIN_enum pin) : _pin(pin) {}
    void init(bool is_input, bool initial_output = false) {
        gpio_init(_pin, is_input ? GPI : GPO, initial_output, GPIO_PIN_CONFIG);
    }
    void setMode(bool is_input) { gpio_dir(_pin, is_input ? GPI : GPO); }

    // GPI

    bool get() { return gpio_get(_pin); }
    bool pressing() { return !get(); }

    // GPO

    void set(bool dat) { gpio_set(_pin, dat); }
    void toggle() { gpio_toggle(_pin); }
};

#endif  // _GPIO_hpp