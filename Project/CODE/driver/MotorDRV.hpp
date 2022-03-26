#ifndef _MotorDRV_hpp
#define _MotorDRV_hpp

#include <limits>

extern "C" {
#include "zf_gpio.h"
#include "zf_pwm.h"
}

class MotorDRV {
 private:
    const PIN_enum DIR_pin;
    const PWMCH_enum PWM_ch;
    const bool invert;
    int32 _deadzone;

 public:
    MotorDRV(PIN_enum dir_pin, PWMCH_enum pwm_ch, bool inverted, int32 deadzone = 0)
        : DIR_pin(dir_pin), PWM_ch(pwm_ch), invert(inverted), _deadzone(deadzone) {}
    void init() const {
        gpio_init(DIR_pin, GPO, 0, GPIO_PIN_CONFIG);
        pwm_init(PWM_ch, 17000, 0);
    }

    template <typename T> inline T limit(T duty, bool use_deadzone = true) const {
        T bound = use_deadzone ? PWM_DUTY_MAX - _deadzone : PWM_DUTY_MAX;
        if (duty > bound) return bound;
        if constexpr (std::numeric_limits<T>::is_signed)
            if (duty < -bound) return -bound;
        return duty;
    }

    inline void setPWM(int32 duty, bool use_deadzone = true) const {
        bool negative = duty < 0;
        if (negative) duty = -duty;
        if (use_deadzone && duty > Param::Motor::deadzone_thresh) duty += _deadzone;
        pwm_duty(PWM_ch, duty);
        gpio_set(DIR_pin, negative ^ invert);
    }

    template <typename T> inline T setPWM_Limit(T duty, bool use_deadzone = true) const {
        duty = limit(duty, use_deadzone);
        setPWM(duty, use_deadzone);
        return duty;
    }

    inline void setDeadzone(int32 deadzone) { _deadzone = deadzone; }
};

#endif  // _MotorDRV_hpp