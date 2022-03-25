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

 public:
    MotorDRV(PIN_enum dir_pin, PWMCH_enum pwm_ch, bool inverted) : DIR_pin(dir_pin), PWM_ch(pwm_ch), invert(inverted) {}
    void init() const {
        gpio_init(DIR_pin, GPO, 0, GPIO_PIN_CONFIG);
        pwm_init(PWM_ch, 17000, 0);
    }

    template <typename T> static constexpr T limit(T duty) {
        if (duty > PWM_DUTY_MAX) return PWM_DUTY_MAX;
        if constexpr (std::numeric_limits<T>::is_signed)
            if (duty < -PWM_DUTY_MAX) return -PWM_DUTY_MAX;
        return duty;
    }

    inline void setPWM(int32 duty) const {
        bool negative = duty < 0;
        if (negative) duty = -duty;
        pwm_duty(PWM_ch, duty);
        gpio_set(DIR_pin, negative ^ invert);
    }

    template <typename T> inline T setPWM_Limit(T duty) const {
        T duty_limit = limit(duty);
        setPWM(duty_limit);
        return duty_limit;
    }
};

#endif  // _MotorDRV_hpp