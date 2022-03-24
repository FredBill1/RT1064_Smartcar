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

    /**
     * @brief 给电机设置PWM脉宽
     *
     * @param duty 要设置的pwm值, 如果是负数则会反向; 绝对值最大为50000(fsl_pwm.h -> PWM_DUTY_MAX)
     */
    template <typename T> inline T setPWM(T duty) const {
        bool negative, dir;
        if constexpr (!std::numeric_limits<T>::is_signed) {
            negative = false, dir = invert;
            if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
        } else {
            negative = duty < 0, dir = negative ^ invert;
            if constexpr (std::numeric_limits<T>::is_integer) {
                if (duty < -PWM_DUTY_MAX || duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
                if (duty < 0) duty = -duty;
            } else {
                if (negative) duty = -duty;
                if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
            }
        }
        pwm_duty(PWM_ch, duty);
        gpio_set(DIR_pin, dir);
        return negative ? -duty : duty;
    }
};

#endif  // _MotorDRV_hpp