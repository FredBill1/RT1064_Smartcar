#ifndef _MotorDRV_hpp
#define _MotorDRV_hpp

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
    MotorDRV(PIN_enum dir_pin, PWMCH_enum pwm_ch, bool inverted);
    void init();

    /**
     * @brief 给电机设置PWM脉宽
     *
     * @param duty 要设置的pwm值, 如果是负数则会反向; 绝对值最大为50000(fsl_pwm.h -> PWM_DUTY_MAX)
     */
    void setPWM(int32 duty);
};

#endif  // _MotorDRV_hpp