#include "MotorDRV.hpp"

#include <algorithm>

MotorDRV::MotorDRV(PIN_enum dir_pin, PWMCH_enum pwm_ch, bool inverted) : DIR_pin(dir_pin), PWM_ch(pwm_ch), invert(inverted) {}
void MotorDRV::init() {
    gpio_init(DIR_pin, GPO, 0, GPIO_PIN_CONFIG);
    pwm_init(PWM_ch, 17000, 0);
}
void MotorDRV::setPWM(int32 duty) {
    pwm_duty(PWM_ch, std::abs(duty));
    gpio_set(DIR_pin, (duty < 0) ^ invert);
}