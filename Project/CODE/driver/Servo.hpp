#ifndef _driver_Servo_hpp
#define _driver_Servo_hpp

extern "C" {
#include "zf_pwm.h"
}

class Servo {
 private:
    const PWMCH_enum pwm;
    const uint32_t freq;
    const uint32_t min_width, max_width;
    uint32_t min_duty, max_duty;
    uint32_t width_to_duty(float width) const { return width * freq * PWM_DUTY_MAX * 1e-6f; }
    float angle_to_width(float angle) const { return angle * (max_width - min_width) / max_angle + min_width; }
    uint32_t angle_to_duty(float angle) const { return width_to_duty(angle_to_width(angle)); }

 public:
    const uint32_t max_angle;
    Servo(PWMCH_enum pwm, uint32_t freq, uint32_t min_width, uint32_t max_width, uint32_t max_angle, float min_lim, float max_lim)
        : pwm(pwm), freq(freq), min_width(min_width), max_width(max_width), max_angle(max_angle) {
        min_duty = angle_to_duty(min_lim);
        max_duty = angle_to_duty(max_lim);
    }
    void max() const { pwm_duty(pwm, max_duty); }
    void min() const { pwm_duty(pwm, min_duty); }
    void init(uint32_t angle) const { pwm_init(pwm, freq, angle_to_duty(angle)); }
    void init_max() const { pwm_init(pwm, freq, max_duty); }
    void init_min() const { pwm_init(pwm, freq, min_duty); }
    void set(uint32_t angle) const { pwm_duty(pwm, angle_to_duty(angle)); }
};

#endif