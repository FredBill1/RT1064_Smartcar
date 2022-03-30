#ifndef _parameters_hpp
#define _parameters_hpp

#include <cstdint>

namespace Param {

constexpr int32_t MotorControlPeriod = 5;  // 电机控制的周期, 单位是tick, 相当于ms

namespace Motor {

constexpr int32_t deadzone_thresh = 200;  // 输出pwm大于此值时把死区pwm加上去
#define MOTOR_ADRC_USE_WC 1               // 是否使用控制器带宽wc而非kp kd作为参数

namespace L1 {
constexpr float kp = 2500;
constexpr float kd = 100;

constexpr float wc = 50;
constexpr float wo = 150;
constexpr float b0 = 5;
constexpr int32_t deadzone = 2700;
}  // namespace L1

namespace L2 {
constexpr float kp = 2500;
constexpr float kd = 100;

constexpr float wc = 50;
constexpr float wo = 150;
constexpr float b0 = 5;
constexpr int32_t deadzone = 3000;
}  // namespace L2

namespace R1 {
constexpr float kp = 2500;
constexpr float kd = 100;

constexpr float wc = 50;
constexpr float wo = 150;
constexpr float b0 = 5;
constexpr int32_t deadzone = 2850;
}  // namespace R1

namespace R2 {
constexpr float kp = 2500;
constexpr float kd = 100;

constexpr float wc = 50;
constexpr float wo = 150;
constexpr float b0 = 5;
constexpr int32_t deadzone = 3550;
}  // namespace R2

}  // namespace Motor

}  // namespace Param

#endif  // _parameters_hpp