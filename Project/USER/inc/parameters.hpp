#ifndef _parameters_hpp
#define _parameters_hpp

#include <cstdint>

namespace Param {

constexpr int32_t MotorControlPeriod = 1;  // 电机控制的周期, 单位是tick, 相当于ms

namespace Motor {

constexpr int32_t deadzone_thresh = 200;  // 输出pwm大于此值时把死区pwm加上去

namespace L1 {
constexpr float wc = 20;
constexpr float wo = 40;
constexpr float b0 = 0.004;
constexpr int32_t deadzone = 2000;
}  // namespace L1

namespace L2 {
constexpr float wc = 20;
constexpr float wo = 40;
constexpr float b0 = 0.004;
constexpr int32_t deadzone = 2000;
}  // namespace L2

namespace R1 {
constexpr float wc = 20;
constexpr float wo = 40;
constexpr float b0 = 0.004;
constexpr int32_t deadzone = 2000;
}  // namespace R1

namespace R2 {
constexpr float wc = 20;
constexpr float wo = 40;
constexpr float b0 = 0.004;
constexpr int32_t deadzone = 2000;
}  // namespace R2

}  // namespace Motor

}  // namespace Param

#endif  // _parameters_hpp