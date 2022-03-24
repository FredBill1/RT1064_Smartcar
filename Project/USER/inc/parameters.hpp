#ifndef _parameters_hpp
#define _parameters_hpp

#include <cstdint>

namespace Param {

constexpr int32_t MotorControlPeriod = 5;  // 电机控制的周期, 单位是tick, 相当于ms

namespace LADRC2 {
namespace L1 {
constexpr float kp = 0.5;
constexpr float kd = 0.1;
constexpr float wo = 0.5;
constexpr float b0 = 0.5;
}  // namespace L1

namespace L2 {
constexpr float kp = 0.5;
constexpr float kd = 0.1;
constexpr float wo = 0.5;
constexpr float b0 = 0.5;
}  // namespace L2

namespace R1 {
constexpr float kp = 0.5;
constexpr float kd = 0.1;
constexpr float wo = 0.5;
constexpr float b0 = 0.5;
}  // namespace R1

namespace R2 {
constexpr float kp = 0.5;
constexpr float kd = 0.1;
constexpr float wo = 0.5;
constexpr float b0 = 0.5;
}  // namespace R2

}  // namespace LADRC2

}  // namespace Param

#endif  // _parameters_hpp