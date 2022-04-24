#ifndef _pose_kalman_utils_hpp
#define _pose_kalman_utils_hpp

#include "pose_kalman/config.hpp"

namespace pose_kalman {

constexpr long double PI_ = 3.141592653589793238462643383279502884197169399375105820974944592307816406L;
constexpr T PI = PI_;
constexpr T PI2 = PI_ * 2;
constexpr T PI_2 = PI_ / 2;
constexpr T PI_4 = PI_ / 4;

constexpr T wrapAngle(T x) {
    while (x > PI) x -= PI2;
    while (x < -PI) x += PI2;
    return x;
}

}  // namespace pose_kalman

#endif  // _pose_kalman_utils_hpp