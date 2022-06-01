#ifndef _pose_kalman_State_hpp
#define _pose_kalman_State_hpp

#include "kalman/Matrix.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {
class State : public Kalman::Vector<T, 6> {
 public:
    KALMAN_VECTOR(State, T, 6)
    //! Position
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t YAW = 2;

    //! Velocity
    static constexpr size_t V_X = 3;
    static constexpr size_t V_Y = 4;
    static constexpr size_t V_YAW = 5;

    static constexpr size_t VELOCITY_POS = V_X;
    static constexpr size_t VELOCITY_SIZE = V_YAW - V_X + 1;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T yaw() const { return (*this)[YAW]; }
    T vX() const { return (*this)[V_X]; }
    T vY() const { return (*this)[V_Y]; }
    T vYaw() const { return (*this)[V_YAW]; }

    T& x() { return (*this)[X]; }
    T& y() { return (*this)[Y]; }
    T& yaw() { return (*this)[YAW]; }
    T& vX() { return (*this)[V_X]; }
    T& vY() { return (*this)[V_Y]; }
    T& vYaw() { return (*this)[V_YAW]; }

    T velocityNorm() const { return segment(VELOCITY_POS, VELOCITY_SIZE).norm(); }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_State_hpp