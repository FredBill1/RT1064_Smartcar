#ifndef _pose_kalman_State_hpp
#define _pose_kalman_State_hpp

#include "kalman/Matrix.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {
class State : public Kalman::Vector<T, 7> {
 public:
    KALMAN_VECTOR(State, T, 7)
    //! Position
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;

    //! Orientation
    static constexpr size_t OX = 2;
    static constexpr size_t OY = 3;

    //! Velocity
    static constexpr size_t V_X = 4;
    static constexpr size_t V_Y = 5;
    static constexpr size_t V_YAW = 6;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T ox() const { return (*this)[OX]; }
    T oy() const { return (*this)[OY]; }
    T vX() const { return (*this)[V_X]; }
    T vY() const { return (*this)[V_Y]; }
    T vYaw() const { return (*this)[V_YAW]; }

    T& x() { return (*this)[X]; }
    T& y() { return (*this)[Y]; }
    T& ox() { return (*this)[OX]; }
    T& oy() { return (*this)[OY]; }
    T& vX() { return (*this)[V_X]; }
    T& vY() { return (*this)[V_Y]; }
    T& vYaw() { return (*this)[V_YAW]; }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_State_hpp