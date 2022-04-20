#ifndef _pose_kalman_Control_hpp
#define _pose_kalman_Control_hpp

#include "kalman/Matrix.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {
class Control : public Kalman::Vector<T, 4> {
 public:
    KALMAN_VECTOR(Control, T, 4)
    //! Velocity
    static constexpr size_t V_X = 0;
    static constexpr size_t V_Y = 1;
    static constexpr size_t V_YAW = 2;

    //! Time
    static constexpr size_t DT = 3;

    T vX() const { return (*this)[V_X]; }
    T vY() const { return (*this)[V_Y]; }
    T vYaw() const { return (*this)[V_YAW]; }
    T dt() const { return (*this)[DT]; }

    T& vX() { return (*this)[V_X]; }
    T& vY() { return (*this)[V_Y]; }
    T& vYaw() { return (*this)[V_YAW]; }
    T& dt() { return (*this)[DT]; }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_Control_hpp