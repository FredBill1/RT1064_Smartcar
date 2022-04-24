#ifndef _pose_kalman_Control_hpp
#define _pose_kalman_Control_hpp

#include "kalman/Matrix.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {
class Control : public Kalman::Vector<T, 1> {
 public:
    KALMAN_VECTOR(Control, T, 1)
    //! Time
    static constexpr size_t DT = 0;

    T dt() const { return (*this)[DT]; }

    T& dt() { return (*this)[DT]; }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_Control_hpp