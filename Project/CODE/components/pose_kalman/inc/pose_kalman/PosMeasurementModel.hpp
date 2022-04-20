#ifndef _pose_kalman_PosMeasurementModel_hpp
#define _pose_kalman_PosMeasurementModel_hpp

#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/SquareRootBase.hpp"

//
#include "pose_kalman/State.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {
class PosMeasurement : public Kalman::Vector<T, 2> {
 public:
    KALMAN_VECTOR(PosMeasurement, T, 2)

    //! Orientation
    static constexpr size_t OX = 0;
    static constexpr size_t OY = 1;

    T ox() const { return (*this)[OX]; }
    T oy() const { return (*this)[OY]; }

    T& ox() { return (*this)[OX]; }
    T& oy() { return (*this)[OY]; }
};

class PosMeasurementModel : public Kalman::LinearizedMeasurementModel<State, PosMeasurement, Kalman::SquareRootBase> {
 public:
    //! State type shortcut definition
    typedef State S;
    //! Measurement type shortcut definition
    typedef PosMeasurement M;
    M h(const S& x) const {
        M measurement;
        measurement.ox() = x.ox();
        measurement.oy() = x.oy();
        return measurement;
    }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_PosMeasurementModel_hpp