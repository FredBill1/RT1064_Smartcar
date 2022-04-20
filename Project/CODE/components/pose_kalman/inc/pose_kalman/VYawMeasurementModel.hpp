#ifndef _pose_kalman_VYawMeasurementModel_hpp
#define _pose_kalman_VYawMeasurementModel_hpp

#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/SquareRootBase.hpp"

//
#include "pose_kalman/State.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {
class VYawMeasurement : public Kalman::Vector<T, 1> {
 public:
    KALMAN_VECTOR(VYawMeasurement, T, 1)

    //! Velocity
    static constexpr size_t V_YAW = 0;

    T vYaw() const { return (*this)[V_YAW]; }
    T& vYaw() { return (*this)[V_YAW]; }
};

class VYawMeasurementModel : public Kalman::LinearizedMeasurementModel<State, VYawMeasurement, Kalman::SquareRootBase> {
 public:
    //! State type shortcut definition
    typedef State S;
    //! Measurement type shortcut definition
    typedef VYawMeasurement M;
    VYawMeasurementModel() {
        H.setZero();
        H(M::V_YAW, S::V_YAW) = 1;
        V.setIdentity();
    }

    M h(const S& x) const {
        M measurement;
        measurement.vYaw() = x.vYaw();
        return measurement;
    }
};
}  // namespace pose_kalman

#endif  // _pose_kalman_VYawMeasurementModel_hpp