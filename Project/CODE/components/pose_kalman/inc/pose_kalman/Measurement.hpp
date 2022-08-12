#ifndef _pose_kalman_Measurement_hpp
#define _pose_kalman_Measurement_hpp

#include "pose_kalman/KFType.hpp"
#include "pose_kalman/State.hpp"
#include "pose_kalman/config.hpp"

namespace pose_kalman {

struct DefaultMeasurementNoiseJacobianUpdater {
    template <typename NoiseJacobianType> void operator()(const State& x, NoiseJacobianType& V) const {
        (void)x;
        (void)V;
    }
};
// Just for example. Don't use.
struct VelocityBasedMeasurementNoiseJacobianUpdater {
    template <typename NoiseJacobianType> void operator()(const State& x, NoiseJacobianType& V) const {
        V.diagonal().setConstant(x.velocityNorm());
    }
};

template <bool _x, bool _y, bool _yaw, bool _vX, bool _vY, bool _vYaw,
          typename NoiseJacobianUpdater = DefaultMeasurementNoiseJacobianUpdater>
struct Measurement {
    static constexpr bool X = _x, Y = _y, YAW = _yaw, V_X = _vX, V_Y = _vY, V_YAW = _vYaw;
    static constexpr int SIZE = _x + _y + _yaw + _vX + _vY + _vYaw;
    class Data : public Kalman::Vector<T, SIZE> {
     public:
        KALMAN_VECTOR(Data, T, SIZE)

        //! Position
        static constexpr size_t X = 0;
        static constexpr size_t Y = X + _x;
        static constexpr size_t YAW = Y + _y;

        //! Velocity
        static constexpr size_t V_X = YAW + _yaw;
        static constexpr size_t V_Y = V_X + _vX;
        static constexpr size_t V_YAW = V_Y + _vY;

        T x() const {
            static_assert(_x, "x is not enabled");
            return (*this)[X];
        }
        T y() const {
            static_assert(_y, "y is not enabled");
            return (*this)[Y];
        }
        T yaw() const {
            static_assert(_yaw, "yaw is not enabled");
            return (*this)[YAW];
        }
        T vX() const {
            static_assert(_vX, "vX is not enabled");
            return (*this)[V_X];
        }
        T vY() const {
            static_assert(_vY, "vY is not enabled");
            return (*this)[V_Y];
        }
        T vYaw() const {
            static_assert(_vYaw, "vYaw is not enabled");
            return (*this)[V_YAW];
        }

        T& x() {
            static_assert(_x, "x is not enabled");
            return (*this)[X];
        }
        T& y() {
            static_assert(_y, "y is not enabled");
            return (*this)[Y];
        }
        T& yaw() {
            static_assert(_yaw, "yaw is not enabled");
            return (*this)[YAW];
        }
        T& vX() {
            static_assert(_vX, "vX is not enabled");
            return (*this)[V_X];
        }
        T& vY() {
            static_assert(_vY, "vY is not enabled");
            return (*this)[V_Y];
        }
        T& vYaw() {
            static_assert(_vYaw, "vYaw is not enabled");
            return (*this)[V_YAW];
        }
    };

    using NoiseJacobian = Kalman::Jacobian<Data, Data>;

    class Model : public Kalman::LinearizedMeasurementModel<State, Data, pose_kalman_COVBASE> {
     public:
        //! State type shortcut definition
        typedef State S;
        //! Measurement type shortcut definition
        typedef Data M;
        Model() {
            this->H.setZero();
            if constexpr (_x) this->H(M::X, S::X) = 1;
            if constexpr (_y) this->H(M::Y, S::Y) = 1;
            if constexpr (_yaw) this->H(M::YAW, S::YAW) = 1;
            if constexpr (_vX) this->H(M::V_X, S::V_X) = 1;
            if constexpr (_vY) this->H(M::V_Y, S::V_Y) = 1;
            if constexpr (_vYaw) this->H(M::V_YAW, S::V_YAW) = 1;
            this->V.setIdentity();
        }

        M h(const S& x) const {
            M measurement;
            if constexpr (_x) measurement.x() = x.x();
            if constexpr (_y) measurement.y() = x.y();
            if constexpr (_yaw) measurement.yaw() = x.yaw();
            if constexpr (_vX) measurement.vX() = x.vX();
            if constexpr (_vY) measurement.vY() = x.vY();
            if constexpr (_vYaw) measurement.vYaw() = x.vYaw();
            return measurement;
        }

        void updateJacobians(const State& x) { NoiseJacobianUpdater()(x, this->V); }
    };
};
}  // namespace pose_kalman

#endif  // _pose_kalman_Measurement_hpp