#ifndef _pose_kalman_IMUMeasurementModel_HPP_
#define _pose_kalman_IMUMeasurementModel_HPP_

#include "SystemModel.hpp"
#include "kalman/LinearizedMeasurementModel.hpp"
namespace Planar_Robot {
template <typename T> class IMUMeasurement : public Kalman::Vector<T, 5> {
 public:
    KALMAN_VECTOR(IMUMeasurement, T, 5)

    //! Orientation
    static constexpr size_t OX = 0;
    static constexpr size_t OY = 1;

    //! Velocity
    static constexpr size_t V_YAW = 2;

    //! Acceleration
    static constexpr size_t A_X = 3;
    static constexpr size_t A_Y = 4;

    T ox() const { return (*this)[OX]; }
    T oy() const { return (*this)[OY]; }
    T vYaw() const { return (*this)[V_YAW]; }
    T aX() const { return (*this)[A_X]; }
    T aY() const { return (*this)[A_Y]; }

    T& ox() { return (*this)[OX]; }
    T& oy() { return (*this)[OY]; }
    T& vYaw() { return (*this)[V_YAW]; }
    T& aX() { return (*this)[A_X]; }
    T& aY() { return (*this)[A_Y]; }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class IMUMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, IMUMeasurement<T>, CovarianceBase> {
 public:
    typedef Planar_Robot::State<T> S;
    typedef Planar_Robot::IMUMeasurement<T> M;
    IMUMeasurementModel() {
        // Setup noise jacobian
        this->H.setIdentity();
        this->V.setIdentity();
    }

    M h(const S& x) const {
        using std::sin, std::cos;
        M measurement;

        measurement.aX() = x.aX() * x.ox() + x.aY() * x.oy();
        measurement.aY() = x.aY() * x.ox() - x.aX() * x.oy();

        measurement.ox() = x.ox();
        measurement.oy() = x.oy();
        measurement.vYaw() = x.vYaw();

        return measurement;
    }

 protected:
    void UpdateJacobians(const S& s) {
        // H = dh/dx (Jacobian of measurement function w.r.t the state)
        this->H.setIdentity();
        // TODO:
        // make a more partial derivative of x.x() w.r.t x.x()
        // this->H(S::X, S::X) = 1;
        // partial derivative of x.x() w.r.t x.theta()
        // this->H(S::X, S::THETA) = 0;
        // partial derivative of x.y() w.r.t x.y()
        // this->H(S::Y, S::Y) = 1;
        // partial derivative of x.y() w.r.t. x.theta()
        // this->H(S::Y, S::THETA) = 0;
        // partia derivative of x.theta() w.r.t. x.theta()
        // this->H(S::THETA, S::THETA) = 1;
    }
};

}  // namespace Planar_Robot
#endif  // _pose_kalman_IMUMeasurementModel_HPP_