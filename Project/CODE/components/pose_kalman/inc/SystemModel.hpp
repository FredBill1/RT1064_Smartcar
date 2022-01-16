#ifndef _pose_kalman_SystemModel_hpp
#define _pose_kalman_SystemModel_hpp

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "kalman/LinearizedMeasurementModel.hpp"
#include "kalman/LinearizedSystemModel.hpp"
namespace Planar_Robot {
template <typename T> class State : public Kalman::Vector<T, 9> {
 public:
    KALMAN_VECTOR(State, T, 9)
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

    //! Acceleration
    static constexpr size_t A_X = 7;
    static constexpr size_t A_Y = 8;

    T x() const { return (*this)[X]; }
    T y() const { return (*this)[Y]; }
    T ox() const { return (*this)[OX]; }
    T oy() const { return (*this)[OY]; }
    T vX() const { return (*this)[V_X]; }
    T vY() const { return (*this)[V_Y]; }
    T vYaw() const { return (*this)[V_YAW]; }
    T aX() const { return (*this)[A_X]; }
    T aY() const { return (*this)[A_Y]; }

    T& x() { return (*this)[X]; }
    T& y() { return (*this)[Y]; }
    T& ox() { return (*this)[OX]; }
    T& oy() { return (*this)[OY]; }
    T& vX() { return (*this)[V_X]; }
    T& vY() { return (*this)[V_Y]; }
    T& vYaw() { return (*this)[V_YAW]; }
    T& aX() { return (*this)[A_X]; }
    T& aY() { return (*this)[A_Y]; }
};

template <typename T> class Control : public Kalman::Vector<T, 4> {
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

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Planar_Robot::State<T> S;
    typedef Planar_Robot::Control<T> C;
    // Definition of (non-linear) state transition function
    S f(const S& x, const C& u) const {
        using std::sin, std::cos, std::atan2;
        S res;

        res.vYaw() = u.vYaw();
        T yaw = atan2(x.oy(), x.ox());
        T res_yaw = yaw + (x.vYaw() + u.vYaw()) / 2 * u.dt();
        res.ox() = cos(res_yaw);
        res.oy() = sin(res_yaw);

        res.vX() = u.vX() * cos(res_yaw) - u.vY() * sin(res_yaw);
        res.vY() = u.vY() * cos(res_yaw) + u.vX() * sin(res_yaw);

        res.x() = x.x() + (x.vX() + res.vX()) / 2 * u.dt();
        res.y() = x.y() + (x.vY() + res.vY()) / 2 * u.dt();

        res.aX() = (res.vX() - x.vX()) / u.dt();
        res.aY() = (res.vY() - x.vY()) / u.dt();

        return res;
    }

 protected:
    // Update jacobian matrices for the system state transition function using current state
    void updataJacbians(const S& x, const C& u) {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setIdentity();
        // this->F.setZero();
        // S x_ = this->f(x, u);
        // T DT_2 = u.dt() / 2;
        // T DTY = -DT_2 * x_.vY();
        // T DTX = DT_2 * x_.vX();
        // T DTTY = DTY * DT_2;
        // T DTTX = DTX * DT_2;
        // // clang-format off
        // this->F << 1, 0, DTY,      DT_2, 0,    DTTY,
        //            0, 1, DTX,      0,    DT_2, DTTX,
        //            0, 0, 1,        0,    0,    DT_2,
        //            0, 0, -x_.vY(), 0,    0,    DTY,
        //            0, 0, x_.vX(),  0,    0,    DTX,
        //            0, 0, 0,        0,    0,    0;
        // // clang-format on

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        // this->W.setZero();
        this->W.setIdentity();
        // float alpha1 = 0.02;
        // float alpha2 = 0.02;
        // float alpha3 = 0.02;
        // float alpha4 = 0.02;
        // Eigen::Matrix<float, 3, 2> V;
        // V(0, 0) = (-std::sin(x.theta()) + std::sin(x.theta() + u.dtheta()*0.01));
        // V(0, 1) = (u.v() * (std::sin(x.theta()) - std::sin(x.theta() + u.dtheta()*0.01)))  +
        //           (u.v() * (std::cos(x.theta() + u.dtheta()*0.01)))*0.01;
        // V(1, 1) = -(u.v() * (std::cos(x.theta()) - std::cos(x.theta() + u.dtheta()*0.01)))  +
        //           (u.v() * (std::sin(x.theta() + u.dtheta()*0.01)))*0.01;
        // V(1, 0) = (std::cos(x.theta()) - std::cos(x.theta() + u.dtheta()*0.01)) ;
        // V(1, 1) = 0.01;

        // Eigen::Matrix2f M;
        // M(0,0) = alpha1 * u.v_x() * u.v_y() + alpha2 * u.dtheta() * u.dtheta();
        // M(0,1) = 0.0;
        // M(1,0) = 0.0;
        // M(1,1) = alpha3 * u.v_x() * u.v_y() + alpha4 * u.dtheta() * u.dtheta();
        // this->W.setZero();
        // this->W = this->W + V * M * V.transpose();
    }
};
}  // namespace Planar_Robot

#endif  //_pose_kalman_SystemModel_hpp