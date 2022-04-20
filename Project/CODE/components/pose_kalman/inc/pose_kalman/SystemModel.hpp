#ifndef _pose_kalman_SystemModel_hpp
#define _pose_kalman_SystemModel_hpp

// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include "apriltag/fmath.hpp"
#include "kalman/LinearizedSystemModel.hpp"
#include "kalman/SquareRootBase.hpp"

//
#include "pose_kalman/Control.hpp"
#include "pose_kalman/State.hpp"
#include "pose_kalman/config.hpp"
namespace pose_kalman {

class SystemModel : public Kalman::LinearizedSystemModel<State, Control, Kalman::SquareRootBase> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pose_kalman::State S;
    typedef pose_kalman::Control C;
    // Definition of (non-linear) state transition function
    SystemModel() {
        this->F.setZero();
        this->F(S::X, S::X) = this->F(S::Y, S::Y) = 1;

        this->W.setIdentity();
    }
    S f(const S& x, const C& u) const {
        using imgProc::apriltag::sinf, imgProc::apriltag::cosf, imgProc::apriltag::atan2f;
        // using std::sin, std::cos, std::atan2;
        S res;

        // Orientation
        T pre_yaw = atan2f(x.oy(), x.ox());
        T cur_yaw = pre_yaw + u.vYaw() * u.dt();
        res.ox() = cosf(cur_yaw);
        res.oy() = sinf(cur_yaw);

        // Velocity
        res.vX() = u.vX() * res.ox() - u.vY() * res.oy();
        res.vY() = u.vX() * res.oy() + u.vY() * res.ox();
        res.vYaw() = u.vYaw();

        // Position
        res.x() = x.x() + res.vX() * u.dt();
        res.y() = x.y() + res.vY() * u.dt();

        return res;
    }

    void updateJacobians(const S& x, const C& u) {
        using imgProc::apriltag::sinf, imgProc::apriltag::cosf, imgProc::apriltag::atan2f;
        // using std::sin, std::cos, std::atan2;

        // F = df/dx (Jacobian of state transition w.r.t. the state)

        T pre_yaw = atan2f(x.oy(), x.ox());
        T cur_yaw = pre_yaw + u.vYaw() * u.dt();
        T res_ox = cosf(cur_yaw);
        T res_oy = sinf(cur_yaw);
        T vXox = u.vX() * res_ox;
        T vXoy = u.vX() * res_oy;
        T vYox = u.vY() * res_ox;
        T vYoy = u.vY() * res_oy;
        T vXoxvYoy = vXox - vYoy;
        T vXoyvYox = vXoy + vYox;

        // this->F(S::X, S::OX) = x.oy() * vXoyvYox * u.dt();
        // this->F(S::X, S::OY) = -x.ox() * vXoyvYox * u.dt();

        // this->F(S::Y, S::OX) = -x.oy() * vXoxvYoy * u.dt();
        // this->F(S::Y, S::OY) = x.ox() * vXoxvYoy * u.dt();

        this->F(S::OX, S::OX) = x.oy() * res_oy;
        this->F(S::OX, S::OY) = -x.ox() * res_oy;

        this->F(S::OY, S::OX) = -x.oy() * res_ox;
        this->F(S::OY, S::OY) = x.ox() * res_ox;

        this->F(S::V_X, S::OX) = x.oy() * vXoyvYox;
        this->F(S::V_X, S::OY) = -x.ox() * vXoyvYox;

        this->F(S::V_Y, S::OX) = -x.oy() * vXoxvYoy;
        this->F(S::V_Y, S::OY) = x.ox() * vXoxvYoy;

        this->F(S::X, S::OX) = this->F(S::V_X, S::OX) * u.dt();
        this->F(S::X, S::OY) = this->F(S::V_X, S::OY) * u.dt();

        this->F(S::Y, S::OX) = this->F(S::V_Y, S::OX) * u.dt();
        this->F(S::Y, S::OY) = this->F(S::V_Y, S::OY) * u.dt();
    }
};
}  // namespace pose_kalman

#endif  //_pose_kalman_SystemModel_hpp