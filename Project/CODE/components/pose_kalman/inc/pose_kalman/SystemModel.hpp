#ifndef _pose_kalman_SystemModel_hpp
#define _pose_kalman_SystemModel_hpp

// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include "kalman/LinearizedSystemModel.hpp"
#include "kalman/SquareRootBase.hpp"

//
#include "pose_kalman/Control.hpp"
#include "pose_kalman/State.hpp"
#include "pose_kalman/config.hpp"
#include "pose_kalman/utils.hpp"
namespace pose_kalman {

class SystemModel : public Kalman::LinearizedSystemModel<State, Control, Kalman::SquareRootBase> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef pose_kalman::State S;
    typedef pose_kalman::Control C;
    // Definition of (non-linear) state transition function
    SystemModel() {
        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F.setIdentity();

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
    }
    S f(const S& x, const C& u) const {
        using std::sin, std::cos;
        T dt = u.dt();
        T sy = sin(x.yaw()), cy = cos(x.yaw());

        S res;

        // Velocity
        res.vX() = x.vX();
        res.vY() = x.vY();
        res.vYaw() = x.vYaw();

        // Position
        res.x() = x.x() + (cy * x.vX() - sy * x.vY()) * dt;
        res.y() = x.y() + (sy * x.vX() + cy * x.vY()) * dt;
        res.yaw() = wrapAngle(x.yaw() + x.vYaw() * dt);

        return res;
    }

    void updateJacobians(const S& x, const C& u) {
        using std::sin, std::cos, std::atan2, std::sqrt;
        T dt = u.dt();
        T sy = sin(x.yaw()), cy = cos(x.yaw());
        T vX = x.vX();  // + aX * dt / 2
        T vY = x.vY();  // + aY * dt / 2

        // F = df/dx (Jacobian of state transition w.r.t. the state)
        this->F(S::X, S::V_X) = cy * dt;
        this->F(S::X, S::V_Y) = -sy * dt;
        this->F(S::Y, S::V_X) = sy * dt;
        this->F(S::Y, S::V_Y) = cy * dt;
        this->F(S::YAW, S::V_YAW) = dt;
        this->F(S::X, S::YAW) = (-sy * vX - cy * vY) * dt * 0.5;
        this->F(S::Y, S::YAW) = (cy * vX - sy * vY) * dt * 0.5;

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        // relative to dt
        this->W.diagonal().setConstant(sqrt(dt));
        if constexpr (useDynamicProcessNoiseCovariance)
            this->W.diagonal().segment(S::VELOCITY_POS, S::VELOCITY_SIZE) *= x.velocityNorm();
    }
};
}  // namespace pose_kalman

#endif  //_pose_kalman_SystemModel_hpp