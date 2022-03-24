// https://github.com/psiorx/ADRC.git

#include "controller/LADRC.hpp"

#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

namespace controller {

void LADRC2::ESO::setParameters(float wo, float b0, float dt) {
    Matrix3f A;
    Vector3f B;
    Vector3f L;
    Matrix<float, 1, 3> C;
    Matrix<float, 3, 2> B_obs_ct;
    Matrix3f A_obs_ct;

    A << 0, 1, 0,  //
        0, 0, 1,   //
        0, 0, 0;

    B << 0, b0, 0;

    C << 1, 0, 0;

    L << 3 * wo, 3 * wo * wo, wo * wo * wo;

    A_obs_ct = A - L * C;
    B_obs_ct << B, L;

    Discretize(A_obs_ct, B_obs_ct, dt);
}

void LADRC2::ESO::Discretize(Matrix3f const& A_obs_ct, Matrix<float, 3, 2> const& B_obs_ct, float dt) {
    Matrix<float, 5, 5> discretization, discretization_exp;

    discretization << A_obs_ct, B_obs_ct, Matrix<float, 2, 5>::Zero();
    discretization_exp = (discretization * dt).exp();

    m_A_obs_dt = discretization_exp.block<3, 3>(0, 0);
    m_B_obs_dt = discretization_exp.block<3, 2>(0, 3);
}

void LADRC2::ESO::setState(Eigen::Vector3f const& xhat) { m_xhat = xhat; }

const Vector3f& LADRC2::ESO::update(float u, float y) {
    Vector2f u_obs{u, y};
    m_xhat.noalias() = m_A_obs_dt * m_xhat;
    m_xhat += m_B_obs_dt * u_obs;
    return m_xhat;
}

void LADRC2::setParameters(float kp, float kd, float wo, float b0, float dt) {
    m_observer.setParameters(wo, b0, dt);
    m_kp = kp;
    m_kd = kd;
    m_b = b0;
}

float LADRC2::Controller(Vector3f const& xhat, float y_desired) {
    float u0 = m_kp * (y_desired - xhat[0]) - m_kd * xhat[1];
    return (u0 - xhat[2]) / m_b;
}

void LADRC2::reset() { m_observer.setState(Vector3f::Zero()); }

float LADRC2::update(float u, float y, float y_desired) {
    const Vector3f& xhat = m_observer.update(u, y);
    return Controller(xhat, y_desired);
}

}  // namespace controller