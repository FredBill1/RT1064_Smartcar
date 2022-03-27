// https://github.com/psiorx/ADRC.git

#ifndef _controller_ADRC_hpp
#define _controller_ADRC_hpp

#include <Eigen/Eigen>

namespace controller {

class LADRC2 {
 protected:
    class ESO {
        Eigen::Vector3f m_xhat;
        Eigen::Matrix3f m_A_obs_dt;
        Eigen::Matrix<float, 3, 2> m_B_obs_dt;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void Discretize(Eigen::Matrix3f const& A_obs_ct, Eigen::Matrix<float, 3, 2> const& B_obs_ct, float dt);

     public:
        ESO() {}
        ESO(float wo, float b0, float dt) { setParameters(wo, b0, dt); }
        void setState(Eigen::Vector3f const& xhat);
        void setParameters(float wo, float b, float dt);
        const Eigen::Vector3f& update(float u, float y);
    };

 private:
    ESO m_observer;
    float m_kp;
    float m_kd;
    float m_b;

    float Controller(Eigen::Vector3f const& xhat, float y_desired);

 public:
    LADRC2() { reset(); }
    LADRC2(float kp, float kd, float wo, float b0, float dt) : LADRC2() { setParameters(kp, kd, wo, b0, dt); }
    LADRC2(float wc, float wo, float b0, float dt) : LADRC2() { setParameters(wc, wo, b0, dt); }
    void setParameters(float kp, float kd, float wo, float b0, float dt);
    void setParameters(float wc, float wo, float b0, float dt) { setParameters(wc * wc, wc + wc, wo, b0, dt); }
    void reset();
    float update(float u, float y, float y_desired);
};

}  // namespace controller

#endif  // _controller_ADRC_hpp