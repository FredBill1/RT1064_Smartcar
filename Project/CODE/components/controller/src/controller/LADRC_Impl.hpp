#ifndef _controller_LADRC_Impl_hpp
#define _controller_LADRC_Impl_hpp

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>

namespace controller {
template <typename Type, int N> static constexpr std::array<Type, N + 1> _binomial_array() {
    std::array<Type, N + 1> res{1};
    Type fact[N + 1]{1};
    for (int i = 1; i <= N; ++i) fact[i] = fact[i - 1] * i;
    for (int i = 1; i <= N; ++i) res[i] = fact[N] / (fact[i] * fact[N - i]);
    return res;
}

template <typename Type, int Order> class LADRC_Impl {
    static constexpr int N = Order + 1;
    using Matrix = Eigen::Matrix<Type, N, N>;
    using Vector = Eigen::Matrix<Type, N, 1>;
    class LESO {
        Vector m_xhat;
        Matrix m_A_obs_dt;
        Eigen::Matrix<Type, N, 2> m_B_obs_dt;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        void Discretize(Matrix const& A_obs_ct, Eigen::Matrix<Type, N, 2> const& B_obs_ct, Type dt) {
            Eigen::Matrix<Type, N + 2, N + 2> discretization, discretization_exp;
            discretization << A_obs_ct, B_obs_ct, Eigen::Matrix<Type, 2, N + 2>::Zero();
            discretization_exp = (discretization * dt).exp();

            m_A_obs_dt = discretization_exp.template block<N, N>(0, 0);
            m_B_obs_dt = discretization_exp.template block<N, 2>(0, N);
        }

     public:
        void setState(Vector const& xhat) { m_xhat = xhat; }
        void setParameters(Type wo, Type b0, Type dt) {
            Matrix A = Matrix::Zero();
            Vector B = Vector::Zero();
            Vector L;
            Eigen::Matrix<Type, 1, N> C = Eigen::Matrix<Type, 1, N>::Zero();
            Eigen::Matrix<Type, N, 2> B_obs_ct;
            Matrix A_obs_ct;

            A.template block<N - 1, N - 1>(0, 1).setIdentity();
            B[N - 2] = b0;
            C[0] = 1;

            constexpr auto binomial_array = _binomial_array<Type, N>();
            Type wo_pow = wo;
            for (int i = 0; i < N; ++i) {
                L[i] = binomial_array[i + 1] * wo_pow;
                wo_pow *= wo;
            }

            A_obs_ct = A - L * C;
            B_obs_ct << B, L;
            Discretize(A_obs_ct, B_obs_ct, dt);
        }

        const Vector& update(Type u, Type y) {
            Eigen::Matrix<Type, 2, 1> u_obs{u, y};
            m_xhat = m_A_obs_dt * m_xhat;
            m_xhat += m_B_obs_dt * u_obs;
            return m_xhat;
        }
    };

    LESO m_observer;
    Type m_k[Order];
    Type m_b;
    Type Controller(Vector const& xhat, float y_desired) {
        float u0 = m_k[0] * (y_desired - xhat[0]);
        for (int i = 1; i < N - 1; ++i) u0 -= m_k[i] * xhat[i];
        u0 -= xhat[N - 1];
        return u0 / m_b;
    }

 public:
    LADRC_Impl() { reset(); }
    LADRC_Impl(Type wc, Type wo, Type b0, Type dt) : LADRC_Impl() { setParameters(wc, wo, b0, dt); }
    void setParameters(Type wc, Type wo, Type b0, Type dt) {
        m_observer.setParameters(wo, b0, dt);

        constexpr auto binomial_array = _binomial_array<Type, Order>();
        Type wc_pow = wc;
        for (int i = Order - 1; i >= 0; --i) {
            m_k[i] = binomial_array[i] * wc_pow;
            wc_pow *= wc;
        }

        m_b = b0;
    }

    void reset() { m_observer.setState(Vector::Zero()); }
    Type update(Type u, Type y, Type y_desired) {
        const Vector& xhat = m_observer.update(u, y);
        return Controller(xhat, y_desired);
    }
};
}  // namespace controller

#endif