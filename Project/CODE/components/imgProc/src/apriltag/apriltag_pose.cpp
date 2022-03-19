#include "apriltag/apriltag_pose.hpp"

#include <cmath>

#include "Eigen/Eigen"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/homography.hpp"
#include "apriltag/internal/utility.hpp"
using namespace Eigen;

namespace imgProc {
namespace apriltag {

constexpr int n_points = 4;

void estimate_pose_for_tag_homography(const apriltag_detection_info& info, apriltag_pose& solution) {
    auto& R = solution.R;
    auto& t = solution.t;
    homography_to_pose(info.det->H, -info.fx, info.fy, info.cx, info.cy, R, t);

    float_t scale = info.tagsize / 2;
    rep(i, 0, 3) t[i] *= scale;
    req(i, 1, 2) {
        t[i] = -t[i];
        rep(j, 0, 3) R[j][i] = -R[j][i];
    }
}

/**
 * Calculate projection operator from image points.
 */
static void calculate_F(const Ref<const Vector3f> v, Ref<Matrix3f> F) {
    F.noalias() = v * v.transpose();
    F /= v.dot(v);
}

static float orthogonal_iteration(const Vector3f v[], const Vector3f p[], Ref<Vector3f> t, Ref<Matrix3f> R, int n_steps) {
    Vector3f p_mean = Vector3f::Zero();
    rep(i, 0, n_points) p_mean += p[i];
    p_mean *= 1.f / n_points;

    Vector3f p_res[n_points];
    rep(i, 0, n_points) p_res[i] = p[i] - p_mean;

    // Compute M1_inv.
    Matrix3f F[n_points];
    Matrix3f avg_F = Matrix3f::Zero();
    rep(i, 0, n_points) {
        calculate_F(v[i], F[i]);
        avg_F += F[i];
    }
    avg_F *= 1.f / n_points;
    Matrix3f M1 = Matrix3f::Identity() - avg_F;
    Matrix3f M1_inv = M1.inverse();

    float prev_error = HUGE_VALF;
    // Iterate.
    rep(i, 0, n_steps) {
        // Calculate translation.
        Vector3f M2 = Vector3f::Zero();
        rep(j, 0, n_points) M2 += (F[j] - Matrix3f::Identity()) * R * p[j];
        M2 *= 1.f / n_points;
        t = M1_inv * M2;

        // Calculate rotation.
        Vector3f q[n_points];
        Vector3f q_mean = Vector3f::Zero();
        rep(j, 0, n_points) {
            q[j] = F[j] * (R * p[j] + t);
            q_mean += q[j];
        }
        q_mean *= 1.f / n_points;

        Matrix3f M3 = Matrix3f::Zero();
        rep(j, 0, n_points) M3 += (q[j] - q_mean) * p_res[j].transpose();
        JacobiSVD<Matrix3f> svd(M3, ComputeThinU | ComputeThinV);
        R = svd.matrixU() * svd.matrixV().transpose();
        // if (R.determinant() < 0) {
        if (R(2, 2) < 0) {
            R(0, 2) = -R(0, 2);
            R(1, 2) = -R(1, 2);
            R(2, 2) = -R(2, 2);
        }

        float error = 0;
        rep(j, 0, 4) {
            Vector3f err_vec = (Matrix3f::Identity() - F[j]) * (R * p[j] + t);
            error += err_vec.dot(err_vec);
        }
        prev_error = error;
    }
    return prev_error;
}

/**
 * Evaluates polynomial p at x.
 */
inline float polyval(float* p, int degree, float x) {
    float ret = 0;
    for (int i = 0; i <= degree; i++) ret += p[i] * (float)std::pow(x, i);
    return ret;
}

/**
 * Numerically solve small degree polynomials. This is a customized method. It
 * ignores roots larger than 1000 and only gives small roots approximately.
 *
 * @param p Array of parameters s.t. p(x) = p[0] + p[1]*x + ...
 * @param degree The degree of p(x).
 * @outparam roots
 * @outparam n_roots
 */
static void solve_poly_approx(float* p, int degree, float* roots, int* n_roots) {
    constexpr int max_degree = 4;
    constexpr int MAX_ROOT = 1000;
    if (degree == 1) {
        if (fabs(p[0]) > MAX_ROOT * fabs(p[1])) {
            *n_roots = 0;
        } else {
            roots[0] = -p[0] / p[1];
            *n_roots = 1;
        }
        return;
    }

    // Calculate roots of derivative.
    float p_der[max_degree];
    for (int i = 0; i < degree; i++) p_der[i] = (i + 1) * p[i + 1];

    float der_roots[max_degree - 1];
    int n_der_roots;
    solve_poly_approx(p_der, degree - 1, der_roots, &n_der_roots);

    // Go through all possibilities for roots of the polynomial.
    *n_roots = 0;
    for (int i = 0; i <= n_der_roots; i++) {
        float min;
        if (i == 0) {
            min = -MAX_ROOT;
        } else {
            min = der_roots[i - 1];
        }

        float max;
        if (i == n_der_roots) {
            max = MAX_ROOT;
        } else {
            max = der_roots[i];
        }

        if (polyval(p, degree, min) * polyval(p, degree, max) < 0) {
            // We have a zero-crossing in this interval, use a combination of Newton' and bisection.
            // Some thanks to Numerical Recipes in C.

            float lower;
            float upper;
            if (polyval(p, degree, min) < polyval(p, degree, max)) {
                lower = min;
                upper = max;
            } else {
                lower = max;
                upper = min;
            }
            float root = 0.5 * (lower + upper);
            float dx_old = upper - lower;
            float dx = dx_old;
            float f = polyval(p, degree, root);
            float df = polyval(p_der, degree - 1, root);

            for (int j = 0; j < 100; j++) {
                if (((f + df * (upper - root)) * (f + df * (lower - root)) > 0) || (fabs(2 * f) > fabs(dx_old * df))) {
                    dx_old = dx;
                    dx = 0.5 * (upper - lower);
                    root = lower + dx;
                } else {
                    dx_old = dx;
                    dx = -f / df;
                    root += dx;
                }

                if (root == upper || root == lower) { break; }

                f = polyval(p, degree, root);
                df = polyval(p_der, degree - 1, root);

                if (f > 0) {
                    upper = root;
                } else {
                    lower = root;
                }
            }

            roots[(*n_roots)++] = root;
        } else if (polyval(p, degree, max) == 0) {
            // float/triple root.
            roots[(*n_roots)++] = max;
        }
    }
}

/**
 * Given a local minima of the pose error tries to find the other minima.
 */
inline bool fix_pose_ambiguities(const Vector3f v[], const Vector3f p[], Ref<Vector3f> t, Ref<Matrix3f> R, Ref<Matrix3f> ret) {
    // 1. Find R_t
    Vector3f R_t_3 = t / t.norm();

    Vector3f e_x{1, 0, 0};
    Vector3f R_t_3_tmp = (e_x.transpose() * R_t_3).value() * R_t_3;
    Vector3f R_t_1_tmp = e_x - R_t_3_tmp;
    Vector3f R_t_1 = R_t_1_tmp / R_t_1_tmp.norm();

    Vector3f R_t_2 = R_t_3.cross(R_t_1);
    Matrix3f R_t{{R_t_1[0], R_t_1[1], R_t_1[2]}, {R_t_2[0], R_t_2[1], R_t_2[2]}, {R_t_3[0], R_t_3[1], R_t_3[2]}};

    // 2. Find R_z
    Matrix3f R_1_prime = R_t * R;
    float r31 = R_1_prime(2, 0);
    float r32 = R_1_prime(2, 1);
    float hypotenuse = sqrtf(r31 * r31 + r32 * r32);
    if (hypotenuse < 1e-100) {
        r31 = 1;
        r32 = 0;
        hypotenuse = 1;
    }
    Matrix3f R_z{{r31 / hypotenuse, -r32 / hypotenuse, 0}, {r32 / hypotenuse, r31 / hypotenuse, 0}, {0, 0, 1}};

    // 3. Calculate parameters of Eos
    Matrix3f R_trans = R_1_prime * R_z;
    float sin_gamma = -R_trans(0, 1);
    float cos_gamma = R_trans(1, 1);
    Matrix3f R_gamma{{cos_gamma, -sin_gamma, 0}, {sin_gamma, cos_gamma, 0}, {0, 0, 1}};
    float sin_beta = -R_trans(2, 0);
    float cos_beta = R_trans(2, 2);
    float t_initial = atan2f(sin_beta, cos_beta);

    Vector3f v_trans[n_points];
    Vector3f p_trans[n_points];
    Matrix3f F_trans[n_points];
    Matrix3f avg_F_trans = Matrix3f::Zero();
    rep(i, 0, n_points) {
        p_trans[i].noalias() = R_z.transpose() * p[i];
        v_trans[i].noalias() = R_t * v[i];
        calculate_F(v_trans[i], F_trans[i]);
        avg_F_trans += F_trans[i];
    }
    avg_F_trans *= 1.f / n_points;
    Matrix3f G_inv_tmp = Matrix3f::Identity() - avg_F_trans;
    Matrix3f G = G_inv_tmp.inverse();
    G *= 1.f / n_points;

    Matrix3f M1{{0, 0, 2}, {0, 0, 0}, {-2, 0, 0}};
    Matrix3f M2{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}};

    Vector3f b0 = Vector3f::Zero();
    Vector3f b1 = Vector3f::Zero();
    Vector3f b2 = Vector3f::Zero();
    rep(i, 0, n_points) {
        b0 += (F_trans[i] - Matrix3f::Identity()) * R_gamma * p_trans[i];
        b1 += (F_trans[i] - Matrix3f::Identity()) * R_gamma * M1 * p_trans[i];
        b2 += (F_trans[i] - Matrix3f::Identity()) * R_gamma * M2 * p_trans[i];
    }
    Vector3f b0_ = G * b0;
    Vector3f b1_ = G * b1;
    Vector3f b2_ = G * b2;

    float a0 = 0, a1 = 0, a2 = 0, a3 = 0, a4 = 0;
    rep(i, 0, n_points) {
        Vector3f c0 = (Matrix3f::Identity() - F_trans[i]) * (R_gamma * p_trans[i] + b0_);
        Vector3f c1 = (Matrix3f::Identity() - F_trans[i]) * (R_gamma * M1 * p_trans[i] + b1_);
        Vector3f c2 = (Matrix3f::Identity() - F_trans[i]) * (R_gamma * M2 * p_trans[i] + b2_);

        a0 += (c0.transpose() * c0).value();
        a1 += 2.f * (c0.transpose() * c1).value();
        a2 += (c1.transpose() * c1).value() + 2.f * (c0.transpose() * c2).value();
        a3 += 2.f * (c1.transpose() * c2).value();
        a4 += (c2.transpose() * c2).value();
    }

    // 4. Solve for minima of Eos.
    float px[5]{a1, 2 * a2 - 4 * a0, 3 * a3 - 3 * a1, 4 * a4 - 2 * a2, -a3};

    float roots[4];
    int n_roots;
    solve_poly_approx(px, 4, roots, &n_roots);

    float minima[4];
    int n_minima = 0;
    for (int i = 0; i < n_roots; i++) {
        float t1 = roots[i];
        float t2 = t1 * t1;
        float t3 = t1 * t2;
        float t4 = t1 * t3;
        float t5 = t1 * t4;
        // Check extrema is a minima.
        if (a2 - 2 * a0 + (3 * a3 - 6 * a1) * t1 + (6 * a4 - 8 * a2 + 10 * a0) * t2 + (-8 * a3 + 6 * a1) * t3 +
                (-6 * a4 + 3 * a2) * t4 + a3 * t5 >=
            0) {
            // And that it corresponds to an angle different than the known minimum.
            float t_cur = 2 * atanf(roots[i]);
            // We only care about finding a second local minima which is qualitatively
            // different than the first.
            if (fabs(t_cur - t_initial) > 0.1) { minima[n_minima++] = roots[i]; }
        }
    }

    // 5. Get poses for minima.
    if (n_minima != 1) return false;
    float t_cur = minima[0];
    Matrix3f R_beta = M2 * t_cur;
    R_beta += M1;
    R_beta *= t_cur;
    R_beta += Matrix3f::Identity();
    R_beta *= 1 / (1 + t_cur * t_cur);
    ret.noalias() = R_t.transpose() * R_gamma * R_beta * R_z;
    return true;
}

/**
 * Estimate tag pose using orthogonal iteration.
 */
inline void estimate_tag_pose_orthogonal_iteration(const apriltag_detection_info& info, float& err1, apriltag_pose& solution1,
                                                   float& err2, apriltag_pose& solution2, int nIters) {
    float scale = info.tagsize * 0.5f;
    Vector3f p[4]{{-scale, -scale, 0}, {scale, -scale, 0}, {scale, scale, 0}, {-scale, scale, 0}};
    Vector3f v[4];
    rep(i, 0, 4) v[i] = {(info.det->p[i][0] - info.cx) / info.fx, (info.det->p[i][1] - info.cy) / info.fy, 1};

    estimate_pose_for_tag_homography(info, solution1);
    err1 = orthogonal_iteration(v, p, Map<Vector3f>(solution1.t), Map<Matrix3f>(solution1.R[0]), nIters);
    if (fix_pose_ambiguities(v, p, Map<Vector3f>(solution1.t), Map<Matrix3f>(solution1.R[0]), Map<Matrix3f>(solution2.R[0]))) {
        err2 = orthogonal_iteration(v, p, Map<Vector3f>(solution2.t), Map<Matrix3f>(solution2.R[0]), nIters);
    } else
        err2 = HUGE_VALF;
}

float estimate_tag_pose(const apriltag_detection_info& info, apriltag_pose& pose) {
    float err1, err2;
    apriltag_pose pose2;
    estimate_tag_pose_orthogonal_iteration(info, err1, pose, err2, pose2, 50);
    if (err1 <= err2) return err1;
    pose = pose2;
    return err2;
}

det_poses_t& estimate_poses(apriltag_detection_info& info, detections_t& dets) {
    det_poses_t& det_poses = *new (staticBuffer.allocate(sizeof(det_poses_t))) det_poses_t(det_poses_alloc_t{staticBuffer});
    det_poses.reserve(std::distance(dets.begin(), dets.end()));
    for (apriltag_detection* det_p : dets) {
        det_poses.emplace_back(*det_p);
        info.det = det_p;
        apriltag_pose& pose = det_poses.back().pose;
        pose.err = estimate_tag_pose(info, pose);
        // estimate_pose_for_tag_homography(info, det_poses.back().pose);
    }
    return det_poses;
}

void tag_pose_to_camera(const apriltag_pose& pose, const float_t src[3], float_t dst[3]) {
    matrix_transform(pose.R[0], pose.t, src, dst);
}

void camera_info_to_image(const apriltag_detection_info& info, const float_t src[3], float_t dst[2]) {
    camera_to_image(info.fx, info.fy, info.cx, info.cy, src, dst);
}

void tag_pose_to_image(const apriltag_detection_info& info, const apriltag_pose& pose, const float_t src[3], float_t dst[2]) {
    float_t cam[3];
    tag_pose_to_camera(pose, src, cam);
    camera_info_to_image(info, cam, dst);
}

}  // namespace apriltag
}  // namespace imgProc