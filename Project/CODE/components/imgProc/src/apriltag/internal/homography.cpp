#include "apriltag/internal/homography.hpp"

#include "Eigen/Eigen"
#include "apriltag/fmath.hpp"

namespace imgProc {
namespace apriltag {

void homography_compute2(float_t dst[3][3], const float_t c[4][4]) {
    // clang-format off
    float_t A[]{
        c[0][0], c[0][1], 1,       0,       0, 0, -c[0][0]*c[0][2], -c[0][1]*c[0][2], c[0][2],
              0,       0, 0, c[0][0], c[0][1], 1, -c[0][0]*c[0][3], -c[0][1]*c[0][3], c[0][3],
        c[1][0], c[1][1], 1,       0,       0, 0, -c[1][0]*c[1][2], -c[1][1]*c[1][2], c[1][2],
              0,       0, 0, c[1][0], c[1][1], 1, -c[1][0]*c[1][3], -c[1][1]*c[1][3], c[1][3],
        c[2][0], c[2][1], 1,       0,       0, 0, -c[2][0]*c[2][2], -c[2][1]*c[2][2], c[2][2],
              0,       0, 0, c[2][0], c[2][1], 1, -c[2][0]*c[2][3], -c[2][1]*c[2][3], c[2][3],
        c[3][0], c[3][1], 1,       0,       0, 0, -c[3][0]*c[3][2], -c[3][1]*c[3][2], c[3][2],
              0,       0, 0, c[3][0], c[3][1], 1, -c[3][0]*c[3][3], -c[3][1]*c[3][3], c[3][3],
    };
    // clang-format on

    // Eliminate.
    for (int col = 0; col < 8; col++) {
        // Find best row to swap with.
        float_t max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++) {
            float_t val = fabs(A[row * 9 + col]);
            if (val > max_val) {
                max_val = val;
                max_val_idx = row;
            }
        }

        // Swap to get best row.
        if (max_val_idx != col) {
            for (int i = col; i < 9; i++) {
                float_t tmp = A[col * 9 + i];
                A[col * 9 + i] = A[max_val_idx * 9 + i];
                A[max_val_idx * 9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++) {
            float_t f = A[i * 9 + col] / A[col * 9 + col];
            A[i * 9 + col] = 0;
            for (int j = col + 1; j < 9; j++) A[i * 9 + j] -= f * A[col * 9 + j];
        }
    }

    // Back solve.
    for (int col = 7; col >= 0; col--) {
        float_t sum = 0;
        for (int i = col + 1; i < 8; i++) { sum += A[col * 9 + i] * A[i * 9 + 8]; }
        A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
    }
    // clang-format off
    dst[0][0] = A[8 ], dst[0][1] = A[17], dst[0][2] = A[26];
    dst[1][0] = A[35], dst[1][1] = A[44], dst[1][2] = A[53];
    dst[2][0] = A[62], dst[2][1] = A[71], dst[2][2] = 1;
    // clang-format on
}

void homography_project(const float_t H[3][3], float_t x, float_t y, float_t* ox, float_t* oy) {
    float_t xx = H[0][0] * x + H[0][1] * y + H[0][2];
    float_t yy = H[1][0] * x + H[1][1] * y + H[1][2];
    float_t zz = H[2][0] * x + H[2][1] * y + H[2][2];
    *ox = xx / zz;
    *oy = yy / zz;
}

void homography_to_pose(const float_t H[3][3], float_t fx, float_t fy, float_t cx, float_t cy, float_t R[3][3], float_t t[3]) {
    using namespace Eigen;
    Matrix<float_t, 3, 3> _R;
    _R(2, 0) = H[2][0];
    _R(2, 1) = H[2][1];
    t[2] = H[2][2];
    _R(0, 0) = (H[0][0] - cx * _R(2, 0)) / fx;
    _R(0, 1) = (H[0][1] - cx * _R(2, 1)) / fx;
    t[0] = (H[0][2] - cx * t[2]) / fx;
    _R(1, 0) = (H[1][0] - cy * _R(2, 0)) / fy;
    _R(1, 1) = (H[1][1] - cy * _R(2, 1)) / fy;
    t[1] = (H[1][2] - cy * t[2]) / fy;

    // compute the scale by requiring that the rotation columns are unit length
    // (Use geometric average of the two length vectors we have)
    float_t length1 = sqrtf(_R(0, 0) * _R(0, 0) + _R(1, 0) * _R(1, 0) + _R(2, 0) * _R(2, 0));
    float_t length2 = sqrtf(_R(0, 1) * _R(0, 1) + _R(1, 1) * _R(1, 1) + _R(2, 1) * _R(2, 1));
    float_t s = 1.0 / sqrtf(length1 * length2);

    // get sign of S by requiring the tag to be in front the camera;
    // we assume camera looks in the -Z direction.
    if (t[2] > 0) s = -s;

    _R(2, 0) *= s;
    _R(2, 1) *= s;
    t[2] *= s;
    _R(0, 0) *= s;
    _R(0, 1) *= s;
    t[0] *= s;
    _R(1, 0) *= s;
    _R(1, 1) *= s;
    t[1] *= s;

    // now recover [_R(0,2) _R(1,2) _R(2,2)] by noting that it is the cross product of the other two columns.
    _R(0, 2) = _R(1, 0) * _R(2, 1) - _R(2, 0) * _R(1, 1);
    _R(1, 2) = _R(2, 0) * _R(0, 1) - _R(0, 0) * _R(2, 1);
    _R(2, 2) = _R(0, 0) * _R(1, 1) - _R(1, 0) * _R(0, 1);

    JacobiSVD<Matrix<float_t, 3, 3>> svd(_R, ComputeThinU | ComputeThinV);
    Map<Matrix<float_t, 3, 3>>(R[0], 3, 3).noalias() = svd.matrixU() * svd.matrixV().transpose();
}

void matrix_transform(const float_t R[9], const float_t t[3], const float_t v[3], float_t dst[3]) {
    using namespace Eigen;
    Map<const Matrix<float_t, 3, 3>> RM(R);
    Map<const Matrix<float_t, 3, 1>> tM(t);
    Map<const Matrix<float_t, 3, 1>> vM(v);
    Map<Matrix<float_t, 3, 1>> dstM(dst);
    dstM.noalias() = RM * vM;
    dstM += tM;
}

void camera_to_image(float_t fx, float_t fy, float_t cx, float_t cy, const float_t src[3], float_t dst[2]) {
    dst[0] = fx * src[0] / src[2] + cx;
    dst[1] = fy * src[1] / src[2] + cy;
}

}  // namespace apriltag
}  // namespace imgProc