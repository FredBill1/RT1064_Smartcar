#include "apriltag/internal/homography.hpp"

#include <cmath>

#include "Eigen/Eigen"

namespace imgProc {
namespace apriltag {

void homography_compute2(double dst[3][3], double c[4][4]) {
    // clang-format off
    double A[]{
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
        double max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++) {
            double val = std::abs(A[row * 9 + col]);
            if (val > max_val) {
                max_val = val;
                max_val_idx = row;
            }
        }

        // Swap to get best row.
        if (max_val_idx != col) {
            for (int i = col; i < 9; i++) {
                double tmp = A[col * 9 + i];
                A[col * 9 + i] = A[max_val_idx * 9 + i];
                A[max_val_idx * 9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++) {
            double f = A[i * 9 + col] / A[col * 9 + col];
            A[i * 9 + col] = 0;
            for (int j = col + 1; j < 9; j++) A[i * 9 + j] -= f * A[col * 9 + j];
        }
    }

    // Back solve.
    for (int col = 7; col >= 0; col--) {
        double sum = 0;
        for (int i = col + 1; i < 8; i++) { sum += A[col * 9 + i] * A[i * 9 + 8]; }
        A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
    }
    // clang-format off
    dst[0][0] = A[8 ], dst[0][1] = A[17], dst[0][2] = A[26];
    dst[1][0] = A[35], dst[1][1] = A[44], dst[1][2] = A[53];
    dst[2][0] = A[62], dst[2][1] = A[71], dst[2][2] = 1;
    // clang-format on
}

void homography_project(const double H[3][3], double x, double y, double *ox, double *oy) {
    double xx = H[0][0] * x + H[0][1] * y + H[0][2];
    double yy = H[1][0] * x + H[1][1] * y + H[1][2];
    double zz = H[2][0] * x + H[2][1] * y + H[2][2];
    *ox = xx / zz;
    *oy = yy / zz;
}

void homography_to_pose(const double H[3][3], double fx, double fy, double cx, double cy, double R[3][3], double t[3]) {
    using namespace Eigen;
    Matrix3d _R;
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
    double length1 = std::sqrt(_R(0, 0) * _R(0, 0) + _R(1, 0) * _R(1, 0) + _R(2, 0) * _R(2, 0));
    double length2 = std::sqrt(_R(0, 1) * _R(0, 1) + _R(1, 1) * _R(1, 1) + _R(2, 1) * _R(2, 1));
    double s = 1.0 / std::sqrt(length1 * length2);

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

    JacobiSVD<Matrix3d> svd(_R, ComputeThinU | ComputeThinV);
    Map<Matrix3d>(R[0], 3, 3).noalias() = svd.matrixU() * svd.matrixV().transpose();
}

}  // namespace apriltag
}  // namespace imgProc