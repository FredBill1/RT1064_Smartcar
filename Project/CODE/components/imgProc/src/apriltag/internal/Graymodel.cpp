#include "apriltag/internal/Graymodel.hpp"

#include <rtthread.h>

#include "apriltag/internal/apriltag_math.hpp"

namespace imgProc {
namespace apriltag {

graymodel::graymodel() { rt_memset(this, 0, sizeof(graymodel)); }

void graymodel::add(float_t x, float_t y, float_t gray) {
    // update upper right entries of A = J'J
    A[0][0] += x * x;
    A[0][1] += x * y;
    A[0][2] += x;
    A[1][1] += y * y;
    A[1][2] += y;
    A[2][2] += 1;

    // update B = J'gray
    B[0] += x * gray;
    B[1] += y * gray;
    B[2] += gray;
}

void graymodel::solve() { mat33_sym_solve((float_t*)A, B, C); }

float_t graymodel::interpolate(float_t x, float_t y) { return C[0] * x + C[1] * y + C[2]; }

}  // namespace apriltag
}  // namespace imgProc