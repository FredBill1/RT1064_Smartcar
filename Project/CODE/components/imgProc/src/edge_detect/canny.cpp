#include "edge_detect/canny.hpp"

#include <utility>

#include "apriltag/config.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "edge_detect/conv.hpp"
#include "imgProc/CoordStack.hpp"
#include "imgProc/common.hpp"

#ifndef M_PI
#define M_PI 3.14159265f
#endif

namespace imgProc {
namespace edge_detect {
using namespace apriltag;

gvec_t* canny(uint8_t* src, int low_thresh, int high_thresh) {
    // 1. Noise Reduction with a Gaussian filter
    sepconv3(src, kernel_gauss_3, 1.0f / 16.0f, 0.0f);

    // 2. Finding Image Gradients
    gvec_t* gm = (gvec_t*)staticBuffer.allocate(N * M * sizeof(*gm));
    // 2. Finding Image Gradients
    for (int y = 1; y < N - 1; y++) {
        for (int x = 1; x < M - 1; x++) {
            int vx = 0, vy = 0;
            // sobel kernel in the horizontal direction
            vx = (int)src[(y - 1) * M + x - 1] - (int)src[(y - 1) * M + x + 1] + ((int)src[(y + 0) * M + x - 1] << 1) -
                 ((int)src[(y + 0) * M + x + 1] << 1) + (int)src[(y + 1) * M + x - 1] - (int)src[(y + 1) * M + x + 1];

            // sobel kernel in the vertical direction
            vy = (int)src[(y - 1) * M + x - 1] + ((int)src[(y - 1) * M + x + 0] << 1) + (int)src[(y - 1) * M + x + 1] -
                 (int)src[(y + 1) * M + x - 1] - ((int)src[(y + 1) * M + x + 0] << 1) - (int)src[(y + 1) * M + x + 1];

            // Find magnitude
            int g = (int)sqrtf(vx * vx + vy * vy);
            // Find the direction and round angle to 0, 45, 90 or 135
            float theta = atan2f(vy, vx);
            if (theta < 0) theta += M_PI;
            int t = (int)(theta * (180.0f / M_PI));

            if (t < 22) {
                t = 0;
            } else if (t < 67) {
                t = 45;
            } else if (t < 112) {
                t = 90;
            } else if (t < 160) {
                t = 135;
            } else {
                t = 0;
            }

            gm[y * M + x].t = t;
            gm[y * M + x].g = g;
        }
    }

    // 3. Non-maximum Suppression
    CoordStack stack;
    for (int y = 0; y < N; y++) {
        for (int x = 0; x < M; x++) {
            int i = y * M + x;
            gvec_t *va, *vb, *vc = &gm[y * M + x];

            // Clear the borders
            if (y == (0) || y == (N - 1) || x == (0) || x == (M - 1)) {
                src[i] = 0;
                continue;
            }

            if (vc->g < low_thresh) {
                // Not an edge
                src[i] = 0;
                continue;
                // Strong edge
            } else if (vc->g >= high_thresh) {
                src[i] = 255;
            } else {  // Weak edge
                src[i] = 1;
                continue;
            }

#define CANNY_CHECK_                                         \
    if (src[i] == 255) stack.push({(int16_t)x, (int16_t)y}); \
    else                                                     \
        src[i] = 0;
#define CANNY_CHECK1(dx, dy) \
    if (vc->g > gm[(y - dy) * M + (x - dx)].g && vc->g >= gm[(y + dy) * M + (x + dx)].g) { CANNY_CHECK_ }
#define CANNY_CHECK2(dx, dy) \
    if (vc->g > gm[(y - dy) * M + (x - dx)].g && vc->g > gm[(y + dy) * M + (x + dx)].g) { CANNY_CHECK_ }

            switch (vc->t) {
            case 0: CANNY_CHECK1(1, 0); break;
            case 45: CANNY_CHECK2(1, 1); break;
            case 90: CANNY_CHECK1(0, 1); break;
            default: CANNY_CHECK2(-1, 1); break;
            }
        }
    }
    // 4. Hysteresis Thresholding
    while (!stack.empty()) {
        Coordinate xy = stack.pop();
        int x = xy.x, y = xy.y;
        uint8_t* m = src + (y * M + x);
#define CANNY_PUSH(dx, dy) \
    if (*(m + ((dy)*M + (dx))) == 1) *(m + ((dy)*M + (dx))) = 255, stack.push({(int16_t)(x + (dx)), (int16_t)(y + (dy))});
        if (y > 0) {
            if (x > 0) CANNY_PUSH(-1, -1);
            CANNY_PUSH(0, -1);
            if (x < M - 1) CANNY_PUSH(1, -1);
        }
        if (x > 0) CANNY_PUSH(-1, 0);
        if (x < M - 1) CANNY_PUSH(1, 0);
        if (y < N - 1) {
            if (x > 0) CANNY_PUSH(-1, 1);
            CANNY_PUSH(0, 1);
            if (x < M - 1) CANNY_PUSH(1, 1);
        }
    }
#undef CANNY_PUSH
    // staticBuffer.pop(N * M * sizeof(*gm));  // gvec_t* gm
    return gm;
}

}  // namespace edge_detect
}  // namespace imgProc