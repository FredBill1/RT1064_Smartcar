#include "edge_detect/canny.hpp"

#include "apriltag/config.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "edge_detect/conv.hpp"

namespace imgProc {
namespace edge_detect {
using namespace apriltag;

typedef struct gvec {
    uint16_t t;
    uint16_t g;
} gvec_t;

void canny(uint8_t* src, int low_thresh, int high_thresh) {
    // 1. Noise Reduction with a Gaussian filter
    sepconv3(src, kernel_gauss_3, 1.0f / 16.0f, 0.0f);

    // 2. Finding Image Gradients
    gvec_t* gm = (gvec_t*)staticBuffer.allocate(N * M * sizeof(*gm));
    // 2. Finding Image Gradients
    for (int gy = 1, y = 1; y < N - 1; y++, gy++) {
        for (int gx = 1, x = 1; x < M - 1; x++, gx++) {
            int vx = 0, vy = 0;
            // sobel kernel in the horizontal direction
            vx = src[(y - 1) * M + x - 1] - src[(y - 1) * M + x + 1] + (src[(y + 0) * M + x - 1] << 1) -
                 (src[(y + 0) * M + x + 1] << 1) + src[(y + 1) * M + x - 1] - src[(y + 1) * M + x + 1];

            // sobel kernel in the vertical direction
            vy = src[(y - 1) * M + x - 1] + (src[(y - 1) * M + x + 0] << 1) + src[(y - 1) * M + x + 1] -
                 src[(y + 1) * M + x - 1] - (src[(y + 1) * M + x + 0] << 1) - src[(y + 1) * M + x + 1];

            // Find magnitude
            int g = (int)sqrtf(vx * vx + vy * vy);
            // Find the direction and round angle to 0, 45, 90 or 135
            int t = (int)fabs((atan2f(vy, vx) * (180.0f / 3.14159265358979323846f)));
            if (t < 22) {
                t = 0;
            } else if (t < 67) {
                t = 45;
            } else if (t < 112) {
                t = 90;
            } else if (t < 160) {
                t = 135;
            } else if (t <= 180) {
                t = 0;
            }

            gm[gy * M + gx].t = t;
            gm[gy * M + gx].g = g;
        }
    }

    // 3. Hysteresis Thresholding
    // 4. Non-maximum Suppression and output
    for (int gy = 0, y = 0; y < N; y++, gy++) {
        for (int gx = 0, x = 0; x < M; x++, gx++) {
            int i = y * M + x;
            gvec_t *va = NULL, *vb = NULL, *vc = &gm[gy * M + gx];

            // Clear the borders
            if (y == (0) || y == (N - 1) || x == (0) || x == (M - 1)) {
                src[i] = 0;
                continue;
            }

            if (vc->g < low_thresh) {
                // Not an edge
                src[i] = 0;
                continue;
                // Check if strong or weak edge
            } else if (vc->g >= high_thresh || gm[(gy - 1) * M + (gx - 1)].g >= high_thresh ||
                       gm[(gy - 1) * M + (gx + 0)].g >= high_thresh || gm[(gy - 1) * M + (gx + 1)].g >= high_thresh ||
                       gm[(gy + 0) * M + (gx - 1)].g >= high_thresh || gm[(gy + 0) * M + (gx + 1)].g >= high_thresh ||
                       gm[(gy + 1) * M + (gx - 1)].g >= high_thresh || gm[(gy + 1) * M + (gx + 0)].g >= high_thresh ||
                       gm[(gy + 1) * M + (gx + 1)].g >= high_thresh) {
                vc->g = vc->g;
            } else {  // Not an edge
                src[i] = 0;
                continue;
            }

            switch (vc->t) {
            case 0: {
                va = &gm[(gy + 0) * M + (gx - 1)];
                vb = &gm[(gy + 0) * M + (gx + 1)];
                break;
            }

            case 45: {
                va = &gm[(gy + 1) * M + (gx - 1)];
                vb = &gm[(gy - 1) * M + (gx + 1)];
                break;
            }

            case 90: {
                va = &gm[(gy + 1) * M + (gx + 0)];
                vb = &gm[(gy - 1) * M + (gx + 0)];
                break;
            }

            case 135: {
                va = &gm[(gy + 1) * M + (gx + 1)];
                vb = &gm[(gy - 1) * M + (gx - 1)];
                break;
            }
            }

            if (!(vc->g > va->g && vc->g > vb->g)) {
                src[i] = 0;
            } else {
                src[i] = 255;
            }
        }
    }
    staticBuffer.pop(N * M * sizeof(*gm));  // gvec_t* gm
}

}  // namespace edge_detect
}  // namespace imgProc