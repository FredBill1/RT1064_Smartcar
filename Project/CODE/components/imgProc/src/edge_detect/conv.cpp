#include "edge_detect/conv.hpp"

#include "apriltag/config.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/utility.hpp"
#include "arm_math.h"

namespace imgProc {
namespace edge_detect {
using namespace apriltag;
// Image kernels
extern const int8_t kernel_gauss_3[3 * 3]{1, 2, 1, 2, 4, 2, 1, 2, 1};
extern const int8_t kernel_gauss_5[5 * 5]{1, 4, 6, 4, 1, 4, 16, 24, 16, 4, 6, 24, 36, 24, 6, 4, 16, 24, 16, 4, 1, 4, 6, 4, 1};
extern const int kernel_laplacian_3[3 * 3]{-1, -1, -1, -1, 8, -1, -1, -1, -1};
extern const int kernel_high_pass_3[3 * 3]{-1, -1, -1, -1, +8, -1, -1, -1, -1};

void sepconv3(uint8_t* img, const int8_t* krn, const float m, const int b) {
    constexpr int ksize = 3;
    // TODO: Support RGB
    int* buffer = (int*)staticBuffer.allocate(M * sizeof(*buffer) * 2);

    // NOTE: This doesn't deal with borders right now. Adding if
    // statements in the inner loop will slow it down significantly.
    for (int y = 0; y < N - ksize; y++) {
        for (int x = 0; x < M; x++) {
            int acc = 0;
            // if (IM_X_INSIDE(img, x+k) && IM_Y_INSIDE(img, y+j))
            acc = __SMLAD(krn[0], img[(y + 0) * M + x], acc);
            acc = __SMLAD(krn[1], img[(y + 1) * M + x], acc);
            acc = __SMLAD(krn[2], img[(y + 2) * M + x], acc);
            buffer[((y & 1) * M) + x] = acc;
        }
        if (y > 0) {
            // flush buffer
            for (int x = 0; x < M - ksize; x++) {
                int acc = 0;
                acc = __SMLAD(krn[0], buffer[((y - 1) % 2) * M + x + 0], acc);
                acc = __SMLAD(krn[1], buffer[((y - 1) % 2) * M + x + 1], acc);
                acc = __SMLAD(krn[2], buffer[((y - 1) % 2) * M + x + 2], acc);
                acc = (acc * m) + b;  // scale, offset, and clamp
                acc = max(min(acc, 255), 0);
                img[y * M + (x + 1)] = acc;
            }
        }
    }
    staticBuffer.pop(M * sizeof(*buffer) * 2);  // int* buffer
}

}  // namespace edge_detect
}  // namespace imgProc