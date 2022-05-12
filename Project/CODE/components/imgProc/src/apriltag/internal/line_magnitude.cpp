#include "apriltag/internal/line_magnitude.hpp"

#include <rtthread.h>

#include "RectConfig.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/decode_quad.hpp"
#include "apriltag/internal/utility.hpp"
#include "bresenham.hpp"
namespace imgProc {
namespace apriltag {

float_t pixel_magnitude(const uint8_t* img, int x, int y) {
    int pixel;
    int x_acc = 0, y_acc = 0;

    const uint8_t* row_ptr = img + max(0, y - 1) * M;

    // (-1,-1)
    pixel = *(row_ptr + max(0, x - 1));
    x_acc += pixel;
    y_acc += pixel;

    // (0,-1)
    pixel = *(row_ptr + x);
    y_acc += pixel << 1;

    // (1,-1)
    pixel = *(row_ptr + min(M - 1, x + 1));
    x_acc -= pixel;
    y_acc += pixel;

    if (y) row_ptr += M;

    // (-1,0)
    pixel = *(row_ptr + max(0, x - 1));
    x_acc += pixel << 1;

    // (1,0)
    pixel = *(row_ptr + min(M - 1, x + 1));
    x_acc -= pixel << 1;

    if (y < N - 1) row_ptr += M;

    // (-1,1)
    pixel = *(row_ptr + max(0, x - 1));
    x_acc += pixel;
    y_acc -= pixel;

    // (0,1)
    pixel = *(row_ptr + x);
    y_acc -= pixel << 1;

    // (1,1)
    pixel = *(row_ptr + min(M - 1, x + 1));
    x_acc -= pixel;
    y_acc -= pixel;

    return sqrtf((x_acc * x_acc) + (y_acc * y_acc));
}

static inline float_t line_magnitude(uint8_t* img, int x1, int y1, int x2, int y2) {
    float res = 0;
    drawLine(x1, y1, x2, y2, [&](int x, int y) { res += pixel_magnitude(img, x, y); });
    return res;
}

rects_t& rects_magnitude(uint8_t* img, quads_t& quads) {
    rects_t& rects = *new (staticBuffer.allocate(sizeof(rects_t))) rects_t(rects_alloc_t{staticBuffer});
    for (auto& quad : quads) {
        refine_edges(img, &quad);
        float_t magnitude = 0;
        rep(i, 0, 4) magnitude += line_magnitude(img, quad.p[i][0], quad.p[i][1], quad.p[(i + 1) & 3][0], quad.p[(i + 1) & 3][1]);
        if (magnitude < min_magnitude) continue;
        rects.push_front(new (staticBuffer.allocate(sizeof(rect))) rect);
        auto& r = *rects.front();
        rt_memcpy(r.p[0], quad.p[0], sizeof(r.p));
        r.magnitude = magnitude;
    }
    return rects;
}

}  // namespace apriltag
}  // namespace imgProc