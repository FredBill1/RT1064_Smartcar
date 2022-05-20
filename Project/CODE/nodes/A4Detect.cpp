#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
#include "zf_gpio.h"
}

#include <utility>

#include "apriltag/fmath.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/fit_quad.hpp"
#include "apriltag/visualization.hpp"
#include "bresenham.hpp"
#include "devices.hpp"
#include "edge_detect/canny.hpp"
#include "edge_detect/show_edge.hpp"

namespace imgProc {
using namespace apriltag;
namespace edge_detect {
using std::pair;

#define PIXEL(img, x, y) (*(img + (y)*M + (x)))
#define DRAW(img, x, y) (*(img + ((y) & ~3) * M + ((x) & ~3)))
#define CIRCLE(img, x, y, r, color)                                                            \
    drawCircle((y) >> 2, (x) >> 2, (r), [img](int i, int j) {                                  \
        if (0 <= i && i < N / 4 && 0 <= j && j < M / 4) *(img + ((i * M + j) << 2)) = (color); \
    })
static inline pair<int, int> find_farthest(uint8_t* img, bool visualize = false) {
    constexpr float start_dist = 50;

    int res_i = -1, res_j = -1;
    int res_dist2 = 0;
    for (int degree = 0; degree < 360; degree += 10) {
        float x = M / 2, y = N / 2;
        float rad = degree * (3.14159265f / 180);
        float dx = cosf(rad), dy = sinf(rad);
        x += dx * start_dist, y += dy * start_dist;
        int i, j, dist2;
        for (;;) {
            x += dx, y += dy;
            i = y, j = x;
            if (!(0 <= i && i < N && 0 <= j && j < M)) break;
            if (*(img + (i * M + j)) == 255) {
                dist2 = (i - N / 2) * (i - N / 2) + (j - M / 2) * (j - M / 2);
                if (dist2 > res_dist2) {
                    res_dist2 = dist2;
                    res_i = i, res_j = j;
                }
                break;
            }
            if (visualize) *(img + ((i & ~3) * M + (j & ~3))) = 2;  // RED
        }
    }
    if (visualize) CIRCLE(img, res_j, res_i, 6, 3);
    return {res_i, res_j};
}

// 王福生,齐国清.二值图像中目标物体轮廓的边界跟踪算法[J].大连海事大学学报,2006(01):62-64+67.DOI:10.16411/j.cnki.issn1006-7736.2006.01.017.
static inline pair<Coordinate*, int> edgeTrace(uint8_t* img, int X, int Y) {
    constexpr int dxy[8][2]{{1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}};
    Coordinate* coords = (Coordinate*)staticBuffer.peek();
    int n = 0, x = X, y = Y;
    for (;;) {
        coords[n].x = x, coords[n].y = y, ++n;
        PIXEL(img, x, y) = 0;
        for (int t = 0, dir = int((1.125f - atan2f(y - N / 2, x - M / 2) / PI_f) * 4) & 7; t < 8; ++t, dir = (dir + 1) & 7) {
            int u = x + dxy[dir][0], v = y + dxy[dir][1];
            if (u == 0 || u == M - 1 || v == 0 || v == N - 1) return {nullptr, 0};
            if (PIXEL(img, u, v) == 255) {
                x = u, y = v;
                goto edgeTrace_success;
            }
        }
        break;
    edgeTrace_success:;
    }
    staticBuffer.allocate(n * sizeof(Coordinate));
    for (int i = 0; i < n; ++i) DRAW(img, coords[i].x, coords[i].y) = 2;
    return {coords, n};
}

static void A4DetectEntry() {
    int32_t pre_time = rt_tick_get();

    for (;;) {
        bool enabled = slave_switch[2].get();  // 拨码开关决定是否进行可视化，因为可视化会消耗时间

        staticBuffer.reset();

        uint8_t* img = mt9v03x_csi_image_take();
        if (!enabled) {
            show_grayscale(img);
            continue;
        }

        canny(img, 50, 100);  // 边缘检测

        auto [y, x] = find_farthest(img);  // 找到最远的点
        if (y != -1) {
            auto [coords, sz] = edgeTrace(img, x, y);

            ips114_showint32(188, 1, sz, 4);
        }

        show_edge(img);  // 显示边缘图片

        mt9v03x_csi_image_release();  // 释放图片

        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // 显示耗时/ms
        pre_time = cur_time;
    }
}
}  // namespace edge_detect
}  // namespace imgProc

bool A4DetectNode() { return FuncThread(imgProc::edge_detect::A4DetectEntry, "A4Detect", 4096, 2, 1000); }
