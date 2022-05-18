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

static inline pair<int, int> find_farthest(uint8_t* img, bool visualize = false) {
    constexpr float start_dist = 50;

    int res_i = N / 2, res_j = M / 2;
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
    if (visualize)
        drawCircle(res_i / 4, res_j / 4, 6, [img](int i, int j) {
            *(img + (i * M + j) * 4) = 3;  // BLUE
        });
    return {res_i, res_j};
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

        find_farthest(img, true);  // 找到最远的点

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
