#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
}

#include <cmath>

#include "apriltag/apriltag.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/homography.hpp"
#include "apriltag/undisort.hpp"
#include "apriltag/visualization.hpp"
#include "devices.hpp"
//
#include "RectConfig.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

namespace imgProc {
namespace apriltag {

// https://en.wikipedia.org/wiki/Line–line_intersection
// p1.....p4
// ..\.../..
// ...res...
// ../...\..
// p3.....p2
static inline bool line_intersection(const float_t p1[2], const float_t p2[2], const float_t p3[2], const float_t p4[2],
                                     float_t res[2]) {
    float_t x1x2 = p1[0] - p2[0], x3x4 = p3[0] - p4[0], y1y2 = p1[1] - p2[1], y3y4 = p3[1] - p4[1];
    float_t D = (x1x2) * (y3y4) - (y1y2) * (x3x4);
    if (D == 0) return false;
    float_t x1y2y1x2 = p1[0] * p2[1] - p1[1] * p2[0], x3y4y3x4 = p3[0] * p4[1] - p3[1] * p4[0];
    res[0] = (x1y2y1x2 * x3x4 - x1x2 * x3y4y3x4) / D;
    res[1] = (x1y2y1x2 * y3y4 - y1y2 * x3y4y3x4) / D;
    return true;
}

static inline float_t vector_cos(float_t x1, float_t y1, float_t x2, float_t y2) {
    return (x1 * x2 + y1 * y2) / (sqrtf(x1 * x1 + y1 * y1) * sqrtf(x2 * x2 + y2 * y2));
}

static float_t Cam2Base[3][3];

static inline bool checkRect(rect& r) {
    // 在0-180度之间, cos值随着角度增大而减小
    static const float_t min_rect_cos = std::cos(max_rect_angle * M_PI / 180),
                         max_rect_cos = std::cos(min_rect_angle * M_PI / 180);
    for (int i = 0; i < 4; ++i) homography_project(Cam2Base, r.p[i][0], r.p[i][1], r.p_proj[i], r.p_proj[i] + 1);
    for (int i = 0; i < 4; ++i) {
        float_t dx = r.p_proj[i][0] - r.p_proj[(i + 1) & 3][0], dy = r.p_proj[i][1] - r.p_proj[(i + 1) & 3][1];
        float_t d2 = dx * dx + dy * dy;
        if (d2 < min_rect_size * min_rect_size || d2 > max_rect_size * max_rect_size) return false;
        float_t c = vector_cos(r.p_proj[(i + 1) & 3][0] - r.p_proj[i][0], r.p_proj[(i + 1) & 3][1] - r.p_proj[i][1],
                               r.p_proj[(i + 3) & 3][0] - r.p_proj[i][0], r.p_proj[(i + 3) & 3][1] - r.p_proj[i][1]);
        if (c < min_rect_cos || c > max_rect_cos) return false;
    }
    if (!line_intersection(r.p_proj[0], r.p_proj[2], r.p_proj[1], r.p_proj[3], r.c_proj)) return false;
    return true;
}

static inline void filterRects(rects_t& rects) {
    for (auto it = rects.before_begin();; ++it) {
        while (next(it) != rects.end() && !checkRect(**next(it))) rects.erase_after(it);
        if (next(it) == rects.end()) break;
    }
}

static void testFindRectEntry() {
    // AT_DTCM_SECTION_ALIGN(static uint8_t img[N * M], 64);

    homography_compute2(Cam2Base, Cam2Base_corr);

    int32_t pre_time = rt_tick_get();

    for (;;) {
        bool visualize = slave_switch[2].get();  // 拨码开关决定是否进行可视化，因为可视化会消耗时间

        // uint8_t* src = mt9v03x_csi_image_take();
        // undisort_I(src, img);  // 矫正图像畸变
        uint8_t* img = mt9v03x_csi_image_take();

        rects_t& rects = find_rects(img, min_magnitude);
        if (visualize) plot_rects(img, rects, GREEN);

        filterRects(rects);
        if (visualize) plot_rects(img, rects, RED);

        if (visualize) show_plot_grayscale(img);

        mt9v03x_csi_image_release();  // 释放图片

        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // 显示耗时/ms
        pre_time = cur_time;
    }
}
}  // namespace apriltag
}  // namespace imgProc

bool testFindRectNode() { return FuncThread(imgProc::apriltag::testFindRectEntry, "testFindRect", 4096, 2, 1000); }
