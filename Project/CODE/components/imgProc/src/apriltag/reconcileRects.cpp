#include "apriltag/reconcileRects.hpp"

#include "RectConfig.hpp"
#include "apriltag/fmath.hpp"
#include "apriltag/internal/g2d.hpp"
#include "apriltag/internal/homography.hpp"
#include "utils/excuteOnce.hpp"

namespace imgProc {
namespace apriltag {

static inline bool checkRect(rect& r) {
    static float_t Cam2Base[3][3];
    excuteOnce(homography_compute2(Cam2Base, Cam2Base_corr));

    // 在0-180度之间, cos值随着角度增大而减小
    static const float_t min_rect_cos = cosf(max_rect_angle * PI_f / 180), max_rect_cos = cosf(min_rect_angle * PI_f / 180);
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

static inline bool rectIntersect(const rect& r1, const rect& r2) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float_t tmp[2];
            if (line_segment_intersection(r1.p_proj[i], r1.p_proj[(i + 1) & 3], r2.p_proj[j], r2.p_proj[(j + 1) & 3], tmp))
                return true;
        }
    if (polygon_contains_point(r1.p_proj, 4, r2.c_proj)) return true;
    if (polygon_contains_point(r2.p_proj, 4, r1.c_proj)) return true;
    return false;
}

void reconcileRects(rects_t& rects) {
    for (auto it = rects.before_begin();; ++it) {
        while (next(it) != rects.end() && !checkRect(**next(it))) rects.erase_after(it);
        if (next(it) == rects.end()) break;
    }
    rects.sort([](const rect* r1, const rect* r2) { return r1->magnitude > r2->magnitude; });
    for (auto front = rects.begin(); front != rects.end(); ++front) {
        for (decltype(front) it = front, cur; it != rects.end(); ++it) {
            if (cur = it; ++cur == rects.end()) break;
            if (rectIntersect(**cur, **front)) {
                // 上面以经按降序排列了, 所以可以直接删
                // if ((*cur)->magnitude > (*front)->magnitude) std::swap(*cur, *front);
                rects.erase_after(it);
            }
        }
    }
}

}  // namespace apriltag
}  // namespace imgProc
