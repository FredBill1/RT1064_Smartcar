#include "apriltag/internal/segmentation.hpp"

#include <rtthread.h>

#include <algorithm>

#include "apriltag/internal/Hashmap.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/UnionBuffer.hpp"
#include "apriltag/internal/utility.hpp"

namespace imgProc {
namespace apriltag {

#define DO_UNIONFIND(dx, dy) \
    if (img(y + dy, x + dx) == v) uf.merge(y* M + x, (y + dy) * M + x + dx);
inline void do_unionfind_first_line(const QuadImg_t& img) {
    Unionfind_t& uf = unionBuffer.segmentation.uf;
    constexpr int_fast32_t y = 0;
    uint8_t v;
    for (int_fast32_t x = 1; x < M - 1; x++) {
        v = img(y, x);
        if (v == 1) continue;
        DO_UNIONFIND(-1, 0);
    }
}
inline void do_unionfind_line(const QuadImg_t& img, int_fast32_t y) {
    Unionfind_t& uf = unionBuffer.segmentation.uf;
    uint8_t v_m1_m1, v_0_m1 = img(y - 1, 0), v_1_m1 = img(y - 1, 1), v_m1_0, v = img(y, 0);
    for (int_fast32_t x = 1; x < M - 1; x++) {
        v_m1_m1 = v_0_m1, v_0_m1 = v_1_m1, v_1_m1 = img(y - 1, x + 1), v_m1_0 = v, v = img(y, x);
        if (v == 1) continue;
        DO_UNIONFIND(-1, 0);
        if (x == 1 || !((v_m1_0 == v_m1_m1) && (v_m1_m1 == v_0_m1))) { DO_UNIONFIND(0, -1); }
        if (v == 3) {
            if (x == 1 || !(v_m1_0 == v_m1_m1 || v_0_m1 == v_m1_m1)) { DO_UNIONFIND(-1, -1); }
            if (!(v_0_m1 == v_1_m1)) { DO_UNIONFIND(1, -1); }
        }
    }
}
#undef DO_UNIONFIND
void unionfind_connected(const QuadImg_t& img) {
    Unionfind_t& uf = unionBuffer.segmentation.uf;
    uf.reset();
    do_unionfind_first_line(img);
    for (int_fast32_t y = 1; y < N; ++y) do_unionfind_line(img, y);

    // rep(i, 0, N - 1) rep(j, 1, M) if (img(i, j) != 1) {
    //     const auto cur = img(i, j);
    //     if (img(i, j - 1) == cur) uf.merge(i * M + j, i * M + j - 1);
    //     if ((j == 1 || !(img(i, j - 1) == img(i - 1, j - 1) && img(i - 1, j - 1) == img(i - 1, j))) && img(i - 1, j) == cur)
    //         uf.merge(i * M + j, (i - 1) * M + j);
    //     if (cur == 3) {}
    //     if (i > 0 && (j == 0 || img(i, j - 1) != img(i - 1, j - 1) || img(i - 1, j - 1) != img(i, j - 1)) && img(i - 1, j) ==
    //     cur)
    //         uf.merge(i * M + j, (i - 1) * M + j);
    // }
}

inline ID_t hashPt2(uint64_t x, uint64_t y) { return (((x << 32) + y) * 2654435761) >> 32; }

clusters_t* gradient_clusters(const QuadImg_t& img) {
    Unionfind_t& uf = unionBuffer.segmentation.uf;

    constexpr int_fast32_t dxy[][2]{{1, 0}, {0, 1}, {-1, 1}, {1, 1}};
    staticBuffer.reset();
    Hashmap& dict = Hashmap::create(unionBuffer.segmentation.hashmapbuf, staticBuffer);
    rep(y, 1, N - 1) rep(x, 1, M - 1) {
        const auto v0 = img(y, x);
        if (v0 == 1 || uf.size(y * M + x) < CLUSTER_MIN_SIZE) continue;
        for (auto [dx, dy] : dxy) {
            int_fast32_t x1 = x + dx, y1 = y + dy;
            uint8_t v1 = img(y1, x1);
            if (v0 + v1 != 3 || uf.size(y1 * M + x1) < CLUSTER_MIN_SIZE) continue;
            auto [a, b] = std::minmax(uf[y * M + x], uf[y1 * M + x1]);
            ID_t id = hashPt2(a, b);
            List_pt_t*& list = dict[id];
            if (!list) list = new (staticBuffer.allocate(sizeof(List_pt_t))) List_pt_t(List_pt_alloc_t{staticBuffer});
            if (staticBuffer.overflow()) {
                rt_kprintf("[%s] staticBuffer overflow\r\n", __func__);
                RT_ASSERT(0);
            }
            list->push_front({uint16_t(2 * x + dx), uint16_t(2 * y + dy), int16_t(dx * ((int32_t)v1 - v0)),
                              int16_t(dy * ((int_fast32_t)v1 - v0)), 0.f});
        }
    }
    clusters_t* clusters = new (staticBuffer.allocate(sizeof(clusters_t))) clusters_t(clusters_alloc_t{staticBuffer});
    dict.for_each([clusters](List_pt_t*& list) { clusters->emplace_front(list), list = nullptr; });
    rt_kprintf("used: %d\r\n", staticBuffer.usage());
    return clusters;
}

}  // namespace apriltag
}  // namespace imgProc