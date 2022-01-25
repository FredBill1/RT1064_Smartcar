#include "apriltag/internal/segmentation.hpp"

#include <rtthread.h>

#include <algorithm>

#include "apriltag/internal/Hashmap.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/UnionBuffer.hpp"
#include "apriltag/internal/utility.hpp"

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

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
            int_fast32_t r0 = uf[y * M + x], r1 = uf[y1 * M + x1];
            if (r0 > r1) std::swap(r0, r1);
            List_pt_t*& list = dict[hashPt2(r0, r1)];
            if (!list) list = new (staticBuffer.allocate(sizeof(List_pt_t))) List_pt_t(List_pt_alloc_t{staticBuffer});
            int_fast32_t dif = ((int_fast32_t)v1 - v0) / 3 * 255;
            list->push_front({uint16_t(2 * x + dx), uint16_t(2 * y + dy), int16_t(dx * dif), int16_t(dy * dif), 0.f});
        }
    }
    clusters_t* clusters = new (staticBuffer.allocate(sizeof(clusters_t))) clusters_t(clusters_alloc_t{staticBuffer});
    dict.for_each([clusters](List_pt_t*& list) { clusters->push_front(list); });
    return clusters;
}

void show_unionfind() {
    ips114_set_region(0, 0, M - 1, N - 1);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) {
            uint64_t cur = unionBuffer.segmentation.uf[i * M + j];
            cur *= int(1e9 + 7);
            cur &= 0xFFFF;
            ips114_writedata_16bit(cur);
        }
}

void show_clusters(const clusters_t& clusters) {
    uint64_t cur = 2333;
    for (auto& cluster : clusters) {
        cur *= int(1e9 + 7);
        cur &= 0xFFFF;
        for (auto& p : *cluster) {
            int i = p.y / 2, j = p.x / 2;
            ips114_set_region(max(0, j - 2), max(0, i - 2), min(M - 1, j + 2), min(N - 1, i + 2));
            rep(u, max(0, j - 2), min(M - 1, j + 2)) rep(v, max(0, i - 2), min(N - 1, i + 2)) ips114_writedata_16bit(cur);
        }
    }
}

}  // namespace apriltag
}  // namespace imgProc