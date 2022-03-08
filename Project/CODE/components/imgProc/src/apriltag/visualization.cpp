#include "apriltag/visualization.hpp"

#include <rtthread.h>

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include "apriltag/config.hpp"
#include "apriltag/internal/UnionBuffer.hpp"
#include "apriltag/internal/utility.hpp"

namespace imgProc {
namespace apriltag {

void show_grayscale(const uint8_t* img) {
    ips114_set_region(0, 0, M / 4 - 1, N / 4 - 1);
    rep(i, 0, N / 4) rep(j, 0, M / 4) {
        uint16_t cur = img[(i * M + j) * 4];
        uint16_t color = (0x001f & ((cur) >> 3)) << 11;
        color = color | (((0x003f) & ((cur) >> 2)) << 5);
        color = color | (0x001f & ((cur) >> 3));
        ips114_writedata_16bit(color);
    }
}

void show_threshim(const QuadImg_t& img) {
    ips114_set_region(0, 0, M / 4 - 1, N / 4 - 1);
    rep(i, 0, N / 4) rep(j, 0, M / 4) {
        uint16_t color;
        switch (img(i * 2, j * 2)) {
        case 0: color = BLACK; break;
        case 1: color = GRAY; break;
        case 2: color = RED; break;
        default: color = WHITE; break;
        }
        ips114_writedata_16bit(color);
    }
}

void plot(int_fast32_t i, int_fast32_t j, uint16_t color, int_fast32_t size) {
    if (!(0 <= i && i < N && 0 <= j && j < M)) return;
    i /= 4, j /= 4;
    auto il = max(0, i - size), ir = min(N / 4 - 1, i + size), jl = max(0, j - size), jr = min(M / 4 - 1, j + size);
    ips114_set_region(jl, il, jr, ir);
    req(u, il, ir) req(v, jl, jr) ips114_writedata_16bit(color);
}

inline uint8_t& imgIdx(uint8_t* img, int_fast32_t i, int_fast32_t j) { return img[i * M + j]; }
inline uint8_t imgIdx(const uint8_t* img, int_fast32_t i, int_fast32_t j) { return img[i * M + j]; }

constexpr uint8_t HEADER[4]{0x00, 0xff, 0x80, 0x7f};
inline void setHeader(uint8_t* img, int_fast32_t i, int_fast32_t j) { rep(t, 0, 4) imgIdx(img, i, j + t) = HEADER[t]; }
inline bool checkHeader(const uint8_t* img, int_fast32_t i, int_fast32_t j) {
    rep(t, 0, 4) if (imgIdx(img, i, j + t) != HEADER[t]) return false;
    return true;
}

void plotImg(uint8_t* img, int_fast32_t i, int_fast32_t j, uint16_t color, int_fast32_t size) {
    if (!(0 <= i && i < N && 0 <= j && j < M)) return;
    constexpr int_fast32_t mask = ~3;
    i &= mask, j &= mask;
    for (int_fast32_t u = max(0, i - size * 4); u < min(i + size * 4, N); u += 4)
        for (int_fast32_t v = max(0, j - size * 4); v < min(j + size * 4, M); v += 4) {
            rep(t, 0, 4) imgIdx(img, u, v + t) = HEADER[t];
            imgIdx(img, u + 1, v) = color >> 8;
            imgIdx(img, u + 1, v + 1) = color;
        }
}

void show_plot_grayscale(const uint8_t* img) {
    ips114_set_region(0, 0, M / 4 - 1, N / 4 - 1);
    rep(i, 0, N / 4) rep(j, 0, M / 4) {
        uint16_t color;
        if (checkHeader(img, i << 2, j << 2)) {
            color = ((uint16_t)imgIdx(img, (i << 2) + 1, j << 2) << 8) + imgIdx(img, (i << 2) + 1, (j << 2) + 1);
        } else {
            uint16_t cur = imgIdx(img, i << 2, j << 2);
            color = (0x001f & ((cur) >> 3)) << 11;
            color = color | (((0x003f) & ((cur) >> 2)) << 5);
            color = color | (0x001f & ((cur) >> 3));
        }
        ips114_writedata_16bit(color);
    }
}

void show_unionfind() {
    ips114_set_region(0, 0, M / 4 - 1, N / 4 - 1);
    for (int i = 0; i < (N / 4); ++i)
        for (int j = 0; j < (M / 4); ++j) {
            uint64_t cur = unionBuffer.segmentation.uf[(i * (M / quad_decimate) + j) * 2];
            cur *= int(1e9 + 7);
            cur &= 0xFFFF;
            ips114_writedata_16bit(cur);
        }
}

void show_cluster(const List_pt_t& cluster, uint16_t color, int32_t delay) {
    for (auto& p : cluster) {
        plot(p.y, p.x, color);
        if (delay) rt_thread_mdelay(delay);
    }
}

void show_clusters(const clusters_t& clusters) {
    uint64_t cur = 2333;
    for (auto& cluster : clusters) {
        cur *= int(1e9 + 7);
        cur &= 0xFFFF;
        show_cluster(*cluster, cur);
    }
}

void show_quads(quads_t& quads) {
    for (auto& quad : quads) rep(i, 0, 4) plot(quad.p[i][1], quad.p[i][0]);
}

}  // namespace apriltag
}  // namespace imgProc