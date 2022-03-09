#include "apriltag/visualization.hpp"

#include <rtthread.h>

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include "apriltag/config.hpp"
#include "apriltag/internal/globalVariables.hpp"
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
    int_fast32_t ur = min(N - 1, i + size * 4), vr = min(M - 1, j + size * 4);
    for (int_fast32_t u = max(0, i - size * 4); u <= ur; u += 4)
        for (int_fast32_t v = max(0, j - size * 4); v <= vr; v += 4) {
            rep(t, 0, 4) imgIdx(img, u, v + t) = HEADER[t];
            imgIdx(img, u + 1, v) = color >> 8;
            imgIdx(img, u + 1, v + 1) = color;
        }
}

void lineImg(uint8_t* img, int_fast32_t i0, int_fast32_t j0, int_fast32_t i1, int_fast32_t j1, uint16_t color,
             int_fast32_t size) {
    int_fast32_t dx = abs(i1 - i0), sx = i0 < i1 ? 1 : -1, dy = -abs(j1 - j0), sy = j0 < j1 ? 1 : -1, err = dx + dy;
    for (;;) {
        plotImg(img, i0, j0, color, size);
        if (i0 == i1 && j0 == j1) break;
        int_fast32_t e2 = 2 * err;
        if (e2 >= dy) err += dy, i0 += sx;
        if (e2 <= dx) err += dx, j0 += sy;
    }
}

void plot_tag_det(uint8_t* img, apriltag_detection& det) {
    for (int i = 0; i < 4; i++) lineImg(img, det.p[(i + 1) & 3][1], det.p[(i + 1) & 3][0], det.p[i][1], det.p[i][0], RED);
    plotImg(img, det.p[0][1], det.p[0][0], BLUE, 2);
    plotInt(img, det.c[1], det.c[0], det.id, 2, true);
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

void plotChar(uint8_t* img, int_fast32_t i, int_fast32_t j, char dat) {
    dat -= 32;
    rep(di, 0, 16) {
        auto tmp = tft_ascii[dat][di];
        rep(dj, 0, 8) {
            uint16_t color = ((tmp >> dj) & 1) ? IPS114_PENCOLOR : IPS114_BGCOLOR;
            plotImg(img, i + (di << 2), j + (dj << 2), color, 0);
        }
    }
}

void plotInt(uint8_t* img, int_fast32_t i, int_fast32_t j, int32_t dat, int_fast32_t len, bool atCenter) {
    if (len <= 0) return;
    if (atCenter) j -= len * 8 << 1, i -= 16 << 1;
    if (dat < 0) plotChar(img, i, j, '-'), j += 8 << 2, dat = -dat, --len;
    j += len * 8 << 2;
    while (len--) j -= 8 << 2, plotChar(img, i, j, '0' + (dat % 10)), dat /= 10;
}

void show_unionfind() {
    ips114_set_region(0, 0, M / 4 - 1, N / 4 - 1);
    for (int i = 0; i < (N / 4); ++i)
        for (int j = 0; j < (M / 4); ++j) {
            uint64_t cur = uf[(i * (M / quad_decimate) + j) * 2];
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

void show_clusterImg(uint8_t* img, const List_pt_t& cluster, uint16_t color, int32_t delay) {
    for (auto& p : cluster) {
        plotImg(img, p.y, p.x, color);
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
void show_clustersImg(uint8_t* img, const clusters_t& clusters) {
    uint64_t cur = 2333;
    for (auto& cluster : clusters) {
        cur *= int(1e9 + 7);
        cur &= 0xFFFF;
        show_clusterImg(img, *cluster, cur);
    }
}

void show_quads(quads_t& quads) {
    for (auto& quad : quads) rep(i, 0, 4) plot(quad.p[i][1], quad.p[i][0]);
}
void show_quadsImg(uint8_t* img, quads_t& quads) {
    for (auto& quad : quads) rep(i, 0, 4) plotImg(img, quad.p[i][1], quad.p[i][0]);
}
}  // namespace apriltag
}  // namespace imgProc