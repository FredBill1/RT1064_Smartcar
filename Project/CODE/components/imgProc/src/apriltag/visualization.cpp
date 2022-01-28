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
        plot(p.y / 2, p.x / 2, color);
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