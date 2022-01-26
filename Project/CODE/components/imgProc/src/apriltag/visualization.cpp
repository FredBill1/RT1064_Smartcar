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

void plot(int_fast32_t i, int_fast32_t j, uint16_t color) {
    if (!(0 <= i && i < N && 0 <= j && j < M)) return;
    auto il = max(0, i - 2), ir = min(N - 1, i + 2), jl = max(0, j - 2), jr = min(M - 1, j + 2);
    ips114_set_region(jl, il, jr, ir);
    req(u, il, ir) req(v, jl, jr) ips114_writedata_16bit(color);
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