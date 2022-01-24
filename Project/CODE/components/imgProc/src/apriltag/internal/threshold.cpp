#include "apriltag/internal/threshold.hpp"

#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/UnionBuffer.hpp"
#include "apriltag/internal/utility.hpp"

namespace imgProc {
namespace apriltag {

void threshold(uint8_t* src, QuadImg_t& dst) {
    auto& im_max = unionBuffer.threshold.im_max;
    auto& im_min = unionBuffer.threshold.im_min;
    auto& im_max2 = unionBuffer.threshold.im_max2;
    auto& im_min2 = unionBuffer.threshold.im_min2;

    rep(i, 0, TN) rep(j, 0, TM) {
        im_max[i][j] = 0, im_min[i][j] = 255;
        rep(u, 0, TILESZ) rep(v, 0, TILESZ) {
            uint8_t val = src[(i * TILESZ + u) * M + j * TILESZ + v];
            chkmax(im_max[i][j], val), chkmin(im_min[i][j], val);
        }
    }

    rep(i, 0, TN) rep(j, 0, TM) {
        im_max2[i][j] = 0, im_min2[i][j] = 255;
        req(u, max(0, i - 1), min(TN - 1, i + 1)) req(v, max(0, j - 1), min(TM - 1, j + 1)) chkmax(im_max2[i][j], im_max[u][v]),
            chkmin(im_min2[i][j], im_min[u][v]);
    }

    rep(i, 0, TN) rep(j, 0, TM) {
        if (im_max2[i][j] - im_min2[i][j] < THRESH_MIN_DIFF)
            rep(u, 0, TILESZ) rep(v, 0, TILESZ) dst.set(i * TILESZ + u, j * TILESZ + v, 1);
        else {
            int_fast16_t thresh = ((int_fast16_t)im_max2[i][j] + im_min2[i][j]) >> 1;
            rep(u, 0, TILESZ) rep(v, 0, TILESZ)
                dst.set(i * TILESZ + u, j * TILESZ + v, src[(i * TILESZ + u) * M + j * TILESZ + v] > thresh ? 3 : 0);
        }
    }
}

}  // namespace apriltag
}  // namespace imgProc