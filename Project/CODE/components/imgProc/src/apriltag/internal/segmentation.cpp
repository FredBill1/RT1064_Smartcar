#include "apriltag/internal/segmentation.hpp"

#include "apriltag/internal/utility.hpp"

namespace imgProc {
namespace apriltag {

void unionfind_connected(Unionfind_t& uf, const QuadImg_t& img) {
    uf.reset();
    rep(i, 0, N - 1) rep(j, 1, M) if (img(i, j) != 1) {
        const auto cur = img(i, j);
        if (img(i, j - 1) == cur) uf.merge(i * M + j, i * M + j - 1);
        if (i > 0 && (j == 0 || img(i, j - 1) != img(i - 1, j - 1) || img(i - 1, j - 1) != img(i, j - 1)) && img(i - 1, j) == cur)
            uf.merge(i * M + j, (i - 1) * M + j);
    }
}

}  // namespace apriltag
}  // namespace imgProc