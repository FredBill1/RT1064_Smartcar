#ifndef _apriltag_Unionfind_hpp
#define _apriltag_Unionfind_hpp

#include <cstdint>

namespace imgProc {
namespace apriltag {

template <typename T, int_fast32_t Size> class Unionfind {
 public:
    using data_t = T;

 protected:
    data_t fa[Size];
    int32_t sz[Size];

 public:
    inline void reset() {
        for (int_fast32_t i = 0; i < Size; ++i) sz[i] = 1, fa[i] = i;
    }
    inline data_t find(data_t x) {
        if (fa[x] == x) return x;
        data_t t = x;
        while (t != fa[t]) t = fa[t];
        while (x != fa[x]) {
            data_t u = fa[x];
            fa[x] = t;
            x = u;
        }
        return x;
    }
    inline void merge(data_t x, data_t y) {
        data_t xr = find(x), yr = find(y);
        if (xr == yr) return;
        fa[xr] = yr;
        sz[yr] += sz[xr];
    }
    inline data_t operator[](data_t x) { return find(x); }
    inline int32_t size(data_t x) { return sz[find(x)]; }
};

}  // namespace apriltag

}  // namespace imgProc

#endif  // _apriltag_Unionfind_hpp