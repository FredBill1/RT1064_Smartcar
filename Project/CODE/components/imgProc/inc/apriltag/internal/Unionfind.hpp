#ifndef _apriltag_Unionfind_hpp
#define _apriltag_Unionfind_hpp

#include <algorithm>
#include <cstdint>

namespace imgProc {
namespace apriltag {

template <typename T, int_fast32_t Size> class Unionfind {
 public:
    using data_t = T;

 protected:
    struct Node {
        data_t fa;
        int32_t sz;
    } node[Size];

 public:
    inline void reset() {
        for (int_fast32_t i = 0; i < Size; ++i) node[i].sz = 1, node[i].fa = i;
    }
    inline data_t find(data_t x) {
        if (node[x].fa == x) return x;
        data_t t = x;
        while (t != node[t].fa) t = node[t].fa;
        while (x != node[x].fa) {
            data_t u = node[x].fa;
            node[x].fa = t;
            x = u;
        }
        return x;
    }
    inline void merge(data_t x, data_t y) {
        data_t xr = find(x), yr = find(y);
        if (xr == yr) return;
        if (node[xr].sz > node[yr].sz) std::swap(xr, yr);
        node[xr].fa = yr;
        node[yr].sz += node[xr].sz;
    }
    inline data_t operator[](data_t x) { return find(x); }
    inline int32_t size(data_t x) { return node[find(x)].sz; }
};

}  // namespace apriltag

}  // namespace imgProc

#endif  // _apriltag_Unionfind_hpp