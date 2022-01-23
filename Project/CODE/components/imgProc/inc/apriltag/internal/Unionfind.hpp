#ifndef _apriltag_Unionfind_hpp
#define _apriltag_Unionfind_hpp

#include <cstdint>

namespace imgProc {
namespace apriltag {

template <typename T, int_fast32_t Size> class Uinonfind {
 public:
    using data_t = T;
    static constexpr int_fast32_t size = Size;

 protected:
    data_t fa[Size];

 public:
    inline void reset() {
        for (int_fast32_t i = 0; i < Size; ++i) fa[i] = i;
    }
    inline data_t find(data_t x) {
        data_t t = x;
        while (t != fa[t]) t = fa[t];
        while (x != fa[x]) {
            data_t u = fa[x];
            fa[x] = t;
            x = u;
        }
        return x;
    }
    inline void merge(data_t x, data_t y) { fa[find(x)] = find(y); }
    inline data_t operator[](data_t x) { return find(x); }
};

}  // namespace apriltag

}  // namespace imgProc

#endif  // _apriltag_Unionfind_hpp