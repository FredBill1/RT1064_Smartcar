#ifndef _BinaryImg_hpp
#define _BinaryImg_hpp

#include <cstdint>

namespace imgProc {

template <int_fast32_t Height, int_fast32_t Width> class BinaryImg {
 public:
    static constexpr int_fast32_t N = Height, M = Width;

 protected:
    using data_t = uint_fast8_t;
    static constexpr int_fast32_t SZ = sizeof(data_t);
    data_t data[(N * M + SZ - 1) / SZ];
    inline constexpr data_t idx(int_fast32_t i, int_fast32_t j) const { return i * M + j; }

 public:
    inline uint8_t operator()(int_fast32_t i, int_fast32_t j) const {
        return (data[idx(i, j) / SZ] & (1 << (idx(i, j) & (SZ - 1)))) ? 255 : 0;
    }
    inline void set(int_fast32_t i, int_fast32_t j, bool v) {
        if (v) {
            data[idx(i, j) / SZ] |= data_t(1) << (idx(i, j) & (SZ - 1));
        } else {
            data[idx(i, j) / SZ] &= ~(data_t(1) << (idx(i, j) & (SZ - 1)));
        }
    }
};

template <int_fast32_t Height, int_fast32_t Width> class QuadImg {
 public:
    static constexpr int_fast32_t N = Height, M = Width, Plot_sz = 3;

 protected:
    using data_t = uint_fast8_t;
    static constexpr int_fast32_t SZ = sizeof(data_t);
    data_t data[(N * M * 2 + SZ - 1) / SZ];
    inline constexpr data_t idx(int_fast32_t i, int_fast32_t j) const { return (i * M + j) << 1; }

 public:
    inline uint8_t operator()(int_fast32_t i, int_fast32_t j) const { return data[idx(i, j) / SZ] >> (idx(i, j) & (SZ - 1)) & 3; }
    inline void set(int_fast32_t i, int_fast32_t j, uint8_t v) {
        int_fast32_t I = idx(i, j) / SZ, J = idx(i, j) & (SZ - 1);
        data[I] &= ~(data_t(3) << J);
        data[I] |= data_t(v) << J;
    }
    inline void plot(int_fast32_t i, int_fast32_t j) {
        for (int_fast32_t u = (i - Plot_sz >= 0 ? i - Plot_sz : 0); u < (i + Plot_sz < N ? i + Plot_sz : N - 1); ++u)
            for (int_fast32_t v = (j - Plot_sz >= 0 ? j - Plot_sz : 0); v < (j + Plot_sz < M ? j + Plot_sz : M - 1); ++v)
                set(u, v, 2);
    }
};

}  // namespace imgProc

#endif  // _BinaryImg_hpp