#include <cstdint>

#include "apriltag/config.hpp"
namespace imgProc {
namespace apriltag {
constexpr int16_t mapxI[]{
#include "mapxI.txt"
};
constexpr int16_t mapyI[]{
#include "mapyI.txt"
};

void undisort_I(const uint8_t* src, uint8_t* dst) {
    auto *x = mapxI, *y = mapyI;
    for (int i = 0; i < N * M; ++i) *dst++ = *(src + *y++ * M + *x++);
}

}  // namespace apriltag
}  // namespace imgProc