#ifndef _apriltag_config_hpp
#define _apriltag_config_hpp

#include <cstdint>

namespace imgProc {
namespace apriltag {

constexpr int_fast32_t N = 480;  // image height
constexpr int_fast32_t M = 752;  // image width

// Thresholding
constexpr int_fast32_t TILESZ = 4;
constexpr int_fast32_t TN = N / TILESZ, TM = M / TILESZ;
constexpr int_fast32_t THRESH_MIN_DIFF = 5;

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_config_hpp