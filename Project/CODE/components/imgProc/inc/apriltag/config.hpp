#ifndef _apriltag_config_hpp
#define _apriltag_config_hpp

#include <cstdint>

namespace imgProc {
namespace apriltag {

constexpr int_fast32_t N = 120;  // image height
constexpr int_fast32_t M = 188;  // image width

constexpr int_fast32_t STATICBUFFER_SIZE = 1024 * 1024 * 10;

// Thresholding
constexpr int_fast32_t TILESZ = 4;
constexpr int_fast32_t TN = N / TILESZ, TM = M / TILESZ;
constexpr int_fast32_t THRESH_MIN_DIFF = 5;

// Segmentation
constexpr int_fast32_t HASH_BUCKET_CNT = N * M + 1;
constexpr int_fast32_t CLUSTER_MIN_SIZE = 25;

// fit quad
constexpr int_fast32_t max_nmaxima = 10;  // how many corner candidates to consider when segmenting a group of pixels into a quad.
constexpr float max_line_fit_mse = 10.0f;
constexpr float cos_critical_rad = 0.98480775301220802032;

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_config_hpp