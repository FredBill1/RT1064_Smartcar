#ifndef _imgProc_edge_detect_A4Detect_hpp
#define _imgProc_edge_detect_A4Detect_hpp

#include <cstdint>

#include "apriltag/apriltag.hpp"
#include "imgProc/common.hpp"

namespace imgProc {
namespace edge_detect {

constexpr int target_coords_maxn = 24;
extern apriltag::quad target_quad;
extern Coordinate target_coords[target_coords_maxn];
extern int target_coords_cnt;
extern apriltag::float_t target_coords_corr[target_coords_maxn][2];

bool A4Detect(uint8_t* img, apriltag::float_t borderWidth = 7, apriltag::float_t borderHeight = 5, int low_thresh = 50,
              int high_thresh = 100);

void draw_corr(apriltag::float_t target_coords_corr[][2], int target_coords_cnt, apriltag::float_t borderWidth = 7,
               apriltag::float_t borderHeight = 5, uint16_t color = 0xF800);

}  // namespace edge_detect
}  // namespace imgProc

#endif  // _imgProc_edge_detect_A4Detect_hpp