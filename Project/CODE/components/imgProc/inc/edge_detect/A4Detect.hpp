#ifndef _imgProc_edge_detect_A4Detect_hpp
#define _imgProc_edge_detect_A4Detect_hpp

#include <cstdint>

#include "imgProc/common.hpp"

namespace imgProc {
namespace edge_detect {

constexpr int target_coords_maxn = 20;
extern Coordinate target_coords[target_coords_maxn];
extern int target_coords_cnt;

bool A4Detect(uint8_t* img, int low_thresh = 50, int high_thresh = 100);

}  // namespace edge_detect
}  // namespace imgProc

#endif  // _imgProc_edge_detect_A4Detect_hpp