#ifndef _apriltag_homography_hpp
#define _apriltag_homography_hpp

#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

void homography_compute2(float_t dst[3][3], float_t c[4][4]);

void homography_project(const float_t H[3][3], float_t x, float_t y, float_t *ox, float_t *oy);

void homography_to_pose(const float_t H[3][3], float_t fx, float_t fy, float_t cx, float_t cy, float_t R[3][3], float_t t[3]);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_homography_hpp