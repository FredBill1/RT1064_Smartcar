#ifndef _apriltag_pose_hpp
#define _apriltag_pose_hpp

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

struct apriltag_detection_info {
    apriltag_detection* det;
    float_t tagsize;  // In meters.
    float_t fx;       // In pixels.
    float_t fy;       // In pixels.
    float_t cx;       // In pixels.
    float_t cy;       // In pixels.
};

struct apriltag_pose {
    float_t R[3][3];
    float_t t[3];
};

void estimate_pose_for_tag_homography(apriltag_detection_info& info, apriltag_pose& solution);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_pose_hpp