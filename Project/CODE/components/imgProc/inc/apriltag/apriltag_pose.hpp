#ifndef _apriltag_pose_hpp
#define _apriltag_pose_hpp

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

struct apriltag_detection_info {
    apriltag_detection* det;
    double tagsize;  // In meters.
    double fx;       // In pixels.
    double fy;       // In pixels.
    double cx;       // In pixels.
    double cy;       // In pixels.
};

struct apriltag_pose {
    double R[3][3];
    double t[3];
};

void estimate_pose_for_tag_homography(apriltag_detection_info& info, apriltag_pose& solution);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_pose_hpp