#ifndef _apriltag_visualization_pose_cpp
#define _apriltag_visualization_pose_cpp

#include "apriltag/apriltag_pose.hpp"

namespace imgProc {
namespace apriltag {

void plot_pose_axis(uint8_t* img, const apriltag_detection_info& info, const apriltag_pose& pose);

void plot_pose_cube(uint8_t* img, const apriltag_detection_info& info, const apriltag_pose& pose, bool plotID = true);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_visualization_pose_cpp