#ifndef _apriltag_visualization_pose_cpp
#define _apriltag_visualization_pose_cpp

#include "apriltag/apriltag_pose.hpp"

namespace imgProc {
namespace apriltag {

void plot_pose_axis(uint8_t* img, apriltag_detection_info& info, apriltag_pose& solution);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_visualization_pose_cpp