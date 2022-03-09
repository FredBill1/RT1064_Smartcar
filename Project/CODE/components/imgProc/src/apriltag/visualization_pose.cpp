#include "apriltag/visualization_pose.hpp"

#include "apriltag/internal/homography.hpp"
#include "apriltag/visualization.hpp"

namespace imgProc {
namespace apriltag {

void plot_pose_axis(uint8_t* img, apriltag_detection_info& info, apriltag_pose& solution) {
    float x0, y0, x1, y1;
    camera_to_image(info.fx, info.fy, info.cx, info.cy, solution.t[0], solution.t[1], solution.t[2], &x0, &y0);
    constexpr uint16_t colors[3]{0xF800, 0x07E0, 0x001F};
    for (int i = 0; i < 3; ++i) {
        float_t v[3]{}, dst[3];
        v[i] = info.tagsize;
        matrix_transform(solution.R[0], solution.t, v, dst);
        camera_to_image(info.fx, info.fy, info.cx, info.cy, dst[0], dst[1], dst[2], &x1, &y1);
        lineImg(img, y0, x0, y1, x1, colors[i]);
    }
}

}  // namespace apriltag
}  // namespace imgProc
