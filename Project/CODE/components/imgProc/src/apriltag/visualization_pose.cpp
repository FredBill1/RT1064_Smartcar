#include "apriltag/visualization_pose.hpp"

#include "apriltag/internal/homography.hpp"
#include "apriltag/visualization.hpp"

namespace imgProc {
namespace apriltag {

void plot_pose_axis(uint8_t* img, const apriltag_detection_info& info, const apriltag_pose& pose) {
    float_t p0[2], p1[2];
    float_t O[3]{};
    tag_pose_to_image(info, pose, O, p0);
    constexpr uint16_t colors[3]{0xF800, 0x07E0, 0x001F};  // RGB
    for (int i = 0; i < 3; ++i) {
        float_t v[3]{}, dst[3];
        v[i] = info.tagsize;
        tag_pose_to_image(info, pose, v, p1);
        lineImg(img, p0[1], p0[0], p1[1], p1[0], colors[i]);
    }
}

}  // namespace apriltag
}  // namespace imgProc
