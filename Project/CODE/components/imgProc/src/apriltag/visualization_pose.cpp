#include "apriltag/visualization_pose.hpp"

#include "apriltag/internal/homography.hpp"
#include "apriltag/internal/utility.hpp"
#include "apriltag/visualization.hpp"

namespace imgProc {
namespace apriltag {

constexpr uint16_t colors[]{0xF800, 0x07E0, 0x001F};  // RGB

void plot_pose_axis(uint8_t* img, const apriltag_detection_info& info, const apriltag_pose& pose) {
    float_t p0[2], p1[2];
    float_t O[3]{};
    tag_pose_to_image(info, pose, O, p0);
    rep(i, 0, 3) {
        float_t v[3]{}, dst[3];
        v[i] = info.tagsize;
        tag_pose_to_image(info, pose, v, p1);
        lineImg(img, p0[1], p0[0], p1[1], p1[0], colors[i]);
    }
}

void plot_pose_cube(uint8_t* img, const apriltag_detection_info& info, const apriltag_pose& pose) {
    float_t p[8][2];
    const float_t X = info.tagsize / 2;
    float_t corner[4][3]{{X, X}, {X, -X}, {-X, -X}, {-X, X}};
    rep(i, 0, 4) {
        tag_pose_to_image(info, pose, corner[i], p[i]);
        corner[i][2] = -2 * X;
        tag_pose_to_image(info, pose, corner[i], p[i + 4]);
    }
    rep(i, 0, 4) lineImg(img, p[i][1], p[i][0], p[((i + 1) & 3)][1], p[((i + 1) & 3)][0], colors[2]);
    rep(i, 0, 4) lineImg(img, p[i][1], p[i][0], p[i + 4][1], p[i + 4][0], colors[1]);
    rep(i, 0, 4) lineImg(img, p[i + 4][1], p[i + 4][0], p[((i + 1) & 3) + 4][1], p[((i + 1) & 3) + 4][0], colors[0]);
}

}  // namespace apriltag
}  // namespace imgProc
