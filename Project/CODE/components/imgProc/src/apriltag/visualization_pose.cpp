#include "apriltag/visualization_pose.hpp"

#include <algorithm>

#include "apriltag/internal/homography.hpp"
#include "apriltag/internal/utility.hpp"
#include "apriltag/visualization.hpp"
namespace imgProc {
namespace apriltag {

constexpr uint16_t colors[]{0xF800, 0x07E0, 0x001F, 0xFFFF};  // RGB,White

void plot_pose_axis(uint8_t* img, const apriltag_detection_info& info, const apriltag_pose& pose) {
    float_t p0[2], p1[2];
    float_t O[3]{};
    tag_pose_to_image(info, pose, O, p0);
    float_t cam[3][3];
    int idx[3]{0, 1, 2};
    rep(i, 0, 3) {
        float tag[3]{};
        tag[i] = info.tagsize;
        tag_pose_to_camera(pose, tag, cam[i]);
    }
    std::sort(idx, idx + 3, [cam](int i, int j) { return cam[i][2] > cam[j][2]; });
    for (int i : idx) {
        camera_info_to_image(info, cam[i], p1);
        lineImg(img, p0[1], p0[0], p1[1], p1[0], colors[i], 1);
    }
    plotImg(img, p0[1], p0[0], colors[3], 1);
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
    rep(i, 0, 4) lineImg(img, p[i][1], p[i][0], p[((i + 1) & 3)][1], p[((i + 1) & 3)][0], colors[2], 0);
    rep(i, 0, 4) lineImg(img, p[i][1], p[i][0], p[i + 4][1], p[i + 4][0], colors[1], 0);
    rep(i, 0, 4) lineImg(img, p[i + 4][1], p[i + 4][0], p[((i + 1) & 3) + 4][1], p[((i + 1) & 3) + 4][0], colors[0], 0);
}

void polt_pose_err(uint8_t* img, const apriltag_detection& det, const apriltag_pose& pose) {
    plotInt(img, det.c[1], det.c[0], pose.err * 1e9, 4, true);
}

void plot_det_poses(uint8_t* img, const apriltag_detection_info& info, const det_poses_t& det_poses) {
    for (auto& [det, pose] : det_poses) {
        // polt_pose_err(img, det, pose);  // 画出tag pose的err
        // plot_tag_det(img, det);           // 画出tag边框和id
        // plot_pose_axis(img, info, pose);  // 画出tag的坐标系，RGB分别为xyz轴
        plot_pose_cube(img, info, pose);  // 画出tag的3d方块
    }
}

}  // namespace apriltag
}  // namespace imgProc
