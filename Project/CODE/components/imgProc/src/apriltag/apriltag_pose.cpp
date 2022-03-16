#include "apriltag/apriltag_pose.hpp"

#include "apriltag/internal/homography.hpp"
#include "apriltag/internal/utility.hpp"

namespace imgProc {
namespace apriltag {

void estimate_pose_for_tag_homography(const apriltag_detection_info& info, apriltag_pose& solution) {
    auto& R = solution.R;
    auto& t = solution.t;
    homography_to_pose(info.det->H, -info.fx, info.fy, info.cx, info.cy, R, t);

    float_t scale = info.tagsize / 2;
    rep(i, 0, 3) t[i] *= scale;
    req(i, 1, 2) {
        t[i] = -t[i];
        rep(j, 0, 3) R[j][i] = -R[j][i];
    }
}

det_poses_t& estimate_poses(apriltag_detection_info& info, detections_t& dets) {
    det_poses_t& det_poses = *new (staticBuffer.allocate(sizeof(det_poses_t))) det_poses_t(det_poses_alloc_t{staticBuffer});
    det_poses.reserve(std::distance(dets.begin(), dets.end()));
    for (apriltag_detection* det_p : dets) {
        det_poses.emplace_back(*det_p);
        info.det = det_p;
        estimate_pose_for_tag_homography(info, det_poses.back().pose);
    }
    return det_poses;
}

void tag_pose_to_camera(const apriltag_pose& pose, const float_t src[3], float_t dst[3]) {
    matrix_transform(pose.R[0], pose.t, src, dst);
}

void camera_info_to_image(const apriltag_detection_info& info, const float_t src[3], float_t dst[2]) {
    camera_to_image(info.fx, info.fy, info.cx, info.cy, src, dst);
}

void tag_pose_to_image(const apriltag_detection_info& info, const apriltag_pose& pose, const float_t src[3], float_t dst[2]) {
    float_t cam[3];
    tag_pose_to_camera(pose, src, cam);
    camera_info_to_image(info, cam, dst);
}

}  // namespace apriltag
}  // namespace imgProc