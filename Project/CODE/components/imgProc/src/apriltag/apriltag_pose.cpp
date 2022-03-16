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

det_2ds_t& poese_to_det_2ds(const det_poses_t& det_poses, const float_t shift_dist) {
    const float_t shift[3]{0, 0, shift_dist};
    float_t res[3];
    det_2ds_t& det_2ds = *new (staticBuffer.allocate(sizeof(det_2ds_t))) det_2ds_t(det_2ds_alloc_t{staticBuffer});
    det_2ds.reserve(det_poses.size());
    for (auto& [det, pose] : det_poses) {
        det_2ds.emplace_back(det);
        tag_pose_to_camera(pose, shift, res);
        float_t* xy = det_2ds.back().xy;
        xy[0] = res[2], xy[1] = -res[0];
    }
    return det_2ds;
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