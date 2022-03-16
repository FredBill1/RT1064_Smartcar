#ifndef _apriltag_pose_hpp
#define _apriltag_pose_hpp

#include <utility>
#include <vector>

#include "apriltag/apriltag.hpp"
#include "apriltag/internal/StaticBuffer.hpp"

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
    float_t R[3][3];  // 旋转矩阵
    float_t t[3];     // 平移向量
};

struct det_pose {
    const apriltag_detection& det;
    apriltag_pose pose;
    det_pose(const apriltag_detection& _det) : det(_det) {}
};
using det_poses_alloc_t = StaticAllocator<det_pose>;
using det_poses_t = std::vector<det_pose, det_poses_alloc_t>;

struct det_2d {
    const apriltag_detection& det;
    float_t xy[2];
    det_2d(const apriltag_detection& _det) : det(_det) {}
};
using det_2ds_alloc_t = StaticAllocator<det_2d>;
using det_2ds_t = std::vector<det_2d, det_2ds_alloc_t>;

void estimate_pose_for_tag_homography(const apriltag_detection_info& info, apriltag_pose& solution);

det_poses_t& estimate_poses(apriltag_detection_info& info, detections_t& dets);

det_2ds_t& poese_to_det_2ds(const det_poses_t& det_poses, const float_t shift_dist);

void tag_pose_to_camera(const apriltag_pose& pose, const float_t src[3], float_t dst[3]);

void camera_info_to_image(const apriltag_detection_info& info, const float_t src[3], float_t dst[2]);

void tag_pose_to_image(const apriltag_detection_info& info, const apriltag_pose& pose, const float_t src[3], float_t dst[2]);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_pose_hpp