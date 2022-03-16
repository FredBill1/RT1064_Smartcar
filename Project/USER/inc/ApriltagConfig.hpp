#ifndef _AprilTagConfig_hpp
#define _AprilTagConfig_hpp

namespace imgProc {
namespace apriltag {

constexpr int maxhamming = 1;              // 解码tag时最多错几位
constexpr float_t tagsize = 0.1145454545;  // apriltag的边长，单位是米
constexpr float_t shift_dist = 0.05;       // 3维信息转二维坐标时tag平面向后方平移的距离

// 相机标定参数
constexpr float_t fx = 397.38711548;  // In pixels.
constexpr float_t fy = 399.70840454;  // In pixels.
constexpr float_t cx = 369.06676129;  // In pixels.
constexpr float_t cy = 226.30497101;  // In pixels.

}  // namespace apriltag
}  // namespace imgProc

#endif  // _AprilTagConfig_hpp