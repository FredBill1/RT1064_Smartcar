#ifndef _AprilTagConfig_hpp
#define _AprilTagConfig_hpp

namespace imgProc {
namespace apriltag {

constexpr int maxhamming = 1;               // 解码tag时最多错几位
constexpr float_t tagsize = 0.18 * 7 / 11;  // apriltag的边长，单位是米
constexpr float_t shift_dist = 0.05;        // 3维信息转二维坐标时tag平面向后方平移的距离

// 相机标定参数
constexpr float_t fx = 589.70233154;  // In pixels.
constexpr float_t fy = 648.32983398;  // In pixels.
constexpr float_t cx = 376.15877696;  // In pixels.
constexpr float_t cy = 242.75541548;  // In pixels.

}  // namespace apriltag
}  // namespace imgProc

#endif  // _AprilTagConfig_hpp