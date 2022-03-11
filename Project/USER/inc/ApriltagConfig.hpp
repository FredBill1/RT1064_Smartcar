#ifndef _AprilTagConfig_hpp
#define _AprilTagConfig_hpp

namespace imgProc {
namespace apriltag {

constexpr int maxhamming = 1;    // 解码tag时最多错几位
constexpr float_t tagsize = 10;  // apriltag的边长，单位自定

// 相机标定参数
constexpr float_t fx = 397.0867192;   // In pixels.
constexpr float_t fy = 396.83938378;  // In pixels.
constexpr float_t cx = 369.35730722;  // In pixels.
constexpr float_t cy = 226.60933851;  // In pixels.

}  // namespace apriltag
}  // namespace imgProc

#endif  // _AprilTagConfig_hpp