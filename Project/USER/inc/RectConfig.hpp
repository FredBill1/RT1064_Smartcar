#ifndef _RectConfig_hpp
#define _RectConfig_hpp

#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

// 发送到串口的最大Rect数量
constexpr int max_rect_cnt = 5;

// 有效Rect的最小边界梯度和
constexpr float_t min_magnitude = 7000;

// Rect边长的范围, 单位是m
constexpr float_t min_rect_size = 0.09;
constexpr float_t max_rect_size = 0.15;

// Rect相邻两边夹角范围, 单位是度
constexpr float_t min_rect_angle = 70;
constexpr float_t max_rect_angle = 110;

// 相机到底盘的透视变换对应坐标点
// clang-format off
constexpr float_t Cam2Base_corr[4][4]{   // {{图像x_i, 图像y_i, 底盘x_i, 底盘y_i}, 0<=i<=3}
    {64,  454, 0.195,  0.210},  // 左下角
    {688, 450, 0.207, -0.200},  // 右下角
    {720, 27,  0.654, -0.434},  // 右上角
    {20,  22,  0.721,  0.540},  // 左上角
};
// clang-format on

}  // namespace apriltag
}  // namespace imgProc

#endif  // _RectConfig_hpp