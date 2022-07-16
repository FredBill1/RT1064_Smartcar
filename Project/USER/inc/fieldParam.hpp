#ifndef _fieldParam_hpp
#define _fieldParam_hpp

#include <cstdint>

#include "pose_kalman/utils.hpp"
using pose_kalman::PI, pose_kalman::PI_2;

// 单位都是米

// A4纸上边界的宽高
constexpr float borderWidth = 7;
constexpr float borderHeight = 5;

// 赛场一格的大小，用于对齐A4纸识别结果
constexpr bool squareAlign = true;  // 是否对A4识别结果进行对齐
constexpr float squareSize = 0.2;

// 实际赛场的宽高
constexpr float fieldWidth = 5;
constexpr float fieldHeight = 4;

// 搬运时边界外的距离
constexpr float carryExtendPadding = 0.5;  // 向外
constexpr float carrySidePadding = 0.5;    // 角落里向中间

// 场地内rect距离边界的最小距离
constexpr float rectPadding = 0.1;

// 车在场地内使rect生效的最小距离
constexpr bool useRectBasePadding = false;
constexpr float rectBasePadding = 0.1;

// rect识别坐标的最大偏移距离
constexpr float rectMaxDistError = 0.5;

// 初始位置
constexpr float garage_position1[2]{0.105, 0.4};
constexpr float garage_position2[2]{0.105, -0.4};
constexpr float initial_position[3]{0.105, -0.141, PI_2};

#endif  // _fieldParam_hpp