#ifndef _fieldParam_hpp
#define _fieldParam_hpp

#include <cstdint>

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
constexpr float carryPadding = 0.3;

// 场地内rect距离边界的最小距离
constexpr float rectPadding = 0.1;

// 车在场地内使rect生效的最小距离
constexpr float rectBasePadding = 0.1;

// rect识别坐标的最大偏移距离
constexpr float rectMaxDistErrorSquared = 0.5 * 0.5;

#endif  // _fieldParam_hpp