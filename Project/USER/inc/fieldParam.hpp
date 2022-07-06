#ifndef _fieldParam_hpp
#define _fieldParam_hpp

#include <cstdint>

// 单位都是米

// A4纸上边界的宽高
constexpr float borderWidth = 7;
constexpr float borderHeight = 5;

// 赛场一格的大小，用于对齐A4纸识别结果
constexpr float squareSize = 0.2;

// 实际赛场的宽高
constexpr float fieldWidth = 5;
constexpr float fieldHeight = 4;

// 搬运时边界外的距离
constexpr float carryPadding = 0.3;

// rect识别坐标的最大偏移距离
constexpr float rectMaxDistErrorSquared = 0.5 * 0.5;

#endif  // _fieldParam_hpp