#ifndef _fmath_hpp
#define _fmath_hpp

namespace imgProc {
namespace apriltag {

constexpr double PI_d = 3.14159265358979323846;
constexpr float PI_f = 3.14159265358979323846f;

float sqrtf(float x);
float fabs(float x);
int floorf(float x);
int ceilf(float x);
int roundf(float x);

float sinf(float x);
float cosf(float x);
float atanf(float x);
float atan2f(float y, float x);

}  // namespace apriltag
}  // namespace imgProc
#endif  // _fmath_hpp