#include "apriltag/fmath.hpp"

#define USE_ARM_MATH 1
#include <cmath>

#if (USE_ARM_MATH)
#include <arm_math.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265f
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632f
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816f
#endif

#if (USE_ARM_MATH)
static inline int fast_ceilf(float x) {
    float fi;
    int i;
    x += 0.99999f;
#if defined(__CC_ARM)
    __asm {
		vcvt.s32.f32 fi, x
		vmov  i, fi
    }
#else
    asm volatile("vcvt.S32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));

#endif
    return i;
}

static inline int fast_floorf(float x) {
    float fi;
    int i;
#if defined(__CC_ARM)
    __asm {
		vcvt.s32.f32 fi, x
		vmov  i, fi
    }
#else
    asm volatile("vcvt.S32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));
#endif
    return i;
}

static inline int fast_roundf(float x) {
    float fi;
    int i;
#if defined(__CC_ARM)
    __asm("");
    { vcvtr.s32.f32 fi, x vmov i, fi }
#else
    asm volatile("vcvtr.s32.f32  %[r], %[x]\n" : [r] "=t"(i) : [x] "t"(x));
#endif
    return i;
}

static inline float fast_atanf(float xx) {
    float x, y, z;
    int sign;
    x = xx;
    /* make argument positive and save the sign */
    if (xx < 0.0f) {
        sign = -1;
        x = -xx;
    } else {
        sign = 1;
        x = xx;
    }
    /* range reduction */
    if (x > 2.414213562373095f) { /* tan 3pi/8 */
        y = M_PI_2;
        x = -(1.0f / x);
    } else if (x > 0.4142135623730950f) { /* tan pi/8 */
        y = M_PI_4;
        x = (x - 1.0f) / (x + 1.0f);
    } else
        y = 0.0f;
    z = x * x;
    y += (((8.05374449538e-2f * z - 1.38776856032E-1f) * z + 1.99777106478E-1f) * z - 3.33329491539E-1f) * z * x + x;
    if (sign < 0) y = -y;
    return (y);
}

static inline float fast_atan2f(float y, float x) {
    if (x > 0 && y >= 0) return fast_atanf(y / x);
    if (x < 0 && y >= 0) return M_PI - fast_atanf(-y / x);
    if (x < 0 && y < 0) return M_PI + fast_atanf(y / x);
    if (x > 0 && y < 0) return 2 * M_PI - fast_atanf(-y / x);
    return (y == 0) ? 0 : ((y > 0) ? M_PI : -M_PI);
}

// #define fast_sqrtf sqrtf
static inline float fast_sqrtf(float x) {
#if defined(__CC_ARM)
    __asm {
		VSQRT.F32	x,	x
    }
#else
    asm volatile("vsqrt.f32 %[r], %[x]\n" : [r] "=t"(x) : [x] "t"(x));
#endif
    return x;
}
#endif

namespace imgProc {
namespace apriltag {
#if (USE_ARM_MATH)
float sqrtf(float x) { return fast_sqrtf(x); }
float fabs(float x) { return fabsf(x); }
int floorf(float x) { return fast_floorf(x); }
int ceilf(float x) { return fast_ceilf(x); }
int roundf(float x) { return fast_roundf(x); }

float sinf(float x) { return arm_sin_f32(x); }
float cosf(float x) { return arm_cos_f32(x); }
float atanf(float x) { return fast_atanf(x); }
float atan2f(float y, float x) { return fast_atan2f(y, x); }

#else
float sqrtf(float x) { return std::sqrt(x); }
float fabs(float x) { return std::fabs(x); }
int floorf(float x) { return std::floor(x); }
int ceilf(float x) { return std::ceil(x); }
int roundf(float x) { return std::round(x); }

float sinf(float x) { return std::sin(x); }
float cosf(float x) { return std::cos(x); }
float atanf(float x) { return std::atan(x); }
float atan2f(float y, float x) { return std::atan2(y, x); }
#endif
}  // namespace apriltag
}  // namespace imgProc