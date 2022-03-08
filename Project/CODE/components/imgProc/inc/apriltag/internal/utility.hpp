#ifndef _imgProc_utility_hpp
#define _imgProc_utility_hpp

#include <cstdint>

#define rep(i, s, e) for (int_fast32_t i = s; i < e; ++i)
#define req(i, s, e) for (int_fast32_t i = s; i <= e; ++i)

namespace imgProc {
template <typename T, typename U> inline void chkmax(T& x, U y) {
    if (y > x) x = y;
}
template <typename T, typename U> inline void chkmin(T& x, U y) {
    if (y < x) x = y;
}
template <typename T, typename U> inline constexpr auto min(T x, U y) -> decltype(x + y) { return x < y ? x : y; }
template <typename T, typename U> inline constexpr auto max(T x, U y) -> decltype(x + y) { return x > y ? x : y; }
template <typename T, typename U, typename V> inline constexpr T constrain(T x, U l, V r) { return min(max(x, l), r); }
template <typename T, typename U> inline constexpr void swap(T& x, U& y) {
    T t = x;
    x = y, y = t;
}
template <typename T> inline constexpr T abs(T x) { return x > 0 ? x : -x; }
template <typename T> inline constexpr T sign(T x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

template <typename T> inline constexpr T sq(T x) { return x * x; }

}  // namespace imgProc

#endif  // _imgProc_utility_hpp