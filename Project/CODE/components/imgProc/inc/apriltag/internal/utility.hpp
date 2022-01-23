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

}  // namespace imgProc

#endif  // _imgProc_utility_hpp