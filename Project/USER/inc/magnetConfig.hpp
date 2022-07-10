#ifndef _magnetConfig_hpp
#define _magnetConfig_hpp

namespace magnet {

constexpr int cnt = 4;
constexpr float T_length = 0.3;
constexpr float T_shift = 0.025;
constexpr float T_extend = 0.25;

constexpr float pos[][2]{
    // Ç°
    {T_length / 2 + T_shift, T_extend},
    {T_length / 2 + T_shift, -T_extend},
    // ÖÐ
    {T_shift, T_extend},
    {T_shift, -T_extend},
    // ºó
    {-T_length / 2 + T_shift, T_extend},
    {-T_length / 2 + T_shift, -T_extend},
};

}  // namespace magnet

#endif  // _magnetConfig_hpp