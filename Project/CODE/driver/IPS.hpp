#ifndef _IPS_hpp
#define _IPS_hpp

#include "BinaryImg.hpp"

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

class IPS {
 protected:
    static const int N = 8, M = 30;
    char _buf[N][M];
    bool _ret = false;
    unsigned char _i = 0, _j = 0;
    void nextLine();

 public:
    enum Color {
        // clang-format off
        Red    = 0xF800,
        Blue   = 0x001F,
        Yellow = 0xFFE0,
        Green  = 0x07E0,
        White  = 0xFFFF,
        Black  = 0x0000,
        Gray   = 0X8430,
        Brown  = 0XBC40,
        Purple = 0XF81F,
        Pink   = 0XFE19,
        // clang-format on
    };
    void init();
    void showChar(int i, int j, char c);
    void putchar(char c);
    void flush();
    void printStr(const char* s);
    void printf(const char* fmt, ...);
};

template <int_fast32_t height, int_fast32_t width> IPS& operator<<(IPS& ips, const imgProc::QuadImg<height, width>& img) {
    constexpr uint16 coord_x = width > IPS114_X_MAX ? IPS114_X_MAX : width;
    constexpr uint16 coord_y = height > IPS114_Y_MAX ? IPS114_Y_MAX : height;
    ips114_set_region(0, 0, coord_x - 1, coord_y - 1);
    for (int32 j = 0; j < coord_y; j++) {
        for (int32 i = 0; i < coord_x; i++) {
            uint8_t cur = img(j * height / coord_y, i * width / coord_x);
            uint16_t color;
            switch (cur) {
            case 0: color = IPS::Black; break;
            case 1: color = IPS::Gray; break;
            case 2: color = IPS::Red; break;
            default: color = IPS::White; break;
            }
            ips114_writedata_16bit(color);
        }
    }
    return ips;
}

#endif  // _IPS_hpp