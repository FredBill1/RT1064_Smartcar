#ifndef _IPS_hpp
#define _IPS_hpp

#include "BinaryImg.hpp"

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
    void puts(const char* s) { printStr(s), putchar('\n'), flush(); }
    void printf(const char* fmt, ...);
};

#endif  // _IPS_hpp