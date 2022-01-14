#include "IPS.hpp"

#include <rtthread.h>

#include <cstdarg>
#include <cstdio>

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

void IPS::init() {
    memset(_buf, ' ', sizeof(_buf));
    ips114_init();
}

void IPS::showChar(int i, int j, char c) { ips114_showchar(j * 8, i * 16, c); }

void IPS::flush() {
    if (_j)
        for (int j = _j + 1; j < M; ++j) _buf[_i][_j] = ' ';
    for (int i = 0; i < N; ++i) {
        int line = (_i - i + N - 1) % N;
        for (int j = 0; j < M; ++j) showChar(N - i - 1, j, _buf[line][j]);
    }
}

void IPS::putchar(char c) {
    if (c == '\n') ++_i, _j = 0;
    else if (c == '\r')
        _j = 0;
    else {
        _buf[_i][_j++] = c;
        if (_j == M) _j = 0, ++_i;
    }
    if (_i == N) _i = 0;
}

void IPS::printStr(const char* s) {
    while (*s) putchar(*s++);
}

void IPS::printf(const char* fmt, ...) {
    va_list args1;
    va_start(args1, fmt);
    va_list args2;
    va_copy(args2, args1);
    char* buf = new char[1 + vsnprintf(nullptr, 0, fmt, args1)];
    va_end(args1);
    vsprintf(buf, fmt, args2);
    va_end(args2);
    printStr(buf);
    delete[] buf;
}
