#ifndef _SerialIO_hpp
#define _SerialIO_hpp

#include <rtthread.h>

#include <type_traits>
extern "C" {
#include "zf_uart.h"
}

class SerialIO : private lpuart_handle_t {
 private:
    rt_mailbox_t rx_mb;
    uint8 rx_bf;
    UARTN_enum uartn;

    static void rx_cb(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

 public:
    void init(const char *name, UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin);
    uint8 getchar();
    void putchar(uint8 c);
    void getbuff(uint8 *buf, uint32 len);
    void putbuff(uint8 *buf, uint32 len);
    template <typename T> std::enable_if_t<std::is_arithmetic<T>::value, void> read(T &x);
    template <typename T> std::enable_if_t<std::is_arithmetic<T>::value, void> write(T x);
    template <typename T, typename... U> void read(T &x, U &...y);
    template <typename T, typename... U> void write(T x, U... y);
};

template <typename T> std::enable_if_t<std::is_arithmetic<T>::value, void> SerialIO::read(T &x) {
    double tmp;
    getbuff((uint8 *)&tmp, 8), x = (T)tmp;
}
template <typename T, typename... U> void SerialIO::read(T &x, U &...y) { read(x), read(y...); }

template <typename T> std::enable_if_t<std::is_arithmetic<T>::value, void> SerialIO::write(T x) {
    double tmp = (double)x;
    putbuff((uint8 *)&tmp, 8);
}
template <typename T, typename... U> void SerialIO::write(T x, U... y) { write(x), write(y...); }

#endif  // _SerialIO_hpp