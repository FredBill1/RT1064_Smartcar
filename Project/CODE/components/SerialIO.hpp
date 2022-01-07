#ifndef _SerialIO_hpp
#define _SerialIO_hpp

#include <rtthread.h>

#include <type_traits>
#include <utility>
extern "C" {
#include "zf_uart.h"
}

class SerialIO : private lpuart_handle_t {
 private:
    rt_mailbox_t rx_mb;
    uint8 rx_bf;
    UARTN_enum uartn;
    const rt_int32_t TIMEOUT;
    static void rx_cb(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

 public:
    static const uint8 HEADER[4];
    SerialIO(rt_int32_t timeout_ms = 100) : TIMEOUT(rt_tick_from_millisecond(timeout_ms)) {}
    void init(const char *mb_name, UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin,
              rt_size_t mb_size = 32);

    uint8 getchar();
    void putchar(uint8 c);
    bool getbuff(uint8 *buf, uint32 len, rt_int32_t timeout = RT_WAITING_FOREVER);
    void putbuff(const uint8 *buf, uint32 len);

    template <typename Data_t, typename T> std::enable_if_t<std::is_arithmetic<T>::value, void> write(T x);
    template <typename Data_t, typename T, typename... U> void write(T x, U... y);
    template <typename... Ts> void writeF(Ts &&...ts) { write<float>(std::forward<Ts>(ts)...); };
    template <typename... Ts> void writeD(Ts &&...ts) { write<double>(std::forward<Ts>(ts)...); };

    template <typename Data_t, typename T> std::enable_if_t<std::is_arithmetic<T>::value, bool> read(T &x);
    template <typename Data_t, typename T, typename... U> bool read(T &x, U &...y);
    template <typename... Ts> bool readF(Ts &&...ts) { return read<float>(std::forward<Ts>(ts)...); };
    template <typename... Ts> bool readD(Ts &&...ts) { return read<double>(std::forward<Ts>(ts)...); };

    void sendHeader();
    void waitHeader();
};

template <typename Data_t, typename T> std::enable_if_t<std::is_arithmetic<T>::value, bool> SerialIO::read(T &x) {
    uint8 tmp[sizeof(Data_t) + 1], sum = 0;
    if (getbuff(tmp, sizeof(Data_t) + 1, TIMEOUT)) {
        for (int i = 0; i < sizeof(Data_t); ++i) sum += tmp[i];
        if (sum == tmp[sizeof(Data_t)]) {
            x = (T) * (Data_t *)tmp;
            return true;
        }
    }
    return false;
}
template <typename Data_t, typename T, typename... U> bool SerialIO::read(T &x, U &...y) {
    return read<Data_t>(x) && read<Data_t>(y...);
}

template <typename Data_t, typename T> std::enable_if_t<std::is_arithmetic<T>::value, void> SerialIO::write(T x) {
    uint8 tmp[sizeof(Data_t)], sum = 0;
    *(Data_t *)tmp = (Data_t)x;
    for (int i = 0; i < sizeof(Data_t); ++i) sum += tmp[i];
    putbuff(tmp, sizeof(Data_t));
    putchar(sum);
}
template <typename Data_t, typename T, typename... U> void SerialIO::write(T x, U... y) {
    write<Data_t>(x), write<Data_t>(y...);
}

#endif  // _SerialIO_hpp