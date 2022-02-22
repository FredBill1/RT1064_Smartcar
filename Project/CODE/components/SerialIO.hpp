#ifndef _SerialIO_hpp
#define _SerialIO_hpp

#include <rtthread.h>

#include <cstdint>

extern "C" {
#include "zf_uart.h"
}
#include "utils/Mailbox.hpp"
#include "utils/Semaphore.hpp"

class SerialIO {
 public:
    static constexpr size_t HeaderSize = 4;
    static inline constexpr uint8_t HEADER[HeaderSize] = {0x00, 0xff, 0x80, 0x7f};
    static void applyHeader(uint8_t *data) {
        for (int i = 0; i < sizeof(HEADER); ++i) data[i] = HEADER[i];
    }

    class TxXfer : protected lpuart_transfer_t {
        Semaphore sem;

     public:
        TxXfer(TxXfer &&) = delete;
        TxXfer(uint8_t *data, size_t size, const char *name = "xfer") : sem(name, 1) {
            dataSize = size;
            this->data = data;
        }
        bool txFinished() { return sem.wait(0); }
        void setTxFin() { sem.release(); }
    };

    template <typename T, int cnt, int extra_bytes = 0, bool checkSum = true> class TxUtil : public TxXfer {
     private:
        uint8_t _data[HeaderSize + extra_bytes + cnt * (sizeof(T) + checkSum)];

     public:
        TxUtil(const char *name = "xfer") : TxXfer(_data, sizeof(_data), name) { applyHeader(_data); }
        void set(int i, T val) {
            uint8_t *buf = (uint8_t *)&val;
            int I = HeaderSize + extra_bytes + i * (sizeof(T) + checkSum);
            uint8_t &sum = _data[I + sizeof(T)];
            if constexpr (checkSum) sum = 0;
            for (int j = 0; j < sizeof(T); ++j) {
                _data[I + j] = buf[j];
                if constexpr (checkSum) sum += buf[j];
            }
        }
        template <typename... U> inline void setAll(U... val) {
            static_assert(sizeof...(val) == cnt, "number of arguments is not equal to data count");
            int i = 0;
            ((set(i++, val)), ...);
        }
        template <typename U> inline void setArr(U *arr) {
            for (int i = 0; i < cnt; ++i) set(i, arr[i]);
        }
        uint8_t *ptr() { return _data; }
        uint8_t *extra_data() { return _data + HeaderSize; }
        static constexpr size_t size() { return sizeof(_data); }
    };

 private:
    const UARTN_enum _uartn;
    const uint32_t _baud;
    const UARTPIN_enum _tx_pin, _rx_pin;
    const uint32_t timeout;

    uint8_t rx_bf;
    Mailbox rx_mb, tx_mb;
    Semaphore tx_sem;
    TxXfer *txXfer;

    lpuart_handle_t handle;
    static inline LPUART_Type *const UARTN[] = LPUART_BASE_PTRS;
    static void xferCB(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData) {
        ((SerialIO *)userData)->xferCB(status);
    }
    void resetRxData() {
        handle.rxData = &rx_bf;
        handle.rxDataSize = 1;
    }
    void xferCB(status_t status);
    bool trySendNewData(bool needSem);

 public:
    SerialIO(SerialIO &&) = delete;
    SerialIO(UARTN_enum uartn, uint32_t baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin, uint32_t rx_sz, uint32_t tx_sz,
             uint32_t timeout_ms = 100);
    void init();
    void send(TxXfer &xfer);
    bool getchar(uint8_t &data);
    bool getbuf(uint8_t *buf, size_t cnt);

 private:
    bool waitHeaderUtil();

 public:
    void waitHeader();

 private:
    template <typename T, bool checkSum = true> bool getTbuf(T *buf, size_t cnt) {
        if constexpr (!checkSum) return getbuf((uint8_t *)buf, cnt * sizeof(T));
        for (size_t i = 0; i < cnt; ++i) {
            uint8_t *ptr = (uint8_t *)(buf + i), sum = 0, tmp = 0;
            if (!(getbuf(ptr, sizeof(T)) && getchar(tmp))) return false;
            for (size_t i = 0; i < sizeof(T); ++i) sum += ptr[i];
            if (sum != tmp) return false;
        }
        return true;
    }

 public:
    template <typename T, bool checkSum = true, typename... U> bool getData(U &...data) {
        T buf[sizeof...(data)];
        if (!getTbuf<T, checkSum>(buf, sizeof...(data))) return false;
        size_t i = 0;
        ((data = buf[i++]), ...);
        return true;
    }
    template <typename T, size_t cnt, bool checkSum = true, typename U> bool getArr(U *data) {
        T buf[cnt];
        if (!getTbuf<T, checkSum>(buf, cnt)) return false;
        for (size_t i = 0; i < cnt; ++i) data[i] = buf[i];
        return true;
    }
};

#endif  // _SerialIO_hpp