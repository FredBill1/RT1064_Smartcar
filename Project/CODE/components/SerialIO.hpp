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
        TxXfer(const char *name) : sem(name, 1) {}
        TxXfer(void *data, size_t size, const char *name = "xfer") : TxXfer(name) { setData(data, size); }
        void setData(void *data, size_t size) {
            dataSize = size;
            this->data = (uint8_t *)data;
        }
        void setSize(size_t size) { dataSize = size; }
        bool txFinished(int32_t timeout_ms = 0) { return sem.wait(timeout_ms); }
        void setTxFin() { sem.release(); }
    };

    template <typename T> class TxWirter {
        uint8_t *const data;

     public:
        TxWirter(uint8_t *data) : data(data) {}
        void set(int i, T val) {
            uint8_t *buf = (uint8_t *)&val;
            int I = i * (sizeof(T) + 1);
            uint8_t &sum = data[I + sizeof(T)];
            sum = 0;
            for (int j = 0; j < sizeof(T); ++j) {
                data[I + j] = buf[j];
                sum += buf[j];
            }
        }
        static constexpr int getSize(int cnt = 1) { return (sizeof(T) + 1) * cnt; }
        template <typename... U> inline void setAll(U... val) {
            int i = 0;
            ((set(i++, val)), ...);
        }
        template <typename U> inline void setArr(U *arr, int cnt) {
            for (int i = 0; i < cnt; ++i) set(i, arr[i]);
        }
    };

    template <typename T, int cnt, bool withID = false, bool checkSum = true> class TxUtil : public TxXfer {
     private:
        uint8_t _data[HeaderSize + withID + cnt * (sizeof(T) + checkSum)];

     public:
        TxUtil(const char *name) : TxXfer(_data, sizeof(_data), name) {
            static_assert(!withID, "if `withID` is set `true`, you should specify the `id` using the other ctor");
            applyHeader(_data);
        }
        TxUtil(const char *name, uint8_t id) : TxXfer(_data, sizeof(_data), name) {
            static_assert(
                withID,
                "if `withID` is set `false`, you should not specify the `id` using this ctor, use the other ctor instead");
            _data[HeaderSize] = id;
            applyHeader(_data);
        }
        void set(int i, T val) {
            uint8_t *buf = (uint8_t *)&val;
            int I = HeaderSize + withID + i * (sizeof(T) + checkSum);
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
        uint8_t *Data() { return _data + HeaderSize + withID; }
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