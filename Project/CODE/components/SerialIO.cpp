#include "SerialIO.hpp"

void SerialIO::init(const char *mb_name, UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin,
                    rt_size_t mb_size) {
    this->uartn = uartn;
    rx_mb = rt_mb_create(mb_name, mb_size, RT_IPC_FLAG_FIFO);
    uart_init(uartn, baud, tx_pin, rx_pin);
    uart_set_handle(uartn, this, rx_cb, NULL, 0, &rx_bf, 1);
    uart_rx_irq(uartn, 1);
}

void SerialIO::rx_cb(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData) {
    SerialIO &self = *(SerialIO *)handle;
    if (status == kStatus_LPUART_RxIdle) rt_mb_send(self.rx_mb, self.rx_bf);
    self.rxDataSize = 1;
    self.rxData = &self.rx_bf;
}

uint8 SerialIO::getchar() {
    rt_ubase_t dat;
    rt_mb_recv(rx_mb, &dat, RT_WAITING_FOREVER);
    return (uint8)dat;
}
void SerialIO::putchar(uint8 c) { uart_putchar(uartn, c); }

bool SerialIO::getbuff(uint8 *buf, uint32 len, rt_int32_t timeout) {
    rt_ubase_t dat;
    for (uint32 i = 0; i < len; ++i) {
        rt_err_t ret = rt_mb_recv(rx_mb, &dat, timeout);
        if (ret != RT_EOK) return false;
        buf[i] = (uint8)dat;
    }
    return true;
}

void SerialIO::putbuff(const uint8 *buf, uint32 len) {
    extern LPUART_Type *UARTN[];
    LPUART_WriteBlocking(UARTN[uartn], buf, len);
}

const uint8 SerialIO::HEADER[4] = {0x00, 0xff, 0x80, 0x7f};
void SerialIO::sendHeader() { putbuff(HEADER, 4); }

void SerialIO::waitHeader() {
    rt_tick_t stamp[4];
    {
        rt_tick_t cur = rt_tick_get();
        for (int i = 0; i < 4; ++i) stamp[i] = cur;
    }
    uint8 buf[4];
    for (int i = 0;; i = (i + 1) & 3) {
        buf[i] = getchar();
        stamp[i] = rt_tick_get();
        if (stamp[(i + 1) & 3] - stamp[i] > (TIMEOUT << 2)) continue;
        bool success = true;
        for (int j = 0; j < 4; ++j)
            if (buf[(i + j + 1) & 3] != HEADER[j]) {
                success = false;
                break;
            }
        if (success) return;
    }
}

void SerialIO::sendTail() { putchar(0x00), putchar(0x00), putchar(0x80), putchar(0x7f); }