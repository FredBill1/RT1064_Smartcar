#include "SerialIO.hpp"

void SerialIO::init(const char *name, UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin) {
    this->uartn = uartn;
    rx_mb = rt_mb_create(name, 32, RT_IPC_FLAG_FIFO);
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

void SerialIO::putbuff(uint8 *buf, uint32 len) { uart_putbuff(uartn, buf, len); }
