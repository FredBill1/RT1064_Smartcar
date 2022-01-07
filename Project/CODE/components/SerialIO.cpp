#include "SerialIO.hpp"

void SerialIO::init(const char *name, UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin) {
    mb = rt_mb_create(name, 10, RT_IPC_FLAG_FIFO);
    uart_init(uartn, baud, tx_pin, rx_pin);
    uart_set_handle(uartn, this, rx_cb, NULL, 0, &rx_bf, 1);
    uart_rx_irq(uartn, 1);
}

void SerialIO::rx_cb(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData) {
    SerialIO &self = *(SerialIO *)handle;
    if (status == kStatus_LPUART_RxIdle) rt_mb_send(self.mb, self.rx_bf);
    self.rxDataSize = 1;
    self.rxData = &self.rx_bf;
}

uint8 SerialIO::getchar() {
    rt_ubase_t dat;
    rt_mb_recv(mb, &dat, RT_WAITING_FOREVER);
    return (uint8)dat;
}