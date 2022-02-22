#include "SerialIO.hpp"

SerialIO::SerialIO(UARTN_enum uartn, uint32_t baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin, uint32_t rx_sz, uint32_t tx_sz,
                   uint32_t timeout_ms)
    : _uartn(uartn), _baud(baud), _tx_pin(tx_pin), _rx_pin(rx_pin), timeout(timeout_ms), txXfer(nullptr) {
    char name[] = "UART_rx";
    name[4] = '0' + (int)uartn;
    rx_mb.init(name, rx_sz);
    name[5] = 't';
    tx_mb.init(name, tx_sz);
    tx_sem.init(name, 1);
}

void SerialIO::init() {
    uart_init(_uartn, _baud, _tx_pin, _rx_pin);
    LPUART_TransferCreateHandle(UARTN[_uartn], &handle, xferCB, this);
    resetRxData();
    LPUART_EnableInterrupts(UARTN[_uartn], kLPUART_RxDataRegFullInterruptEnable);
}

bool SerialIO::trySendNewData(bool needSem) {
    if (needSem && !tx_sem.wait(0)) return false;
    rt_ubase_t p;
    if (!tx_mb.get_nowait(p)) return false;
    txXfer = (TxXfer *)p;
    LPUART_TransferSendNonBlocking(UARTN[_uartn], &handle, (lpuart_transfer_t *)txXfer);
    return true;
}

void SerialIO::xferCB(status_t status) {
    if (status == kStatus_LPUART_RxIdle) {
        rx_mb.instant_put(rx_bf);
        resetRxData();
    } else if (status == kStatus_LPUART_TxIdle) {
        txXfer->setTxFin();
        if (!trySendNewData(false)) tx_sem.release();
    }
}

void SerialIO::send(TxXfer &xfer) {
    tx_mb.instant_put((rt_ubase_t)&xfer);
    trySendNewData(true);
}

bool SerialIO::getchar(uint8_t &data) {
    rt_ubase_t tmp;
    if (!rx_mb.get(tmp, timeout)) return false;
    data = tmp;
    return true;
}

bool SerialIO::getbuf(uint8_t *buf, size_t cnt) {
    for (int i = 0; i < cnt; ++i)
        if (!getchar(buf[i])) return false;
    return true;
}

bool SerialIO::waitHeaderUtil() {
    static_assert(HeaderSize == 4);
    uint8_t buf[4] = {1, 1, 1, 1};
    for (int i = 0;; i = (i + 1) & 3) {
        if (!getchar(buf[i])) return false;
        bool success = true;
        for (int j = 0; j < 4; ++j) {
            if (buf[(i + j + 1) & 3] != HEADER[j]) {
                success = false;
                break;
            }
        }
        if (success) return true;
    }
}

void SerialIO::waitHeader() {
    while (!waitHeaderUtil())
        ;
}
