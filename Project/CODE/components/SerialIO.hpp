#ifndef _SerialIO_hpp
#define _SerialIO_hpp

#include <rtthread.h>
extern "C" {
#include "zf_uart.h"
}

class SerialIO : private lpuart_handle_t {
 private:
    rt_mailbox_t mb;
    uint8 rx_bf;

    static void rx_cb(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

 public:
    void init(const char *name, UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin);
    uint8 getchar();
};

#endif  // _SerialIO_hpp