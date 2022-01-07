extern "C" {
#include "headfile.h"
int main(void);
}

#include "SerialIO.hpp"

SerialIO wireless;

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);

    wireless.init("wireless", WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);

    EnableGlobalIRQ(0);

    while (1) {
        //此处编写需要循环执行的代码
        char t = wireless.getchar();
        rt_kprintf("%c\n", t);
        gpio_toggle(B9);
        // rt_thread_mdelay(100);
    }
}
