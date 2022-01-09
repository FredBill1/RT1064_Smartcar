extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "SerialIO.hpp"
SerialIO wireless;

double a = 1e9 + 7, b = 1e18, c = std::numeric_limits<double>::max();

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    // rt_thread_mdelay(500);

    wireless.init("wireless", WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);
    EnableGlobalIRQ(0);

    while (1) {
        //此处编写需要循环执行的代码
        wireless.sendHeader();
        wireless.putchar(3);
        wireless.writeD(a, b, c);
        gpio_toggle(B9);
        rt_thread_mdelay(1000);
    }
}
