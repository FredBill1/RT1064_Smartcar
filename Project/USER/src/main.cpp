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
    // wireless.init("wireless", USART_2, 115200, UART2_TX_B18, UART2_RX_B19); // 编码器
    // wireless.init("wireless", USART_3, 115200, UART3_TX_C8 , UART3_RX_C9 );
    // wireless.init("wireless", USART_4, 115200, UART4_TX_C16, UART4_RX_C17);
    // wireless.init("wireless", USART_5, 115200, UART5_TX_C28, UART5_RX_C29);
    EnableGlobalIRQ(0);
    const uint8 ttt[]{"  hello\n\r"};
    while (1) {
        //此处编写需要循环执行的代码
        // wireless.sendHeader();
        // wireless.putchar(3);
        // wireless.writeD(a, b, c);
        wireless.putchar(wireless.getchar());
        wireless.putbuff(ttt, sizeof(ttt) - 1);
        gpio_toggle(B9);
        // rt_thread_mdelay(1000);
    }
}
