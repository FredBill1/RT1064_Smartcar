extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "Encoder.hpp"
#include "SerialIO.hpp"
SerialIO wireless;
QTimer encoder1(QTIMER_1, QTIMER1_TIMER0_C0, QTIMER1_TIMER1_C1);
QTimer encoder2(QTIMER_1, QTIMER1_TIMER2_C2, QTIMER1_TIMER3_C24);
QTimer encoder3(QTIMER_2, QTIMER2_TIMER0_C3, QTIMER2_TIMER3_C25);
// QTimer encoder4(QTIMER_3, QTIMER3_TIMER2_B18, QTIMER3_TIMER3_B19);
QTimer encoder4(QTIMER_3, QTIMER3_TIMER0_B16, QTIMER3_TIMER1_B17);

double a = 1e9 + 7, b = 1e18, c = std::numeric_limits<double>::max();

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)
    rt_thread_mdelay(300);

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    // rt_thread_mdelay(500);
    encoder1.init();
    encoder2.init();
    encoder3.init();
    encoder4.init();
    ips114_init();

    wireless.init("wireless", WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);
    // wireless.init("wireless", USART_2, 115200, UART2_TX_B18, UART2_RX_B19); // 编码器
    // wireless.init("wireless", USART_3, 115200, UART3_TX_C8 , UART3_RX_C9 );
    // wireless.init("wireless", USART_4, 115200, UART4_TX_C16, UART4_RX_C17);
    // wireless.init("wireless", USART_5, 115200, UART5_TX_C28, UART5_RX_C29);
    EnableGlobalIRQ(0);
    ips114_clear(WHITE);
    const uint8 ttt[]{"  hello\n\r"};
    while (1) {
        //此处编写需要循环执行的代码
        // wireless.sendHeader();
        // wireless.putchar(3);
        // wireless.writeD(a, b, c);
        // wireless.putchar(wireless.getchar());
        // wireless.putbuff(ttt, sizeof(ttt) - 1);
        // gpio_toggle(B9);
        ips114_showint16(0, 1, encoder1.get());
        ips114_showint16(0, 2, encoder2.get());
        ips114_showint16(0, 3, encoder3.get());
        ips114_showint16(0, 4, encoder4.get());
        rt_thread_mdelay(100);
        // rt_thread_mdelay(1000);
    }
}
