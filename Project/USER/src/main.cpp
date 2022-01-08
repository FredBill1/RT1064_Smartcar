extern "C" {
#include "headfile.h"
int main(void);
}

#include <Eigen/Eigen>

#include "SerialIO.hpp"

SerialIO wireless;

double aa, bb, cc;

template <typename T> inline void showMat(T& x) {
    for (int i = 0; i < x.rows(); ++i) {
        for (int j = 0; j < x.cols(); ++j) rt_kprintf("%d ", x(i, j));
        rt_kputs("\n\r");
    }
}

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    // rt_thread_mdelay(500);

    wireless.init("wireless", WIRELESS_UART, WIRELESS_UART_BAUD, WIRELESS_UART_TX, WIRELESS_UART_RX);
    EnableGlobalIRQ(0);

    Eigen::Matrix<int, 3, 3> x;
    x << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    auto y = x.adjoint();
    showMat(x);
    showMat(y);
    showMat(x * y);

    while (1) {
        //此处编写需要循环执行的代码
        wireless.waitHeader();
        if (wireless.readD(aa, bb, cc)) {
            rt_kprintf("%d %d %d\n", (int)aa, (int)bb, (int)cc);
            wireless.sendHeader();
            wireless.writeD(aa, bb, cc);
        } else
            rt_kputs("error\n");
        gpio_toggle(B9);
    }
}
