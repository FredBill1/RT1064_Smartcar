#include "devices.hpp"

SerialIO uart2(100);
SerialIO uart3(100);
SerialIO uart4(100);
SerialIO uart5(100);
SerialIO wireless(100);

ICM20948 imu(ICM20948_CONFIG);
IPS ips;

void initDevices() {
    uart2.init("UART2", UART2_CONFIG);
    uart3.init("UART3", UART3_CONFIG);
    uart4.init("UART4", UART4_CONFIG);
    uart5.init("UART5", UART5_CONFIG);
    wireless.init("Wireless", UART8_CONFIG);
    imu.init();
}