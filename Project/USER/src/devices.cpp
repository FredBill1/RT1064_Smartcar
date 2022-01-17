#include "devices.hpp"

SerialIO uart2(100);
SerialIO uart3(100);
SerialIO uart4(100);
SerialIO uart5(100);
SerialIO wireless(500);

ICM20948 imu(ICM20948_CONFIG);
IPS ips;

MotorDRV motorDrvL1(MOTORDRV_L1_CONFIG);
MotorDRV motorDrvL2(MOTORDRV_L2_CONFIG);
MotorDRV motorDrvR1(MOTORDRV_R1_CONFIG);
MotorDRV motorDrvR2(MOTORDRV_R2_CONFIG);

void initDevices() {
    ips.init();
    uart2.init("UART2", UART2_CONFIG);
    uart3.init("UART3", UART3_CONFIG);
    uart4.init("UART4", UART4_CONFIG);
    uart5.init("UART5", UART5_CONFIG);
    wireless.init("Wireless", UART8_CONFIG);

    motorDrvL1.init();
    motorDrvL2.init();
    motorDrvR1.init();
    motorDrvR2.init();

    // imu.init();
}