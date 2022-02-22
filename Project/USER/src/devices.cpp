#include "devices.hpp"

SerialIO uart2(UART2_CONFIG);
SerialIO uart3(UART3_CONFIG);
SerialIO uart4(UART4_CONFIG);
SerialIO uart5(UART5_CONFIG);
SerialIO wireless(UART8_CONFIG);

ICM20948 imu(ICM20948_CONFIG);
IPS ips;

MotorDRV motorDrvL1(MOTORDRV_L1_CONFIG);
MotorDRV motorDrvL2(MOTORDRV_L2_CONFIG);
MotorDRV motorDrvR1(MOTORDRV_R1_CONFIG);
MotorDRV motorDrvR2(MOTORDRV_R2_CONFIG);

void initDevices() {
    ips.init();

    uart2.init();
    uart3.init();
    uart4.init();
    uart5.init();
    wireless.init();

    motorDrvL1.init();
    motorDrvL2.init();
    motorDrvR1.init();
    motorDrvR2.init();

    // imu.init();
}