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

GPIO led(B9);
GPIO beep(B11);

GPIO btn_c4(C4);
GPIO btn_c26(C26);
GPIO btn_c27(C27);
GPIO btn_c31(C31);
GPIO switch_d27(D27);
GPIO switch_d4(D4);

void initDevices() {
    // motor pwm
    motorDrvL1.init();
    motorDrvL2.init();
    motorDrvR1.init();
    motorDrvR2.init();

    rt_thread_mdelay(500);

    // screen
    ips.init();

    // serial
    uart2.init();
    uart3.init();
    uart4.init();
    uart5.init();
    wireless.init();

    // gpio
    led.init(false);
    beep.init(false);
    btn_c4.init(true);
    btn_c26.init(true);
    btn_c27.init(true);
    btn_c31.init(true);
    switch_d27.init(true);
    switch_d4.init(true);

    ips.puts("Initialization Complete.");
}