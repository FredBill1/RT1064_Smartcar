#include "devices.hpp"

SerialIO uart2(UART2_CONFIG);
SerialIO uart3(UART3_CONFIG);
SerialIO uart4(UART4_CONFIG);
SerialIO uart5(UART5_CONFIG);
SerialIO wireless(UART8_CONFIG);

ICM20948 imu(ICM20948_CONFIG);
IPS ips;

extern const MotorDRV motorDrvL1(MOTORDRV_L1_CONFIG);
extern const MotorDRV motorDrvL2(MOTORDRV_L2_CONFIG);
extern const MotorDRV motorDrvR1(MOTORDRV_R1_CONFIG);
extern const MotorDRV motorDrvR2(MOTORDRV_R2_CONFIG);

extern const QTimer qtimerL1(EncoderL1_CONFIG);
extern const QTimer qtimerL2(EncoderL2_CONFIG);
extern const QTimer qtimerR1(EncoderR1_CONFIG);
extern const QTimer qtimerR2(EncoderR2_CONFIG);

Encoder encoderL1(qtimerL1);
Encoder encoderL2(qtimerL2);
Encoder encoderR1(qtimerR1);
Encoder encoderR2(qtimerR2);

extern const GPIO led(B9);
extern const GPIO beep(B11);

extern const GPIO btn_c4(C4);
extern const GPIO btn_c26(C26);
extern const GPIO btn_c27(C27);
extern const GPIO btn_c31(C31);
extern const GPIO switch_d27(D27);
extern const GPIO switch_d4(D4);

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

    // qtimer
    qtimerL1.init();
    qtimerL2.init();
    qtimerR1.init();
    qtimerR2.init();

    // encoder
    encoderL1.init();
    encoderL2.init();
    encoderR1.init();
    encoderR2.init();

    ips.puts("Initialization Complete.");
}