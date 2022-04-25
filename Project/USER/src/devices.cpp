#include "devices.hpp"

#include "parameters.hpp"

SerialIO uart2(UART2_CONFIG);
SerialIO uart3(UART3_CONFIG);
SerialIO uart4(UART4_CONFIG);
SerialIO uart5(UART5_CONFIG);
SerialIO wireless(UART8_CONFIG);

ICM20948 imu(ICM20948_CONFIG);
IPS ips;

MotorDRV motorDrvL1(MOTORDRV_L1_CONFIG, Param::Motor::L1::deadzone);
MotorDRV motorDrvL2(MOTORDRV_L2_CONFIG, Param::Motor::L2::deadzone);
MotorDRV motorDrvR1(MOTORDRV_R1_CONFIG, Param::Motor::R1::deadzone);
MotorDRV motorDrvR2(MOTORDRV_R2_CONFIG, Param::Motor::R2::deadzone);

extern const QTimer qtimerL1(EncoderL1_CONFIG);
extern const QTimer qtimerL2(EncoderL2_CONFIG);
extern const QTimer qtimerR1(EncoderR1_CONFIG);
extern const QTimer qtimerR2(EncoderR2_CONFIG);

Encoder encoderL1(qtimerL1);
Encoder encoderL2(qtimerL2);
Encoder encoderR1(qtimerR1);
Encoder encoderR2(qtimerR2);

// clang-format off
controller::LADRC controllerL1(Param::Motor::L1::wc, Param::Motor::L1::wo, Param::Motor::L1::b0, Param::MotorControlPeriod * 0.001f);
controller::LADRC controllerL2(Param::Motor::L2::wc, Param::Motor::L2::wo, Param::Motor::L2::b0, Param::MotorControlPeriod * 0.001f);
controller::LADRC controllerR1(Param::Motor::R1::wc, Param::Motor::R1::wo, Param::Motor::R1::b0, Param::MotorControlPeriod * 0.001f);
controller::LADRC controllerR2(Param::Motor::R2::wc, Param::Motor::R2::wo, Param::Motor::R2::b0, Param::MotorControlPeriod * 0.001f);
// clang-format on

MotorCtrl motorCtrlL1(motorDrvL1, encoderL1, controllerL1);
MotorCtrl motorCtrlL2(motorDrvL2, encoderL2, controllerL2);
MotorCtrl motorCtrlR1(motorDrvR1, encoderR1, controllerR1);
MotorCtrl motorCtrlR2(motorDrvR2, encoderR2, controllerR2);

MoveBase moveBase;

Systick systick;

extern const GPIO led(B9);
extern const GPIO beep(B11);

extern const GPIO btn_c4(C4);
extern const GPIO btn_c26(C26);
extern const GPIO btn_c27(C27);
extern const GPIO btn_c31(C31);
extern const GPIO switch_d27(D27);
extern const GPIO switch_d4(D4);

extern const GPIO master_key[5]{MASTER_KEY};
extern const GPIO master_switch[3]{MASTER_SWITCH};
extern const GPIO slave_key[5]{SLAVE_KEY};
extern const GPIO slave_switch[3]{SLAVE_SWITCH};

void initDevices() {
    // motor pwm
    motorDrvL1.init();
    motorDrvL2.init();
    motorDrvR1.init();
    motorDrvR2.init();

    rt_thread_mdelay(500);
    systick.init();

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
    // btn_c4.init(true);
    // btn_c26.init(true);
    // btn_c27.init(true);
    // btn_c31.init(true);
    // switch_d27.init(true);
    // switch_d4.init(true);
    for (auto& gpio : master_key) gpio.init(true);
    for (auto& gpio : master_switch) gpio.init(true);
    for (auto& gpio : slave_key) gpio.init(true);
    for (auto& gpio : slave_switch) gpio.init(true);

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