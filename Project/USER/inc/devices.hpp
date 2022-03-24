#ifndef _devices_hpp
#define _devices_hpp

#include "Encoder.hpp"
#include "GPIO.hpp"
#include "ICM20948.hpp"
#include "IPS.hpp"
#include "MotorDRV.hpp"
#include "QTimer.hpp"
#include "SerialIO.hpp"

//
#include "PinConfig.h"

extern const GPIO led;
extern const GPIO beep;

extern const GPIO btn_c4;
extern const GPIO btn_c26;
extern const GPIO btn_c27;
extern const GPIO btn_c31;
extern const GPIO switch_d27;
extern const GPIO switch_d4;

extern SerialIO uart2;
extern SerialIO uart3;
extern SerialIO uart4;
extern SerialIO uart5;
extern SerialIO wireless;

extern ICM20948 imu;

extern IPS ips;

extern const MotorDRV motorDrvL1, motorDrvL2, motorDrvR1, motorDrvR2;

extern const QTimer qtimerL1, qtimerL2, qtimerR1, qtimerR2;
extern Encoder encoderL1, encoderL2, encoderR1, encoderR2;

void initDevices();

#endif  // _devices_hpp