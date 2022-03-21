#ifndef _devices_hpp
#define _devices_hpp

#include "Encoder.hpp"
#include "GPIO.hpp"
#include "ICM20948.hpp"
#include "IPS.hpp"
#include "MotorDRV.hpp"
#include "SerialIO.hpp"

//
#include "PinConfig.h"

extern GPIO led;
extern GPIO beep;

extern GPIO btn_c4;
extern GPIO btn_c26;
extern GPIO btn_c27;
extern GPIO btn_c31;
extern GPIO switch_d27;
extern GPIO switch_d4;

extern SerialIO uart2;
extern SerialIO uart3;
extern SerialIO uart4;
extern SerialIO uart5;
extern SerialIO wireless;

extern ICM20948 imu;

extern IPS ips;

extern MotorDRV motorDrvL1, motorDrvL2, motorDrvR1, motorDrvR2;

void initDevices();

#endif  // _devices_hpp